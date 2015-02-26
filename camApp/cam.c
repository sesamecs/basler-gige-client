/*
  This file is part of basler-gige-client.

  basler-gige-client is free software: you can redistribute it and/or modify
  it under the terms of the GNU Lesser General Public License as published by
  the Free Software Foundation.

  basler-gige-client is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public License
  along with basler-gige-client.  If not, see <http://www.gnu.org/licenses/>.

  Copyright (C) SESAME, Yazan Dabain <yazan.dabain@sesame.org.jo> 2014, 2015
*/

// system libraries
#include <math.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <pthread.h>

// SDL, OpenGL and AntTweakBar
#include <SDL.h>
#include <SDL_opengl.h>
#include <AntTweakBar.h>

// EPICS channel access
#include "cadef.h"
#include "dbDefs.h"

// Common header
#include "common.h"

// Colormaps
#include "colormap.h"

// Image saving
#include "img_save.h"

// Definitions
#define SHOW_DEBUG 0
#define TARGET_FPS 20

#define CAM_MAX_WIDTH 1296
#define CAM_MAX_HEIGHT 966
#define WIN_WIDTH 800
#define WIN_HEIGHT 600
#define DEPTH 32
#define LEFT_BAR_WIDTH 200
#define SHOW_AREA 0

#define ENFORCE(test, msg) if (!(test)) {fprintf(stderr, (msg)); exit(1);}

typedef enum { SOFTWARE, HARDWARE } TriggerSource;
typedef enum { ENABLED, DISABLED } CameraCaptureState;
typedef enum { MANUAL, AUTOMATIC } GainControl;

struct Image {
  struct GSPixel  original[CAM_MAX_WIDTH * CAM_MAX_HEIGHT]; // grayscale camera output (unprocessed)
  struct RGBPixel output[CAM_MAX_WIDTH * CAM_MAX_HEIGHT]; // processed RGB image
  unsigned long xprofile[CAM_MAX_WIDTH];                  // sum of grayscale component across a row
  unsigned long yprofile[CAM_MAX_HEIGHT];                 // sum of grayscale component across a column
  GLuint textureId;                                       // OpenGL texture id
  bool needs_texture_update;                              // flag to signal that the texture needs an update
                                                          // this flag is needed because the update needs to
                                                          // happen in the same thread that created the OpenGL context
  pthread_rwlock_t lock;                                  // read-write lock to synchronize access
};

union PVValue { // holder of the pv value
  long lng;
  TriggerSource trigger_source;
  GainControl gain_control;
};

struct PVCollection {
  chid get_pv;         // pv from the device input
  chid set_pv;         // pv for device output
  chid process_pv;     // pv used to trigger driver input processing
  union PVValue value; // union holding the value from the device input
};

// Global variables
static char *group_name; // camera pv name prefix (eg. TL1-DI-CAM1)

// PVs
static chid video_chid;      // waveform pv representing camera output
static chid cam_enable_chid; // binary pv to enable/disable camera

// PV collections
static struct PVCollection exposure_pv;
static struct PVCollection width_pv;
static struct PVCollection height_pv;
static struct PVCollection offx_pv;
static struct PVCollection offy_pv;
static struct PVCollection trigger_pv;
static struct PVCollection gain_pv;
static struct PVCollection gain_control_pv;

// general state
static CameraCaptureState camera_enabled = DISABLED;
static bool pv_connected = false;
static bool initialized = false;
static bool got_frame = false;
static float fps = 0.0;
static int win_width = WIN_WIDTH;
static int win_height = WIN_HEIGHT;
static int cam_render_offset_x = 0;
static int cam_render_offset_y = 0;
static float scale = 1.0;
static char* base_path;

// visualization settings
static struct Colormap colormap;
static bool show_profiles = false;

// image buffers
static struct Image img_pixmap[2];    // double image and texture buffering
static size_t img_current_buffer = 0;
static pthread_mutex_t buffer_switch_mutex;

// AntTweakBar
static TwBar* settings_bar;

static void show_message(const char* message) {
  if (settings_bar) {
    TwSetParam(settings_bar, "message", "label", TW_PARAM_CSTRING, 1, message);
  }
}

// checks whether a connection is establised, possibly waiting max_wait_time_ms
static bool has_connection(chid channel, int max_wait_time_ms) {
  int wait_time = 0;
  enum channel_state chst;
  while ((chst = ca_state(channel)) != cs_conn && wait_time < max_wait_time_ms) {
    usleep(1000);
    wait_time += 1;
  }

  if (chst != cs_conn) {
    show_message("Connection error");
    fprintf(stderr, "connection cannot be established\n");
    return false;
  }

  return true;
}

// mapping function from screen coordinates to camera coordinates in the X axis
static int from_screen_to_camera_x(int screen_x) {
  screen_x -= LEFT_BAR_WIDTH + cam_render_offset_x;
  screen_x /= scale;

  if (screen_x > width_pv.value.lng) screen_x = width_pv.value.lng;
  if (screen_x < 0) screen_x = 0;
  return screen_x;
}

// mapping function from screen coordinates to camera coordinates in the Y axis
static int from_screen_to_camera_y(int screen_y) {
  screen_y = win_height - screen_y;
  screen_y -= cam_render_offset_y;
  screen_y /= scale;

  if (screen_y > height_pv.value.lng) screen_y = height_pv.value.lng;
  if (screen_y < 0) screen_y = 0;
  return height_pv.value.lng - screen_y;
}

static void drawXProfile(struct Image* image) {
  int x;

  #if SHOW_AREA
  glColor4f(1.0, 1.0, 1.0, 0.4);
  glBegin(GL_LINES);

  for (x = LEFT_BAR_WIDTH + cam_render_offset_x; x < win_width - cam_render_offset_x; x++) {
    int col = from_screen_to_camera_x(x);

    float val = image->xprofile[col];
    val /= height_pv.value.lng;
    val *= (win_height - 2 * cam_render_offset_y) * 0.2;
    val /= 256.0;

    glVertex2d(x, cam_render_offset_y);
    glVertex2d(x, cam_render_offset_y + val);
  }

  glEnd();
  #endif

  glColor4f(1.0, 1.0, 1.0, 1.0);
  glBegin(GL_POINTS);

  for (x = LEFT_BAR_WIDTH + cam_render_offset_x; x < win_width - cam_render_offset_x; x++) {
    int col = from_screen_to_camera_x(x);

    float val = image->xprofile[col];
    val /= height_pv.value.lng;
    val *= (win_height - 2 * cam_render_offset_y) * 0.2;
    val /= 256.0;

    glVertex2d(x, cam_render_offset_y + val);
  }

  glEnd();
}

static void drawYProfile(struct Image* image) {
  int y;

  #if SHOW_AREA
  glColor4f(1.0, 1.0, 1.0, 0.4);
  glBegin(GL_LINES);

  for (y = cam_render_offset_y; y < win_height - cam_render_offset_y; y++) {
    int row = from_screen_to_camera_y(y);

    float val = image->yprofile[row];
    val /= width_pv.value.lng;
    val *= (win_width - 2 * cam_render_offset_x - LEFT_BAR_WIDTH) * 0.2;
    val /= 256.0;

    glVertex2d(LEFT_BAR_WIDTH + cam_render_offset_x, y);
    glVertex2d(LEFT_BAR_WIDTH + cam_render_offset_x + val, y);
  }

  glEnd();
  #endif

  glColor4f(1.0, 1.0, 1.0, 1.0);
  glBegin(GL_POINTS);

  for (y = cam_render_offset_y; y < win_height - cam_render_offset_y; y++) {
    int row = from_screen_to_camera_y(y);

    float val = image->yprofile[row];
    val /= width_pv.value.lng;
    val *= (win_width - 2 * cam_render_offset_x - LEFT_BAR_WIDTH) * 0.2;
    val /= 256.0;

    glVertex2d(LEFT_BAR_WIDTH + cam_render_offset_x + val, y);
  }

  glEnd();
}

static void render() {
  glViewport(0, 0, win_width, win_height);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  glOrtho(0, win_width, 0, win_height, 1, -1);

  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();

  glClear(GL_COLOR_BUFFER_BIT);

  int drawing_area_width = win_width - LEFT_BAR_WIDTH;
  int drawing_area_height = win_height;

  float xscale = (float) drawing_area_width / width_pv.value.lng;
  float yscale = (float) drawing_area_height / height_pv.value.lng;

  if (xscale > yscale) {
    scale = yscale;

    int extra_pixels = drawing_area_width - width_pv.value.lng * yscale;
    cam_render_offset_x = extra_pixels / 2;
    cam_render_offset_y = 0;
  } else {
    scale = xscale;

    int extra_pixels = drawing_area_height - height_pv.value.lng * xscale;
    cam_render_offset_x = 0;
    cam_render_offset_y = extra_pixels / 2;
  }

  struct Image* current_image = &img_pixmap[img_current_buffer];
  pthread_rwlock_rdlock(&current_image->lock); // disallow writers to access current_image

  // use current texture
  glBindTexture(GL_TEXTURE_2D, current_image->textureId);
  glBegin(GL_QUADS); // draw textured quad
    glTexCoord2i(0, 1);
    glVertex3f(LEFT_BAR_WIDTH + cam_render_offset_x, cam_render_offset_y, 0);

    glTexCoord2i(1, 1);
    glVertex3f(LEFT_BAR_WIDTH + cam_render_offset_x + width_pv.value.lng * scale, cam_render_offset_y, 0);

    glTexCoord2i(1, 0);
    glVertex3f(LEFT_BAR_WIDTH + cam_render_offset_x + width_pv.value.lng * scale, cam_render_offset_y + height_pv.value.lng * scale, 0);

    glTexCoord2i(0, 0);
    glVertex3f(LEFT_BAR_WIDTH + cam_render_offset_x, cam_render_offset_y + height_pv.value.lng * scale, 0);
  glEnd();

  if (show_profiles) {
    drawXProfile(current_image);
    drawYProfile(current_image);
  }

  pthread_rwlock_unlock(&current_image->lock);

  TwDraw();
  SDL_GL_SwapBuffers();
}

static void update_textures() {
  // texture updates must happen in the thread that has the opengl context
  if (img_pixmap[img_current_buffer].needs_texture_update) {
    glBindTexture(GL_TEXTURE_2D, img_pixmap[img_current_buffer].textureId);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width_pv.value.lng, height_pv.value.lng, 0, GL_RGB, GL_UNSIGNED_BYTE, img_pixmap[img_current_buffer].output);
    img_pixmap[img_current_buffer].needs_texture_update = false;
  }
}

static void black_screen() {
  pthread_mutex_lock(&buffer_switch_mutex);
  size_t img_new_buffer = 1 - img_current_buffer;
  pthread_mutex_unlock(&buffer_switch_mutex);

  struct Image* new_image = &img_pixmap[img_new_buffer];
  pthread_rwlock_wrlock(&new_image->lock);
  // black out pixmap
  memset(&new_image->original, 0, sizeof(new_image->original));
  memset(&new_image->output, 0, sizeof(new_image->output));
  memset(&new_image->xprofile, 0, sizeof(new_image->xprofile));
  memset(&new_image->yprofile, 0, sizeof(new_image->yprofile));
  new_image->needs_texture_update = true;
  pthread_rwlock_unlock(&new_image->lock);

  pthread_mutex_lock(&buffer_switch_mutex);
  img_current_buffer = img_new_buffer;
  pthread_mutex_unlock(&buffer_switch_mutex);
}

static void video_connection_state_callback(struct connection_handler_args args) {
  // warning: this runs in a different thread
  pv_connected = (args.op == CA_OP_CONN_UP);

  if (!pv_connected) {
    show_message("Video is disconnected");
    black_screen();
    camera_enabled = DISABLED;
  }

  #if SHOW_DEBUG
  printf("connection state: %s\n", pv_connected ? "connected" : "disconnected");
  #endif

  if (initialized) {
    char window_caption[1024];
    snprintf(window_caption, sizeof(window_caption), "%s (%s)", group_name, pv_connected ? "connected" : "disconnected");
    SDL_WM_SetCaption(window_caption, NULL);
  }
}

static void enable_cam(CameraCaptureState state) {
  if (!has_connection(cam_enable_chid, 1000)) {
    fprintf(stderr, "cannot enable/disable camera: pv is disconnected\n");
    return;
  }

  ca_put(DBR_LONG, cam_enable_chid, &state);
  ca_pend_io(5.0);

  if (state == DISABLED) fps = 0.0;
}

static void TW_CALL enable_cam_tw(void* clientData) {
  CameraCaptureState state = (CameraCaptureState) clientData; // clientData is the state
  enable_cam(state);
}

static void cam_enable_callback(struct event_handler_args eha) {
  // warning: this runs in a different thread
  if (eha.status != ECA_NORMAL) {
    printf("abnormal status: %d\n", eha.status);
    show_message("Invalid PV state");
  } else {
    camera_enabled = *(int*) eha.dbr == 0 ? ENABLED : DISABLED;
    if (camera_enabled == ENABLED) {
      show_message("Capturing started");
    } else {
      show_message("Capturing stopped");
    }
  }
}

static void video_stream_callback(struct event_handler_args eha) {
  // warning: this runs in a different thread
  if (eha.status != ECA_NORMAL) {
    printf("abnormal status: %d\n", eha.status);
    show_message("Invalid PV state");
  } else {
    got_frame = true;

    static struct timespec last_timestamp;
    struct timespec current_timestamp;
    clock_gettime(CLOCK_REALTIME, &current_timestamp);

    float interval = (current_timestamp.tv_sec + current_timestamp.tv_nsec / 1.0e9) - (last_timestamp.tv_sec + last_timestamp.tv_nsec / 1.0e9);
    last_timestamp = current_timestamp;

    fps = 1.0 / interval;

    #if SHOW_DEBUG
    fprintf(stderr, "got data (addr: %p, len: %lu, frame rate: %0.2f)\n", eha.dbr, eha.count, 1.0 / interval);
    #endif

    unsigned char *pdata = (unsigned char *) eha.dbr;

    pthread_mutex_lock(&buffer_switch_mutex);
    size_t img_new_buffer = 1 - img_current_buffer;
    pthread_mutex_unlock(&buffer_switch_mutex);

    struct Image* new_image = &img_pixmap[img_new_buffer];
    pthread_rwlock_wrlock(&new_image->lock);

    memset(&new_image->xprofile, 0, sizeof(new_image->xprofile));
    memset(&new_image->yprofile, 0, sizeof(new_image->yprofile));

    int i, x = 0, y = 0;
    for (i = 0; i < eha.count && i < width_pv.value.lng * height_pv.value.lng; i++) {
      if (++x == width_pv.value.lng) {
        x = 0;
        y++;
      }

      unsigned char p_value = pdata[i];
      new_image->original[i].v = p_value;
      new_image->output[i].r = colormap.red_transform(p_value);   // apply color transformation
      new_image->output[i].g = colormap.green_transform(p_value); // apply color transformation
      new_image->output[i].b = colormap.blue_transform(p_value);  // apply color transformation

      new_image->xprofile[x] += p_value;
      new_image->yprofile[y] += p_value;
    }
    new_image->needs_texture_update = true; // mark for update on next render
    pthread_rwlock_unlock(&new_image->lock);

    pthread_mutex_lock(&buffer_switch_mutex);
    img_current_buffer = img_new_buffer;
    pthread_mutex_unlock(&buffer_switch_mutex);
  }
}

static void update_value_callback(struct event_handler_args eha) {
  // warning: this runs in a different thread
  if (eha.status != ECA_NORMAL) {
    printf("abnormal status: %d\n", eha.status);
    show_message("Invalid PV state");
  } else {
    // set the provided variable to the value of the pv
    *((long *) eha.usr) = *((long *) eha.dbr); // eha.usr is the pointer to the PVValue associated with the PV

    // if gain control value changed
    if (initialized && (GainControl *)eha.usr == &(gain_control_pv.value.gain_control)) {
      // disable gain field in the tweak bar if the gain control is automatic
      int isReadonly = (gain_control_pv.value.gain_control == AUTOMATIC);
      TwSetParam(settings_bar, "gain", "readonly", TW_PARAM_INT32, 1, &isReadonly);
    }
  }
}

static void TW_CALL tw_bar_set_value_callback(const void *value, void *clientData) {
  struct PVCollection *collection = (struct PVCollection*) clientData;
  long v = *(uint32_t*) value;

  // make sure setter is processed before getter
  CA_SYNC_GID group;
  ca_sg_create(&group);
  ca_sg_put(group, DBR_LONG, collection->set_pv, &v);
  ca_sg_block(group, 5);

  long process_value = 1;
  ca_put(DBR_LONG, collection->process_pv, &process_value);
  ca_flush_io();
}

static void TW_CALL tw_bar_get_value_callback(void *value, void *clientData) {
  struct PVCollection *collection = (struct PVCollection*) clientData;
  *(uint32_t*) value = collection->value.lng;
}

static void TW_CALL tw_bar_get_mouse_x(void *value, void *clientData) {
  int x, y;
  SDL_GetMouseState(&x, &y);
  *(int32_t*) value = (int32_t) from_screen_to_camera_x(x);
}

static void TW_CALL tw_bar_get_mouse_y(void *value, void *clientData) {
  int x, y;
  SDL_GetMouseState(&x, &y);
  *(int32_t*) value = (int32_t) from_screen_to_camera_y(y);
}

static void TW_CALL tw_bar_get_colormap_callback(void *value, void *clientData) {
  *(ColormapType*) value = colormap.type;
}

static void TW_CALL tw_bar_set_colormap_callback(const void *value, void *clientData) {
  ColormapType type = *(ColormapType*) value;
  init_colormap(type, &colormap);
}

static void TW_CALL tw_bar_get_show_profiles_callback(void *value, void *clientData) {
  *(bool*) value = show_profiles;
}

static void TW_CALL tw_bar_set_show_profiles_callback(const void *value, void *clientData) {
  show_profiles = *(bool*) value;
}

static void* take_shot_impl(void* uarg) {
  pthread_mutex_lock(&buffer_switch_mutex);
  struct Image* current_image = &img_pixmap[img_current_buffer];
  pthread_mutex_unlock(&buffer_switch_mutex);

  char date[128];
  time_t now = time(NULL);
  struct tm* t = localtime(&now);

  strftime(date, sizeof(date) - 1, "%Y-%m-%d_%H:%M:%S", t);

  char* path = (char*) calloc(strlen(base_path) + 1 /* / */ + strlen(group_name) + 1 /* _ */ + strlen(date) + 4 /* .png */ + 1, sizeof(char));

  strcat(path, base_path);
  if (strlen(base_path) > 0 && base_path[strlen(base_path) - 1] != '/') {
    strcat(path, "/");
  }
  strcat(path, group_name);
  strcat(path, "_");
  strcat(path, date);
  strcat(path, ".png");

  pthread_rwlock_rdlock(&current_image->lock);
  if (img_save_color(current_image->output, width_pv.value.lng, height_pv.value.lng, path)) {
    char msg[1024];
    snprintf(msg, sizeof(msg), "Shot saved to '%s'", path);
    show_message(msg);
  } else {
    show_message("Unable to save shot");
  }
  pthread_rwlock_unlock(&current_image->lock);

  free(path);
  return NULL;
}

static void TW_CALL take_shot(void* clientData) {
  pthread_t thread;
  pthread_create(&thread, NULL, take_shot_impl, NULL);
}

static void init_tw_bar() {
  TwInit(TW_OPENGL, NULL);
  TwWindowSize(WIN_WIDTH, WIN_HEIGHT);

  settings_bar = TwNewBar("main_bar");

  // Sizing group
  TwAddVarCB(settings_bar, "width", TW_TYPE_UINT32, tw_bar_set_value_callback, tw_bar_get_value_callback, &width_pv, "label=Width min=320 max=1296 step=100 group='Image Resolution'");
  TwAddVarCB(settings_bar, "height", TW_TYPE_UINT32, tw_bar_set_value_callback, tw_bar_get_value_callback, &height_pv, "label=Height min=240 max=966 step=100 group='Image Resolution'");

  // Offset group
  TwAddVarCB(settings_bar, "offset_x", TW_TYPE_UINT32, tw_bar_set_value_callback, tw_bar_get_value_callback, &offx_pv, "label=X min=0 max=1296 step=100 keyincr=RIGHT keydecr=LEFT group='Image Offset'");
  TwAddVarCB(settings_bar, "offset_y", TW_TYPE_UINT32, tw_bar_set_value_callback, tw_bar_get_value_callback, &offy_pv, "label=Y min=0 max=966 step=100 keyincr=DOWN keydecr=UP group='Image Offset'");

  // Camera settings
  TwAddVarCB(settings_bar, "exposure", TW_TYPE_UINT32, tw_bar_set_value_callback, tw_bar_get_value_callback, &exposure_pv, "label=Exposure min=16 max=1000000 step=100000 group='Camera Settings'");
  TwAddVarCB(settings_bar, "gain", TW_TYPE_UINT32, tw_bar_set_value_callback, tw_bar_get_value_callback, &gain_pv, "label=Gain min=300 max=850 step=50 group='Camera Settings'");

  TwEnumVal gain_control_ev[] = {{MANUAL, "Manual"}, {AUTOMATIC, "Automatic"}};
  TwType gain_control_type = TwDefineEnum("GainControlType", gain_control_ev, 2);
  TwAddVarCB(settings_bar, "gain_control", gain_control_type, tw_bar_set_value_callback, tw_bar_get_value_callback, &gain_control_pv, "label='Gain Control' group='Camera Settings'");

  TwEnumVal trigger_source_ev[] = {{SOFTWARE, "Software"}, {HARDWARE, "Hardware"}};
  TwType trigger_source_type = TwDefineEnum("TriggerSourceType", trigger_source_ev, 2);
  TwAddVarCB(settings_bar, "trigger_source", trigger_source_type, tw_bar_set_value_callback, tw_bar_get_value_callback, &trigger_pv, "label='Trigger Source' group='Camera Settings'");

  // Mouse Position
  TwAddVarCB(settings_bar, "mouse_x", TW_TYPE_INT32, NULL, tw_bar_get_mouse_x, NULL, "label=X group='Mouse Position in Image'");
  TwAddVarCB(settings_bar, "mouse_y", TW_TYPE_INT32, NULL, tw_bar_get_mouse_y, NULL, "label=Y group='Mouse Position in Image'");

  // Interface settings
  TwEnumVal colormap_ev[] = {{GRAYSCALE, "Grayscale"}, {HOTCOLD, "Hot-cold"}};
  TwType colormap_type = TwDefineEnum("ColormapType", colormap_ev, 2);
  TwAddVarCB(settings_bar, "colormap", colormap_type, tw_bar_set_colormap_callback, tw_bar_get_colormap_callback, NULL, "label=Colormap group=Interface");
  TwAddVarCB(settings_bar, "show_profiles", TW_TYPE_BOOL8, tw_bar_set_show_profiles_callback, tw_bar_get_show_profiles_callback, NULL, "label='Show Profiles' group=Interface");

  // Commands
  TwAddButton(settings_bar, "start_capture", enable_cam_tw, (void*) ENABLED, "label='Start capture' group=Commands");
  TwAddButton(settings_bar, "stop_capture", enable_cam_tw, (void*) DISABLED, "label='Stop capture' group=Commands");
  TwAddButton(settings_bar, "take_shot", take_shot, NULL, "label='Take shot' key=SPACE group=Commands");

  // Status
  TwAddVarRO(settings_bar, "connected", TW_TYPE_BOOL8, &pv_connected, "label=Connected true=Yes false=No group=State");
  TwAddVarRO(settings_bar, "capturing", TW_TYPE_BOOL8, &camera_enabled, "label=Capturing true=No false=Yes group=State");
  TwAddVarRO(settings_bar, "fps", TW_TYPE_FLOAT, &fps, "label=FPS precision=2 group=State");

  // Messages
  TwAddButton(settings_bar, "message", NULL, NULL, "label=' ' group='Last Message'");

  char def[2048];
  snprintf(def, sizeof(def),
    "main_bar label='%s' size='200 %d' refresh=0.5 color=`0 0 0` position=`0 0` movable=false resizable=false iconifiable=false fontresizable=false",
    group_name,
    WIN_HEIGHT
  );
  TwDefine(def);
}

static void init_gl() {
  glEnable(GL_TEXTURE_2D);

  glViewport(0, 0, WIN_WIDTH, WIN_HEIGHT);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  glOrtho(0, WIN_WIDTH, 0, WIN_HEIGHT, 1, -1);

  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();

  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glEnable(GL_BLEND);

  glClearColor(0.0f, 0.0f, 0.0f, 1.0f);

  int i;
  for (i = 0; i < 2; i++) {
    glGenTextures(1, &(img_pixmap[i].textureId));
    glBindTexture(GL_TEXTURE_2D, img_pixmap[i].textureId);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glBindTexture(GL_TEXTURE_2D, 0);
  }

  ENFORCE(glGetError() == GL_NO_ERROR, "opengl has error");
}

static void init_sdl() {
  ENFORCE(SDL_Init(SDL_INIT_VIDEO) >= 0, "sdl initialization failed");
  atexit(SDL_Quit);

  SDL_Surface *screen = SDL_SetVideoMode(WIN_WIDTH, WIN_HEIGHT, DEPTH, SDL_OPENGL | SDL_RESIZABLE);
  ENFORCE(screen != NULL, "invalid SDL screen");

  char window_caption[1024];
  snprintf(window_caption, sizeof(window_caption), "%s (%s)", group_name, pv_connected ? "connected" : "disconnected");
  SDL_WM_SetCaption(window_caption, NULL);
}

static void init_pv_collection(const char *property, bool monitor, long default_value, struct PVCollection *collection) {
  // create get pv chid (eg. TL1-DI-CAM1:getWidth)
  char get_pv_name[1024];
  snprintf(get_pv_name, sizeof(get_pv_name), "%s:get%s", group_name, property);
  SEVCHK(ca_create_channel(get_pv_name, NULL, NULL, CA_PRIORITY_DEFAULT, &(collection->get_pv)), "ca_create_channel");
  if (monitor) {
    SEVCHK(ca_create_subscription(DBR_LONG, 1, collection->get_pv, DBE_VALUE, update_value_callback, &(collection->value.lng), NULL), "ca_create_subscription");
  }

  // create set pv chid (eg. TL1-DI-CAM1:setWidth)
  char set_pv_name[1024];
  snprintf(set_pv_name, sizeof(set_pv_name), "%s:set%s", group_name, property);
  SEVCHK(ca_create_channel(set_pv_name, NULL, NULL, CA_PRIORITY_DEFAULT, &(collection->set_pv)), "ca_create_channel");

  // create proc pv chid (eg. TL1-DI-CAM1:getWidth.PROC)
  char proc_pv_name[1024];
  snprintf(proc_pv_name, sizeof(proc_pv_name), "%s:get%s.PROC", group_name, property);
  SEVCHK(ca_create_channel(proc_pv_name, NULL, NULL, CA_PRIORITY_DEFAULT, &(collection->process_pv)), "ca_create_channel");

  collection->value.lng = default_value;
}

static void init_epics() {
  SEVCHK(ca_context_create(ca_enable_preemptive_callback), "ca_context_create");

  // connect and monitor getImage pv with video callback
  char pv_name_vid[1024];
  snprintf(pv_name_vid, sizeof(pv_name_vid), "%s:getImage", group_name);
  SEVCHK(ca_create_channel(pv_name_vid, video_connection_state_callback, NULL, CA_PRIORITY_DEFAULT, &video_chid), "ca_create_channel");
  SEVCHK(ca_create_subscription(DBR_CHAR, CAM_MAX_WIDTH * CAM_MAX_HEIGHT, video_chid, DBE_VALUE, video_stream_callback, NULL, NULL), "ca_create_subscription");

  // connect the getImage.DISA pv to enable/disable CAM
  char pv_name_enable[1024];
  snprintf(pv_name_enable, sizeof(pv_name_enable), "%s:getImage.DISA", group_name);
  SEVCHK(ca_create_channel(pv_name_enable, NULL, NULL, CA_PRIORITY_DEFAULT, &cam_enable_chid), "ca_create_channel");
  SEVCHK(ca_create_subscription(DBR_INT, 1, cam_enable_chid, DBE_VALUE, cam_enable_callback, NULL, NULL), "ca_create_subscription");

  // initialize the variable pvs
  init_pv_collection("Width", true, CAM_MAX_WIDTH, &width_pv);
  init_pv_collection("Height", true, CAM_MAX_HEIGHT, &height_pv);
  init_pv_collection("OffsetX", true, 0, &offx_pv);
  init_pv_collection("OffsetY", true, 0, &offy_pv);
  init_pv_collection("Exposure", true, 100000, &exposure_pv);
  init_pv_collection("TriggerSource", true, SOFTWARE, &trigger_pv);
  init_pv_collection("Gain", true, 850, &gain_pv);
  init_pv_collection("GainAuto", true, AUTOMATIC, &gain_control_pv);

  SEVCHK(ca_flush_io(), "ca_flush_io");
}

// limits fps to TARGET_FPS using usleep
static void control_fps(int* frames, struct timespec* last_timestamp) {
  (*frames)++;

  struct timespec current_timestamp;
  clock_gettime(CLOCK_REALTIME, &current_timestamp);
  float interval = (current_timestamp.tv_sec + current_timestamp.tv_nsec / 1.0e9) - (last_timestamp->tv_sec + last_timestamp->tv_nsec / 1.0e9);

  float time_per_frame = interval / *frames;
  if (time_per_frame < (1.0 / TARGET_FPS))
  {
    while (true)
    {
      clock_gettime(CLOCK_REALTIME, &current_timestamp);
      interval = (current_timestamp.tv_sec + current_timestamp.tv_nsec / 1.0e9) - (last_timestamp->tv_sec + last_timestamp->tv_nsec / 1.0e9);
      if (interval / *frames >= (0.98 / TARGET_FPS)) break;
      usleep(1000);
    }
  }

  if (interval >= 2.0) {
    if (got_frame) {
      got_frame = false;
    } else {
      fps = 0;
    }
    *last_timestamp = current_timestamp;
    *frames = 0;
  }

  #if SHOW_DEBUG
  float rfps = *frames / interval;
  printf("Main loop fps: %0.2f\n", rfps);
  #endif
}

static void handle_resize(SDL_Event event) {
  SDL_SetVideoMode(event.resize.w, event.resize.h, DEPTH, SDL_OPENGL | SDL_RESIZABLE);
  win_width = event.resize.w;
  win_height = event.resize.h;

  TwWindowSize(win_width, win_height);

  int new_size[2] = {LEFT_BAR_WIDTH, win_height};
  TwSetParam(settings_bar, NULL, "size", TW_PARAM_INT32, 2, new_size);
}

static void main_loop() {
  bool stop = false;

  struct timespec last_timestamp;
  clock_gettime(CLOCK_REALTIME, &last_timestamp);

  int frames = 0;

  while (!stop) {
    update_textures();
    render();
    control_fps(&frames, &last_timestamp);

    SDL_Event event;
    while (SDL_PollEvent(&event)) {
      if (!TwEventSDL(&event, SDL_MAJOR_VERSION, SDL_MINOR_VERSION)) {
        if ((event.type == SDL_QUIT) || (event.type == SDL_KEYDOWN && event.key.keysym.sym == SDLK_q)) {
          stop = true;
          break;
        }

        if (event.type == SDL_VIDEORESIZE) handle_resize(event);
      }
    }
  }
}

static void init_locks() {
  pthread_mutex_init(&buffer_switch_mutex, NULL);
  pthread_rwlock_init(&img_pixmap[0].lock, NULL);
  pthread_rwlock_init(&img_pixmap[1].lock, NULL);
}

static void init_base_path() {
  base_path = getenv("CAM_CLIENT_IMG_DIRECTORY"); // take base path from environment
  if (base_path != NULL) return;
  base_path = getenv("HOME"); // put base path in home directory
  if (base_path != NULL) return;
  base_path = "/tmp/"; // put base path in tmp directory
}

int main(int argc,char **argv) {
  if (argc != 2) {
    fprintf(stderr, "Usage: %s <group>\n", argv[0]);
    exit(1);
  }

  group_name = argv[1];

  init_base_path();
  init_locks();
  init_colormap(HOTCOLD, &colormap);
  init_sdl();
  init_gl();
  init_epics();
  init_tw_bar();

  initialized = true;

  enable_cam(ENABLED);
  main_loop();
  enable_cam(DISABLED);

  TwTerminate();
  ca_context_destroy();

  return 0;
}
