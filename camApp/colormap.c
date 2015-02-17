#include <assert.h>

#include "colormap.h"

static unsigned char identity_transform(unsigned char grayscale) {
  return grayscale;
}

static unsigned char hotcold_red_transform(unsigned char grayscale) {
  if (grayscale <= 64 * 2) return 0;
  if (grayscale >= 64 * 3) return 255;
  return (grayscale - 64 * 3) * 4;
}

static unsigned char hotcold_green_transform(unsigned char grayscale) {
  if (grayscale < 64 * 1) return grayscale * 4;
  if (grayscale <= 64 * 3) return 255;
  return (64 * 4 - grayscale) * 4;
}

static unsigned char hotcold_blue_transform(unsigned char grayscale) {
  if (grayscale <= 64 * 1) return 255;
  if (grayscale >= 64 * 2) return 0;
  return (64 * 2 - grayscale) * 4;
}

static void init_grayscale_colormap(struct Colormap *colormap) {
  colormap->type = GRAYSCALE;
  colormap->red_transform = identity_transform;
  colormap->green_transform = identity_transform;
  colormap->blue_transform = identity_transform;
}

static void init_hotcold_colormap(struct Colormap *colormap) {
  colormap->type = HOTCOLD;
  colormap->red_transform = hotcold_red_transform;
  colormap->green_transform = hotcold_green_transform;
  colormap->blue_transform = hotcold_blue_transform;
}

void init_colormap(ColormapType type, struct Colormap* colormap) {
  switch (type) {
    case GRAYSCALE:
      init_grayscale_colormap(colormap);
      break;
    case HOTCOLD:
      init_hotcold_colormap(colormap);
      break;
    default:
      assert(0);
  }
}
