#include "img_save.h"

#include <stdio.h>
#include <png.h>

bool img_save_color(struct RGBPixel* pixels, int width, int height, const char* filepath) {
  bool success = false;

  FILE* fp = fopen(filepath, "wb");
  if (!fp) {
    fprintf(stderr, "unable to write file '%s'\n", filepath);
    return false;
  }

  png_structp png = png_create_write_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);
  if (!png) {
    fprintf(stderr, "unable to create png struct\n");
    goto file_cleanup;
  }

  png_infop info = png_create_info_struct(png);
  if (!info) {
    fprintf(stderr, "unable to create png info struct\n");
    goto png_cleanup;
  }

  if (setjmp(png_jmpbuf(png))) {
    fprintf(stderr, "png error (setjmp triggered)\n");
    goto info_cleanup;
  }

  png_init_io(png, fp);
  png_set_IHDR(png, info, width, height, 8, PNG_COLOR_TYPE_RGB, PNG_INTERLACE_NONE, PNG_COMPRESSION_TYPE_DEFAULT, PNG_FILTER_TYPE_DEFAULT);
  png_write_info(png, info);

  int row;
  for (row = 0; row < height; row++) {
    png_write_row(png, (png_bytep) (pixels + width * row));
  }

  png_write_end(png, NULL);
  success = true;

info_cleanup:
  png_free_data(png, info, PNG_FREE_ALL, -1);
png_cleanup:
  png_destroy_write_struct(&png, &info);
file_cleanup:
  fclose(fp);

  return success;
}
