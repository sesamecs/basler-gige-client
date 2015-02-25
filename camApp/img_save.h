#ifndef IMGSAVE_H
#define IMGSAVE_H

#include <stdbool.h>

#include "common.h"

bool img_save_color(struct RGBPixel* pixels, int width, int height, const char* filepath);
//bool img_save_gray (struct  GSPixel* pixels, int width, int height, const char* filepath);

#endif
