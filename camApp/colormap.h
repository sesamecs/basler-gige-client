#ifndef COLORMAP_H
#define COLORMAP_H

typedef enum { GRAYSCALE, HOTCOLD } ColormapType;

struct Colormap { // Colormap interface
  ColormapType type;
  unsigned char (*red_transform)(unsigned char);   // color transformation function
  unsigned char (*green_transform)(unsigned char); // color transformation function
  unsigned char (*blue_transform)(unsigned char);  // color transformation function
};

void init_colormap(ColormapType, struct Colormap*);

#endif