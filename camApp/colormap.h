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