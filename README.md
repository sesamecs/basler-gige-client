# Basler GigE Camera EPICS Client

This [EPICS](http://www.aps.anl.gov/epics/) channel access client connects to the EPICS Basler GigE camera [driver](https://github.com/sesamecs/basler-gige-epics).

It provides access to the video stream and image settings (e.g. resolution, gain, trigger source, ...) found on the camera.

The client was developed by [SESAME](http://sesame.org.jo/sesame/) and is currently under use in its booster ring.

## Screenshots

![Screenshot 1](https://raw.githubusercontent.com/sesamecs/basler-gige-client/master/screenshots/TL1-HC.png)
![Screenshot 2](https://raw.githubusercontent.com/sesamecs/basler-gige-client/master/screenshots/VBL-G.png)

## Libraries

* OpenGL under compatibility mode (to run on the stable computers without the newer OpenGL drivers)
* [SDL](https://www.libsdl.org/)
* [AntTweakBar](http://anttweakbar.sourceforge.net/)
* [EPICS CA](http://www.aps.anl.gov/epics/docs/ca.php)

## Licenses

### License

This software is licensed under GNU LGPL version 2.1 license available [here](https://www.gnu.org/licenses/lgpl-2.1.txt).

### SDL license

SDL 1.2 is licensed under the GNU LGPL version 2.1 license available [here](https://www.gnu.org/licenses/lgpl-2.1.txt).

### AntTweakBar

The AntTweakBar library is free to use and redistribute. It is released under the zlib/libpng license available [here](http://opensource.org/licenses/Zlib).
