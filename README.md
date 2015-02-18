# Basler GigE Camera EPICS Client

This [EPICS](http://www.aps.anl.gov/epics/) channel access client connects to the EPICS Basler GigE camera [driver](https://github.com/sesamecs/basler-gige-epics).

It provides access to the video stream and image settings (e.g. resolution, gain, trigger source, ...) found on the camera.

The client was developed by [SESAME](http://sesame.org.jo/sesame/) and is currently under use in its booster ring.

## How To Build

This client uses the EPICS build system. After you have an EPICS environment setup, you can fire `make` in the top directory to build the client.

## How To Run

The client expects a number of PVs on the network provided by the channel access protocol. These PVs are:
* `$(DEVICE):getImage`
* `$(DEVICE):getWidth`, `$(DEVICE):setWidth`
* `$(DEVICE):getHeight`, `$(DEVICE):setHeight`
* `$(DEVICE):getOffsetX`, `$(DEVICE):setOffsetX`
* `$(DEVICE):getOffsetY`, `$(DEVICE):setOffsetY`
* `$(DEVICE):getExposure`, `$(DEVICE):setExposure`
* `$(DEVICE):getTriggerSource`, `$(DEVICE):setTriggerSource`
* `$(DEVICE):getGain`, `$(DEVICE):setGain`
* `$(DEVICE):getGainAuto`, `$(DEVICE):setGainAuto`

The `$(DEVICE)` must be specified when running the binary as the first command-line argument.

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

## More Info About SESAME

[SESAME](http://sesame.org.jo/sesame/) (Synchrotron-light for Experimental Science and Applications in the Middle East) is a “third-generation” synchrotron light source under construction in Allan (Jordan). It will be the Middle East's first major international research centre.

It is a cooperative venture by scientists and governments of the region set up on the model of CERN (European Organization for Nuclear Research). It is being developed under the auspices of UNESCO (United Nations Educational, Scientific and Cultural Organization) following the formal approval given for this by the Organization's Executive Board (164th session, May 2002).
