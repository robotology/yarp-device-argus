
![YARP logo](https://raw.githubusercontent.com/robotology/yarp/master/doc/images/yarp-robot-24.png "yarp-device-argus")
yarp-device-argus
========

This is the [argus](https://www.framos.com/en/products/fsm-imx415c-01s-v1a-23740) device for [YARP](https://www.yarp.it/).
It supports the [FRAMOS](https://www.framos.com/en/product-catalog/modules/framos-sensor-modules) cameras.

The **FRAMOS™** cameras currently compatible with YARP are:
- FRAMOS-IMX415 (obsolete)
- FRAMOS-IMX678

# 1. License

[![License](https://img.shields.io/badge/license-BSD--3--Clause%20%2B%20others-19c2d8.svg)](https://github.com/robotology/yarp-device-argus/blob/main/LICENSE)

This software may be modified and distributed under the terms of the
BSD-3-Clause license. See the accompanying LICENSE file for details.

The argusCamera device uses the
[Libargus API](https://docs.nvidia.com/jetson/l4t-multimedia/group__LibargusAPI.html), which is a module included in the [Jetson Linux Multimedia API](https://docs.nvidia.com/jetson/l4t-multimedia/index.html), released
under the [NVIDIA license](https://docs.nvidia.com/jetson/l4t-multimedia/nvidia_legal.html).
See the relative documentation for the terms of the license.

# 2. How to use FRAMOS cameras as a YARP device

## 2.1. Dependencies
Before proceeding further, please install the following dependencies:

- [Nvidia JetPack v6.2](https://developer.nvidia.com/embedded/jetpack-sdk-62) for `Jetson Orin NX` + [BSP r36.4.3](https://connecttech.com/ftp/Drivers/L4T-Release-Notes/Jetson-Orin-NX-Orin-Nano/ORIN-NX-NANO-36.4.3.pdf) for `Boson for FRAMOS` carrier board
- [YARP 3.12](https://github.com/robotology/yarp/releases/tag/v3.12.0)
- `Jetson Linux Multimedia API` (already installed with JetPack, otherwise it should be installed with `sudo apt install nvidia-l4t-jetson-multimedia-api`)
- [OpenCV](https://opencv.org/) + [CUDA](https://opencv.org/platforms/cuda/) (optional)

> [!Warning]
> Sometimes, on Jetson platforms, an error similar to the one below can occur:
>
> ```sh
> (gst-plugin-scanner:21845): GStreamer-WARNING **: 04:34:02.887: Failed to load plugin '/usr/lib/aarch64-linux-gnu/gstreamer-1.0/libgstlibav.so': /usr/lib/aarch64-linux-gnu/libgomp.so.1: cannot allocate memory in static TLS block
> ```
>
> As stated [here](https://docs.nvidia.com/metropolis/deepstream/dev-guide/text/DS_FAQ.html?highlight=batch%20size#how-to-fix-cannot-allocate-memory-in-static-tls-block-error), this could be solved with:
> `export LD_PRELOAD=/usr/lib/aarch64-linux-gnu/libgomp.so.1`

## 2.2. Configure the hardware

After installing the necessary dependencies, please follow [this guide](https://github.com/robotology/yarp-device-argus/tree/main/doc/setup_orin-nx_framos-imx415.md) to configure and setup the `Jetson Orin NX` board to use the `FRAMOS-IMX678` cameras.

## 2.3. Build and install yarp-device-argus

```bash
mkdir build
cd build
cmake -DCMAKE_INSTALL_PREFIX=<installation_path> ..
make
make install
```

Optionally, GPU usage can be enabled by adding `-DTRY_ACTIVATE_CUDA=true` to the cmake options above.

In order to make the device detectable, add `<installation_path>/share/yarp` to the `YARP_DATA_DIRS` environment variable of the system.

Alternatively, if `YARP` has been installed using the [robotology-superbuild](https://github.com/robotology/robotology-superbuild), it is possible to use `<directory-where-you-downloaded-robotology-superbuild>/build/install` as the `<installation_path>`.

## 2.4. How to run argusCamera driver

From command line:

```bash
yarpdev --device argusCamera --d 0 --period 0.011 --rotation 90.0 --width 1920 --height 1080 --rotation_with_crop false
```

or from the root of the repository:

```bash
yarpdev --from conf/conf.ini
```

Where `conf.ini`:

```ini
device              argusCamera
d                   0
period              0.011
rotation            90.0
width               1920
height              1080
rotation_with_crop  false
```

Otherwise, the device can be launched via `yarprorbotinterface` as:

```bash
yarprobotinterface --config conf/camera.xml
```

# 3. Device documentation

This device driver exposes the `yarp::dev::IFrameGrabberImage` and
`yarp::dev::IFrameGrabberControls` interfaces to read the images and operate on the available settings.
See the documentation for more details about each interface.

| YARP device name | YARP default nws        |
|:----------------:|:-----------------------:|
| `argusCamera`    | `frameGrabber_nws_yarp` |

`yarp-device-argus` is based on [`yarpDeviceParamParserGenerator`](https://github.com/robotology/yarp/discussions/3081). The parameters accepted by this device are:

| Group name      | Parameter name     | Type            | Units   | Default Value  | Required | Description                                    | Notes                            |
|:---------------:|:------------------:|:---------------:|:-------:|:--------------:|:--------:|:----------------------------------------------:|:--------------------------------:|
|                 | d                  | int             | -       | 0              |  No      | Enumeration of the camera device               |                                  |
|                 | period             | double          | seconds | 0.033          |  No      | Refresh period of acquisition of the camera in s| Check the camera documentation for the fps cap |
|                 | rotation           | double          | degrees | 0.0            |  No      | Rotation applied from the center of the image  | Depending the size requested some rotations are not allowed. The rotation worsens the performance of the device. Allowed values: 0.0, 90.0, -90.0, 180.0. |
|                 | width              | int             | pixel   |   640          | No                          | Width of the images requested to the camera                       | The cameras has a value cap for the width of the image that can be provided, check the documentation. Zero or negative value not accepted |
|                 | height             | int             | pixel   |   480          | No                          | Height of the images requested to the camera                       | The cameras has a value cap for the width of the image that can be provided, check the documentation. Zero or negative value not accepted |
|                 | rotation_with_crop | bool            |     -   |   false        | No                          | The rotation, if the param is true, is obtained by swapping x with y                       | The image will have a resolution swapped with respect to what is requested |

**Suggested resolutions**

The performances reported in the table below are referred to the FRAMOS-IMX415 cameras.

|resolution|carrier|fps w/ CUDA|fps w/o CUDA|
|:-:|:-:|:-:|:-:|
|640x480|mjpeg|75|90|
|1920x1080|mjpeg|40|50|
|3840x2160|mjpeg|9|12|

# 4. Information for developers

Maintainers
--------------
This repository is maintained by:

| | | | |
|:---:|:---:|:---:|:---:|
| [<img src="https://github.com/Nicogene.png" width="40">](https://github.com/Nicogene) | [@Nicogene](https://github.com/Nicogene) | [<img src="https://github.com/martinaxgloria.png" width="40">](https://github.com/martinaxgloria) | [@martinaxgloria](https://github.com/martinaxgloria) |
