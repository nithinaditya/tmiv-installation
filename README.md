Test Model for MPEG Immersive Video (TMIV) Build and Installation Instructions
==========================================

## Prerequisites to follow this instruction

The software is ISO C++17 conformant and requires the following external libraries:

* The [{fmt}](https://github.com/fmtlib/fmt) library for string formatting
* The [Catch2](https://github.com/catchorg/Catch2.git) test framework
* The [HEVC test model](https://vcgit.hhi.fraunhofer.de/jct-vc/HM.git) (HM)

Minimal prerequisites are:

* C++17 compiler and build tools
* CMake 3.14 or newer


## Obtain all external libraries

### Catch2 testing framework

1. Visit this URL: https://github.com/catchorg/Catch2/tree/v2.11.1
1. Download ZIP or clone the repo resulting in a directory `/Workspace/Catch2-2.11.1` such that the file `/Workspace/Catch2-2.11.1/README.md` exists.

### {fmt} string formatting library

1. Visit this URL: https://github.com/fmtlib/fmt/tree/7.0.3
1. Download ZIP or clone the repo resulting in a directory `/Workspace/fmt-7.0.3` such that the file `/Workspace/fmt-7.0.3/README.md` exists.

### HEVC test model (HM)

1. Visit this URL: https://vcgit.hhi.fraunhofer.de/jct-vc/HM/-/tree/HM-16.16
1. Download ZIP or clone the repo resulting in a directory `/Workspace/HM-HM-16.16` such that the file `/Workspace/HM-HM-16.16/README` exists.
1. Rename that directory to `/Workspace/HM-16.16`.

### Fraunhofer Versatile Video Encoder (VVenC)

1. Visit this URL: https://github.com/fraunhoferhhi/vvenc/tree/v0.2.0.0
1. Download ZIP or clone the repo resulting in a directory `/Workspace/vvenc-0.2.0.0` such that the file `/Workspace/vvenc-0.2.0.0/README.md` exists.

### Fraunhofer Versatile Video Decoder (VVdeC)

1. Visit this URL: https://github.com/fraunhoferhhi/vvdec/tree/v0.2.0.0
1. Download ZIP or clone the repo resulting in a directory `/Workspace/vvdec-0.2.0.0` such that the file `/Workspace/vvdec-0.2.0.0/README.md` exists.

### The TMIV project

1. Visit this URL: https://gitlab.com/mpeg-i-visual/tmiv/-/tree/v8.0.1
1. Click on the download button (next to the Clone button) and select "zip"
1. Unzip, resulting in a directory `/Workspace/tmiv-v6.1` such that the file `/Workspace/tmiv-v6.1/README.md` exists.
1. Rename to `/Workspace/tmiv` to match with the following instructions.

2. Alternatively you can just clone this repo and continue.


## Building and installing TMIV

Below are two alternative instructions for building: using only command-line tools or using a GUI. Both instructions are compatible with Linux and Windows. Make sure to use forward slashes with CMake, also on Windows.

### Using the command line

To build and install TMIV into the directory `/Workspace/tmiv_install` with sources in the directory `/Workspace/tmiv`, execute:

```shell
mkdir /Workspace/tmiv_build
cd /Workspace/tmiv_build
cmake -DCMAKE_INSTALL_PREFIX=/Workspace/tmiv_install -DCMAKE_BUILD_TYPE=Release /Workspace/tmiv
cmake --build . --parallel --config Release
cmake --build . --parallel --config Release --target install
```

The intermediate directory `/Workspace/tmiv_build` may be deleted when the installation has completed.

### Using a GUI

Open the CMake GUI and specify:

* Where the source directory is: `/Workspace/tmiv`
* Where to build the binaries: `/Workspace/tmiv_build`
* Click Configure, Yes, Finish
* Do not change any flags starting with `CATCH_`
* Set `CMAKE_INSTALL_PREFIX` to `/Workspace/tmiv_install`
* Set `CMAKE_BUILD_TYPE` to `Release` (no need for Visual Studio)
* Click Generate

Build and install the generated project.

For Visual Studio please:
* Manually select `Release` from the drop-down box.
* Perform build by building the `ALL_BUILD` target.
* Perform installation by building the `INSTALL` target.

## Installation result

After installation, the TMIV executables `Encoder`, `Decoder` and `Renderer` will be available under the directory `/Workspace/tmiv_install/bin`.
By default TMIV only builds the HM modules that are required for TMIV (`TLibCommon` and `TLibDecoder`).
When `HM_BUILD_TAPPDECODER` and `HM_BUILD_TAPPENCODER` are selected, then the `TAppDecoder` and `TAppEncoder` tools respectively will also be installed to this directory.
TMIV per default builds `vvencFFapp` and `vvdecapp` from the Fraunhofer VVC implementations.

# Running the software tests

In general and especially when [contributing to TMIV](doc/CONTRIBUTING.md) it is a good idea to run the software tests before starting any (large) experiments. TMIV has two types of software tests that are integrated into the project.

## Unit tests

The unit tests check aspects of the software in isolation and without I/O. Running the 100+ unit tests only takes a couple of seconds in total.

To run all unit tests, execute the following command inside your build directory:

```shell
cmake --build . --parallel --config Release --target test
```
For Visual Studio please substitute test for RUN_TESTS.


