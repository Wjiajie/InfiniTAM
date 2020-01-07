# InfiniTAM-Based Experiment With Dynamic-scene Reconstruction

This project started out about two years ago with a graduate student (me) trying to replicate the results of the KillingFusion (Slavcheva et al., CVPR2017) and, later, the SobolevFusion (Slavcheva et al., CVPR2018) algorithms, without any access to the source code and little guidance from the original authors. It does not currently fully replicate the results, i.e. there is significant drift on the "Snoopy" depth video sequence originally used by the Killing/SobolevFusion authors, as the below figure demonstrates.

The image on the left shows the reconstruction result in the canonical pose for frames 15-65 of the Snoopy sequence. The image on the right shows the continued reconstruction after frame 135, with the noise resulting from drift highlighted.
![Figure showing the result before drift on the left and with significant drift on the right](https://i.ibb.co/x2gj8Q0/snoopy.png)

## What may be going wrong?

I don't know exactly yet, but one thing we do know is that the rigid alignment doesn't always give us the best result. I am using one of InfiniTAM's original trackers, I haven't tried to play with it's settings yet. Mira Slavcheva's algorithms both use SDF-2-SDF Fusion (also published by Slavcheva et al.) for rigid tracking, which she has shown to work better than the existing alternative glorified ICP versions. I will continue to experiment to see if rigid tracking is the issue and potentially implement SDF-2-SDF fusion as a separate Tracker within InfiniTAM.

It could be that some detail is missing from the implementation, and could be that it is near-impossible to tune. A lot of details about the algorithm are missing from the official publications, there are a lot of things that can be interpreted ambiguiously (for instance, when exactly is the data materialized for every stage of the algorithm?) and correspondence I've had with the authors so far appears to be self-contradicting about subtle details, such as ignoring / not ignoring values from certain voxels for part / all of the gradient computation and similar things.

## What details are currently missing that we know of, and what are some known issues?

1. SDF-2-SDF rigid alignment (InfiniTAM's trackers are used instead)
2. Capability to run the optimization in-reverse, in order to forward-animate the more-complete canonical mesh.
3. OpenMP-parallelized CPU implementation currently has a bug
4. ~CUDA implementation is incomplete and has all sorts of bugs~ *[Update]* CUDA implementation is now complete for both the plain voxel array and the voxel block hash indexing and has no difference from the signle-threaded CPU version if you believe the unit tests. However, expanded allocation should be turned on to get 100% match from CPU to GPU during runtime, which slows down the CUDA implementation significantly.
5. OpenCV dependency still needs to be made optional, like the VTK dependency.

## How do I try this code out?

1. You need to be somewhat well-versed in using CMake. 3rd-party requirements are all open-source, and you can glean what you're missing by running the CMake generator.
2. Linux currently is the only officially supported OS. All of the required CMake packages and this code in theory should work on any major platform, so you can try on MacOS or Windows at your own risk, and let me know if you'd like to fix things that are not working on your platform. If this project achieves greater success, then I'll probably start officially supporting the other systems.
3. Build without OpenMP (use CMake to disable it) since it's buggy at the time of writing! I also recommend building with FFMPEG, since that will enable visual debugging/video recording.
4. To obtain the result shown above, download the [original Snoopy sequence](http://campar.in.tum.de/personal/slavcheva/deformable-dataset/index.html), modify Files/infinitam_snoopy_config.json with proper paths for input_and_output_settings_paths, and run like this (modify the path to point to infinitam_snoopy_config.json):

<build_folder>/Apps/InfiniTAM_bpo/InfiniTAM_bpo --config=Files/infinitam_snoopy_config.json --record_reconstruction_video

**Note**: If you build with FFMPEG, this will also record a video in the output folder specified in the config file.

## Is this code being worked on / maintained?

Yes, after a looong break, I'm officially switching to try to do something with it again, at least for awhile. Even if I'm not actively working on it, I do my best to respond to new issues or collaboration requests.

## Will I merge this back into InfiniTAM?

TLDR: Maybe. 

Originally, that was the plan. However, at the time of writing I'm exhausted from trying to stick to the open(for extension)-closed(for modification) principle. The code wasn't originally designed for dynamic-scene fusion, so some things really do need to change to make the new things more maintainable. I've already changed a lot of code from the original InfiniTAM codebase, IMHO, for the better.

If this fork achieves reasonable success, I'll reach out to the InfiniTAM authors and ask whether they'd like to work on integrating my changes into their codebase. I (hope that I) didn't break anything so far, but the original code lacks any hint of continuous integration, which I plan to add to it, at least for my code, shortly.

This code is still privy to the original LICENSE included within.

The Following is the original InfiniTAM README:

# InfiniTAM v3

This is the main branch of the software bundle "InfiniTAM", the current version is actively maintained by:

  Victor Adrian Prisacariu <victor@robots.ox.ac.uk>  
  Olaf Kaehler <olaf@robots.ox.ac.uk>  
  Stuart Golodetz <smg@robots.ox.ac.uk>  
  Michael Sapienza <michael.sapienza@eng.ox.ac.uk>  
  Tommaso Cavallari <tommaso.cavallari@unibo.it>
  
Previous maintainers and contributors are:

  Carl Yuheng Ren <carl@robots.ox.ac.uk>  
  Ming Ming Cheng <cmm.thu@gmail.com>  
  Xin Sun <xin.sun@st-hughs.ox.ac.uk>  
  Philip H.S. Torr <philip.torr@eng.ox.ac.uk>  
  Ian D Reid <ian.reid@adelaide.edu.au>  
  David W Murray <dwm@robots.ox.ac.uk>

For more information about InfiniTAM please visit the project website <http://www.infinitam.org>. 

Other related projects can be found in the Oxford Active Vision Library <http://www.oxvisionlib.org>.

# 1. Building the System

### 1.1 Requirements

Several 3rd party libraries are needed for compiling InfiniTAM. The given version numbers are checked and working, but different versions might be fine as well. Some of the libraries are optional, and skipping them will reduce functionality.

  - cmake (e.g. version 2.8.10.2 or 3.2.3)
    REQUIRED for Linux, unless you write your own build system
    OPTIONAL for MS Windows, if you use MSVC instead
    available at http://www.cmake.org/

  - OpenGL / GLUT (e.g. freeglut 2.8.0 or 3.0.0)
    REQUIRED for the visualisation
    the library should run without
    available at http://freeglut.sourceforge.net/

  - CUDA (e.g. version 6.0 or 7.0)
    OPTIONAL but REQUIRED for all GPU accelerated code
    at least with cmake it is still possible to compile the CPU part without
    available at https://developer.nvidia.com/cuda-downloads

  - OpenNI (e.g. version 2.2.0.33)
    OPTIONAL but REQUIRED to get live images from suitable hardware
    also make sure you have freenect/OpenNI2-FreenectDriver if you need it
    available at http://structure.io/openni

  - libpng (e.g. version 1.6)
    OPTIONAL, allows to read PNG input files
    available at http://libpng.org

  - FFMPEG (e.g. version 2.8.6)
    OPTIONAL, allows writing and playback of lossless FFV1 encoded videos
    available at https://www.ffmpeg.org/

  - librealsense (e.g. github version from 2016-MAR-22)
    OPTIONAL, allows to get live images from Intel Realsense cameras
    available at https://github.com/IntelRealSense/librealsense

  - libuvc (e.g. github version from 2015-OCT-27)
    OPTIONAL, deprecated alternative to librealsense
    currently this works only with branch olafkaehler/master
    available at https://github.com/olafkaehler/libuvc

  - doxygen (e.g. version 1.8.2)
    OPTIONAL, builds a nice reference manual
    available at http://www.doxygen.org/

###1.2 Build Process

  To compile the system, use the standard cmake approach:
```
  $ mkdir build
  $ cd build
  $ cmake /path/to/InfiniTAM -DOPEN_NI_ROOT=/path/to/OpenNI2/
  $ make
```
  To create a doxygen documentation, just run doxygen:
```
  $ doxygen Doxyfile
```
  This will create a new directory doxygen-html/ containing all the
documentation.

### 1.3 Odds and Ends

Padding the data structure ITMVoxel in ITMLibDefines.h with one extra byte may or may not improve the overall performance on certain GPUs. On a NVidia GTX 680 it appears to do, on a GTX 780 it does not. Have a try yourself if you need the speed.

On Mac OS X 10.9 there are currently some issues with libc++ vs. libstdc++ in conjunction with CUDA. They eventually manifest in error messages like:
```
Undefined symbols for architecture x86_64: 
"std::ios_base::Init::Init()", referenced from:
      __GLOBAL__I_a in libITMLib.a(ITMLib_generated_ITMColorTracker_CUDA.cu.o)
      __GLOBAL__I_a in libITMLib.a(ITMLib_generated_ITMDepthTracker_CUDA.cu.o)
     [...]
```
In the current version of InfiniTAM these errors are avoided by specifying ```CMAKE_CXX_FLAGS=-stdlib=libstdc++``` whenever clang is detected as complier. However, future versions of CUDA might not require this anymore or even get confused and/or require ```CUDA_HOST_COMPILER=/usr/bin/clang``` instead.

If a version of GLUT other than freeglut is used, the InfiniTAM sample application has problems on exit, as it is currently not explicitly cleaning up CUDA memory or closing the OpenNI device. Use freeglut to avoid this if you experience any problems.

Some sensors may need a small change to work correctly with OpenNI, the changes are described [here](http://com.occipital.openni.s3.amazonaws.com/Structure%20Sensor%20OpenNI2%20Quick%20Start%20Guide.pdf).


# 2. Sample Programs

The build process should result in an executable InfiniTAM, which is the main sample program. For a version without visualisation, try InfiniTAM_cli. If compiled with OpenNI support, both should run out-of-the-box without problems for live reconstruction. If you have calibration information for your specific device, you can pass it as the first argument to the program, e.g.:
```
  $ ./InfiniTAM Teddy/calib.txt
```
If no OpenNI support has been compiled in, the program can be used for offline processing:
```
  $ ./InfiniTAM Teddy/calib.txt Teddy/Frames/%04i.ppm Teddy/Frames/%04i.pgm
```
The arguments are essentially masks for sprintf and the %04i will be replaced by a running number, accordingly.


# 3. Additional Documentation

Apart from the doxygen documentation there should also be a technical report
shipped along with this package. It is also available from the official project
website. Further technical information is to be found in:

```
@inproceedings{InfiniTAM_ECCV_2016,
  author    = {Olaf K{\"{a}}hler and
               Victor Adrian Prisacariu and
               David W. Murray},
  title     = {Real-Time Large-Scale Dense 3D Reconstruction with Loop Closure},
  booktitle = {Computer Vision - {ECCV} 2016 - 14th European Conference, Amsterdam,
               The Netherlands, October 11-14, 2016, Proceedings, Part {VIII}},
  pages     = {500--516},
  year      = {2016}
}
```

and

```
@article{InfiniTAM_ISMAR_2015,
author = {{K{\"a}hler}, O. and
		  {Prisacariu}, V.~A. and
		  {Ren}, C.~Y. and
		  {Sun}, X. and
		  {Torr}, P.~H.~S and
		  {Murray}, D.~W.},
title = "{Very High Frame Rate Volumetric Integration of Depth Images on Mobile Device}",
journal = "{IEEE Transactions on Visualization and Computer Graphics 
	   (Proceedings International Symposium on Mixed and Augmented Reality 2015}",
volume = {22},
number = {11},
year = 2015
```

and

```
@article{InfiniTAM_arXiv_2017,
author = {V A Prisacariu and O K{\"a}hler and S Golodetz and M Sapienza and T Cavallari and P H S Torr and D W Murray},
title = {{InfiniTAM v3: A Framework for Large-Scale 3D Reconstruction with Loop Closure}},
journal = {arXiv pre-print arXiv:1708.00783v1},
year = {2017}
}
```

------

### History:
- 2017-JUL-23: version 3 release
- 2016-NOV-18: updated to reflect changes to team and project structure
- 2015-JUL-10: updated dependencies, added reference to ISMAR paper
- 2014-OCT-06: initial public release
