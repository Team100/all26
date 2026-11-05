# Raspberry Pi Docs

## Overview

We use Raspberry Pi 5's as "smart cameras", looking at each
frame captured and extracting things the robot cares about:

* Poses of nearby AprilTags
* Bearings to nearby game pieces

The main libraries we use are

* [AprilTag](https://github.com/AprilRobotics/apriltag) -- a umich project from 2011, revised a few times since then.
* [OpenCV](https://en.wikipedia.org/wiki/OpenCV) -- an Intel project from 1999, expanded many times since then.
* [robotpy](https://github.com/robotpy) a port of WPILib to python.

The architecture is very simple:

* A main loop
* A camera captures a frame
* One or two analyzers look at it
* Results are sent over the network

There are a few interesting details:

* To keep the Pi and RoboRIO clocks in sync, we use the 
  [Team 100 "sync" protocol](https://github.com/team100/all26/blob/main/lib/src/main/java/org/team100/lib/network/SYNC.md)
  * The reason we care about the clocks is because we care about exact
    timing of each captured frame: the camera system latency is
    much more than the robot update periodicity, so the robot needs to
    be careful about handling the "delayed" input.
* For many types, we use python `Protocol` with a few
  implementations, e.g. a "fake" camera for testing.
* We run the same code everywhere, so we use the Raspberry Pi
  serial number to select implemenations and capabilities.
* We have our own deployment and management method.

We use simple bespoke code, rather than some FRC vision
thing like Limelight or PhotonVision, to encourage the
students to develop
[agency](https://en.wikipedia.org/wiki/Agency_(philosophy))
to understand and modify their own tools.
These are not complex systems, they should not be
magic black boxes.

The code is organized into many small files, which may be
confusing on first read.  Read the
[code walkthrough](walkthrough.md) to learn what's where.

## "How to" content

* [Code walkthrough](walkthrough.md)
* [Setup your dev environment](setup.md)
* [Run the code and the tests on your laptop](run_locally.md)
* [Setup a Raspberry Pi](pi_setup.md)
* [Deploy to the Raspberry Pi](deploy.md)
* [Use the Raspberry Pi](use.md)

## Background

* [How the AprilTag detector works](detection.md)
* [How we handle lens distortion](distortion.md)
* [A bit on our Python style](python_style.md)
* [Modes of some common sensors](sensor_modes.md)
