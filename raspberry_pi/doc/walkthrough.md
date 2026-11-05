# Team 100 Raspberry Pi Code Walkthrough

## Control flow

The app flow works generally as follows, for the most common setup:

* `runapp.py` loads the deployed zip file and runs main.
* `main.py` instantiates everything and runs a `Looper` in a separate thread.
* `CameraLoop` repeatedly grabs a frame from `Camera` and gives it to `Interpreter`.
  * `RealCamera` captures a frame and its timestamp
  * `DualInterpreter` decodes the frame, gives it to `MonoAnalysis` and `ColorAnalysis`, and shows the result on `Display`.
    * `Apriltags` finds tags, annotates the image, and sends coordinates with `Network`.
    * `Blobs` finds objects, annotates the image, and sends coordinates with `Network`.
    * `RealNetwork` sends messages using WPI's Network Tables.
    * `LinuxDisplay` publishes the annotated image to `localhost:1181`.


## New camera quick start

To set up a new camera:

* In `identity.py`, add an entry, using the Raspberry Pi serial
number, which you can find in `/proc/cpuinfo`,
or by running the code, it will print the serial number found.
Use a name that reflects what the camera is for, or where
it is.  Write this name on the camera itself.
* In `distortion.py`, add an entry using the new identity, with
the distortion parameters you measure using `mrcal`.
* In `intrinsic.py`, add an entry using the new identity, with
the intrinsic matrix you measure using `mrcal`.
* Choose or create a `Config` implementation appropriate for
the camera and what you're doing with it.  Add an entry to the
match statement in `config_factory.py`.
* In `interpreter_factory.py`, add an entry with the analysis
you want to do (e.g. `Apriltags`, `Blobs`).
* If the camera is mounted upside-down, and you want to correct
the orientation here, so that it appears right-side-up in the
viewfinder, you can do that in `config_protocol.py`,
see `transform()`.
* In `model.py`, add an entry using the camera model string,
which is printed when you run the code.
* In `size.py`, add an entry for the camera model, using the
size it reports.  For a high resolution camera, you may want
to downsample it, using a different encoding size than sensor
size.


## Packages

Brief descriptions of what's in each package:

* __analysis__
  * The `MonoAnalysis` and `ColorAnalysis` protocols
  consume images for analysis, along with a separate copy for
  display annotation, and a timestamp.
  * There's currently only one implementation of each,
  `AprilTags` for localization, and `Blobs` for
  (primitive) object detection. These implementations use
  `Network` to send data to the robot.
  * The implementation is chosen in `InterpreterFactory`,
  based on `Identity`.

* __camera__
  * The `Camera` protocol describes camera interactions,
  e.g. `capture_request()` returns a frame, `get_dist()` returns
  the distortion matrix.
  * There is a `RealCamera` implementation for
  real robots, and a `FakeCamera` for testing.
  * The implementation is chosen at runtime by `CameraFactory`,
  so that code running on a Raspberry Pi gets the real one, and
  test code running on your laptop gets the fake one.  If you forgot
  to connect a camera to the Pi, you get another implementation,
  `NoCamera`
  * The `RealCamera` is configured using `Model`, which
  uses the reported name, and affects immutable properties of
  the camera model, currently only `Size`, comprising
  the sensor size and encoded size.

* __camera/config__
  * The `Config` protocol describes camera
  configuration, which includes things like sensor size, exposure duration, etc.  The main purpose of a `Config` is to configure
  the hardware itself, in `RealCamera`.
  * Each real camera may have more than one config 
  implementation, e.g. the Raspberry Pi Global Shutter
  camera can be configured using `ConfigGsMono` or
  `ConfigGsColor`.
  * The implementation is chosen at runtime by the
  `ConfigFactory`, based on Raspberry Pi `Identity`

* __config__
  * The `Identity` enum reflects the Raspberry Pi serial number.
  We use it to keep track of which hardware is performing which task,
  e.g. a front-facing camera might do object detection using color
  frames, and a side-facing camera might do localization in
  monochrome.

* __dashboard__
  * The `Display` protocol describes a way to display images.
  * There is one implementation for linux, and a different one
  for windows, and a fake one for testing.  (TODO: mac?).
  Both the real implementations do the same thing, sending
  frames through the web interface (e.g. 10.1.0.35:1181),
  using the WPILib CameraServer.
  * The implementation is chosen and configured by
  `DisplayFactory`, to match the actual camera resolution,
  with a scale factor.  Don't use full-scale images during
  competition, it slows the frame rate and overloads the network.

* __decoder__
  * The `Decoder` protocol describes a frame decoder that takes
  raw input from the camera and produces something that our
  python code can operate on.
  * There is one implementation per camera encoding.
  * The implementation is specified in the camera `Config`.

* __framework__
  * `Looper` is an abstract base class that loops forever.
  * There are two implementations (elsewhere): `CameraLoop`
  captures one frame per loop, and `SyncLoop` implements the
  client half of the Team 100 clock sync method.

* __interpreter__
  * The `Interpreter` protocol consumes camera frames.
  * There are two implementations, `DualInterpreter`, which
  passes the same frame through 0, 1, or 2 analyzers and displays
  the results, and `Viewfinder`, which displays its input.
  * The implementation is selected in `InterpreterFactory`.

* __network__
  * The `Network` protocol desecribes how the app sends data
  to the robot.
  * There are two implementations, `RealNetwork`, which
  is used by `main.py`, and `FakeNetwork`, which is used
  in tests.
  * There are a few other network-related classes here, e.g.
  the `Calibrate` mode, and support for clock syncing.

* __util__
  * The `util` package contains stuff that doesn't fit anywhere
  else, mostly about time.


## Formality

Our code is written in a somewhat formal way, to make it easier to understand,
(assuming you understand the formalities).

For example, in many places, we use
[Protocol](https://typing.python.org/en/latest/spec/protocol.html),
which is analogous to a
Java [Interface](https://www.w3schools.com/java/java_interface.asp)
type, to make it easier to make small changes.

We use the __factory pattern__ to select protocol
implementations at runtime, usually based on
the serial number of the Raspberry Pi itself (which
reveals its role) or the model name of the camera
(which reveals the type of sensor inside).  Each module
contains one protcol and (usually) one factory.

We also use static type annotations absolutely everywhere, to make it easier
for humans to read (and machines to check) the code.

We use an object-oriented (Java-ish) style, with one 
(CamelCase) class per (snake_case) module.

We use unit tests to verify some aspects of the design, using
canned images (see `camera/images`).
