# GTSAM

This is copied from https://github.com/truher/robot-with-gtsam

It depends on the vendordep, gtsam-vendordep.json, which is a way to
describe a maven dependency.

Eventually, the maven dependency will be resolved online, but for now,
you can build it locally.

Check out truher/gtsam-vendordep and build it.

The 2027 branch is out of date, actually, so fix that.

Build thirdparty-gtsam locally, it ends up in $HOME/releases/maven.

Build gtsam-vendordep locally, it sends output to the same place.

Copy it to wpilib/2027_alpha5/maven. (TODO why?)