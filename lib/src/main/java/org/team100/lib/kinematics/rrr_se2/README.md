# RRR Linkage in SE(2)

A serial planar linkage of three revolute joints.

<img src="image.png" width=400 />

There are two implementations here

* one using the URDF implementation from last year, with iterative inverse.
* one using the new product-of-exponentials method from Modern Robotics, which uses an analytic inverse.

The planar RRR linkage has up to __two possible "postures"__ for a given pose, generally called
"elbow up" and "elbow down" (see diagram).