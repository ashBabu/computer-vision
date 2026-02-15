### Getting started

* `sudo apt install libceres-dev`
* `sudo apt install libeigen3-dev`
* `sudo apt install nlohmann-json3-dev`  # if json parsing is required

In the folder, [simple_example](bundle_adjustment/simple_example/), a toy problem is solved using the instructions given in the [pdf](bundle_adjustment/simple_example/Bundle%20Adjustment.pdf)

A brief background about large bundle adjustment problems for 3D reconstruction is explained in my blog [Revisiting Bundle Adjustment with Ceres](https://ashbabu.github.io/blog/2026/bundle-adjustment-in-large/)

For [BAL](bundle_adjustment/ba_uni_washington/), large scale reconstruction is carried out. The data is taken from [here](https://grail.cs.washington.edu/projects/bal/), specifically the [Dubrovnik](https://grail.cs.washington.edu/projects/bal/dubrovnik.html)

**The Camera Block (<camera_1> to <camera_num_cameras>)**

Each camera in this file is represented by a sequence of 9 parameters, each on its own line. For the [file](problem-16-22106-pre.txt) with 16 cameras, there will be $16 \times 9 = 144$ lines in this section. The 9 parameters for each 
camera are typically ordered as follows:
* Parameters 1-3: Rotation ($R$) Represented as a Rodrigues axis-angle vector ($r_x, r_y, r_z$). The direction of the vector is the axis of rotation, and its magnitude is the angle in radians.
* Parameters 4-6: Translation ($t$) The 3D translation vector ($t_x, t_y, t_z$) that moves a point from world coordinates into the camera's coordinate system.
* Parameter 7: Focal Length ($f$)A single value representing the camera's focal length in pixels.
* Parameters 8-9: Radial Distortion ($k_1, k_2$) These are coefficients used to model the lens distortion (how straight lines might appear curved near the edges of the image).

**The Point Block (<point_1> to <point_num_points>)**

This section contains the 3D coordinates for every landmark in the scene. Each point is represented by 3 parameters on separate lines. For your file with 22,106 points, there will be $22106 \times 3 = 66,318$ 
lines in this section.
* Parameters 1-3: World Coordinates ($X, Y, Z$) These are the estimated 3D positions of the features in the "world" space that the cameras are looking at.

### Camera Projection model
```
P  =  R * X + t       (conversion from world to camera coordinates)
p  = -P / P.z         (perspective division, -ve becoz image plane 
                        behind camera)
p' =  f * r(p) * p    (conversion to pixel coordinates)
                       where P.z is the third (z) coordinate of P. 

r(p) = 1.0 + k1 * ||p||^2 + k2 * ||p||^4.

||p||^2  = p.x * p.x + p.y * p.y
```
In the last equation, $r(p)$ is a function that computes a scaling factor to undo the radial distortion

### References
* [Ceres](http://ceres-solver.org/nnls_tutorial.html#bundle-adjustment)
* [Slambook](https://github.com/gaoxiang12/slambook2/tree/master)  # specifically chapter 8
* [Grail Lab, University of Washington](https://grail.cs.washington.edu/projects/bal/)
