### Getting started

* `sudo apt install libceres-dev`
* `sudo apt install libeigen3-dev`
* `sudo apt install nlohmann-json3-dev`  # if json parsing is required

In the folder, [simple_example](bundle_adjustment/simple_example/), a toy problem is solved using the instructions given in the [pdf](bundle_adjustment/simple_example/Bundle%20Adjustment.pdf)

A brief background about large bundle adjustment problems for 3D reconstruction is explained in my blog [Revisiting Bundle Adjustment with Ceres](https://ashbabu.github.io/blog/2026/bundle-adjustment-in-large/)

For [BAL](bundle_adjustment/ba_uni_washington/), large scale reconstruction is carried out. 

### References
* [Ceres](http://ceres-solver.org/nnls_tutorial.html#bundle-adjustment)
* [Slambook](https://github.com/gaoxiang12/slambook2/tree/master)  # specifically chapter 8
* [Grail Lab, University of Washington](https://grail.cs.washington.edu/projects/bal/)
