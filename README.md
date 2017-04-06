This repository is a C++/OpenCV implementation of the [Stereo Odometry based on careful Feature selection and Tracking](ieeexplore.ieee.org/iel7/7320493/7324045/07324219.pdf), as a part of the course project for [Probabilistic Mobile Robotics](http://home.iitk.ac.in/~gpandey/ee_698g.html).

The MATLAB implementation of this can be found [here](https://github.com/Mayankm96/Stereo-Odometry-SOFT).

## Package `imgpub`

* __img_publisher__: Loads an image from disk and advertises it
under the topic `img_pub`. It has the following parameters:
	* file: the path to the image to load
	* frequency: frequency used for publishing the image 

* __img_filter__: Subscribes to an image topic `img_pub`, performs simple image manipulation, then publishes the resulting image on a separate image topic `img_filt`. It has the following dynamic reconfigure paramters:
	* brightness: an integer value greater than or equal to zero
	* contrast: a floating point value greater than zero

## Package `harris`

* __harris_corners__: Class implementation of node that subscribes to an image topic `img_pub` and publishes the resulting image on the topic `harris_features` showing the Harris corner points detected. It has the following dynamic reconfigure parameters:
	* template size: an odd integer value greater than or equal to three that contains the size of the template for calculating the harris response at a given point
	* threshold: a floating point value greather than zero that sets the lower boundary for a harris response value to be considered a corner. This value is in percent and taken relative to the maximum harris response in the input image