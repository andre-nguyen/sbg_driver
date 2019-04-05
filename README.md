# SBG Driver
Rewritten ROS SBG Ellipse Micro2 driver.

## Changes
* Modified SBG Ecom library to allow low latency operation.
* New time stamp changing node.

## Difference with sbg_driver_ros
* The code is a lot less complicated and some parts are hard coded. But
the essentials are there and the whole callback sequence is much easier
to read.