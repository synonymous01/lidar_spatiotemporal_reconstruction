# 4 Dimensional Spatio-temporal reconstruction of LiDAR pointclouds

We have datasets collected of a wheat field throughout the season. There are a total of 9 separate pointclouds describing the field each week. All of these pointclouds have different alignment due to various pointcloud registrations and our goal is to align them all for further analysis.

_currently a work in progress_

## Current progress
\
I first started by trying to align using basic ICP algorithms and appling them on all the pointclouds. The issue with ICP is that it heavily depends on the initial pose of a moving pointcloud. That works well in terms of real-time scan matching but for fully fledged pointclouds with dense information, a minimizing algorithm like ICP can get stuck in a local minima. \ \
_These two pointclouds align perfectly_ \
\
![image](https://github.com/synonymous01/lidar_spatiotemporal_reconstruction/assets/40025239/6e736863-eb69-471c-b3e8-fab06e75b485)

\
_Whereas here they fail to align_ \
![image](https://github.com/synonymous01/lidar_spatiotemporal_reconstruction/assets/40025239/2d3164f5-3570-47d6-95e3-b126bcdf524f)
\
In this scenario, it makes sense having to do some type of data association across the pointclouds and using those to align them together. All the pointclouds have one universal landmark: the solar plates in the north-western region of the pointclouds. By isolating those plates and running ICP across them, we can get improved results since we get rid points that may interfere with minimizing. 
If ICP is able to align these panels perfectly, applying the same transformations to the whole pointcloud will align them perfectly too.\
After manually isolating the panels and running ICP on them, applying their transformations on the pointcloud we get the following result:
![image](https://github.com/synonymous01/lidar_spatiotemporal_reconstruction/assets/40025239/5e3c7008-da53-4b7c-8bcb-e6a8a34992c4) \

The results look perfect! but if you notice closely, the pointclouds are skewed: the magenta points are lower on one side and higher on the other. A possible reason for this is some pointclouds do not cover the solar panels properly, like below: /
![image](https://github.com/synonymous01/lidar_spatiotemporal_reconstruction/assets/40025239/118de27f-2294-44d4-b28b-138e27322e1f)

I will continue updating this as progress continues.
