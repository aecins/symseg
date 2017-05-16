# SymSeg: Model-Free Object Segmentation in Point Clouds Using the Symmetry Constraint.

## Dependencies ##
- [PCL](https://github.com/PointCloudLibrary/pcl)
- [OctoMap](https://github.com/OctoMap/octomap) (with dynamicEDT3D)

Tested in Ubuntu 14.04 and 16.04 with PCL 1.8.0 and Octomap 1.8.1


## Building ##

To checkout and build *symseg* in the `build` directory, execute the following in a terminal:

```
git clone https://github.com/aecins/symseg.git
cd symseg
mkdir build
cd build
cmake ..
make -j
```

## Running example ##
An example code is provided that segments rotational objects in a scene. To execute it run the following from the `bin` directory:
```
./rotational_segmentation ../sample_scene
```
