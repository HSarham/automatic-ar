# automatic-ar

**This code is the reference implementation of the following paper:**

[H. Sarmadi, R. Muñoz-Salinas, M. A. Berbís and R. Medina-Carnicer, "Simultaneous Multi-View Camera Pose Estimation and Object Tracking With Squared Planar Markers," in IEEE Access, vol. 7, pp. 22927-22940, 2019.](https://doi.org/10.1109/ACCESS.2019.2896648)

## Tested* Dependencies:

### Required:
* [CMake](https://cmake.org/) 3.1
* [OpenCV](https://opencv.org/) 3.2.0
* [Aruco](https://www.uco.es/investiga/grupos/ava/node/26) 3.0.0 (Included in the project)

### Optional:

* [PCL](http://pointclouds.org/) 1.7.2 (for extra visualization)

*The project might work with older dependencies however it has not been tested. The code has been tested on Ubuntu 18.04.

## Installing Dependencies on Ubuntu 18.04
Required:
```shell
sudo apt install build-essential cmake libopencv-dev
```
Optional:
```shell
sudo apt install libpcl-dev
```

## How to compile the code

Like a normal cmake project make a build library:

```shell
mkdir automatic-ar-build
```
change the current directory to the build directory:
```shell
cd automatic-ar-build
```
run cmake to configure the project and generate the makefiles:

```shell
cmake ../automatic-ar/trunk/
```
in case cmake does not find a library you have automatically, you can manually give cmake the path to where the library's cmake configuration file exists. For example:
```shell
cmake ../automatic-ar/trunk/ -DOpenCV_DIR=~/local/share/OpenCV
```
Finally run make to build the binaries
```shell
make
```
You will find the executables to work with in the apps directory.
## Sample data
You can download sample data sets from [here](http://hsarmadi.info/public_files/automatic-ar).

## Usage

### Processing a sequence

1. Unzip the desired data set.
```shell
unzip box.zip
```
2. Do the marker detection by giving the path of the data set to `detect_markers`:
```shell
detect_markers box
```
After unzipping you will find a file name `aruco.detections` in the data folder path.

3. Apply the algorithm by providing the data folder path and the physical size of the marker to `find_solution`:
```shell
find_solution box 0.05
```
Here `0.05` is the input size of each marker in meters.

#### Outputs

The output of `find_solutions` includes several files. Files with the name format 'initial*' store information of the solution after initialization and before optimization. The files with their names 'final*' store information resulted after doing the optimization.

There are also `*.yaml` files that have the relative transformation between cameras, relative transformation between markers, and the relative transformation from the reference marker to the reference camera for each frame stored in the [YAML](http://yaml.org/) format. You can use OpenCV's FileStorage class to read the data from the stored `*.yaml` files.

If you compile with PCL library you will also find `*.cameras.pcd` and `*.markers.pcd` which are point cloud based visualizations for cameras' configuration and markers' configuration respectively. These files could be viewed using `pcl_viewer` from the PCL library.

### Tracking a sequence

For tracking the marker detection is done live so you do not need to do the detection in separate step. However, you need a processed sequence with the same camera and object configuration as your tracking sequence. Let's assume that we want to do tracking in the `box_tracking` sequence using the already processed `box` sequence (you can find both of them in the sample data sets). You just run:
```shell
track box_tracking box/final.solution
```

## Visualization
If you compile with the PCL library you will have automatic 3D visualization when you run `find_solution`. However, if not, you can still visualize the solution using the overlay app:
```shell
overlay pentagonal
```
You can also save overlayed visualization in a video in the dataset folder by using an extra option:
```shell
overlay pentagonal -save-video
```
Also the `track` app has a live overlay visualization in the runtime that does not need PCL.

## Dataset format

Each dataset is defined within a folder. In that folder each camera has a directory with its index as its name. In the cameras folders there is a "calib.xml" file specifying the camera matrix, distortion coefficients and the image size in OpenCV calibration output style for that camera.
