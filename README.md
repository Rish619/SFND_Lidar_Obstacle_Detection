# Sensor Fusion Self-Driving Car Course

<img src="media/Result.gif" width="700" height="400" />

### Welcome to the Sensor Fusion course first Project Lidar Obstacle Detection.
* Ransac function build from scratch to perform plane identification : RansacPlane1
  ```c++
     template<typename PointT>
	 std::unordered_set<int> ProcessPointClouds<PointT>::RansacPlane1(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol) 
  ```
* Cloud Segmentation function responsible for segmenting both plane(inliers passed by RanasacPlane1 function) and obstruction cloud which the input cloud minus all the inliers passed by the RansacPlane1: SegmentPlane
  ```c++
     template<typename PointT>
	 std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
  ```

* Cloud Seperation being used by the Cloud Segmentation, this function subtracts inliers from the total input cloud and returns a pair of point clouds <obstruction cloud, plane cloud> : SeparateClouds
  ```c++
     template<typename PointT>
     std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
  ``` 

* Clustering the obstruction point cloud which returned by the SegmentPlane function, this function is : euclideanCluster
  ```c++
     template<typename PointT>
	 std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::euclideanCluster(typename pcl::PointCloud<PointT>::Ptr cloud, float distanceTol, int minsize, int maxsize)
  ```

* Kdtree implementation from scratch for inserting and searching points in 3d space 

## Evaluation/Observation Results
* Filtering the input point cloud on an average took 4 milliseconds

* Plane Segmentation on an average took 17 milliseconds

* Clustering on an average took 2 milliseconds

* FPS achieved on an average 190 

## Hardware/Testing system
* GPU: GeForce RTX 2060     Driver Version: 440.100      CUDA Version: 10.2  
* CPU: Intel(R) Core(TM) i7-8700 CPU @ 3.20GHz
* OS: Ubuntu 18.04.4 LTS

## Memory Usage
* GPU Core usage: 8MB, GPU MEM: 0%, CPU Core Usage: 100% , CPU Memory Usage: 148MB at a given instance by the environment process

## Installation

### Ubuntu 

```bash
$> sudo apt install libpcl-dev
$> cd ~
$> git clone https://github.com/Rish619/SFND_Lidar_Obstacle_Detection.git
$> cd SFND_Lidar_Obstacle_Detection
$> mkdir build && cd build
$> cmake ..
$> make
$> ./environment
```

### Windows 

http://www.pointclouds.org/downloads/windows.html

### MAC

#### Install via Homebrew
1. install [homebrew](https://brew.sh/)
2. update homebrew 
	```bash
	$> brew update
	```
3. add  homebrew science [tap](https://docs.brew.sh/Taps) 
	```bash
	$> brew tap brewsci/science
	```
4. view pcl install options
	```bash
	$> brew options pcl
	```
5. install PCL 
	```bash
	$> brew install pcl
	```

#### Prebuilt Binaries via Universal Installer
http://www.pointclouds.org/downloads/macosx.html  
NOTE: very old version 

#### Build from Source

[PCL Source Github](https://github.com/PointCloudLibrary/pcl)

[PCL Mac Compilation Docs](http://www.pointclouds.org/documentation/tutorials/compiling_pcl_macosx.php)
