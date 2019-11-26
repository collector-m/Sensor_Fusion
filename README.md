# Sensor Fusion


<font size="3">
    
*  Process raw lidar data with filtering, segmentation, and clustering to detect other vehicles on the road. 

*  Fuse camera images together with lidar point cloud data. You'll extract object features, classify objects, and project the camera image into three dimensions to fuse with lidar data. 

* Analyze radar signatures to detect and track objects. Calculate velocity and orientation by correcting for radial velocity distortions, noise, and occlusions. 

* Fuse data from multiple sources using Kalman filters, and build extended and unscented Kalman filters for tracking nonlinear movement. 
</font>

---

## Projects

### [1. Lidar Obstcle Detection](https://github.com/darrickz/Sensor_Fusion/tree/master/SFND_Lidar_Obstacle_Detection)
* Implemented RANSAC algorithm to separated ground plane from obstacles
* Euclidean clustering algorithm is used to identify obstacles. KD tree implemented from scratch is used to speed up searching point cloud
<table><tr>
<td>
<figure>
    <img  src="./images/Lidar_Obstacle0.gif" alt="Drawing" width="450" height="350"/>
    <p align="center">Lidar Obstcle Detection</p>
</figure></td>

<td><figure>
    <img  src="./images/Lidar_Obstacle2.gif" alt="Drawing" width="450" height="350"/>
    <p align="center">Tracking a bicyclist riding in front of the car</p>
</figure>
  </td>  
  </td>
</tr></table>
<figure>
    <kbd>
    <img  src="./images/Lidar_Obstacle1.gif" alt="Drawing" style="width: 710px;"/>
   </kbd>        
    <p align="center">Tracking a bicyclist riding in front of the car</p>
</figure>

---



### [2. 2D Feature Tracking](https://github.com/darrickz/Sensor_Fusion/tree/master/SFND_2D_Feature_Tracking)
Various combination of keypoint detectors, descriptors and matching schemes are explored. 
<figure>
    <kbd>
    <img  src="./images/2D_Features.gif" alt="Drawing" style="height: 500 width: 1000px;"/>
    </kbd>    
    <p align="center">2D_Features</p>
</figure>

---

### [3. 3D Object Tracking and TTC calculation](https://github.com/darrickz/Sensor_Fusion/tree/master/SFND_3D_Object_Tracking)
* Various combination of keypoint detectors, descriptors and matching schemes are explored. 
* Object detection using the pre-trained YOLO deep-learning framework
* Methods to track objects by matching keypoints and bounding boxes across successive images
* Associating regions in a camera image with lidar points in 3D space
<figure>
    <kbd>
    <img  src="./images/TTC.gif" alt="Drawing" style="height: 500 width: 1000px;"/>
    </kbd>    
    <p align="center">TTC Calculation</p>
</figure>    
