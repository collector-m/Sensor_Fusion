# Sensor Fusion

---
<font size="3">
    
*  Process raw lidar data with filtering, segmentation, and clustering to detect other vehicles on the road.

*  Fuse camera images together with lidar point cloud data. Extract object features, classify objects, and project the camera image into three dimensions to fuse with lidar data. 

*  Analyze radar signatures to detect and track objects. Calculate velocity and orientation by correcting for radial velocity distortions, noise, and occlusions.

*  Fuse data from multiple sources using Unscented Kalman filters, and build extended and unscented Kalman filters for tracking nonlinear movement. 
</font>
---

### Lidar Obstacle Detection
<table><tr>
<td>
<figure>
    <img  src="./images/Lidar_Obstacle.gif" alt="Drawing" style="width: 350px;"/>
    <center>Lidar Obstcle Detection</center>
</figure></td>

<td><figure>
    <img  src="./images/Lidar_Obstacle1.gif" alt="Drawing" style="width: 370px;"/>
    <center>Lidar Obstcle Detection</center>
</figure>
  </td>  </tr></table>

---
---


### 2D Feature Matching

<figure>
    <img  src="./images/2D_Features.gif" alt="Drawing" style="height: 500 width: 1000px;"/>
    <center>2D_Features</center>

