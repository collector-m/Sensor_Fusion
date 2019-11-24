# Sensor Fusion


<font size="3">
    
*  Process raw lidar data with filtering, segmentation, and clustering to detect other vehicles on the road. 

*  Fuse camera images together with lidar point cloud data. You'll extract object features, classify objects, and project the camera image into three dimensions to fuse with lidar data. 

* Analyze radar signatures to detect and track objects. Calculate velocity and orientation by correcting for radial velocity distortions, noise, and occlusions. 

* Fuse data from multiple sources using Kalman filters, and build extended and unscented Kalman filters for tracking nonlinear movement. 
</font>

---

### [Lidar Obstcle Detection](https://github.com/darrickz/Sensor_Fusion/tree/master/SFND_Lidar_Obstacle_Detection)
<table><tr>
<td>
<figure>
    <img  src="./images/Lidar_Obstacle0.gif" alt="Drawing" style="width: 350px;"/>
    <center>Lidar Obstcle Detection</center>
</figure></td>

<td><figure>
    <img  src="./images/Lidar_Obstacle2.gif" alt="Drawing" style="width: 340px;"/>
    <center>Tracking a bicyclist riding in front of the car</center>
</figure>
  </td>  
  </td>
</tr></table>
<figure>
    <img  src="./images/Lidar_Obstacle1.gif" alt="Drawing" style="width: 710px;"/>
    <center>Tracking a bicyclist riding in front of the car</center>
</figure>

---



### [2D Feature Matching](https://github.com/darrickz/Sensor_Fusion/tree/master/SFND_2D_Feature_Tracking)

<figure>
    <img  src="./images/2D_Features.gif" alt="Drawing" style="height: 500 width: 1000px;"/>
    <center>2D_Features</center>
</figure>

---

### [3D Feature Tracking and TTC calculation](https://github.com/darrickz/Sensor_Fusion/tree/master/SFND_3D_Object_Tracking)

<figure>
    <img  src="./images/TTC.gif" alt="Drawing" style="height: 500 width: 1000px;"/>
    <center>TTC Calculation</center>
</figure>    
