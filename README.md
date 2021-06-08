# **Extended Kalman Filter** 

---

**Introduction to Extended Kalman Filter Project**

The goals / steps of this project are the following:
* Implement an extended Kalman filter in C++ to detect a bicycle that travels around your vehicle (stationary) using simulated lidar and radar measurements. 
* Use a Kalman filter, lidar measurements and radar measurements to track the bicycle's position and velocity.

**Note:** 

* Refer to sensor-fusion-ekf-reference.pdf for KF Equations
* Refer to [NPTEL Probabilistic Robotics](https://www.youtube.com/watch?v=j7BVHy231B0&list=PLyqSpQzTE6M_XM9cvjLLO_Azt1FkgPhpH&index=29) for a well-guided
explanation of Bayes Filter, KF and EKF.


[//]: # (Image References)

[image1]: /writeup_images/data.png 
[image2]: /writeup_images/9.png 
[image3]: /writeup_images/36.png 
[image4]: /writeup_images/499.png 


## Rubric Points
### The [rubric points](https://review.udacity.com/#!/rubrics/748/view) for this project are straightforward. They include:
* To ensure code compiles correctly
* Sensor Fusion algorithm follows the general processing flow
* Kalman Filter algorithm handles the first measurements appropriately
* Kalman Filter algorithm first predicts then updates
* Kalman Filter can handle radar and lidar measurements
* Accuracy: px, py, vx, vy output coordinates must have an RMSE <= [.11, .11, 0.52, 0.52] when using the file: "obj_pose-laser-radar-synthetic-input.txt" which is the same data file the simulator uses for Dataset 1. 

---

### Explanation of the Data File

The github repo contains one data file:

* obj_pose-laser-radar-synthetic-input.txt
Here is a screenshot of the first data file:

![Screenshot of Data File][image1]

Each row represents a sensor measurement where the first column tells you if the measurement comes from radar (R) or lidar (L).

For a row containing radar data, the columns are: sensor_type, rho_measured, phi_measured, rhodot_measured, timestamp, x_groundtruth, y_groundtruth, vx_groundtruth, vy_groundtruth, yaw_groundtruth, yawrate_groundtruth.

For a row containing lidar data, the columns are: sensor_type, x_measured, y_measured, timestamp, x_groundtruth, y_groundtruth, vx_groundtruth, vy_groundtruth, yaw_groundtruth, yawrate_groundtruth.

Whereas radar has three measurements (rho, phi, rhodot), lidar has two measurements (x, y).

The measurement values and timestamp are used in the Kalman filter algorithm. Groundtruth, which represents the actual path the bicycle took, is used for calculating root mean squared error.

The simulator will be using this data file, and feed main.cpp values from it one line at a time.

### Reading in the Data

The code that will read in and parse the data files is in the main.cpp file. The main.cpp file creates instances of a MeasurementPackage.

The code reads in the data file line by line. The measurement data for each line gets pushed onto a measurement_pack list. The ground truth (px, py, vx, vy) for each line in the data file gets pushed onto a ground_truth list so RMSE can be calculated later from tools.cpp.

### File Structure

#### Files in the Github src Folder
* main.cpp - communicates with the Term 2 Simulator receiving data measurements, calls a function to run the Kalman filter, calls a function to calculate RMSE
* FusionEKF.cpp - initializes the filter, calls the predict function, calls the update function
* kalman_filter.cpp- defines the predict function, the update function for lidar, and the update function for radar
* tools.cpp- function to calculate RMSE and the Jacobian matrix

#### How the Files Relate to Each Other

Here is a brief overview of what happens when you run the code files:

* Main.cpp reads in the data and sends a sensor measurement to FusionEKF.cpp
* FusionEKF.cpp takes the sensor data and initializes variables and updates variables. The Kalman filter equations are not in this file. FusionEKF.cpp has a   variable called ekf_, which is an instance of a KalmanFilter class. The ekf_ will hold the matrix and vector values. You will also use the ekf_ instance to call the predict and update equations.
* The KalmanFilter class is defined in kalman_filter.cpp and kalman_filter.h. You will only need to modify 'kalman_filter.cpp', which contains functions for the prediction and update steps.


### Code Explained

#### main.cpp

* creating an instance of the FusionEKF class
* Receiving the measurement data calling the ProcessMeasurement() function. ProcessMeasurement() is responsible for the initialization of the Kalman filter as well as calling the prediction and update steps of the Kalman filter. You will be implementing the ProcessMeasurement() function in FusionEKF.cpp:

Finally,

The rest of main.cpp will output the following results to the simulator:

* estimation position
* calculated RMSE

main.cpp will call a function to calculate root mean squared error

#### FusionEKF.cpp

Every time main.cpp calls fusionEKF.ProcessMeasurement(measurement_pack_list[k]), the code in FusionEKF.cpp will run. - If this is the first measurement, the Kalman filter will try to initialize the object's location with the sensor measurement.

**Predict and Update Steps:**

Once the Kalman filter gets initialized, the next iterations of the for loop will call the ProcessMeasurement() function to do the predict and update steps.

You will see references to a variable called ekf_. The ekf_ variable is an instance of the KalmanFilter class. ekf_ is used to store Kalman filter variables (x, P, F, H, R, Q) and call the predict and update functions.

#### kalman_filter.cpp

kalman_filter.h defines the KalmanFilter class containing the x vector as well as the P, F, Q, H and R matrices. The KalmanFilter class also contains functions for the prediction step as well as the Kalman filter update step (lidar) and extended Kalman filter update step (radar).

Because lidar uses linear equations, the update step will use the basic Kalman filter equations. On the other hand, radar uses non-linear equations, so the update step involves linearizing the equations with the Jacobian matrix. The Update function will use the standard Kalman filter equations. The UpdateEKF will use the extended Kalman filter equations.

#### Tools.cpp

This file is relatively straight forward. You will implement functions to calculate root mean squared error and the Jacobian matrix:

### Final Results

Lidar measurements are red circles, radar measurements are blue circles with an arrow pointing in the direction of the observed angle, and estimation markers are green triangles. The video below shows what the simulator looks like when a c++ script is using its Kalman filter to track the object. The simulator provides the script the measured data (either lidar or radar), and the script feeds back the measured estimation marker, and RMSE values from its Kalman filter.

Step: 9
![Start][image2]

Step: 36
![Mid][image3]

Step: 499
![End][image4]

The RMSE readings for (px, py, vx, vy) = (0.096, 0.085, 0.415, 0.432) which meets evaluation criteria.
