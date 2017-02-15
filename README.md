# Robot Localization using Particle Filter
##Overview:
Localization is done to identify robot’s position by using information about its environment. Instead of
checking all possible points in free space that could correspond to robot’s position, a number of random
points are taken to sample the free space for robot’s position. Using sensor information such as laser scan,
the samples are converged and thus the robot’s position is estimated.

##Pseudo-Code:
```
Generate random particles in free space
For a number of iterations:
  Compute probability for each particle using laser scan
  Re-sample using the probability
  Estimate the position of robot
  ```
  
##Result
An example of Localization. The red arrows indicate random particles. The yellow arrow indicates the true position and blue arrow indicates the estimated position.
![alt text](https://github.com/KajalGada/localization-using-particle-filter/blob/master/Images/Screenshot_Localization.png "Example")
