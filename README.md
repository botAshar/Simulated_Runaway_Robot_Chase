# Runaway Robot

This is my solution for the final project Runaway Robot of
[Artificial Intelligence for Robotics](https://www.udacity.com/course/artificial-intelligence-for-robotics--cs373)
in Udacity.

##LOCALIZATION:
    -I have used several different types of partictle filters to try to localize the robot in **Part 2** with noise and the localization of the robot has worked localizing the robot successfully several times even when the noise is equal to the speed/distance traveled by the robot.
    -The **Circle measurement on P_filter.py** code uses the typical particle but the measurements recieved from the robot are transformed on estimated circumferance of the circle that the robot is moving on.
    -The **Particle filter change.py** code uses a amended form of particle filter in which the measurement probability for resampling the particles now includes the factor of how close the robot is on the circle, this results in major error reduction in the estimation of robot position.
    -Finally **Non_random_particle_filer apply.py** code uses another amended form of particle filter which basically uses the noisy circularly transformed measurements to create the all the particles instead of using a random set of particles and it recreates n particles using n noisy measurement after n/4 interval. The rest of particle filter code is typical. This results in much less mean error than the previous techniques.

##HUNTING OF ROBOT:
    -Finally, the **Part 4** and **part 5** are the similar, the only difference being the large noise of 200%. So I have used the  **Non_random_particle_filer** and the hunting of the robot is successful for the noise levels up to 10% to 50% of the speed/distance traveled by the robot, beyond that the hunter robot fails to consistently catch the target.
    -The algorithm used for hunting the robot basically follows the predicted robot positions untils it's too far away from the next prediction, it then predicts the position of the robot in advance using the circular path and the edge length the of the n_gon(33 to 34 sides in this case) and moves straight towards it at the maximum allowable speed. Just two steps before reaching cirular predicted measurements, the hunter robot now starts follow the particle filter predicted positions to catch the robot.

**Also About the Non_random_particle filter**, the particles created by it are still pseudorandom due to the noise in the measurements

