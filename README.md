# final-project-445
Robotic Artist - Final Project for Introduction to Robotics (CSCI 445) 

## Group Members
- Hightower, James, Richard
- Wang, Jonathan, Z [@JWangatang](https://github.com/JWangatang)
- Wenner, Matthew, Asao [@mattwenner](https://github.com/mattwenner)

## Project Description
> You will be working on a robotic artist. Your robot will be equipped with a pen holder allowing you to vertically move the pen. The color of the pen can be changed by a human operator by replacing the actual pen used. The goal is to let the robot re-create artistic pieces by drawing them using various colored pens.

## Challenges
There are several challenges that need to be solved.
1. Your robot’s onboard localization tends to drift over time, making it impossible to follow long paths (as we have seen in previous labs). Fortunately, there is an external camera which will track the robots’ position and orientation. However, this tracking will be infrequent, noisy, and might not always be available (e.g. if your robot is out-of-sight of the camera). Thus, you will need to be able to fuse the onboard and external localization.

2. The environment might contain obstacles. You will be given a map (similar to Lab 10), however some paths might only be drawable from a certain direction to avoid obstacles. Others might be impossible to draw without colliding into an obstacle and should be avoided.

3. You should minimize the manual effort for the human operator. In particular, the number of required pen swaps should be minimal.

4. Your robot should draw as well as possible in a reasonable amount of time. Thus, it needs to decide smartly in which order different parts should be drawn and the controller should be optimized.

5. You will have a limited amount of time to complete the drawing.
