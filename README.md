# RUBRIC: 

Writeup (Due 11/11 at 7PM)Permalink

In your code package create a README.md file to hold your project writeup. Your writeup should touch on the following topics. We expect this writeup to be done in such a way that you are proud to include it as part of your professional portfolio. As such, please make sure to write the report so that it is understandable to an external audience. Make sure to add pictures to your report, links to Youtube videos, embedded animated Gifs (these can be recorded with the tool peek).

    What was the goal of your project? Since everyone is doing a different project, you will have to spend some time setting this context.
    How did you solve the problem (i.e., what methods / algorithms did you use and how do they work)? As above, since not everyone will be familiar with the algorithms you have chosen, you will need to spend some xtime explaining what you did and how everything works.
    Describe a design decision you had to make when working on your project and what you ultimately did (and why)? These design decisions could be particular choices for how you implemented some part of an algorithm or perhaps a decision regarding which of two external packages to use in your project.
    What if any challenges did you face along the way?
    What would you do to improve your project if you had more time?
    Did you learn any interesting lessons for future robotic programming projects? These could relate to working on robotics projects in teams, working on more open-ended (and longer term) problems, or any other relevant topic.

# DRAFT WRITEUP
# Neato Golf (ft. Donkey Kong)
### Bill Le, Oscar Bao, Sam Wisnoski

## Project Goals:      
### Introduction:  

<p align="center">
  <img src="media/donkey_kong_golf_1.jpg" alt="Donkey Kong Golfing" width="400">
</p>

Has this ever happened to you? You are playing golf against Donkey Kong, the greatest golf player to ever walk the earth. You've played a valiant 17 holes, and are up by one point going into the 18th. But oh no! Donkey Kong shoots first, and gets a hole in one! To claim his legendary hoard of golden bananas (the biggest hoard east of the Mississippi) you must match his shot. How could you possibly make that happen? If you're nodding your head along, also having experienced this common scenario, then we have the solution for you. 

**Let us introduce you to Neato Golf: our autonomous golf assistant that ensures you never miss the hole again.**

<p align="center">
  <img src="media/neato_golf_bot.png" alt="Neato Golf Bot and Target" width="400">
</p>

The goal of our project was to design and implement an autonomous robotic system capable of locating a golf ball within a predefined area and pushing it onto our target. We wanted to explore how computer vision, object tracking, and motion planning could be combined to complete a task. Additionally, we had a few extra constraints when defining our project goal:
* We wanted to use a camera that was not attached to the robot, but could still capture robotic movement.
* We wanted to use entirely CV to do our detection, without having to rely on odometry (or any other sensor) data.
* We wanted to practice working in a (relatively) controlled enviornment.
* We wanted to implement an algorithm to allow detection of multiple golf balls at once. 
  
With the above constraints in mind, our solution is as follows: using a bird's-eye-view camera, our CV algorithm identifies the ball, the hole, and the Neato robot’s position (x, y, and heading). Then, with a SORT tracking algorithm and a simple path planner, it directs the robot to nudge the ball into the target. In essence, our project takes the final shot of golf out of your hands, and puts it into the more than capable end-effector of our robot. As long as you can get your ball to that green, you're golden... as golden as a golden banana. 

Not convinced that Neato Golf will work for your needs? Videos of partial and full implementations can be found here:
* Using SORT to track multiple balls: https://drive.google.com/file/d/1GgMcts5O3wgErzWKcIJOUZaHiJTX-XAM/view?resourcekey 
* Video of Neato Golf successfully getting a "hole in one": https://drive.google.com/file/d/1KUzkdHAsaL157LKygHP7LB2IOXnDMdcI/view?resourcekey 


### Individual Learning Goals:   
#### Oscar's learning goals: 

#### Sam's learning goals: 
The most important aspect of this project for me was having direct control over a task, working with a tangible robot to get something done. Although that meant we were pretty much limited to Neatos, it was totally worth it for how rewarding it was to get the full implementation working. Additionally, I wanted to practice more image processing skills step by step, being able to track each mask we applied to the image, rather than just running it through a predefined algorithm that spits out a result. 

#### Bill's learning goals:


## Methodology Overview:   
TBD after we finalize code 

### Main loop: 

#### Step 1: ...

#### Step 2: ...

### Final Result:   


## Design Decisions:   

### Calculating the Neato Heading: 
One major design decision we faced was how to determine the Neato’s heading from soley the camera feed. When first approaching this problem, we considered several options: attaching an apriltag to the robot, using PID control to direct it to each location, or (likely the easiest solution) just using odom data. However, our project goals including not relying on the information from the sensors of the robot and using only CV as much as possible. Since we already needed to get a contour of our Neato in order to find the center, we figured we were more than capable of also finding the heading. To do this, we decided to use OpenCV’s Hough Line Transform (thanks, Image Processing!) to detect the main line along the Neato’s body from the contour image. Once that line was found, we could calculate its slope and use a bit of geometry to determine the robot’s heading angle relative to its center point. Suprisingly, this took us a bit of time to get just right; trying to recall information about the law of sines and how to find the intersection of a line. This approach allowed us to estimate the Neato’s orientation entirely from visual information, without relying on any onboard sensors or additional markers. We were able to track both the position and heading of the Neato using only the camera feed—keeping the setup simple, self-contained, and true to the spirit of our project.


### Implimenting Sort with Path Planning: 


## Challenges:   

## Limitations & Potential Improvements:   

## Conclusion:  
### How It Went:

### Lessons Learned: 
