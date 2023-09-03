# Autonomous-Navigation-Mastering-the-Follow-the-Gap-Technique
This repository delves into the world of autonomous driving, spotlighting the "Follow the Gap" method. By leveraging LiDAR technology, it introduces a reactive obstacle avoidance algorithm. Starting with preprocessing laser scans to dynamically determining optimal driving paths, experience how a car smartly navigates simulated terrains.


## Overview

In the "Follow the Gap" project, the primary focus was to implement a reactive obstacle avoidance algorithm. Using the foundational concepts, I ventured into creating an algorithm that would efficiently make a car navigate autonomously around various simulated terrains. The crux of the implementation resided in analyzing laser scans, detecting gaps, and directing the car towards the most optimal goal point within these gaps.

## Key Components and Features

- **Laser Scan Preprocessing:** The initial step involved obtaining laser scan readings and preparing them for further analysis. The purpose of this preprocessing was to streamline the raw data and make it more apt for detecting obstacles and potential paths.

- **Closest Point Detection and Safety Bubble Creation:** After preprocessing, the closest point in the LiDAR ranges array was pinpointed. Surrounding this closest point, a safety bubble was drawn, ensuring that all points inside this bubble were marked as zero. This process effectively differentiated between obstacle points and the 'gaps' or 'free space'.

- **Gap Detection:** The aim here was to discern the longest 'gap', which essentially means identifying the largest sequence of consecutive non-zero elements in the range array. This 'gap' represents the largest available free space for the car to navigate through.

- **Goal Point Selection within Gap:** The goal wasn't just to find any point within the gap, but the 'best' one. While a simple approach could involve selecting the farthest point within the gap, optimizations were made to enhance driving speed and safety based on the "Better Idea" method discussed during lectures.

- **Car Actuation:** Post the selection of the optimal goal point, the car was directed to move towards it. This was achieved by publishing an AckermannDriveStamped message to the `/drive` topic.

- **Versatility in Development Environment:** The project provided the flexibility to implement the node in either C++ or Python, showcasing adaptability in the choice of programming language.

## Testing

The car's autonomous navigation was validated across two distinct maps:

- **Levine Hall Map:** A primary terrain where the car was expected to showcase its obstacle avoidance prowess.

- **Extra Test Maps:** Two additional maps, namely `levine_blocked.png` (a simpler, empty terrain) and `levine_obs.png` (a challenging terrain with intricate obstacles), were employed to rigorously evaluate the implemented algorithm.

![image](https://github.com/Saibernard/Autonomous-Navigation-Mastering-the-Follow-the-Gap-Technique/assets/112599512/e2bd5499-b01a-4390-b0c5-f5eb0e942d47)


![image](https://github.com/Saibernard/Autonomous-Navigation-Mastering-the-Follow-the-Gap-Technique/assets/112599512/43a31596-920f-4afe-a374-5b84364252ff)


### Gap follow in simulation:
 
 https://youtu.be/wIkvEwAcDU8

### Gap follow algorithm with car overcoming obstacles in simulation:

 https://youtu.be/xYAHQdddTKo

 ### Gap follow implenetation in actual f1tenth car:
 
 https://youtu.be/bYpef-Hq3GA

## Documentation

To ensure comprehensive documentation and showcase the car's obstacle avoidance capabilities in action, screen casts were created, capturing the car's navigation across both test maps.

## Additional Resources

A valuable visual resource that proved instrumental during the course of the project was the UNC Follow the Gap Video.

All the code, implementation details, and further documentation, including the screen casts, can be accessed on this repository.
