# RoboND: Search and Sample Return

The present repository is my solution to the [Udacity Search and Sample Return Project](https://github.com/udacity/RoboND-Rover-Project).

**Note**: The project has been developped with a resolution of 1024 x 768 and graphics quality "Good", resulting in about 40-45 fps.

---

## Notebook Analysis

### Details of functions provided in the notebook
The notebook was used to develop the basic functions associated with perception of the rover.

Here below we can see the steps from the camera of the rover:
- first the picture in front of the rover is captured
- then it is projected to a top view
- after that filters are used to identify ground, rocks and obstacles
- finally they are projected towards the local coordinates associated to the rover and mean direction is extracted

![alt text](misc/project_example.png "Project example")

### Projection of rover capture to world map
Once the pixels were identified in the local coordinates of the rover, we used its position to project them towards the world map and identify the coordinates of explored area, using different colors for rocks, ground, and obstacles.

![alt text](misc/project_example_map.png "Project example map")

---

## Autonomous Navigation and Mapping

### Explanation of `perception_step` function
The `perception_step` function uses the camera associated to the rover and create a world map. It respects the following logic:
* A raw image is captured from the rover camera.
* A perspective transformation is then applied to convert it to a top view. It is calibrated initially by displaying a grid in front of the Rover.
* Some color threshold are applied to identify rocks, ground, and obstacles. Rocks are within a certain RGB range previously defined. Ground is light and is identified with pixels above a certain RGB threshold. Obstacles are the remaining pixels.
* The resulting points are converted to the rover local coordinates.
* The position and yaw of the rover are used to convert the previous local coordinates to world coordinates.
* Those positions are used to define polar coordinates as well as updating world coordinates.

**Note**: Prior to updating the world coordinates, we ensure that the roll and pitch of the rover are low. It improves greatly the mapping accuracy.

### Explanation of `decision_step()` function
The `decision_step` function uses the processing performed in the previous perception step to decide what action the rover needs to take. It respects the following logic:
* If the rover is moving:
    * We check whether the path seems to be clear right ahead of us. This is performed by checking how many pixels within a distance of 4m and an angle of [-10 deg ; +10 deg] are identified as being part of the navigable area.
    * If the path ahead is clear, we go full speed and accelerate unless we are already above the speed limit.
    * If the path ahead is not clear, we slow down and target to go to a stop.
    * The direction we follow when moving is calculated as the average angles 6 intervals of 1m each, to ensure we don't have a too big impact of the points further away, represented by more pixels. Also, we give a bias by observing only the points in the [-30 deg, +40 deg] range in order to go more towards the left, which helps visit the entire map. Finally, we apply a scaling factor to increase the effect of our decision and limit the steering angle to what is reachable by the rover.
* If the rover is not moving:
    * We check if the path seems to be clear right ahead of us as per the previous methodology but with a higher threshold in term of number of pixels identified as being part of the navigable area.
    * If the path is clear, we start moving again.
    * If the path is not clear, we rotate towards the right. It ensures that we continue following the left wall, as done previously.

## Autonomous navigation
A lot of steps have been necessary to improve the autonomous navigation and mapping of the rover:
* The target area (distance and angle) to check if the path ahead was clear was tuned up manually, as well as the number of navigable pixels to be part of this area.
* The target area to check which direction to follow had to be tuned up as well.
* The direction to follow was mainly influenced by the points far ahead as they were represented by more pixels. It was corrected by giving equal weights to average direction based on equal intervals of distance.
* A bias had to be introduced in the area to look at for which direction to follow to prevent from visiting all the time the same areas. The idea was to try to follow all the time the left wall.
* The roll and pitch of the rover had to be monitored to ensure we had a good mapping accuracy, occuring only when those values were low.

Here are potential improvements that could be made:
* The speed, acceleration, and braking of the rover could be increased.
* There should be a better way to check if an obstacle such as a rock is in front of us. Currently we risk going towards it as the average direction could be in its middle.
* We should decide which areas to explore based on the previously visited areas.
* We should ensure that we visit each border of the map by ensuring that all sides are limited by some obstacles.

![alt text](misc/autonomous_navigation.png "Autonomous navigation")