# Caltech-ME-EE-CS129-SP23-24
Code for the ME/EE/CS129 - Experimental Robotics Course - Caltech - Spring 23/24

# Course Description
[Caltech Course Catalog 2023-24](https://www.cms.caltech.edu/academics/courses/mecsee-129)

# Project
The project's goal was to create robust and reactive robot that can traverse a Manhattan styled map and execute various functionalities. The robot encompasses a system of 3 IR sensors, a system of 3 ultrasound sensors, and a magnetometer to retrieve information and 2 DC motors to execute the motion. Simple detector functions have been executed in the code to make the sensor data reliable. With these equipments, the robot is capable of executing the following functionalities.

- Basic Line and Tunnel following.
    - capable of autnomously switching between following dark lines and following side tunnels.
    - distinguishes between several components of the map: nodes, deadends, streets
- Manual Driving by the user.
- Undirected Exploration.
    - traverses around the map, obtains information, builds the map.
- Driving to a target on the known map.
    - implements Dijkstra's algorithm to find the shortest path, and traverses down the path to reach the goal.
    - reacts to obstacles and blockages and replans to find a new path.
- Driving to a target not on the known map (Directed Exploration).
    - utilizes Dijkstra's algorithm and explores the map with the intention of finding the target.
    - reacts to obstacles and blockages and replans to find a new path.

# Contributors
Myself and [Edward Ju](https://github.com/Edju03)
