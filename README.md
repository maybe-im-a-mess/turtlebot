# PLV2 Project

by Olha Solodovnyk
(00818239)

### GETTING STARTED

**Requirements:**
- ROS framework installed
- Gazebo installed
- All the dependencies from requirements.txt

**To start the simulation:**
1. Clone the project using `git clone`.
2. Go to _/simulation_ folder.
3. Check if you have Python library mako installed:
`which mako-render`.
If not, you can use `pip install mako`.
4. Create worlds and models:
`make`.
5. Now you can see the worlds in your _/simulation_ folder.

**To run the bot in the simulation:**
1. Run the world you want to use:
`./play world_1_1.sdf`.
2. Run the python file with the challenge you want.

### About the challenges:

**Challenge 1**: The robot can drive to the red wall as close as it can get and stops without colliding with the wall.

**Challenge 2**: The robot can drive to the red wall, and stop at a safe distance. Then it rotates counter-clockwise and drives close to the wooden wall, after what it stops. For this challenge the laser sensor data is used to rotate 90 degrees.

**Challenge 3**: This challenge consists of the same actions as the previous one, but instead of laser sensors, the position and orientation published in `/odom` are used.

**Challenge 4**: The robot is located in a small maze (2X2 cells). It finds a way to the cell with the red wall and stops in front of it without any collision.

**Challenge 5**: The robot is located in a bigger maze (5x5 cells). It finds a way to the cell with red wall, touches the red wall in the shortest time possible without any other collisions.
