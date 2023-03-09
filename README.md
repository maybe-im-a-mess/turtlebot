GETTING STARTED

Requirements:
- ROS framework installed
- Gazebo installed
- All the dependencies from requirements.txt

To start the simulation:
1. Go to "simulation" folder.
2. Check if you have Python library mako installed:
which mako-render
If not, you can use pip install mako.
3. Create worlds and models:
make
4. Now you can see the worlds in your "simulation" folder.

To run the bot in the simulation:
1. Run the world you want to use:
./play world_1_1.sdf
2. Run the python file with the challenge you want.