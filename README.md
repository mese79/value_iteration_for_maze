## Value Iteration for Maze

In a maze world we have a grid of cells including Goals and Pitfalls surrounded by walls.  
A robot in this world can move in one of the four relative directions (Up, Down, Left, and Right). However, there is some noise in robot movements: when the robot tries to move in any given direction, there is a probability that it actually moves to a different direction prependicular to the intended direction.   
For example, taking the direction Left moves the robot to the left cell 80% of the time, to the cell above 10% of the time, and to the cell below 10% of the time. If the robot hits a wall, it will stay to the current cell (it will not move).  

#### Maze Configuration
In `main.cpp`, through lines 15-35 you can config your maze world.  

#### Requirements
- C++ >= 17
- cmake
- Python >= 3.6
- numpy (pip install numpy)
- pygame (pip install pygame)


#### Building & Running
```bash
mkdir build
cd build
cmake ..
make
```

Then run the built project by:
```bash
./value-iteration
```

You can visualize the result by:  

```bash
python ./show.py
```

