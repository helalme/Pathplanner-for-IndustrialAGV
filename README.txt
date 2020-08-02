#### Background:
For industrial applications, often there is a need for an inexpensive and faster decision making AGV (automated guided vehicle). To achieved this goal,
a research project was designed in 2009 combining hardware (vehicle), algorithms and its controlling software. Perception, localization, path-planning
and controlling of that AGV etc. were the core components of that AGV. In this repository, mainly Pathplannar (PP) module (with A* algorithm) is presented,
along with the Tracker module.

#### Module description:
During the operation of an AGV, the control software is supposed to call these two modules (Pathplannar and Tracker) at each time step (e.g. 1 sec).
The PP module gets the input data (coordinates of objects) at each time step from a camera that captures moving objects and process their geometries.
Two modules are the developers copy, not part of the production software. That's why some data are in a simplified format, e.g. the world model (perception area).
At each time step, the PP module calculates the possible shortest distance from its current location to the destination using A* algorithm. Since different
obstacles in the world-model are moving, so the shortest path is also changing at each time step. As a result, it is also necessary to track the position 
of the AVG at each timestep. That's why a tracker is also necessary.

#### ppVisualization.py: 
This is a procedural version (not object oriented) of the pathplanner module. It gives shortest distance implementing A* and shows the
calculated path along with graphical representation obstacles or objects. All objects are assumed rectangular. A sample output of this program is shown in the .PNG file.
      
#### ppCoordinates_OOP.py: 
This one is object-oriented version where graphical representation of the shortest path and obstacles are not shown. Output of this program is the shortest distance
from the start to destination. Corresponding (x,y) coordinates will be printed in a list.  

#### ppModule.py: 
This is the real pathplanner module that was integrated into the main controlling software. It will not run independently as it communicates to other modules.

#### tracker.py:
This module is for real time tracking of AGV, it communicates with ppModule and gives the coordinates of theAGV. This is not 100% complete, rather a part of
developers version. Since it is connected to main control software, it will not run independently. 	

Note:
All of these scripts were developed originally in python 2.x in 2011, however, later these were converted to python 3.x standard using the tool 2to3.py   
