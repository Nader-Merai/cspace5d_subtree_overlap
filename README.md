
# X,Y,Z,$\theta$,V planning in continuous space with soft duplicate detection using subtree overlap.

### About soft duplicate detection
Please refer to this paper: 

### Dependencies:
This package is only dependent on the sbpl library.
So install the sbpl library first: based on [this](https://github.com/SushiStar/sbpl_cspace) repo, which is from the Search-based planning Lab.

### Run this program:
To build this program, under the ```lmp``` directory, run:
```
mkdir build && cd build
cmake ../ && make
```
To run the planner:
```
./xythetac <mapname> <mapnamestl> <start&goal> <mode>
```
-The \<mapname\> parameter is replaced by the map filename, which is in the format```(*.cfg)```  specifically, the```(map.cfg)``` file found under ```map```, the same as the one in the sbpl library.
-The \<mapnamestl\> parameter is replaced by the .stl map filename, which is in the format```(*.stl)``` found under ```map```.
-The \<start&goal\> parameter is the start and goal setup file, which is in ```*.sg``` format, found under```map```.
-The  \<mode\> parameter decides the planning algorithm used. 0 is for original soft duplicate detection, 1 is for subtree overlap labeler, and 2 is for subtree overlap hash table (hash table needs to be generated).
