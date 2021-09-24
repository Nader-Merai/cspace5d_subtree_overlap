
# X,Y,Z,theta,V planning in continuous space with soft duplicate detection using subtree overlap.

### About soft duplicate detection
Please refer to this paper: 

### Dependencies:
This package is only dependent on the sbpl library.
So install the sbpl library first: based on [this](https://github.com/Nader-Merai/sbpl_cspace) repo, which is from the Search-based planning Lab.

### Run this program:
To build this program, under the ```lmp``` directory, run:
```
mkdir build && cd build
cmake ../ && make
```
To run the planner:
```
./xyzyaw <mapname> <mapnamestl> <start&goal> <mode> <producehashtable>
```
-The \<mapname\> parameter is replaced by the map filename, which is in the format```(*.cfg)```  specifically, the```(map.cfg)``` file found under ```map```, the same as the one in the sbpl library. <br />
-The \<mapnamestl\> parameter is replaced by the .stl map filename, which is in the format```(*.stl)``` found under ```map```. <br />
-The \<start&goal\> parameter is the start and goal setup file, which is in ```*.sg``` format, found under```map```. <br />
-The  \<mode\> parameter decides the planning algorithm used. 0 is for original soft duplicate detection, 1 is for subtree overlap labeler, and 2 is for subtree overlap hash table. <br />
-The  \<producehashtable\> parameter with a value of 1 produces the hash table values into "Hash_Table_Values.txt", and with a value of 0 reads the hash table values from "Hash_Table_Values.txt". value of 1 must be used at least once to generate the hash table values, and after that, value 0 may be used to avoid hash table values recalculation.

### Reproducing results and plots:
The directory by the name of ```err_bars_mean_std_err``` is for reproducing results, specifically raw results, error bars, and Average +- Standard Error statistics. <br />
In order to get the results, the following steps must be done:<br />
-Build the program as shown above<br />
-Go into the ```err_bars_mean_std_err``` directory<br />
-Run: chmod 777 Copy_Scripts.txt Make_Plot.txt<br />
-Run: ./Copy_Scripts, or copy each script to the specified directory in the "Copy_Scripts.txt" file<br />
-Go into the specified directory of each copied script and run it.<br />
-Go back into the ```err_bars_mean_std_err``` directory<br />
-There you will find all the raw data.<br />
-To reproduce the plots, Run: ./Make_Plot.txt<br />
-To view the Average +- Standard Error statistics of a certain run, Run: python3 Average_STD ${path_to_raw_data_run}