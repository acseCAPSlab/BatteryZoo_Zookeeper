# BatteryZoo - Zookeeper (Optimal Control) 

This repository is a demonstration of Optimal Control using the [BatteryZoo](https://github.com/acseCAPSlab/BatteryZoo). Here, a fast charging optimal control problem \[needs citation\] is implemented in [ICLOCS2](http://www.ee.ic.ac.uk/ICLOCS/) using the [OPTI Toolbox's](https://github.com/jonathancurrie/OPTI) IPOPT solver \[needs citation\].

## Running the problem

The ```main_BatteryCharging.m``` is the main file inside of the ```SinglePhase``` folder, which can be run for individual batteries of the user's choice. The battery database, at the time of writing, provides data for 40 batteries which can selected by setting ```batt_idx``` in ```BatteryCharging(BattData, batt_idx)``` function call in the main file. An example is shown in the figure below.

![image](https://github.com/acseCAPSlab/SheffieldBatteryLibrary_Control/blob/main/figs/StatePlots_Using_BatchFit.png)

### Running a batch operation 

Addition file ```BatchRunBattOpt.m``` is provided to run the optimal control problem to a batch of batteries which can be chosen by editing the ```indexlist``` variable iniside of the file. An example result of the run is shown below.

![image](https://github.com/acseCAPSlab/SheffieldBatteryLibrary_Control/blob/main/figs/MultipleBatteryBatchRun.png)


## Dependecies
- [ICLOCS2](http://www.ee.ic.ac.uk/ICLOCS/)
- [BatteryZoo](https://github.com/acseCAPSlab/BatteryZoo)
- [OPTI Toolbox](https://github.com/jonathancurrie/OPTI)
