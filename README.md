# STP4: Spatio Temporal Path Planning based on Pedestrian Trajectory Prediction in dense crowds

This repository includes path planning methods with simulated crowds of people dataset in 2D space.

- Codes : 2D-A* search (two_maze.py) and STP4 (spatiotempral.py)
- Dataset : time-series of pedestrians' maps used for path planning codes

  Dataset 1: Prediction map - 50pedestrians * 50scenes
    [https://data.airc.aist.go.jp/dense_crowds_dataset/50pedestrians_50scenes.tar.xz(1.7GB)](https://data.airc.aist.go.jp/dense_crowds_dataset/50pedestrians_50scenes.tar.xz)

  Dataset 2: True trajectory - 50pedestrians * 50scenes
    [https://data.airc.aist.go.jp/dense_crowds_dataset/50pedestrians_50scenes_true_trajectory.tar.xz(94MB)](https://data.airc.aist.go.jp/dense_crowds_dataset/50pedestrians_50scenes_true_trajectory.tar.xz)

## **demo video:**

It can simulate robots navigation in dense crowds of pedestrian.

https://user-images.githubusercontent.com/120366557/220135318-1b6ae2e7-c329-4853-ac9a-53303ea7f1af.mp4


## **Using the code:**

You can output path planning results as png files for creating a demo video by running the following command.

- Execute navigation with 2D-A*.
```bash
python3 two_maze.py -i [Path of input image/origin] -o [Path of output directory for simulation images] --st_r [y of a start] --st_c [x of a start] --go_r [y of a goal] --go_c [x of a goal]
```
- Execute navigation with STP4.
```bash
python3 spatiotemporal.py -i [Path of input image] -o [Path of output directory for simulation images] --st_r [y of a start] --st_c [x of a start] --go_r [y of a goal] --go_c [x of a goal]
```

### Examples
- Execute 2D-A* with the above demo video setup. The -i option must be specified to use the current information. Since no future information is used, dataset1 and 2 will have the same result.
```bash
python3 two_maze.py -i 50pedestrians_50scenes/000/origin/ -o output_dir --st_r 270 --st_c 160 --go_r 130 --go_c 240  (2D-A*:1S→1G)
python3 two_maze.py -i 50pedestrians_50scenes/000/origin/ -o output_dir --st_r 270 --st_c 120 --go_r 130 --go_c 280  (2D-A*:2S→2G)
python3 two_maze.py -i 50pedestrians_50scenes/000/origin/ -o output_dir --st_r 270 --st_c 80 --go_r 130 --go_c 320   (2D-A*:3S→3G)
```
- Execute STP4 with the same scenario using the prediction map (Dataset 1)
```bash
python3 spatiotemporal.py -i 50pedestrians_50scenes/000/ -o output_dir --st_r 270 --st_c 160 --go_r 130 --go_c 240   (Proposed:1S→1G)
python3 spatiotemporal.py -i 50pedestrians_50scenes/000/ -o output_dir --st_r 270 --st_c 120 --go_r 130 --go_c 280   (Proposed:2S→2G)
python3 spatiotemporal.py -i 50pedestrians_50scenes/000/ -o output_dir --st_r 270 --st_c 80 --go_r 130 --go_c 320    (Proposed:3S→3G)
```

- Execute STP4 with the same scenario using the ground truth map (Dataset 2)
```bash
python3 spatiotemporal.py -i 50pedestrians_50scenes_true_trajectory/000/ -o output_dir --st_r 270 --st_c 160 --go_r 130 --go_c 240   (Proposed:1S→1G)
python3 spatiotemporal.py -i 50pedestrians_50scenes_true_trajectory/000/ -o output_dir --st_r 270 --st_c 120 --go_r 130 --go_c 280   (Proposed:2S→2G)
python3 spatiotemporal.py -i 50pedestrians_50scenes_true_trajectory/000/ -o output_dir --st_r 270 --st_c 80 --go_r 130 --go_c 320    (Proposed:3S→3G)
```

## **Dense crowds dataset structure**
- Dataset : time-series of pedestrians' maps used for path planning codes

  Dataset 1: Prediction map - 50pedestrians * 50scenes
    [https://data.airc.aist.go.jp/dense_crowds_dataset/50pedestrians_50scenes.tar.xz(1.7GB)](https://data.airc.aist.go.jp/dense_crowds_dataset/50pedestrians_50scenes.tar.xz)

  Dataset 2: True trajectory - 50pedestrians * 50scenes
    [https://data.airc.aist.go.jp/dense_crowds_dataset/50pedestrians_50scenes_true_trajectory.tar.xz(94MB)](https://data.airc.aist.go.jp/dense_crowds_dataset/50pedestrians_50scenes_true_trajectory.tar.xz)

Datasets 1 and 2 are the same scene with the difference between predictions and true values.

The parameters of Dataset 1 and Dataset 2 are shown in the following table. 

|  Setting  |  Value  |
| :----: | :----: |
|  Pedestrians motion  |  ORCA  |
|  Preferred velocity  |  1.0[m/s]  |
|  Max. speed  |  2.0[m/s]  |
|  Length of data  |  45[s]  |
|  One time step  |  0.05[s] |
|  Predicted step  |  20[step]  |
| Map size  |  20[m] $\times$ 20[m]  |
|  One pixel  |  0.05[m]  |


The directory structure of dataset is shown in the following tree.

"t" is the future time steps, with t = 0 being the current time.

"T" is the current time.

```
.
├── 000 (1th scene)
│   ├── 000 (T=0.05[s], prediction maps[Dataset 1] or true trajectory[Dataset 2])
│   │   ├── 00.png  (t=1)
|   |   ...
│   │   ├── 19.png  (t=20)
│   │   └── vel.png (vel.png is an empty map using as the start node [t=0 map])
|   ...
│   ├── 899 (T=45.0[s], prediction maps[Dataset 1] or true trajectory[Dataset 2])
│   │   ├── 00.png
|   |   ...
│   │   ├── 19.png
│   │   └── vel.png
│   └── origin (2Dmap at T=0.05~45.0[s])
│       ├── 000.png
|       ...
│       └── 899.png
...
└── 049 (50th scene)
    ├── 000
    │   ├── 00.png
    |   ...
    │   ├── 19.png
    │   └── vel.png
    ...
    ├── 899
    │   ├── 00.png
    |   ...
    │   ├── 19.png
    │   └── vel.png
    └── origin
        ├── 000.png
        ...
        └── 899.png

```

We shall not be responsible for any loss, damages and troubles when you use our dataset.



  

## **LICENSE:**
Copyright 2023 National Institute of Advanced Industrial Science and Technology (AIST), Japan.

This program is licensed under the Apache License, Version2.0.

This program uses code derived from davecom/ClasicComputerScienceProblemsInPython((c) 2018 David Kopec).

The three contributions are as follows
- Enabled input from costmaps
- Implementation of spatiotemporal search
- Addition of robot simulation



[davecom/ClassicComputerScienceProblemsInPython](https://github.com/davecom/ClassicComputerScienceProblemsInPython)

