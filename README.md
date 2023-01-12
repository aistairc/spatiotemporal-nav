# Spatiotemporal Navigation based on Pedestrian Trajectory Prediction in Dense Crowds

Spatiotemporal Navigation is planning method over multiple-time prediction maps with the future in mind.

This method ensures smooth moving by considering future pedestrian position.

## **demo video:**


https://user-images.githubusercontent.com/120366557/211988441-c04a7e78-3c66-48be-b7b8-bfcdeeb9a47c.mp4



## **Dataset**
link

## **Using the code:**

You can output png files for creating a demo video by running the following command.

```bash
python3 two_maze.py -i [Path of input image/origin] -o [Path of output directory for simulation images] --st_r [y of a start] --st_c [x of a start] --go_r [y of a goal] --go_c [x of a goal]
python3 two_maze.py -i origin_pred_map/000/origin/ -o output_dir --st_r 270 --st_c 160 --go_r 130 --go_c 240
```

```bash
python3 spatiotemporal.py -i [Path of input image] -o [Path of output directory for simulation images] --st_r [y of a start] --st_c [x of a start] --go_r [y of a goal] --go_c [x of a goal]
python3 spatiotemporal.py -i origin_pred_map/000/ -o output_dir --st_r 270 --st_c 160 --go_r 130 --go_c 240
```

## **LICENSE:**
Copyright 2023 National Institute of Advanced Industrial Science and Technology (AIST), Japan.

This program is licensed under the Apache License, Version2.0.

This program uses code derived from davecom/ClasicComputerScienceProblemsInPython((c) 2018 David Kopec).

The three contributions are as follows
- Enabled input from costmaps
- Implementation of spatiotemporal search
- Addition of robot simulation



[davecom/ClassicComputerScienceProblemsInPython](https://github.com/davecom/ClassicComputerScienceProblemsInPython)

