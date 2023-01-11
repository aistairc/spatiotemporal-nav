# Spatiotemporal Navigation based on Pedestrian Trajectory Prediction in Dense Crowds

## Yuta Sato, Yoko Sasaki, Hiroshi Takemura


## **Using the code:**

```bash
python3 two_maze.py -i [Path of input image/origin] -o [Path of output directory for simulation images] --st_r [y of a start] --st_c [x of a start] --go_r [y of a goal] --go_c [x of a goal]
python3 two_maze.py -i origin_pred_map/000/origin/ -o output_dir --st_r 270 --st_c 160 --go_r 130 --go_c 240
```

```bash
python3 spatiotemporal.py -i [Path of input image] -o [Path of output directory for simulation images] --st_r [y of a start] --st_c [x of a start] --go_r [y of a goal] --go_c [x of a goal]
python3 spatiotemporal.py -i origin_pred_map/000/ -o output_dir --st_r 270 --st_c 160 --go_r 130 --go_c 240
```

## **LICENSE:**
This package uses code derived from davecom/ClasicComputerScienceProblemsInPython(Copyright 2018 David Kopec).
The two contributions are as follows
- Enabled input from images
- Implementation of spatiotemporal search
- Addition of robot simulation
(c) 2023 Yuta Sato


[davecom/ClassicComputerScienceProblemsInPython](https://github.com/davecom/ClassicComputerScienceProblemsInPython)

