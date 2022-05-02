# PointCloud matching
The goal of this project is to take 2 different .pcd files (model and scene),
and get the position(s) of model(s) (and their transformations) in scene if there is any.

For now, I am following some PCL tutorials. Now it does correspondece grouping between mug model and table scene with 3 mugs.
![img.png](img.png) MODEL_cup_pink.pcd + SCENE_table_with_mugs.pcd = all models in scenes

## Ubuntu env:
Download PCL: https://pointclouds.org/downloads/ 
### To run this project in terminal:
1. Go to the `/build` folder.
2. `cmake ..`
3. `make`
4. `./main.cpp model.pcd scene.pcd`


To display help, run `./main.cpp -h`
### Data
In `/pcd_files` directory you can find models with `MODEL_cup_` prefix (mug), and scenes with `SCENE_table_` prefix (table with mugs).

In `/pcd_real_scenes` directory you can clouds from PGM files. In `/pcd_valve_resized` there are valve models in scale 1:1 to scenes.


To view the .pcd files in Ubuntu `pcl_viewer -multiview 1 <pcd_filepath>`
