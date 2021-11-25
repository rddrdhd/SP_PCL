#PointCloud matching
The goal of this project is to take 2 different .pcd files (model and scene),
and get the position(s) of model(s) in scene (same subset of points) if there is any.

For now, I am following some PCL tutorials. Now it does correspondece grouping between mug model and table scene with 3 mugs.

##Ubuntu env:

###To run this project:
1. In the terminal, go to the `/build` folder.
2. `cmake ..`
3. `make`
4. `./main.cpp ../pcd_files/MODEL_cup_pink.pcd ../pcd_files/SCENE_table_with_mugs.pcd` 

###Models
In `/pcd_files` folder you can find models with `MODEL_ prefix` (mug), and scenes with `SCENE_` prefix (table with mugs).
###To view the .pcd files in Ubuntu
1. `sudo apt-get install pcl-tools`
2. `pcl_viewer -multiview 1 <pcd_filepath>`

