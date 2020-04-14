
#!/bin/bash
# Armin Niedermueller 


echo "Starting Both 3D Cams, 3D Registration, MoveIt! and Rviz"
sleep 1

echo "Starting Intel D435 3D Camera Nodes"
gnome-terminal -e "roslaunch registration_3d start_3d_cams.launch"
sleep 3

echo "Starting ICP Alignment"
gnome-terminal -e "bash -c 'rosrun registration_3d preprocess_align_publish allsteps=true algorithm=nlicp /cam_1/depth/color/points /cam_2/depth/color/points'"
sleep 8

echo "Aligning 3D Cameras to Desk"
gnome-terminal -e "rosrun tf static_transform_publisher 0.55 1.1 1.5 0 0.1 2.44 world cam_2_depth_optical_frame 3" 
sleep 3

echo "Preprocess Point Clouds"
gnome-terminal -e "bash -c 'rosrun collision_avoidance pointcloud_preprocessing passthrough=true'"
sleep 3

echo "Starting MoveIt! and Rviz"
gnome-terminal -e "bash -c 'roslaunch collision_avoidance collision_avoidance.launch'"
sleep 1

echo "Finished"
