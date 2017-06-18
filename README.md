# iai-gazebo-models

##hmmwv oshkosh and fiat500 notes:

-all model.sdf files reads the meshes from  <Package_name>/Assets/

###Editing the models:
-the sdf is separated to parts (body wheels suspension file ender etc) for editing ease, 
to edit the sdf edit the appropriate file in the parent model (ex hmmwv/sdf_assets and not hmmwv_velodyne) and then
run the script BuildSDFModelFromAssets.sh to apply the changes to the model.sdf file that gazebo reads.
The velodyne versions script reads the same files of the main model except for the ender that includes the velodyne

-The full assembly blender file is also there. tweaks to the models can be done in the blend file and exported to the dae
(make sure "Apply modifiers" and "Selected Only" are ticked when exporting a part with blender)
 In case the model is very dark edit the emission value in the .dae file to match 50% of the diffuse color in the materials(Edffects_library)

##Launch files and packages

roslaunch hmmwv hmmwv_empty_world.launch
roslaunch oshkosh_model oshkosh_empty_world.launch
roslaunch velodyne velodyne_world.launch
roslaunch velodyne velodyne_16_world.launch



ps:
The hammvee model is old, use hmmwv instead