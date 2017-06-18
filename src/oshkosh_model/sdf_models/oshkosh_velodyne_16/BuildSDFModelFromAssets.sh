#This script merges all the oshkosh sdf_assets+oshkosh_velodyne ender file into the final model.sdf
rm -f model.sdf
cat ../oshkosh/sdf_assets/Body.xml ../oshkosh/sdf_assets/Suspension.xml ../oshkosh/sdf_assets/Steering.xml ../oshkosh/sdf_assets/Wheels.xml sdf_assets/Ender.xml > model.sdf
echo model.sdf built!  && sleep 1

