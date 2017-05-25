#This script merges all the sdf_assets into the final model.sdf
rm -f model.sdf
cat ../hmmwv/sdf_assets/Body.xml ../hmmwv/sdf_assets/Suspension.xml ../hmmwv/sdf_assets/Steering.xml ../hmmwv/sdf_assets/Wheels.xml sdf_assets/Ender.xml > model.sdf
echo model.sdf built!  && sleep 1

