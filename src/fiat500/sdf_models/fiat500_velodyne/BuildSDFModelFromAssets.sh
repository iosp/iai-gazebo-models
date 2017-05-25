#This script merges all the sdf_assets into the final model.sdf
rm -f model.sdf
cat ../fiat500/sdf_assets/Body.xml ../fiat500/sdf_assets/Suspension.xml ../fiat500/sdf_assets/Steering.xml ../fiat500/sdf_assets/Wheels.xml sdf_assets/Ender.xml > model.sdf
echo model.sdf built!  && sleep 1

