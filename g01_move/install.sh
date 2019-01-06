#/bin/bash install.sh
cd ../marrtino/marrtino_navigation/config
mkdir ../back
mkdir ../back/config
#backup current config
cp * ../back/config
#overwrite with ours
cp ../../../g01_move/conf/yaml ./
#backup move_base.launch
cd ../launch
mkdir ../back/launch
cp move_base.launch ../back/launch
#overwrite
cp ../../../g01_move/conf/launch/move_base.launch ./
