#/bin/bash install.sh
cd ../marrtino/marrtino_navigation/
mkdir ./back
cp config/ back/ -r
cp launch/ back/ -r
cp maps/ back/ -r

#overwrite with ours
cp ../../g01_move/conf/* ./ -r