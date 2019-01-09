#/bin/sh uninstall.sh
cd ../marrtino/marrtino_navigation/

#delete current config
rm config -rf
rm launch -rf
rm maps -rf

#write backed up ones
cp ./back/* ./ -r

#delete back
rm ./back -rf
