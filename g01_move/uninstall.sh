#/bin/sh uninstall.sh
cd ../marrtino/marrtino_navigation/config
#delete current config
rm *
#write backed up ones
cp ../back/config/* ./
#delete and restore move_base
cd ../launch
rm move_base.launch
cp ../back/launch/* ./
#delete back
rm ../back -rf
