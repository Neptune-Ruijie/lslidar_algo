#!/bin/sh

## 这个脚本工作在远端，使用ssh执行

echo PWD=$PWD

WS_DIR=$1
PACKAGE=$2
OUTPUT=$3
source /opt/ros/foxy/setup.bash 
cd ${WS_DIR} 

echo "++++ => Clean workspace.."
rm -rf install build log 
tar xf ${PACKAGE}.tgz -C ${WS_DIR}/src/${PACKAGE}
echo "++++ => Build packages.."
colcon build --merge-install 
echo "++++ => Archive ${OUTPUT}.."
cd install && tar czf ../${OUTPUT} include lib share 
echo "++++ => Done"
