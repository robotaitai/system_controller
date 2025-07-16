#!/bin/bash
echo "Fixing package.xml name tag..."
sed -i 's/<n>system_controller<\/n>/<name>system_controller<\/name>/' src/system_controller/package.xml
echo "Fixed! Now you can run: colcon build" 