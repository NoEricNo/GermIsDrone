#!/bin/bash
# Setup script to apply PX4 modifications

echo 'Setting up PX4 with hormone modifications...'

# Clone PX4 if not exists
if [ ! -d 'PX4-Autopilot' ]; then
    git clone https://github.com/PX4/PX4-Autopilot.git --recursive
fi

cd PX4-Autopilot

# Apply modifications
cp ../px4_modifications/hormone_broadcast.msg msg/
cp -r ../px4_modifications/hormone_source/ src/modules/

echo 'Building PX4 with hormone system...'
make px4_sitl_default

echo 'PX4 hormone system ready!'
