#! /usr/bin/bash

echo "This script should executed either at workspace/src dir or at package dir"

echo "Getting ament_uncrustify config file"
wget https://raw.githubusercontent.com/ament/ament_lint/rolling/ament_uncrustify/ament_uncrustify/configuration/ament_code_style.cfg
mv ament_code_style.cfg uncrustify.cfg

# Check working dir
WORKINGDIR=$(pwd)
PACKAGEDIR='as2_platform_dji_psdk'
if [[ "$WORKINGDIR" == *"$PACKAGEDIR" ]]; then
# Change to workspace/src dir
  cd ..
else
# Already at ws/src dir, mv uncrustify config file
  mv uncrustify.cfg as2_platform_dji_psdk/
fi

echo "Installing third parties dependencies..."

# Import source repos
vcs import < as2_platform_dji_psdk/depends.repos

cd ThirdParties/Payload-SDK

git checkout release/v3.5

# Ignore Payload_SDK since is already built
touch COLCON_IGNORE
