#!/bin/bash

lib_to_modify=/home/jacobmosier/Desktop/ROSTeleop/install/robot/lib/robot/libCTRE_Phoenix.so
exe_to_modify=/home/jacobmosier/Desktop/ROSTeleop/install/robot/lib/robot/main
new_rpath=/home/jacobmosier/Desktop/ROSTeleop/install/robot/lib/robot



# Use patchelf to set the new RPATH
patchelf --set-rpath "$new_rpath" "$lib_to_modify"
patchelf --set-rpath "$new_rpath" "$exe_to_modify"

