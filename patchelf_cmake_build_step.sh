#!/bin/bash

# Get the current directory using pwd
base_dir=$(pwd)

lib_to_modify="$base_dir/install/robot/lib/robot/libCTRE_Phoenix.so"
exe_to_modify="$base_dir/install/robot/lib/robot/main"
new_rpath="$base_dir/install/robot/lib/robot"

# Use patchelf to set the new RPATH
patchelf --set-rpath "$new_rpath" "$lib_to_modify"
patchelf --set-rpath "$new_rpath" "$exe_to_modify"