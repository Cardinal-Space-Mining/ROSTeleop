# ROSTeleop

# Loading the CANABLE on the Linux Machine
1. `sudo slcand -o -c -s0 /dev/ttyACM0 can0` (On my machine this is only needed the first time time)
2. `sudo ip link set $interface type can bitrate 1000000`
3. `sudo ip link set $interface up`
4. `sudo ip link set $interface txqueuelen 1000`

# Mission Control
Using SDL2 for gamepad input. Using IMGUI for options and stuff

# Building
`colcon build --executor parallel --cmake-args=-DCMAKE_EXPORT_COMPILE_COMMANDS:BOOL=ON`