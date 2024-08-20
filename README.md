# ROSTeleop

# Loading the CANABLE on the Linux Machine
1. `sudo slcand -o -c -s0 /dev/ttyACM0 can0` (On my machine this is only needed the first time time)
2. `sudo ip link set can0 type can bitrate 1000000`
3. `sudo ip link set can0 up`
4. `sudo ip link set can0 txqueuelen 1000`

# Mission Control
Using SDL2 for gamepad input. Using IMGUI for options and stuff

# Building
`colcon build --executor parallel --cmake-args=-DCMAKE_EXPORT_COMPILE_COMMANDS:BOOL=ON`

# Requirements 
Build Dependency: [patchelf](https://github.com/NixOS/patchelf) (`sudo apt install patchelf`)

# Errata:
* Pheonix Tuner X works with Pheonix 5. 
* While I have a shell script to bring up the can interface, you can use `udevadm` (see [here](https://forum.linuxfoundation.org/discussion/859554/udev-how-to-set-a-rule-depending-on-the-manufacturer-or-the-serial-number)) and `systemd` (see [here](https://www.pragmaticlinux.com/2021/07/automatically-bring-up-a-socketcan-interface-on-boot/)). In fact, this is the prefered method for robots as the rules will keep devices on the same interface even after replugging or restarting the devices.
* While I have a script to patch the proper rpath settings in, you can configure cmake to preserve the rpath of a compiled exe, and patch the distributed shared object once so everything works nicely
* When working with unmanaged Pheonix 5, you need to feed a watchdog using `ctre::phoenix::unmanaged::Unmanaged::FeedEnable` which is poorly documented in the TalonSRX lit.

# Todo 
1. Use ROS2 Joy Package
2. Implement State machine that takes Joy Messages and outputs motor commands