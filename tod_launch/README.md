# tod_launch
Once built, the software can be launched:
  * Source the workspace:
    `source devel/setup.zsh # or setup.bash - depending on your shell`
  * Launch the nodes:
    * Operator side:
      `roslaunch tod_launch operator.launch`
    * Vehicle side:
      `roslaunch tod_launch vehicle.launch`
    * Both sides:
      `roslaunch tod_launch both.launch`

### Launch Configuration
The launch files can be configured as required.
  * `vehicleID`: Identifier (string) of the vehicle that is being teleoperated. Each
  vehicle requires the provision of a configuration in the `tod_vehicle_interface/tod_vehicle_config` package and a bridge package
  under `tod_vehicle_interface` (for vehicle side only).
  * `launchPackageName`: Flag (boolean) to enable/disable launch of a package.
  Useful if certain packages are not needed, or could not be built due missing
  dependencies.
  * `mode`: Mode (string) how the software should be launched (relevant for vehicle side only).
    * `vehicle`: Used to teleoperate an actual or simulated vehicle. Launches all drivers and
    vehicle hardware interfaces as specified in the bridge package of the respective `vehicleID`.
    * `onlySim`: Launches only the vehicle simulation node.
