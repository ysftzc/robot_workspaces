### Run integration tests

The [integration_launch_testing](./integration_launch_testing) package provides functional integration tests for Franka controllers and the gripper. These tests require **real hardware**: a Franka robot (e.g. FR3) on the network. The default `robot_ip` is `172.16.0.2`. Use `--skip-gripper` if no gripper is attached.

**Note:** To build this package and activate the smoke tests, you need to enable the `BUILD_TESTING` CMake flag when building the workspace:

```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
```

**Run all integration tests:**

```bash
# If you want to use an IP other than 172.16.0.2
colcon build --packages-select integration_launch_testing --cmake-args -DROBOT_IP=<robot-ip> -DBUILD_TESTING=ON
colcon test --packages-select integration_launch_testing --event-handlers console_direct+
# to inspect the results
colcon test-result --all --verbose
```