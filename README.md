# as2_platform_dji_psdk

[![build](https://github.com/aerostack2/as2_platform_dji_psdk/actions/workflows/build_humble.yaml/badge.svg)](https://github.com/aerostack2/as2_platform_dji_psdk/actions/workflows/build_humble.yaml)
[![ROS 2 Humble](https://img.shields.io/badge/ROS_2-Humble-22314E?logo=ros)](https://docs.ros.org/en/humble/)
[![License](https://img.shields.io/badge/License-BSD_3--Clause-blue.svg)](LICENSE)
[![arXiv](https://img.shields.io/badge/arXiv-2303.18237-b31b1b.svg)](https://arxiv.org/abs/2303.18237)

[Aerostack2](https://github.com/aerostack2/aerostack2) platform for DJI drones that use the
[DJI Payload SDK](https://github.com/dji-sdk/Payload-SDK) (PSDK), such as the Matrice 30,
Matrice 300 RTK and Matrice 350 RTK.

The platform talks to the drone through [`psdk_ros2`](https://github.com/umdlife/psdk_ros2), so a
running deployment has two nodes: the `psdk_wrapper` node, which owns the connection to the drone,
and the Aerostack2 platform node, which translates Aerostack2 commands into `psdk_wrapper` topics
and services.

> [!NOTE]
> For older drones that only support the deprecated Onboard SDK, such as the Matrice 210 V2, use
> [`as2_platform_dji_osdk`](https://github.com/aerostack2/as2_platform_dji_osdk).

> [!IMPORTANT]
> This package can't be released to the ROS build farm or listed on the ROS Index yet, because its
> `psdk_interfaces` dependency isn't released for any ROS 2 distribution newer than Humble. Building
> it from source, together with [`psdk_ros2`](https://github.com/umdlife/psdk_ros2), works.

## Requirements

- Ubuntu 22.04 with [ROS 2 Humble](https://docs.ros.org/en/humble/Installation.html)
- [Aerostack2](https://aerostack2.github.io/_00_getting_started/index.html)
- [`psdk_ros2`](https://umdlife.github.io/psdk_ros2/documentation/GettingStarted.html), which
  provides `psdk_interfaces` and the `psdk_wrapper` node
- A DJI developer app (`app_name`, `app_id`, `app_key` and `app_license`) from the
  [DJI Developer Center](https://developer.dji.com/)

## Build

From the root of your workspace:

```bash
rosdep install -y -r --ignore-src --from-paths src --packages-up-to as2_platform_dji_psdk
colcon build --packages-up-to as2_platform_dji_psdk
```

## Usage

Set your DJI app credentials in `config/psdk_authentication.yaml`:

```yaml
/**:
  ros__parameters:
    app_name: "YourPSDK"
    app_id: "YourID"
    app_key: "YourAppKey"
    app_license: "YourAppLicense"
    developer_account: "YourDeveloperAccount"
    baudrate: "921600"
```

Set the serial, USB bulk or network devices that connect to the drone in
`config/link_config.json`.

Then launch the `psdk_wrapper` node and the platform:

```bash
ros2 launch as2_platform_dji_psdk psdk_wrapper.launch.py namespace:=drone0
ros2 launch as2_platform_dji_psdk as2_platform_dji_psdk.launch.py namespace:=drone0
```

### `psdk_wrapper.launch.py`

| Argument | Default | Description |
| --- | --- | --- |
| `namespace` | `$AEROSTACK2_SIMULATION_DRONE_ID` | Drone namespace |
| `psdk_authentication_params_file` | `config/psdk_authentication.yaml` | DJI app credentials |
| `link_config_file_path` | `config/link_config.json` | Connection to the drone |
| `hms_return_codes_path` | `config/hms_2023_08_22.json` | Known DJI HMS return codes |
| `psdk_params_file` | `config/psdk_params.yaml` | Wrapper modules, frames and data rates |
| `tf_frame_prefix` | `''` | TF frame prefix |

The wrapper's GPS, IMU and main camera topics are remapped to the Aerostack2
`sensor_measurements/` namespace.

### `as2_platform_dji_psdk.launch.py`

| Argument | Default | Description |
| --- | --- | --- |
| `namespace` | `$AEROSTACK2_SIMULATION_DRONE_ID` | Drone namespace |
| `log_level` | `info` | Log severity level |
| `control_modes_file` | `config/control_modes.yaml` | Platform control modes |
| `config_file` | `config/platform_config_file.yaml` | Platform parameters |

Platform parameters (command and info rates, camera, gimbal, TF timeout) are in
`config/platform_config_file.yaml`, and each one can also be overridden as a launch argument.

## Test

```bash
colcon test --packages-select as2_platform_dji_psdk
colcon test-result --verbose
```

## Citation

If you use Aerostack2 in your research, please cite:

> M. Fernandez-Cortizas, M. Molina, P. Arias-Perez, R. Perez-Segui, D. Perez-Saura and P. Campoy,
> 2023, ["Aerostack2: A software framework for developing multi-robot aerial systems"](https://arxiv.org/abs/2303.18237),
> arXiv 2303.18237.

## License

[BSD 3-Clause](LICENSE)
