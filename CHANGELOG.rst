^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package as2_platform_dji_psdk
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.0 (2024-08-20)
------------------
* [fix] psdk_wrapper.launch.py by removing tab in default config files
* [feat] Update changes in launcher and params to psdk_ros2 v1.3.0
* [feat] Update launcher with param utils
* [test] flake8 passing
* [ci] new ci added
* [fix] velocity sub topic
* [feat] Add tf_frame_prefix param
* [feat] Improve gimbal using TF
* [fix] Avoid sending duplicated gimbal command
* [fix] platform take off
* [feat] Add gimbal
* [refactor] Code refractor without impl
* [feat] Add HMS file
* [feat] Add gimbal and camera intialization flags
* [fix] speed state and yaw reference frames
* [feat] obtain ctrl authority
* [feat] change to SynchronousServiceClient
* [test] enable test dir
* [docs] add copyright
* [style] add hints to platform virtual methods
* [test] Use ament_lint_auto
* [build] Add psdk_interfaces dependency
* Contributors: Miguel Fernandez-Cortizas, Rafael Perez-Segui, Santiago Tapia-Fernández, pariaspe
