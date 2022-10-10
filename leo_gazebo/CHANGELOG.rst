^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package leo_gazebo
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.0 (2022-10-10)
------------------
* Add launch file for marsyard2022 world
* Contributors: Błażej Sowa

1.0.1 (2022-02-01)
------------------

1.0.0 (2022-01-31)
------------------
* Fix catkin_lint warnings/errors
* Set starting position for marsyard2021 world
* Reformat launch files
* relay odometry to wheel_odom_with_covariance topic
* Remove odom_relay script
* Update diff_drive_controller parameters
* Split the leo_gazebo package into multiple packages
* Update author and maintainer info

0.2.0 (2020-12-18)
------------------
* Specify minimum version for leo_description dependency
* Abandon usage of tf_prefix parameter since it is deprecated
* Use new URDF from the leo_description package
* Pass all args from main launch file
* Change gazebo_differential_plugin to leo_gazebo_differential_plugin

0.1.3 (2020-12-05)
------------------
* Use gazebo_dev dependency instead of gazebo (fix building on debian)
* Update package description

0.1.2 (2020-10-06)
------------------

0.1.1 (2020-09-25)
------------------
* Fixed binary release build
* Added gazebo to dependencies

0.1.0 (2020-09-24)
------------------
* Added marsyard world and marsyard terrain model
* Initial package version
