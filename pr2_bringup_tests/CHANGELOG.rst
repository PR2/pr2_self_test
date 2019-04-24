^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package pr2_bringup_tests
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------

1.0.14 (2019-03-27)
-------------------

1.0.13 (2019-03-26)
-------------------
* updated maintainer status
* Contributors: David Feil-Seifer

1.0.12 (2015-09-22 23:58)
-------------------------
* Merge branch 'hydro-devel' of https://github.com/pr2/pr2_self_test into hydro-devel
* Contributors: TheDash

1.0.11 (2015-09-22 12:23)
-------------------------

1.0.10 (2015-09-18)
-------------------

1.0.9 (2015-02-11)
------------------

1.0.8 (2015-02-10)
------------------

1.0.7 (2014-12-02)
------------------

1.0.6 (2014-11-28 19:38)
------------------------

1.0.5 (2014-11-28 02:19:40)
---------------------------

1.0.4 (2014-11-28 02:19:12)
---------------------------

1.0.3 (2014-10-15)
------------------
* Removed mainpage.dox;updated maintainership
* Contributors: TheDash

1.0.2 (2014-09-16)
------------------
* Versions updated
* Changelogs
* Added install targets to pr2_bringup_tests
* Removed unneceesary packagexml and cmake
* Catkinizes and compiles in hydro if you have the hydro-devel of pr2_apps/pr2_mannequin_mode
* Removed manifest.xml
* Update calibration verify to Groovy.
* Move wg_hardware_test to github.
* Update version of wg_hardware_test to use.
* Update life_test rosinstall for electric install.
* Lock wg_hardware_test to current version before changing dependencies.
* PR2 life test rosinstall file uses special version of camera_calibration
* Calibration check for PR2 production calibration, `#4956 <https://github.com/PR2/pr2_self_test/issues/4956>`_
* Removing stale calibration items from pr2_bringup_tests. OK by Vijay
* Only using the wge100 camera package for camera FW installation
* Camera drivers trunk to pr2_bringup_tests rosinstall file
* Removing vestigial dependency on compressed_image_transport
* Correcting size of checkerboard used for prosilica calibration
* Adding camera_calibration package to pr2_bringup_tests
* Removing user paths from rosinstall file
* Controllers in pr2_bringup_tests load using pkg/Name syntax. `#4534 <https://github.com/PR2/pr2_self_test/issues/4534>`_
* Bumping qual pr2_calibration stack to cturtle. Fixing url for relased stacks too
* Life test rosinstall file now works as cturtle overlay
* rosinstall file for pr2_bringup_tests
* Cleanup of pr2_self_test stack before 0.1 release
* Added platform tags for Ubuntu 9.04, 9.10, and 10.04.
* New life_test.rosinstall file with proper locking
* Fixing typos.  Adding scatterplots for preburn
* Putting together calibration scripts for pre/post burn-in checks
* No only grabbing pr2_bringup_tests from pr2_self_test for calibration bringup
* Renaming calibration stack local name
* Adding stub for calibration bringup
* Prosilica settings launch files
* Added dash to version tag for pr2_arm_navigation revision
* Adding calibrate_prosilica launch file to pr2_bringup_tests. `#4249 <https://github.com/PR2/pr2_self_test/issues/4249>`_
* Removing prosilica from pr2_bringup_tests
* Added launch file to calibrate prosilica camera
* Adding rev number for pr2_arm_navigation_tests in life_test rosinstall file. Blocked by release of `#4028 <https://github.com/PR2/pr2_self_test/issues/4028>`_
* Changing topic of IMU chart
* Forearm cameras will trigger off etherCAT chain
* pr2_bringup_tests depends on image view
* Adding script to check IMU data
* Script to check gripper accelerometers with rxplot
* Using mannequin mode for pr2_bringup_tests
* Gripper LED launch file name change
* ROS install for PR2 during burn in tests
* Moving pr2_bringup_tests package to pr2_self_test
* Contributors: TheDash, ahendrix, gerkey, vpradeep, watts
