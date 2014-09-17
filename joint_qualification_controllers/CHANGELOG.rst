^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package joint_qualification_controllers
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.1 (2014-09-16)
------------------
* Added install targets
* Removed unneceesary packagexml and cmake
* Added include statement in CMakeList
* Removed manifest.xml
* joint_qualification_controllers catkinized and compiles in hydro
* Clean up.
* Export HysteresisController2 plugin
* Implementation of new Hysteresis Controller. Needs testing.
* Start work on repeat hysteresis controller.
* Removing unnecessary msg/srv exports in manifest
* Doc updates on all controllers
* Fix spelling of qualification for controller plugin list
* CB test controller now checks for proper configuration for lift/flex tests
* Joint qualification controllers now loading using correct class name
* Typo in CB check controller
* Joint qualification controllers now use pluginlib 1.1x series declare macro. `#4053 <https://github.com/PR2/pr2_self_test/issues/4053>`_
* Cleanup of pr2_self_test stack before 0.1 release
* Added platform tags for Ubuntu 9.04, 9.10, and 10.04.
* Added URDF dependency to joint_qual_controllers, fixed INFO publishing
* Correct tolerance on CB bar adjustment
* CB controller now looks for tolerances for CB adjustments. `#4224 <https://github.com/PR2/pr2_self_test/issues/4224>`_
* Fixed syntax error in CB controller
* Correct number of flex positions in CB test controller
* Adding dependency on realtime tools to joint_qualification_controllers
* Moving joint_qualification_controllers to pr2_self_test
* Contributors: TheDash, ahendrix, gerkey, watts
