^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package joint_qualification_controllers
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.14 (2019-03-27)
-------------------

1.0.13 (2019-03-26)
-------------------
* updated maintainer status
* Merge pull request `#4 <https://github.com/PR2/pr2_self_test/issues/4>`_ from mikaelarguedas/update_pluginlib_macros
  update to use non deprecated pluginlib macro
* update to use non deprecated pluginlib macro
* Contributors: Austin, David Feil-Seifer, Mikael Arguedas

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
* Added install files line for controller_plugins.xml
* Contributors: dash

1.0.3 (2014-10-15)
------------------

1.0.2 (2014-09-16)
------------------
* Version #s
* Versions updated
* Updated versions
* Fixed jqc
* Changelogs
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
