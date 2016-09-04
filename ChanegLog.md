**DELETE the old version before installing a new one**

_**BUT** do not delete the TCA.user and config.xml files to preserve your settings_

* **v3.2.3**
    * Bugfixes:
        * Fixed rendering of surface markers.
        * Fixed incorrect calculation of Landing Trajectory in some cases.
        * Fixed a bug in maneuver execution that caused it to block sometimes when a maneuver is finished.
        * Fixed and improved landing algorithm of Deorbit Autopilot.
        * Fixed Radar bug that was caused by not resetting the Altimeter.
    * Improved ToOrbit autopilot algorithm.
    * Improved low altitude landing algorithm.
    * Ballistic Jump now uses the Minimum Energy trajectory to conserve fuel. Also fixed instability in gradient-descent optimization of the jump trajectory.
    * Improved Deorbit Autopilot:
        * Better initial orbit calculation. 
        * Added orbit correction that ensures that dorbiting ship will have at least 2km above the target after the main deceleration.
        * Improved landing algorithm.
        * Improved detection and handling of landscape obstacles.
    * Attitude Control uses new Oscillation Detector to quench sporadic oscillations in control output.
    * Improved PersistentRotation management. 
    * The "Enabled" button now flashes red if TCA is disabled when flying.

* v3.2.2
    * Bugfixes:
        * TCA no longer activates a part where it is installed (parts that combine command module with engines should work fine now).
        * Fixed incompatibility with RealismOvehaul.
        * Fixed and improved initial trajectory calculation in DeorbitAutopilot/BallisticJump.
        * In career game: locked maneuver nodes no longer cause NREs in ToOrbit autopilot.
    * Improved orbital landing algorithm; should work better with both stock and FAR aerodynamics.
    * TCA is now aware of PersistentRotation and will wait for the ship to stop rotation before entering the TimeWarp.
    * Added "Status" page to TCA Manual. It shows a warning if TCA is installed improperly. And in career mode it displays purchased TCA modules (including the main subsystem) and their statuses.

* v3.2.1
    * Numerous fixes and improvements to:
        * *Deorbit autopilot*
        * *Landing autopilot*
        * *Attitude* and *Bearing* control
    * Compiled against *AT_Utils* library that will now be common to all my mods. It is distributed with TCA, so no additional action is required.
    * The naming convention of the `.glob` and `.user` files has changed: these files are now named `ThrottleControlledAvionics.glob` and `ThrottleControlledAvionics.user`. If you have done any customizations, don't forget to rename your `TCA.user` file accordingly.

* v3.2.0
    * Added orbital autopilots ([see the video](https://www.youtube.com/watch?v=l1e2brjWgBA)):
        * **ToOrbit autopilot** that achieves a circular orbit defined by radius and inclination, prograde or retrograde. It is smart about high orbits as it uses the double-burn technique, which is how the real satellites get to the geostationary orbit.
        * **Rendezvous autopilot** that can bring a ship to its target-in-orbit from anywhere in the same sphere of influence: from orbit, from a suborbital trajectory and from the surface.
        * **Deorbit autopilot** that lands a ship from orbit precisely at the selected target.
        * **Ballistic jump** that brings a ship to a target located far away on the surface of the same planet using a ballistic trajectory. It is, of course, better to use it in vacuum or sparse atmosphere.
    * Added **VTOL Control** module that allows you to control a VTOL like a copter in a remote-controller style ([see the video](https://www.youtube.com/watch?v=VWFwzOA9YG0)).
    * **Renamed** two engine modes. I hope this way it's more intuitive:
        * Main = "Thrust & Maneuver"
        * Balanced = "Thrust"
    * **Control-freak special**: added a set of switches to the advanced tab that allow to disable automatic usage of Staging, Landing Gear, Brakes and Parachutes.
    * Added **verbose status messages** at the bottom of the TCA window. *These are persistent and may be dismissed by clicking on them.*
    * **Numerous bugfixes and improvements** in almost every module.

* v3.1.0
    * Added **bearing controls** to the main GUI and keyboard. Works independently of CruiseControl, e.g. you may set the bearing in Stop or Level mode.
    * Added **UnBalanced** engine mode which is a mix of Main and Manual: like Main it responses to altitude/vert.velocity controls, but like Manual it is not used for controlled torque generation. Like Manual engine it may be balanced with other engines in Main, Maneuver or Balanced modes. The main use: to make single-engine balanced-by-design VTOLs and rocket landers.
    * Added **configurable button colors**. See the top the *TCA.glob* file for details.
    * Settings in *TCA.glob* could (and should) now be overwritten in GameData/ThrottleControlledAvionics/**TCA.user** file. Read **[the manual](https://github.com/qfeys/ThrottleControlledAvionics/blob/master/GameData/ThrottleControlledAvionics/INSTRUCTIONS.md#tcaglob--the-global-tca-settings)** for details.

* v3.0.1
    * Fixed single-engine optimization bug.
    * Fixed onStage profile update bug.
    * Improved HSC behaviour at very low maximum TWR of slow engines by adding correction for negative vertical speed.
    * TCA GUI now only shows when the active vessel is loaded and unpacked.

* v3.0.0
    * TCA is now a Part Module running in a ship's cockpit/probe-core. This means **many ships may run TCA simultaneously**.
    * TCA has **TONS** of new features. Please, read **[The Manual](https://github.com/qfeys/ThrottleControlledAvionics/blob/master/GameData/ThrottleControlledAvionics/INSTRUCTIONS.md)**.
    * These features are divided into modules and distributed along the TechTree in career mode. Even more, there are cross-module dependencies that do not follow the stock tree. Please, consult **[TCA Tech Tree](https://drive.google.com/file/d/0B3yiXDvgwkLMMWNHTUdwMkpRYUk/view)** graph to know more.

**Acknowledgments**: I want to express extreme gratitude to **[smjjames](http://forum.kerbalspaceprogram.com/index.php?/profile/134004-smjjames/)**, who helped me so much with the testing and proposed countless improvements. Without it this version would never have come to a release!

* v2.3.0.1
    * Fixed issues with engines' balancing when Vertical Speed Control is disabled.
    * Fixed handling of flameouted engines.
    * Fixed issues with changing TCA key on some OSes.

* v2.3.0
    * Added "Balanced Thrust" engine Role. With it you may use jets and other slow-response engines as a main lifting force; they are still unusable for maneuvering.
    * Improved Vertical Speed Control System: actual vertical speed is much closer to the set-point now.
    * Improved flip-over prevention system of Kill Horizontal Speed mode: it now considers current TWR and decreases the pitch angle if it drops too much. Also useful for jets, as they tend to loose power when turned against the air stream.
    * Several smaller improvements.

* v2.2.1.1 - compatible with KSP-1.0.4
    * Fixed engines' velCurve effect calculation when balancing engines.
    * Added atmCurve effect calculation.

* v2.2.1 - compatible with KSP-1.0.2
    * Fixed compatibility issues with the new KSP-API
    * Added UseStockAppLauncher flag into TCA.glob to force the use of AppLauncher even if the Toolbar is installed.

* v2.2.0
    * Added **engine modes**: Main, Maneuver, Manual.
    * Improved balancing of engines that have velocity curves.
    * Torque from **SRBs** and other throttle-locked engines is now **accounted for**.
    * Tuning parameters are now hidden when Autotuning is enabled.
    * Many **performance improvements**.
    * Fixed the conflict with FMRS window.

* v2.1.0
    * Added an **autopilot to kill horizontal speed**. Useful for landing.
    * Added **per-game configuration**; vessel configs are not lost on game switch anymore.
    * Fixed the regression in optimization algorithm and **improved balancing of asymmetric designs**.
    * Fixed problems with steering from the External Command Seat.
    * Other minor fixes.

* v2.0.2
    * **Integrated TCA into career mode.** You will have to purchase it in Specialized Controls to use it.
    * Improved optimization algorithm. Controls should be smoother and thrust should not decrease when torque demand is inadequate.
    * TCA key binding is configurable from the GUI now.
    * Added an option to control vertical speed slider with throttle controls.    

* v2.0.1
    * Modified optimization algorithm to work properly with engines which minThrust is not zero; Vertical Speed Factor is now also taken into account.
    * Corrected Toolbar/AppLauncher icon switching with TCA state.
    * When Loosing Altitude, vertical speed is still displayed.
    * Positions of the windows are now properly saved and loaded.
    * Fixed KSP-AVC .version file to comply with the strict JSON and to work with CKAN.

* v2.0.0
    * Drastically **improved in-flight stability** by completely changing the way engines' limits are calculated. Now it is done via iterative optimization of the error function.
    * Added **smooth vertical speed control** which allows not only hovering, but also fast and agile horizontal flight; even with very unbalanced designs.
    * Added **several configurable parameters** that allow to tune TCA even for the most bizarre ship designs.
    * Added **automatic parameter tuning** that works well with stock SAS and standard ship builds.
    * Added **sophisticated settings system** with separate Global, Per-Vessel and Named configurations. Configuration files are automatically generated on first launch and are located under **`Plugins/PluginData/ThrottleControlledAvionics`**
        * The default **TCA.glob** file contains, among other presets, the **key binding** for enabling TCA.

* v1.4.1

    * KSP-0.90 compatible
    * Made Toolbar support optional.
    * Added basic support of the stock AppLauncher.
    * Added KSP-AVC support.
    * Fixed the "NullRef on engine destroyed" bug.
    * Optimized performance a little.
    * Changed Mod folder structure to canonical form.

* Unofficial v1.3

    * Checks whether engines are ignited or not, so engines that are not yet staged or deactivated buy GUI or actiongroup are unavailable for the calculations.
    * Uses an engineWrapper class (from EngineIgnitor suggested by camlost) to support engines of the type ModuleEnginesFX (like RAPIERs) along normal engines.
    * Supports parts with multiple engine parts (such as RAPIERs)
    * No longer stops looking for engines after finding an SRB.   
    * I hope the problem Beetlecat was having is fixed as well. (Since TCA now detects engines much better)
