**DELETE the old version before installing a new one**
    
_**BUT** do not delete the TCA.conf and config.xml file to preserve your settings_

* **v2.3.0**
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
