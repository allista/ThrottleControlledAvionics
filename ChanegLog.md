**DELETE the old version before installing a new one**
    
_**BUT** do not delete the TCA.conf file to preserve your settings_

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
    
I hope the problem Beetlecat was having is fixed as well. (Since TCA now detects engines much better)
