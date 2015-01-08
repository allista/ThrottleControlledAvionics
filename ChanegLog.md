**DELETE the old version before installing a new one**
    
_**BUT** do not delete the TCASave.txt file to preserve your settings_

* **v1.4.1**

    * **KSP-0.90 compatible**             
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
