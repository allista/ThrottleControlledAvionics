//  Author:
//       Allis Tauri <allista@gmail.com>
//
//  Copyright (c) 2015 Allis Tauri
//
// This work is licensed under the Creative Commons Attribution-ShareAlike 4.0 International License. 
// To view a copy of this license, visit http://creativecommons.org/licenses/by-sa/4.0/ 
// or send a letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.

using System;
using System.Reflection;
using AT_Utils;

// Information about this assembly is defined by the following attributes. 
// Change them to the values specific to your project.

[assembly: AssemblyTitle("ThrottleControlledAvionics")]
[assembly: AssemblyDescription("Plugin for the Kerbal Space Program")]
[assembly: AssemblyConfiguration("")]
[assembly: AssemblyCompany("")]
[assembly: AssemblyProduct("")]
[assembly: AssemblyCopyright("Allis Tauri")]
[assembly: AssemblyTrademark("")]
[assembly: AssemblyCulture("")]

// The assembly version has the format "{Major}.{Minor}.{Build}.{Revision}".
// The form "{Major}.{Minor}.*" will automatically update the build and revision,
// and "{Major}.{Minor}.{Build}.*" will update just the revision.

#if NIGHTBUILD
[assembly: AssemblyVersion("3.4.*")]
#else
[assembly: AssemblyVersion("3.5.0.0")]
#endif
[assembly: KSPAssembly("ThrottleControlledAvionics", 3, 5)]

// The following attributes are used to specify the signing key for the assembly, 
// if desired. See the Mono documentation for more information about signing.

//[assembly: AssemblyDelaySign(false)]
//[assembly: AssemblyKeyFile("")]

namespace ThrottleControlledAvionics
{
	public class ModInfo : KSP_AVC_Info
	{
		public ModInfo()
		{
			MinKSPVersion = new Version(1,3,1);
			MaxKSPVersion = new Version(1,3,1);

			VersionURL   = "https://raw.githubusercontent.com/allista/ThrottleControlledAvionics/master/GameData/ThrottleControlledAvionics/ThrottleControlledAvionics.version";
			UpgradeURL   = "http://spacedock.info/mod/198/Throttle%20Controlled%20Avionics";
			ChangeLogURL = "https://raw.githubusercontent.com/allista/ThrottleControlledAvionics/master/ChanegLog.md";
		}
	}
}
