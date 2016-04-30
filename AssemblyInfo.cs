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

[assembly: AssemblyVersion("3.2.0.0")]
[assembly: KSPAssembly("ThrottleControlledAvionics", 3, 2)]

// The following attributes are used to specify the signing key for the assembly, 
// if desired. See the Mono documentation for more information about signing.

//[assembly: AssemblyDelaySign(false)]
//[assembly: AssemblyKeyFile("")]

namespace ThrottleControlledAvionics
{
	public static class KSP_AVC_Info
	{
		public static readonly string  Name          = "ThrottleControlledAvionics";
		public static readonly Version HangarVersion = Assembly.GetCallingAssembly().GetName().Version;
		public static readonly Version MinKSPVersion = new Version(1,0,5);
		public static readonly Version MaxKSPVersion = new Version(1,0,5);
		public static readonly string  VersionURL    = "https://raw.githubusercontent.com/qfeys/ThrottleControlledAvionics/master/GameData/ThrottleControlledAvionics/ThrottleControlledAvionics.version";
		public static readonly string  UpgradeURL    = "http://spacedock.info/mod/198/Throttle%20Controlled%20Avionics";
		public static readonly string  ChangeLogURL  = "https://raw.githubusercontent.com/qfeys/ThrottleControlledAvionics/master/ChanegLog.md";
		public static readonly string  VersionFile   = "ThrottleControlledAvionics.version";
	}
}