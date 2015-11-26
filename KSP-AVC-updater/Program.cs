//  Author:
//       Allis Tauri <allista@gmail.com>
//
//  Copyright (c) 2015 Allis Tauri
//
// This work is licensed under the Creative Commons Attribution-ShareAlike 4.0 International License. 
// To view a copy of this license, visit http://creativecommons.org/licenses/by-sa/4.0/ 
// or send a letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.

﻿using System.IO;
using ThrottleControlledAvionics;

namespace KSPAVCupdater
{
	class MainClass
	{
		public static void Main(string[] args)
		{
			using(var file = new StreamWriter(KSP_AVC_Info.VersionFile))
			{
			file.WriteLine(
@"{{ 
    ""NAME"":""{0}"",
    ""URL"":""{1}"",
    ""DOWNLOAD"":""{2}"",
    ""CHANGE_LOG_URL"":""{3}"",
    ""VERSION"":
     {{", KSP_AVC_Info.Name, KSP_AVC_Info.VersionURL, KSP_AVC_Info.UpgradeURL, KSP_AVC_Info.ChangeLogURL);
			file.WriteLine("         \"MAJOR\":{0},", KSP_AVC_Info.HangarVersion.Major);
			file.WriteLine("         \"MINOR\":{0},", KSP_AVC_Info.HangarVersion.Minor);
			file.WriteLine("         \"PATCH\":{0},", KSP_AVC_Info.HangarVersion.Build);
			file.WriteLine("         \"BUILD\":{0}", KSP_AVC_Info.HangarVersion.Revision);
			file.WriteLine(
@"     },
    ""KSP_VERSION_MIN"":
     {");
			file.WriteLine("         \"MAJOR\":{0},", KSP_AVC_Info.MinKSPVersion.Major);
			file.WriteLine("         \"MINOR\":{0},", KSP_AVC_Info.MinKSPVersion.Minor);
			file.WriteLine("         \"PATCH\":{0}", KSP_AVC_Info.MinKSPVersion.Build);
			file.WriteLine(
@"     },
    ""KSP_VERSION_MAX"":
     {");
			file.WriteLine("         \"MAJOR\":{0},", KSP_AVC_Info.MaxKSPVersion.Major);
			file.WriteLine("         \"MINOR\":{0},", KSP_AVC_Info.MaxKSPVersion.Minor);
			file.WriteLine("         \"PATCH\":{0}", KSP_AVC_Info.MaxKSPVersion.Build);
			file.WriteLine(
@"     }
}");
			}
		}
	}
}
