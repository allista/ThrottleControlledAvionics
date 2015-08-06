//   CruiseControl.cs
//
//  Author:
//       Allis Tauri <allista@gmail.com>
//
//  Copyright (c) 2015 Allis Tauri
//
// This work is licensed under the Creative Commons Attribution 4.0 International License. 
// To view a copy of this license, visit http://creativecommons.org/licenses/by/4.0/ 
// or send a letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.

using System;
using UnityEngine;

namespace ThrottleControlledAvionics
{
	public class CruiseControl : TCAModule
	{
		public class Config : ModuleConfig
		{
			new public const string NODE_NAME = "CC";

			[Persistent] public Vector3d Starboard;
		}
		static Config CC { get { return TCAConfiguration.Globals.CC; } }

		Vector3d starboard(Vector3d hV)
		{ return QuaternionD.FromToRotation(VSL.up, Vector3d.up)*Vector3d.Cross(hV, VSL.up); }
	}
}

