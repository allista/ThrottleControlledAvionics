//   MacroProcessor.cs
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
	public class MacroProcessor : TCAModule
	{
//		public class Config : ModuleConfig
//		{
//			new public const string NODE_NAME = "MAC";
//
//		}
//		static Config MAC { get { return TCAScenario.Globals.MAC; } }

		public MacroProcessor(VesselWrapper vsl) { VSL = vsl; }

		protected override void UpdateState()
		{ IsActive = CFG.MacroIsActive && CFG.SelectedMacro != null; }

		protected override void Update()
		{
			if(!IsActive) return;
			CFG.MacroIsActive &= CFG.SelectedMacro.Execute(VSL);
			if(!CFG.MacroIsActive) CFG.SelectedMacro.Rewind();
		}
	}
}

