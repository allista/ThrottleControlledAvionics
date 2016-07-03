//  Author:
//       Allis Tauri <allista@gmail.com>
//
//  Copyright (c) 2016 Allis Tauri
//
// This work is licensed under the Creative Commons Attribution-ShareAlike 4.0 International License. 
// To view a copy of this license, visit http://creativecommons.org/licenses/by-sa/4.0/ 
// or send a letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.
//
using System;
using UnityEngine;
using AT_Utils;

namespace ThrottleControlledAvionics
{
	public class TogglesPanel : ControlPanel
	{
		public TogglesPanel(ModuleTCA tca) : base(tca) {}

		VTOLControl VTOL;
		VTOLAssist VLA;
		FlightStabilizer STB;

		public override void Draw()
		{
			GUILayout.BeginHorizontal();
			if(VTOL != null) 
			{
				if(Utils.ButtonSwitch("VTOL Mode", CFG.CTRL[ControlMode.VTOL], 
				                      "Keyboard controls thrust direction instead of torque", GUILayout.ExpandWidth(true)))
					CFG.CTRL.XToggle(ControlMode.VTOL);
			}
			if(VLA != null) 
				Utils.ButtonSwitch("VTOL Assist", ref CFG.VTOLAssistON, 
				                   "Assist with vertical takeoff and landing", GUILayout.ExpandWidth(true));
			if(STB != null) 
				Utils.ButtonSwitch("Flight Stabilizer", ref CFG.StabilizeFlight, 
				                   "Try to stabilize flight if spinning uncontrollably", GUILayout.ExpandWidth(true));
			Utils.ButtonSwitch("AutoGear", ref CFG.AutoGear, 
			                   "Automatically deploy/retract landing gear when needed", GUILayout.ExpandWidth(true));
			Utils.ButtonSwitch("AutoBrakes", ref CFG.AutoBrakes, 
			                   "Automatically ebable/disable brakes when needed", GUILayout.ExpandWidth(true));
			Utils.ButtonSwitch("AutoStage", ref CFG.AutoStage, 
			                   "Automatically activate next stage when previous falmeouted", GUILayout.ExpandWidth(true));
			Utils.ButtonSwitch("AutoChute", ref CFG.AutoParachutes, 
			                   "Automatically activate parachutes when needed", GUILayout.ExpandWidth(true));
			GUILayout.EndHorizontal();
		}
	}
}

