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

namespace ThrottleControlledAvionics
{
	public class InOrbitPanel : ControlPanel
	{
		public InOrbitPanel(ModuleTCA tca) : base(tca) {}

		TimeWarpControl WRP;
		MatchVelocityAutopilot MVA;
		ManeuverAutopilot MAP;
		DeorbitAutopilot DEO;

		public override void Draw()
		{
			if(!VSL.InOrbit || (MVA == null && MAP == null)) return;
			GUILayout.BeginHorizontal();
			var tVessel = VSL.TargetVessel;
			var MVA_aplicable = tVessel != null && tVessel.situation == Vessel.Situations.ORBITING && !CFG.AP[Autopilot.Maneuver];
			var DEO_aplicable = tVessel != null && tVessel.LandedOrSplashed || VSL.Target is WayPoint && !CFG.AP[Autopilot.Maneuver];
			if(WRP != null && (VSL.Info.Countdown > 0 || VSL.HasManeuverNode || VSL.HasTarget)) WRP.Draw();
			if(MAP != null && VSL.HasManeuverNode) MAP.Draw();
			if(DEO != null && DEO_aplicable) DEO.Draw();
			if(MVA != null && MVA_aplicable) MVA.Draw();
			if(VSL.Info.Countdown >= 0)
				GUILayout.Label(string.Format("Countdown: {0:F1}s", VSL.Info.Countdown), Styles.white, GUILayout.ExpandWidth(true));
			if(VSL.Info.TTB >= 0)
				GUILayout.Label(string.Format("Full Thrust: {0:F1}s", VSL.Info.TTB), Styles.yellow, GUILayout.ExpandWidth(true));
			GUILayout.EndHorizontal();
		}
	}
}

