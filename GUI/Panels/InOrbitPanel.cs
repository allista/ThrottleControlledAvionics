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
		RendezvouAutopilot REN;
		PointNavigator PN;

		public override void Draw()
		{
			GUILayout.BeginHorizontal();
			if(WRP != null) WRP.Draw();
			if(MAP != null) MAP.Draw();
			if(MVA != null) MVA.Draw();
			if(REN != null) REN.Draw();
			if(DEO != null) DEO.Draw();
			if(PN  != null) TCAGui.NavigationControls.AddSingleWaypointInMapView();
			GUILayout.Label(new GUIContent(VSL.Info.Countdown >= 0? 
			                               string.Format("-{0:F1}s", VSL.Info.Countdown) : "", 
			                               "Countdown" ),
			                VSL.Info.Countdown > 10? Styles.white : Styles.red, 
			                GUILayout.ExpandWidth(true));
			GUILayout.Label(new GUIContent(VSL.Info.TTB >= 0? 
			                               string.Format("{0:F1}s", VSL.Info.TTB) : "",
			                               "Full Thrust Duration"), 
			                Styles.yellow, GUILayout.ExpandWidth(true));
			GUILayout.EndHorizontal();
		}
	}
}

