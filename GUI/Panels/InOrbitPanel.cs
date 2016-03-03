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
		PointNavigator PN;

		bool draw_MVA;
		bool draw_DEO;
		bool draw_WRP;
		bool draw_MAP;
		bool draw_PN;
		bool draw;

		public override void Update()
		{
			var tVessel = VSL.TargetVessel;
			draw_MVA = MVA != null && (CFG.AP1.Any(Autopilot1.MatchVel, Autopilot1.MatchVelNear) || (VSL.InOrbit && tVessel != null && tVessel.situation == Vessel.Situations.ORBITING) && !CFG.AP1[Autopilot1.Maneuver]);
			draw_DEO = DEO != null && (CFG.AP2[Autopilot2.Deorbit] || (VSL.InOrbit && tVessel != null && tVessel.LandedOrSplashed || VSL.Target is WayPoint) && !CFG.AP1[Autopilot1.Maneuver]);
			draw_MAP = MAP != null && VSL.HasManeuverNode;
			draw_WRP = WRP != null && (VSL.Info.Countdown > 0 || draw_MVA || draw_DEO || draw_MAP);
			draw_PN  = PN  != null && !VSL.OnPlanet && !CFG.AP1[Autopilot1.Maneuver] && !CFG.AP2[Autopilot2.Deorbit];
			draw = draw_MVA || draw_DEO || draw_MAP || draw_WRP || draw_PN;
		}

		public override void Draw()
		{
			if(!draw) return;
			GUILayout.BeginHorizontal();
			if(draw_WRP) WRP.Draw();
			if(draw_MAP) MAP.Draw();
			if(draw_PN) ThrottleControlledAvionics.NavigationControls.AddSingleWaypointInMapView();
			if(draw_DEO) DEO.Draw();
			if(draw_MVA) MVA.Draw();
			if(VSL.Info.Countdown >= 0)
				GUILayout.Label(string.Format("Countdown: {0:F1}s", VSL.Info.Countdown), Styles.white, GUILayout.ExpandWidth(true));
			if(VSL.Info.TTB >= 0)
				GUILayout.Label(string.Format("Full Thrust: {0:F1}s", VSL.Info.TTB), Styles.yellow, GUILayout.ExpandWidth(true));
			GUILayout.EndHorizontal();
		}
	}
}

