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

		public override void Draw()
		{
			if(!VSL.InOrbit || (MVA == null && MAP == null)) return;
			GUILayout.BeginHorizontal();
			if(WRP != null && VSL.Info.Countdown >= 0 && Utils.ButtonSwitch("Warp", CFG.WarpToNode, "Warp to the burn", GUILayout.ExpandWidth(false)))
			{
				CFG.WarpToNode = !CFG.WarpToNode;
				if(!CFG.WarpToNode) TimeWarp.SetRate(0, false);
			}
			if(MAP != null && VSL.HasManeuverNode) 
			{
				if(GUILayout.Button(CFG.AP[Autopilot.Maneuver]? "Abort Maneuver" : "Execute Maneuver", 
				                    CFG.AP[Autopilot.Maneuver]? Styles.red_button : Styles.green_button, GUILayout.ExpandWidth(true)))
					CFG.AP.XToggle(Autopilot.Maneuver);
			}
			if(MVA != null && VSL.HasTarget && !CFG.AP[Autopilot.Maneuver])
			{
				if(Utils.ButtonSwitch("Match Velocity", CFG.AP[Autopilot.MatchVel], 
				                      "Match orbital velocity with the target", GUILayout.ExpandWidth(true)))
					CFG.AP.XToggle(Autopilot.MatchVel);
				if(Utils.ButtonSwitch("Brake Near Target", CFG.AP[Autopilot.MatchVelNear], 
				                      "Match orbital velocity with the target at nearest point", GUILayout.ExpandWidth(true)))
					CFG.AP.XToggle(Autopilot.MatchVelNear);
			}
			if(VSL.Info.Countdown >= 0)
				GUILayout.Label(string.Format("Countdown: {0:F1}s", VSL.Info.Countdown), Styles.white, GUILayout.ExpandWidth(true));
			if(VSL.Info.TTB >= 0)
				GUILayout.Label(string.Format("Full Thrust: {0:F1}s", VSL.Info.TTB), Styles.yellow, GUILayout.ExpandWidth(true));
			GUILayout.EndHorizontal();
		}
	}
}

