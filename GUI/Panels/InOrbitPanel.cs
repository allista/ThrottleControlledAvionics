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
	public class InOrbitPanel : ControlPanel
	{
		const int orb_width = 250;
		const int orb_height = 100;
		static Rect orbit_editor = new Rect((Screen.width-orb_width)/2, 
		                               		(Screen.height-orb_height)/2, 
		                                	orb_width, orb_width);
		readonly int orb_editor_ID;

		public InOrbitPanel(ModuleTCA tca) : base(tca) 
		{
			var rnd = new System.Random();
			orb_editor_ID = rnd.Next();
		}

		TimeWarpControl WRP;
		MatchVelocityAutopilot MVA;
		ManeuverAutopilot MAP;
		DeorbitAutopilot DEO;
		RendezvousAutopilot REN;
		ToOrbitAutopilot ORB;
		PointNavigator PN;

		public override void Draw()
		{
			GUILayout.BeginHorizontal();
			if(WRP != null) WRP.Draw();
			if(MAP != null) MAP.Draw();
			if(MVA != null) MVA.Draw();
			if(ORB != null) ORB.Draw();
			if(REN != null) REN.Draw();
			if(DEO != null) DEO.Draw();
			#if DEBUG
			if(Utils.ButtonSwitch("DBG", ref TrajectoryCalculator.setp_by_step_computation, 
			                      "Toggles step-by-step trajectory computation", GUILayout.ExpandWidth(false)) &&
			   TrajectoryCalculator.setp_by_step_computation)
				MapView.EnterMapView();
			#endif
			if(PN  != null) TCAGui.NavigationControls.AddSingleWaypointInMapView();
			GUILayout.Label(new GUIContent(VSL.Info.Countdown >= 0? 
			                               string.Format("{0:F1}s", VSL.Info.Countdown) : "", 
			                               "Countdown" ),
			                VSL.Info.Countdown > 10? Styles.white : Styles.red, 
			                GUILayout.ExpandWidth(true));
			GUILayout.Label(new GUIContent(VSL.Info.TTB >= 0? 
			                               string.Format("{0:F1}s", VSL.Info.TTB) : "",
			                               "Thrust Duration"), 
			                Styles.yellow, GUILayout.ExpandWidth(true));
			GUILayout.EndHorizontal();
		}

		void draw_orbit_editor(int windowID)
		{
			ORB.DrawOrbitEditor();
			AddonWindowBase.TooltipAndDrag(orbit_editor);
		}

		public void OrbitEditorWindow()
		{
			if(ORB == null || !ORB.ShowEditor) 
			{
				Utils.LockIfMouseOver("TCAOrbitEditor", orbit_editor, false);
				return;
			}
			Utils.LockIfMouseOver("TCAOrbitEditor", orbit_editor);
			orbit_editor = 
				GUILayout.Window(orb_editor_ID, 
				                 orbit_editor, 
				                 draw_orbit_editor, 
				                 "Target Orbit Editor",
				                 GUILayout.Width(orb_width),
				                 GUILayout.Height(orb_height)).clampToScreen();
		}
	}
}

