//   OrbitalTab.cs
//
//  Author:
//       Allis Tauri <allista@gmail.com>
//
//  Copyright (c) 2017 Allis Tauri

using System;
using UnityEngine;
using AT_Utils;

namespace ThrottleControlledAvionics
{
	public class OrbitalTab : ControlTab
	{
		const int orb_width = 350;
		const int orb_height = 100;
		static Rect orbit_editor = new Rect((Screen.width-orb_width)/2, 
		                                    (Screen.height-orb_height)/2, 
		                                    orb_width, orb_height);
		readonly int orb_editor_ID;

		public OrbitalTab(ModuleTCA tca) : base(tca) 
		{
			var rnd = new System.Random();
			orb_editor_ID = rnd.Next();
		}

		MatchVelocityAutopilot MVA;
		DeorbitAutopilot DEO;
		RendezvousAutopilot REN;
		ToOrbitAutopilot ORB;
		PointNavigator PN;

		public override void Draw()
		{
			GUILayout.BeginHorizontal();
			if(MVA != null) MVA.Draw();
			GUILayout.EndHorizontal();
			if(PN  != null && UI.NAV != null) 
				UI.NAV.TargetUI();
			GUILayout.BeginHorizontal();
			if(ORB != null) ORB.Draw();
			if(REN != null) REN.Draw();
			if(DEO != null) DEO.Draw();
			GUILayout.EndHorizontal();
			if(DEO != null && CFG.AP2[Autopilot2.Deorbit])
				DEO.DrawDeorbitSettings();
			#if DEBUG
			if(Utils.ButtonSwitch("DBG", ref TrajectoryCalculator.setp_by_step_computation, 
			                      "Toggles step-by-step trajectory computation", GUILayout.ExpandWidth(true)) &&
			   TrajectoryCalculator.setp_by_step_computation)
				MapView.EnterMapView();
			#endif
		}

		void draw_orbit_editor(int windowID)
		{
			ORB.DrawOrbitEditor();
			GUIWindowBase.TooltipsAndDragWindow();
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