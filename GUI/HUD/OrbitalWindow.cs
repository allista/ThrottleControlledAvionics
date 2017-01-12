//  Author:
//       Allis Tauri <allista@gmail.com>
//
//  Copyright (c) 2016 Allis Tauri
//
// This work is licensed under the Creative Commons Attribution-ShareAlike 4.0 International License. 
// To view a copy of this license, visit http://creativecommons.org/licenses/by-sa/4.0/ 
// or send a letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.

using UnityEngine;
using AT_Utils;

namespace ThrottleControlledAvionics
{
	public class OrbitalControlWindow : ControlWindow
	{
		class OrbitEditor : ControlDialog
		{
			public OrbitEditor()
			{
				width = 350;
				height = 100;
				Title = "Target Orbit Editor";
			}

			ToOrbitAutopilot ORB;

			protected override bool can_draw() 
			{ return ORB != null && ORB.ShowEditor; }

			protected override void MainWindow(int windowID)
			{
				ORB.DrawOrbitEditor();
				GUIWindowBase.TooltipsAndDragWindow();
			}
		}

		class DeorbitSettings : ControlDialog
		{
			public DeorbitSettings()
			{ Title = "Landing Settings"; }

			DeorbitAutopilot DEO;

			protected override bool can_draw() 
			{ return DEO != null && CFG.AP2[Autopilot2.Deorbit]; }

			protected override void MainWindow(int windowID)
			{ 
				DEO.DrawDeorbitSettings(); 
				GUIWindowBase.TooltipsAndDragWindow();
			}
		}

		class InfoWindow : ControlSubwindow
		{
			protected override bool can_draw()
			{ return VSL.Info.Countdown >= 0 || VSL.Info.TTB >= 0; }

			protected override void MainWindow(int windowID)
			{ 
				GUILayout.BeginVertical(Styles.white_on_black);
				VSL.Info.Draw();
				GUILayout.EndVertical();
				TooltipManager.GetTooltip();
			}
		}

		OrbitEditor orbit_editor;
		DeorbitSettings deorbit_settings;

		[SubwindowSpec(AnchorPosition.BottomLeft, 65, -1, yRelative = true)]
		InfoWindow info_window;

		TimeWarpControl WRP;
		MatchVelocityAutopilot MVA;
		ManeuverAutopilot MAN;
		DeorbitAutopilot DEO;
		RendezvousAutopilot REN;
		ToOrbitAutopilot ORB;
		PointNavigator PN;

		public OrbitalControlWindow() { Anchor = AnchorPosition.BottomLeft; }

		[ConfigOption] bool show_autopilots = true;

		static GUIContent show_button = new GUIContent("▶", "Show More");
		static GUIContent hide_button = new GUIContent("◀", "Hide Controls");

		protected override void DrawContent()
		{
			GUILayout.BeginHorizontal();
			if(WRP != null) WRP.Draw();
			if(MAN != null && VSL.HasManeuverNode) MAN.Draw();
			if(!show_autopilots)
			{
				if(GUILayout.Button(show_button, Styles.enabled_button, GUILayout.ExpandWidth(false)))
					show_autopilots = true;
			}
			else
			{
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
				if(PN != null) 
					TCAGui.Instance.NavigationControls.TargetUI();
				if(GUILayout.Button(hide_button, Styles.enabled_button, GUILayout.ExpandWidth(false)))
					show_autopilots = false;
			}
			GUILayout.EndHorizontal();
		}

		public override Rect Draw()
		{
			orbit_editor.Draw();
			deorbit_settings.Draw();
			return base.Draw();
		}
	}
}
