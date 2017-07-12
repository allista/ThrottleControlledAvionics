//   OrbitalTab.cs
//
//  Author:
//       Allis Tauri <allista@gmail.com>
//
//  Copyright (c) 2017 Allis Tauri

using System;
using UnityEngine;
using AT_Utils;

#if DEBUG
using System.Linq;
using System.Collections.Generic;
#endif

namespace ThrottleControlledAvionics
{
	public class OrbitalTab : ControlTab
	{
        public OrbitalTab(ModuleTCA tca) : base(tca) {}

		MatchVelocityAutopilot MVA;
		DeorbitAutopilot DEO;
		RendezvousAutopilot REN;
		ToOrbitAutopilot ORB;
		[InternalModule]
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
            if(ORB != null && ORB.ShowEditor)
                ORB.DrawOrbitEditor();
            if(REN != null && REN.ShowOptions)
            {
                REN.DrawOptions();
                REN.DrawBestTrajectories();
            }
			if(DEO != null && CFG.AP2[Autopilot2.Deorbit])
				DEO.DrawDeorbitSettings();
			#if DEBUG
			if(Utils.ButtonSwitch("DBG", ref TrajectoryCalculator.setp_by_step_computation, 
			                      "Toggles step-by-step trajectory computation", GUILayout.ExpandWidth(true)) &&
			   TrajectoryCalculator.setp_by_step_computation)
				MapView.EnterMapView();
			#endif
		}
	}
}