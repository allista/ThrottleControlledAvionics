//   OrbitalTab.cs
//
//  Author:
//       Allis Tauri <allista@gmail.com>
//
//  Copyright (c) 2017 Allis Tauri

using UnityEngine;
using AT_Utils;


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

        public override void OnRenderObject()
        {
            if(DEO != null)
                DEO.DrawTrajectory();
        }

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
            if(ORB != null && ORB.ShowOptions)
                ORB.DrawOptions();
            if(REN != null && REN.ShowOptions)
            {
                REN.DrawOptions();
                REN.DrawBestTrajectories();
            }
            if(DEO != null && DEO.ShowOptions)
				DEO.DrawOptions();
			#if DEBUG
			if(Utils.ButtonSwitch("DBG", ref TrajectoryCalculator.setp_by_step_computation, 
			                      "Toggles step-by-step trajectory computation", GUILayout.ExpandWidth(true)) &&
			   TrajectoryCalculator.setp_by_step_computation)
				MapView.EnterMapView();
			#endif
		}
	}
}