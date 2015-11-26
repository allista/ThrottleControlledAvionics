//  Author:
//       Allis Tauri <allista@gmail.com>
//
//  Copyright (c) 2015 Allis Tauri
//
// This work is licensed under the Creative Commons Attribution-ShareAlike 4.0 International License. 
// To view a copy of this license, visit http://creativecommons.org/licenses/by-sa/4.0/ 
// or send a letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.

using System;
using UnityEngine;

namespace ThrottleControlledAvionics
{
	public class ManeuverAutopilot : TCAModule
	{
		public ManeuverAutopilot(ModuleTCA tca) { TCA = tca; }

		protected ManeuverNode Node;
		protected PatchedConicSolver Solver { get { return VSL.vessel.patchedConicSolver; } }

		public override void Init()
		{
			base.Init();
			CFG.AP.AddCallback(Autopilot.Maneuver, Enable);
			if(CFG.AP[Autopilot.Maneuver]) Enable();
		}

		protected override void UpdateState()
		{ 
			IsActive = 
				CFG.AP[Autopilot.Maneuver] && 
				Node != null && VSL.MaxThrustM > 0 &&
				GameVariables.Instance
				.GetOrbitDisplayMode(ScenarioUpgradeableFacilities.GetFacilityLevel(SpaceCenterFacility.TrackingStation)) == GameVariables.OrbitDisplayMode.PatchedConics;
		}

		public override void Enable(bool enable = true)
		{
			if(enable)
			{
				if(!VSL.HasManeuverNode) 
				{ CFG.AP[Autopilot.Maneuver] = false; return; }
				CFG.AT.On(Attitude.ManeuverNode);
				Node = Solver.maneuverNodes[0];
				TCA.THR.Throttle = 0;
				CFG.DisableVSC();
			}
			else 
			{
				TimeWarp.SetRate(0, false);
				reset();
			}
		}

		void reset()
		{
			if(Working) TCA.THR.Throttle = 0;
			if(CFG.AT[Attitude.ManeuverNode])
				CFG.AT.On(Attitude.KillRotation);
			CFG.AP.OffIfOn(Autopilot.Maneuver);
			VSL.Countdown = 0;
			VSL.TTB = 0;
			Working = false;
			Node = null;
		}

		public float TTB(float dV, float thrust, float throttle)
		{
			return CheatOptions.InfiniteFuel?
				VSL.M*dV/thrust/throttle : 
				VSL.M*(Mathf.Exp(dV/thrust/throttle)-1)/VSL.MaxMassFlow/throttle;
		}

		protected override void Update()
		{
			if(!IsActive) return;
			if(!VSL.HasManeuverNode || Node != Solver.maneuverNodes[0])
			{ reset(); return; }
			var dV = Node.GetBurnVector(VSL.orbit);
			var dVrem = (float)dV.magnitude;
			//end if below the minimum dV
			if(dVrem < GLB.THR.MinDeltaV) 
			{ Node.RemoveSelf(); reset(); return; }
			//orient along the burning vector
			CFG.AT.OnIfNot(Attitude.ManeuverNode);
			//calculate remaining time to the full thrust burn
			if(!Working)
			{
				VSL.TTB = TTB(dVrem, VSL.MaxThrustM, TCA.THR.NextThrottle(dVrem, VSL.MaxThrustM, 1));
				var burn = Node.UT-VSL.TTB/2f;
				if(CFG.WarpToNode && TCA.ATC.Aligned)
					TCA.WRP.WarpToTime = burn-TCA.ATC.AttitudeError;
				VSL.Countdown = burn-VSL.UT;
				if(VSL.Countdown > 0) return;
				VSL.Countdown = 0;
				Working = true;
			}
			TCA.THR.DeltaV = dVrem;
			if(TCA.ATC.AttitudeError > GLB.ATC.AttitudeErrorThreshold)
				TCA.TRA.AddDeltaV(-VSL.LocalDir(dV));
		}
	}
}
