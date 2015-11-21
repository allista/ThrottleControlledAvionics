//   ManeuverAutopilot.cs
//
//  Author:
//       Allis Tauri <allista@gmail.com>
//
//  Copyright (c) 2015 Allis Tauri
//
// This work is licensed under the Creative Commons Attribution 4.0 International License. 
// To view a copy of this license, visit http://creativecommons.org/licenses/by/4.0/ 
// or send a letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.

using System;
using UnityEngine;

namespace ThrottleControlledAvionics
{
	public class ManeuverAutopilot : TCAModule
	{
		public class Config : ModuleConfig
		{
			new public const string NODE_NAME = "MAN";

			[Persistent] public float DewarpTime = 20f;  //sec
			[Persistent] public float MaxWarp    = 10000f;
		}
		static Config MAN { get { return TCAScenario.Globals.MAN; } }

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
			var dVrem = (float)Node.GetBurnVector(VSL.vessel.orbit).magnitude;
			//end if below the minimum dV
			if(dVrem < GLB.THR.MinDeltaV) 
			{ Node.RemoveSelf(); reset(); return; }
			//orient along the burning vector
			CFG.AT.OnIfNot(Attitude.ManeuverNode);
			//calculate remaining time to the full thrust burn
			if(!Working)
			{
				VSL.TTB = TTB(dVrem, VSL.MaxThrustM, TCA.THR.NextThrottle(dVrem, VSL.MaxThrustM, 1));
				VSL.Countdown = Node.UT-VSL.TTB/2f-VSL.UT;
				var DewarpTime = VSL.Countdown-(MAN.DewarpTime+(1+TimeWarp.deltaTime)*(TimeWarp.CurrentRateIndex+1))-TCA.ATC.AttitudeError;
				if(TimeWarp.CurrentRateIndex > 0 && DewarpTime < 0)
					TimeWarp.SetRate(TimeWarp.CurrentRateIndex-1, false);
				else if(CFG.WarpToNode && TCA.ATC.Aligned && 
				        DewarpTime > TimeWarp.CurrentRate && 
				        TimeWarp.CurrentRateIndex < TimeWarp.fetch.warpRates.Length-1 && 
				        TimeWarp.fetch.warpRates[TimeWarp.CurrentRateIndex+1] <= MAN.MaxWarp &&
				        VSL.AbsAltitude > TimeWarp.fetch.GetAltitudeLimit(TimeWarp.CurrentRateIndex+1, VSL.mainBody))
					TimeWarp.SetRate(TimeWarp.CurrentRateIndex+1, false);
				if(VSL.Countdown > 0) return;
				VSL.Countdown = 0;
				Working = true;
			}
			TCA.THR.DeltaV = dVrem;
		}
	}
}
