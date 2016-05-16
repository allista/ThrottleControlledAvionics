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
	[CareerPart]
	[RequireModules(typeof(AttitudeControl),
	                typeof(ThrottleControl),
	                typeof(TranslationControl),
	                typeof(TimeWarpControl))]
	public class ManeuverAutopilot : TCAModule
	{
		public class Config : TCAModule.ModuleConfig
		{
			[Persistent] public float WrapThreshold = 600f; //s
		}
		static Config MAN { get { return TCAScenario.Globals.MAN; } }

		public ManeuverAutopilot(ModuleTCA tca) : base(tca) {}

		ThrottleControl THR;
		TimeWarpControl WRP;
		AttitudeControl ATC;
		TranslationControl TRA;

		protected ManeuverNode Node;
		protected PatchedConicSolver Solver { get { return VSL.vessel.patchedConicSolver; } }
		protected Timer AlignedTimer = new Timer();
		FuzzyThreshold<float> dVrem = new FuzzyThreshold<float>(1, 0.5f);
		public float MinDeltaV = 1;
		public bool UntilMinimum;

		public override void Init()
		{
			base.Init();
			UntilMinimum = false;
			MinDeltaV = GLB.THR.MinDeltaV;
			CFG.AP1.AddHandler(this, Autopilot1.Maneuver);
		}

		protected override void UpdateState()
		{ 
			base.UpdateState();
			var HasPatchedConics = GameVariables.Instance
				.GetOrbitDisplayMode(ScenarioUpgradeableFacilities.GetFacilityLevel(SpaceCenterFacility.TrackingStation)) == GameVariables.OrbitDisplayMode.PatchedConics;
			IsActive = CFG.Enabled && CFG.AP1[Autopilot1.Maneuver] &&  Node != null && VSL.Engines.MaxThrustM > 0 && HasPatchedConics;
			ControlsActive = IsActive || VSL.HasManeuverNode && HasPatchedConics;
		}

		public void ManeuverCallback(Multiplexer.Command cmd)
		{
			switch(cmd)
			{
			case Multiplexer.Command.Resume:
			case Multiplexer.Command.On:
				if(!VSL.HasManeuverNode) 
				{ CFG.AP1.Off(); return; }
				CFG.AT.On(Attitude.ManeuverNode);
				Node = Solver.maneuverNodes[0];
				THR.Throttle = 0;
				CFG.DisableVSC();
				break;

			case Multiplexer.Command.Off:
				TimeWarp.SetRate(0, false);
				reset();
				break;
			}
		}

		protected override void reset()
		{
			if(Working) THR.Throttle = 0;
			if(CFG.AT[Attitude.ManeuverNode])
				CFG.AT.On(Attitude.KillRotation);
			CFG.AP1.OffIfOn(Autopilot1.Maneuver);
			AlignedTimer.Reset();
			UntilMinimum = false;
			MinDeltaV = GLB.THR.MinDeltaV;
			VSL.Info.Countdown = 0;
			VSL.Info.TTB = 0;
			Working = false;
			Node = null;
		}

		public static void AddNode(VesselWrapper VSL, Vector3d dV, double UT)
		{
			var node = VSL.vessel.patchedConicSolver.AddManeuverNode(UT);
			var norm = VSL.orbit.GetOrbitNormal().normalized;
			var prograde = VSL.orbit.getOrbitalVelocityAtUT(UT).normalized;
			var radial = Vector3d.Cross(prograde, norm).normalized;
			node.DeltaV = new Vector3d(Vector3d.Dot(dV, radial),
			                           Vector3d.Dot(dV, norm),
			                           Vector3d.Dot(dV, prograde));
			VSL.vessel.patchedConicSolver.UpdateFlightPlan();
		}

		public static float TTB(VesselWrapper VSL, float dV, float throttle = 1)
		{
			return CheatOptions.InfiniteFuel?
				VSL.Physics.M*dV/VSL.Engines.MaxThrustM/throttle : 
				VSL.Physics.M*(1-Mathf.Exp(-dV/VSL.Engines.MaxThrustM*VSL.Engines.MaxMassFlow))
				/(VSL.Engines.MaxMassFlow*throttle);
		}

		public float TTB(float dV, float throttle = 1) { return TTB(VSL, dV, throttle); }

		protected override void Update()
		{
			if(!IsActive) return;
			if(!VSL.HasManeuverNode || Node != Solver.maneuverNodes[0]) { reset(); return; }
			var dV = Node.GetBurnVector(VSL.orbit);
			var dVm = (float)dV.magnitude;
			//end if below the minimum dV
			if(dVm < MinDeltaV || UntilMinimum && dVm > dVrem.Value) { Node.RemoveSelf(); reset(); return; }
			dVrem.Value = dVm;
			VSL.ActivateNextStageOnFlameout();
			//orient along the burning vector
			if(dVrem && VSL.Controls.RCSAvailable) 
				CFG.AT.OnIfNot(Attitude.KillRotation);
			else CFG.AT.OnIfNot(Attitude.ManeuverNode);
			//calculate remaining time to the full thrust burn
			if(!Working)
			{
				var ttb = TTB(dVrem, THR.NextThrottle(dVrem, 1));
				if(float.IsNaN(ttb)) return;
				VSL.Info.TTB = ttb;
				var burn = Node.UT-VSL.Info.TTB/2f;
				if(CFG.WarpToNode && WRP.WarpToTime < 0) 
				{
					if((burn-VSL.Physics.UT)/dVrem > MAN.WrapThreshold) WRP.WarpToTime = burn-180;
					else AlignedTimer.RunIf(() => WRP.WarpToTime = burn-ATC.AttitudeError, ATC.Aligned);
				}
				VSL.Info.Countdown = burn-VSL.Physics.UT;
				if(TimeWarp.CurrentRate > 1 || VSL.Info.Countdown > 0) return;
//				Log("TTB {0}, burn {1}, countdown {2}", VSL.Info.TTB, burn, VSL.Info.Countdown);//debug
				VSL.Info.Countdown = 0;
				Working = true;
			}
			if(VSL.Controls.TranslationAvailable)
			{
			    if(dVrem || ATC.AttitudeError > GLB.ATCB.AttitudeErrorThreshold)
					TRA.AddDeltaV(-VSL.LocalDir(dV));
				if(dVrem && VSL.Controls.RCSAvailable) THR.Throttle = 0;
				else THR.DeltaV = dVrem;
			}
			else THR.DeltaV = dVrem;
//			LogF("\ndVrem: {}\nAttitudeError {}, DeltaV: {}, Throttle {}", dVrem, ATC.AttitudeError, THR.DeltaV, THR.Throttle);//debug
		}

		public override void Draw()
		{
			if(ControlsActive)
			{
				if(GUILayout.Button(CFG.AP1[Autopilot1.Maneuver]? "Abort Maneuver" : "Execute Node", 
				                    CFG.AP1[Autopilot1.Maneuver]? Styles.danger_button : Styles.active_button, 
				                    GUILayout.Width(110)))
					CFG.AP1.XToggle(Autopilot1.Maneuver);
			}
			else GUILayout.Label("Execute Node", Styles.inactive_button, GUILayout.Width(110));
		}
	}
}
