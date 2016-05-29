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

		ManeuverNode Node;
		PatchedConicSolver Solver { get { return VSL.vessel.patchedConicSolver; } }
		Timer AlignedTimer = new Timer();

		ManeuverExecutor Executor;
		public float MinDeltaV = 1;

		public override void Init()
		{
			base.Init();
			MinDeltaV = GLB.THR.MinDeltaV;
			CFG.AP1.AddHandler(this, Autopilot1.Maneuver);
			Executor = new ManeuverExecutor(TCA);
		}

		protected override void UpdateState()
		{ 
			base.UpdateState();
			var HasPatchedConics = GameVariables.Instance
				.GetOrbitDisplayMode(ScenarioUpgradeableFacilities.GetFacilityLevel(SpaceCenterFacility.TrackingStation)) == GameVariables.OrbitDisplayMode.PatchedConics;
			IsActive = CFG.Enabled && CFG.AP1[Autopilot1.Maneuver] && Node != null && HasPatchedConics;
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
				if(VSL.Engines.MaxDeltaV < (float)Node.DeltaV.magnitude)
					Status("yellow", "WARNING: there may be not enough propellant for the maneuver");
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
			Executor.Reset();
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

		bool StartCondition(float dV)
		{
			if(Working) return true;
			var ttb = VSL.Engines.TTB(dV, THR.NextThrottle(dV, 1));
			if(float.IsNaN(ttb)) return false;
			VSL.Info.TTB = ttb;
			var burn = Node.UT-VSL.Info.TTB/2f;
			if(CFG.WarpToNode && WRP.WarpToTime < 0) 
			{
				if((burn-VSL.Physics.UT)/dV > MAN.WrapThreshold) WRP.WarpToTime = burn-180;
				else AlignedTimer.RunIf(() => WRP.WarpToTime = burn-ATC.AttitudeError, ATC.Aligned);
			}
			VSL.Info.Countdown = burn-VSL.Physics.UT;
			if(TimeWarp.CurrentRate > 1 || VSL.Info.Countdown > 0) return false;
//			Log("TTB {0}, burn {1}, countdown {2}", VSL.Info.TTB, burn, VSL.Info.Countdown);//debug
			VSL.Info.Countdown = 0;
			Working = true;
			return true;
		}

		protected override void Update()
		{
			if(!IsActive) return;
			if(!VSL.HasManeuverNode || Node != Solver.maneuverNodes[0]) { reset(); return; }
			if(Executor.Execute(Node.GetBurnVector(VSL.orbit), MinDeltaV, StartCondition)) return;
			Node.RemoveSelf();
			reset();
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


	public class ManeuverExecutor : TCAComponent
	{
		ThrottleControl THR;
		TranslationControl TRA;
		AttitudeControl ATC;

		public delegate bool ManeuverCondition(float dV);
		readonly FuzzyThreshold<double> dVrem = new FuzzyThreshold<double>(1, 0.5f);

		public ManeuverExecutor(ModuleTCA tca) : base(tca) { InitModuleFields(); }

		public void Reset() { dVrem.Value = dVrem.Upper+1; }

		public bool Execute(Vector3d dV, float MinDeltaV = 0.1f, ManeuverCondition condition = null)
		{
			dVrem.Value = dV.magnitude;
			//end if below the minimum dV
			if(dVrem < MinDeltaV) return false;
			THR.Throttle = 0;
			VSL.Engines.ActivateEnginesIfNeeded();
			VSL.Engines.ActivateNextStageOnFlameout();
			//orient along the burning vector
			if(dVrem && VSL.Controls.RCSAvailableInDirection(-dV)) 
				CFG.AT.OnIfNot(Attitude.KillRotation);
			else 
			{
				CFG.AT.OnIfNot(Attitude.Custom);
				ATC.SetThrustDirW(-dV);
			}
			//check the condition
			if(condition != null && !condition((float)dVrem)) return true;
			if(VSL.Controls.TranslationAvailable)
			{
				if(dVrem || ATC.AttitudeError > GLB.ATCB.AttitudeErrorThreshold)
					TRA.AddDeltaV(-VSL.LocalDir(dV));
				if(dVrem && CFG.AT[Attitude.KillRotation]) 
				{
					var errorF = Utils.ClampL(Vector3.Dot(VSL.Engines.Thrust.normalized, -dV.normalized), 0);
					THR.DeltaV = (float)dVrem * errorF*errorF;
				}
				else THR.DeltaV = (float)dVrem;
			}
			else THR.DeltaV = (float)dVrem;
//			LogF("\ndVrem: {}\nAttitudeError {}, DeltaV: {}, Throttle {}, RCS {}", 
//			     dVrem, ATC.AttitudeError, THR.DeltaV, THR.Throttle, may_use_RCS);//debug
			return true;
		}
	}
}
