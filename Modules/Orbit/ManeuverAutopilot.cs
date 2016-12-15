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
using AT_Utils;

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
		static Config MAN { get { return Globals.Instance.MAN; } }

		public ManeuverAutopilot(ModuleTCA tca) : base(tca) {}

		ThrottleControl THR;

		ManeuverNode Node;
		double InitialDeltaV;
		double ThresholdDeltaV;
		PatchedConicSolver Solver { get { return VSL.vessel.patchedConicSolver; } }

		public double NodeUT { get { return Node != null? Node.UT : -1; } }
		public Vector3d NodeDeltaV { get; private set; }

		ManeuverExecutor Executor;
		public float MinDeltaV = 1;
		bool within_threshold;
		double min_deltaV = double.MaxValue;

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
			IsActive &= Node != null;
			ControlsActive &= IsActive || TCAScenario.HavePatchedConics && VSL.HasManeuverNode;
		}

		public void ManeuverCallback(Multiplexer.Command cmd)
		{
			switch(cmd)
			{
			case Multiplexer.Command.Resume:
			case Multiplexer.Command.On:
				if(!TCAScenario.HavePatchedConics)
				{
					Status("yellow", "WARNING: maneuver nodes are not yet available. Upgrade the Tracking Station.");
					CFG.AP1.Off(); 
					return;
				}
				if(!VSL.HasManeuverNode) 
				{ CFG.AP1.Off(); return; }
				VSL.Controls.StopWarp();
				CFG.AT.On(Attitude.ManeuverNode);
				Node = Solver.maneuverNodes[0];
				InitialDeltaV = Node.DeltaV.magnitude;
				ThresholdDeltaV = Math.Min(InitialDeltaV, 10);
				NodeDeltaV = Node.GetBurnVector(VSL.orbit);
				min_deltaV = double.MaxValue;
				within_threshold = false;
				if(VSL.Engines.MaxDeltaV < InitialDeltaV)
					Status("yellow", "WARNING: there may be not enough propellant for the maneuver");
				THR.Throttle = 0;
				CFG.DisableVSC();
				break;

			case Multiplexer.Command.Off:
				TimeWarp.SetRate(0, false);
				CFG.AT.On(Attitude.KillRotation);
				reset();
				break;
			}
		}

		protected override void reset()
		{
			base.reset();
			if(Working) THR.Throttle = 0;
			if(CFG.AT[Attitude.ManeuverNode])
				CFG.AT.On(Attitude.KillRotation);
			CFG.AP1.OffIfOn(Autopilot1.Maneuver);
			Executor.Reset();
			NodeDeltaV = Vector3d.zero;
			MinDeltaV = GLB.THR.MinDeltaV;
			min_deltaV = double.MaxValue;
			within_threshold = false;
			VSL.Info.Countdown = 0;
			VSL.Info.TTB = 0;
			Working = false;
			Node = null;
		}

		public static void AddNode(VesselWrapper VSL, Vector3d dV, double UT)
		{
			var node = VSL.vessel.patchedConicSolver.AddManeuverNode(UT);
			var norm = node.patch.GetOrbitNormal().normalized;
			var prograde = node.patch.getOrbitalVelocityAtUT(UT).normalized;
			var radial = Vector3d.Cross(prograde, norm).normalized;
			node.DeltaV = new Vector3d(Vector3d.Dot(dV, radial),
			                           Vector3d.Dot(dV, norm),
			                           Vector3d.Dot(dV, prograde));
			VSL.vessel.patchedConicSolver.UpdateFlightPlan();
		}

		bool StartCondition(float dV)
		{
			if(Working) return true;
			var ttb = VSL.Engines.TTB(dV);
			if(float.IsNaN(ttb)) return false;
			VSL.Info.TTB = ttb;
			var burn = Node.UT-VSL.Info.TTB/2f;
			if(CFG.WarpToNode && VSL.Controls.WarpToTime < 0) 
			{
				if((burn-VSL.Physics.UT)/dV > MAN.WrapThreshold ||
				   TCAScenario.HavePersistentRotation && burn-VSL.Physics.UT > 180+GLB.WRP.DewarpTime)
					VSL.Controls.WarpToTime = burn-180;
				else if(VSL.Controls.CanWarp) 
					VSL.Controls.WarpToTime = burn-VSL.Controls.MinAlignmentTime;
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
			NodeDeltaV = Node.GetBurnVector(VSL.orbit);
			if(Executor.Execute(NodeDeltaV, MinDeltaV, StartCondition)) 
			{
				within_threshold |= Executor.RemainingDeltaV < ThresholdDeltaV;
				if(within_threshold)
				{
					var dV = Executor.RemainingDeltaV;
					if(dV < min_deltaV) { min_deltaV = dV; return; }
					else if(dV-min_deltaV < GLB.THR.MinDeltaV) return;
				}
				else return;
			}
			Node.RemoveSelf();
			reset();
		}

		public override void Draw()
		{
			if(ControlsActive)
			{
				if(GUILayout.Button(CFG.AP1[Autopilot1.Maneuver]? "Abort Maneuver" : "Execute Node", 
				                    CFG.AP1[Autopilot1.Maneuver]? Styles.danger_button : Styles.active_button, 
				                    GUILayout.ExpandWidth(false)))
					CFG.AP1.XToggle(Autopilot1.Maneuver);
			}
			else GUILayout.Label("Execute Node", Styles.inactive_button, GUILayout.ExpandWidth(false));
		}
	}
}
