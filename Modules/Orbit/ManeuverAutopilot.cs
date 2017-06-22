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

			//controls best engine cluster calculation
			[Persistent] public float ClosestCluster   = 5f;   //s
			[Persistent] public float EfficientCluster = 0.1f; //fraction of vessel mass
			[Persistent] public float EfficiencyWeight = 10;   //how much the fuel mass will affect cluster selection
		}
		static Config MAN { get { return Globals.Instance.MAN; } }

        public enum Stage { WAITING, IN_PROGRESS, FINISHED }

		public ManeuverAutopilot(ModuleTCA tca) : base(tca) {}

		ThrottleControl THR;

		ManeuverNode Node;
		double InitialDeltaV;
		double ThresholdDeltaV;
		PatchedConicSolver Solver { get { return VSL.vessel.patchedConicSolver; } }

		public double NodeUT { get { return Node != null? Node.UT : -1; } }
		public Vector3d NodeDeltaV { get; private set; }
        public CelestialBody NodeCB { get; private set; }
        public Orbit TargetOrbit { get; private set; }
        public Stage ManeuverStage { get; private set; }

		ManeuverExecutor Executor;
		public float MinDeltaV = 1;
		bool within_threshold;
		double min_deltaV = double.MaxValue;

        public void AddCourseCorrection(Vector3d dV)
        { Executor.AddCourseCorrection(dV); }

        public void UpdateNode()
        {
            if(VSL.HasManeuverNode)
                update_maneuver_node();
        }

        void update_maneuver_node()
        {
            Node = Solver.maneuverNodes[0];
            InitialDeltaV = Node.DeltaV.magnitude;
            ThresholdDeltaV = Math.Min(InitialDeltaV, 10);
            NodeDeltaV = Node.GetBurnVector(VSL.orbit);
            NodeCB = Node.patch.referenceBody;
            TargetOrbit = Node.nextPatch;
            if(VSL.Engines.MaxDeltaV < InitialDeltaV)
                Status("yellow", "WARNING: there may be not enough propellant for the maneuver");
        }

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
			ControlsActive &= IsActive || TCAScenario.HavePatchedConics && VSL.Engines.HaveThrusters && VSL.HasManeuverNode;
		}

		public void ManeuverCallback(Multiplexer.Command cmd)
		{
			switch(cmd)
			{
			case Multiplexer.Command.Resume:
			case Multiplexer.Command.On:
                ManeuverStage = Stage.WAITING;
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
                update_maneuver_node();
				min_deltaV = double.MaxValue;
				within_threshold = false;
				THR.Throttle = 0;
				CFG.DisableVSC();
				break;

			case Multiplexer.Command.Off:
                VSL.Controls.StopWarp();
                if(!CFG.WarpToNode && TimeWarp.CurrentRateIndex > 0)
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
            NodeCB = null;
            MinDeltaV = GLB.THR.MinDeltaV;
			min_deltaV = double.MaxValue;
			within_threshold = false;
			VSL.Info.Countdown = 0;
			VSL.Info.TTB = 0;
			Working = false;
			Node = null;
		}

        public static Vector3d Orbital2NodeDeltaV(Orbit o, Vector3d orbitalDeltaV, double UT)
        {
            var norm = o.GetOrbitNormal().normalized;
            var prograde = o.getOrbitalVelocityAtUT(UT).normalized;
            var radial = Vector3d.Cross(prograde, norm).normalized;
            return new Vector3d(Vector3d.Dot(orbitalDeltaV, radial),
                                Vector3d.Dot(orbitalDeltaV, norm),
                                Vector3d.Dot(orbitalDeltaV, prograde));
        }

		public static void AddNode(VesselWrapper VSL, Vector3d dV, double UT)
		{
			var node = VSL.vessel.patchedConicSolver.AddManeuverNode(UT);
            node.DeltaV = Orbital2NodeDeltaV(node.patch, dV, UT);
			VSL.vessel.patchedConicSolver.UpdateFlightPlan();
//            VSL.Log("AddNode: {} : {}", UT, node.DeltaV);//debug
		}

        public static void AddNodeRaw(VesselWrapper VSL, Vector3d NodeV, double UT)
        {
            var node = VSL.vessel.patchedConicSolver.AddManeuverNode(UT);
            node.DeltaV = NodeV;
            VSL.vessel.patchedConicSolver.UpdateFlightPlan();
//            VSL.Log("AddNodeRaw: {} : {}", UT, node.DeltaV);//debug
        }

		bool StartCondition(float dV)
		{
			if(Working) return true;
			var ttb = VSL.Engines.TTB_Precise(dV);
//            Log("dV {}, TTB {}", dV, ttb);//debug
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
            //emergency dewarping
            if(!CFG.WarpToNode && TimeWarp.CurrentRate > 1 && VSL.Info.Countdown < GLB.WRP.DewarpTime)
                TimeWarp.SetRate(0, true);
			if(VSL.Info.Countdown > 0) return false;
//            Log("burn {}, countdown {}", burn, VSL.Info.Countdown);//debug
			VSL.Info.Countdown = 0;
            ManeuverStage = Stage.IN_PROGRESS;
			Working = true;
			return true;
		}

		protected override void Update()
		{
			if(!IsActive) return;
			if(!VSL.HasManeuverNode || 
               Node != Solver.maneuverNodes[0] ||
               NodeCB != Node.patch.referenceBody) 
            { 
                Message("Maneuver has been interrupted.");
                reset(); 
                return; 
            }
            if(!VSL.Engines.HaveThrusters && !VSL.Engines.HaveNextStageEngines)
            {
                Message("Out of fuel");
                reset();
                return;
            }
            //update the node
            NodeDeltaV = (TargetOrbit.GetFrameVelAtUT(NodeUT)-VSL.orbit.GetFrameVelAtUT(NodeUT)).xzy;
            if(Executor.Execute(NodeDeltaV, MinDeltaV, StartCondition)) 
			{
				within_threshold |= Executor.RemainingDeltaV < ThresholdDeltaV;
				if(within_threshold)
				{
					VSL.Controls.GimbalLimit = 0;
					var dV = Executor.RemainingDeltaV;
					if(dV < min_deltaV) { min_deltaV = dV; return; }
					if(dV-min_deltaV < MinDeltaV) return;
				}
				else return;
			}
            ManeuverStage = Stage.FINISHED;
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
