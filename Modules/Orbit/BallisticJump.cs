//  Author:
//       Allis Tauri <allista@gmail.com>
//
//  Copyright (c) 2016 Allis Tauri
//
// This work is licensed under the Creative Commons Attribution-ShareAlike 4.0 International License. 
// To view a copy of this license, visit http://creativecommons.org/licenses/by-sa/4.0/ 
// or send a letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.
//
using System;
using System.Collections.Generic;
using UnityEngine;
using AT_Utils;

namespace ThrottleControlledAvionics
{
	[CareerPart]
	[RequireModules(typeof(AttitudeControl),
	                typeof(BearingControl), 
	                typeof(ThrottleControl), 
	                typeof(ManeuverAutopilot), 
	                typeof(AutoLander))]
	public class BallisticJump : LandingTrajectoryAutopilot
	{
		public new class Config : TCAModule.ModuleConfig
		{
			[Persistent] public float StartOffset      = 30f;  //s
			[Persistent] public float StartInclination = 30f;  //deg
			[Persistent] public float StartAltitude    = 100f; //m
			[Persistent] public float InclinationF     = 2f;
			[Persistent] public float ObstacleOffset   = 50f;

            [Persistent] public PIDf_Controller FallCorrectionPID = new PIDf_Controller(1, 0.5f, 0, 0, float.PositiveInfinity);

			public float StartTangent = 1;

			public override void Init()
			{
				StartTangent = 1/Mathf.Tan(Utils.Clamp(StartInclination, 10, 80)*Mathf.Deg2Rad);
			}
		}
		static Config BJ { get { return Globals.Instance.BJ; } }

		public BallisticJump(ModuleTCA tca) : base(tca) {}

		public enum Stage { None, Start, GainAltitude, Compute, Accelerate, CorrectAltitude, CorrectTrajectory, Coast, Wait }
		[Persistent] public Stage stage;
		float StartAltitude;

        PIDf_Controller fall_correction = new PIDf_Controller(1, 0.5f, 0, 0, float.PositiveInfinity);

		public override void Init()
		{
			base.Init();
            fall_correction.setPID(BJ.FallCorrectionPID);
			CFG.AP2.AddHandler(this, Autopilot2.BallisticJump);
		}

		protected override void reset()
		{
			base.reset();
			stage = Stage.None;
			StartAltitude = BJ.StartAltitude;
			CFG.AP1.Off();
		}

		protected override bool check_target()
		{
			if(!base.check_target()) return false;
			if(VesselOrbit.PeR > Body.Radius)
			{
				Status("yellow", "Cannot perform <b>Ballistic Jump</b> from orbit.\n" +
				       "Use <b>Land at Target</b> instead.");
				return false;
			}
			//compute initial orbit estimation using LambertSolver
			var solver = new LambertSolver(VesselOrbit, CFG.Target.RelOrbPos(Body), VSL.Physics.UT);
			var dV = solver.dV4TransferME();
			if(Vector3d.Dot(VesselOrbit.vel+dV, VesselOrbit.pos) < 0)
				dV = -2*VesselOrbit.vel-dV;
			var trj = new LandingTrajectory(VSL, dV, VSL.Physics.UT, CFG.Target, TargetAltitude, false);
			if(trj.TransferTime < ManeuverOffset)
			{
				Status("yellow", "The target is too close for the jump.\n" +
				       "Use <b>Go To</b> instead.");
				return false;
			}
			return true;
		}

		public void BallisticJumpCallback(Multiplexer.Command cmd)
		{
			switch(cmd)
			{
			case Multiplexer.Command.Resume:
				if(!check_patched_conics()) return;
				UseTarget();
				NeedCPSWhenMooving();
				if(VSL.HasManeuverNode) 
					CFG.AP1.OnIfNot(Autopilot1.Maneuver);
				if(stage == Stage.None && !landing) 
					stage = Stage.Start;
				break;

			case Multiplexer.Command.On:
				reset();
				if(setup())
				{
					SaveGame("before_jump");
					goto case Multiplexer.Command.Resume;
				}
				CFG.AP2.Off();
				return;

			case Multiplexer.Command.Off:
                ReleaseCPS();
				SetTarget();
				reset();
				break;
			}
		}

        class LandingSiteOptimizer : LandingSiteOptimizerBase
        {
            readonly BallisticJump m;
            readonly Vector3d direction;
            readonly double velocity;

            public LandingSiteOptimizer(BallisticJump module, double velocity, Vector3d direction, float dtol) : base(dtol)
            { 
                m = module;
                this.direction = direction;
                this.velocity = velocity;
            }

            public override IEnumerator<LandingTrajectory> GetEnumerator()
            {
                var targetAlt = m.TargetAltitude;
                var dir = direction;
                var V = velocity;
                LandingTrajectory prev = null, cur = null;
                while(continue_calculation(prev, cur))
                {       
                    prev = cur;
                    var startUT = m.VSL.Physics.UT+BJ.StartOffset;
                    cur = new LandingTrajectory(m.VSL, 
                                                dir*V-m.VesselOrbit.getOrbitalVelocityAtUT(startUT), 
                                                startUT, m.CFG.Target, targetAlt);
                    if(Best == null || cur.Quality < Best.Quality) Best = cur;
                    targetAlt = cur.TargetAltitude;
                    V += cur.DeltaR*(1-m.CFG.Target.AngleTo(m.VSL)/Math.PI*0.9)*m.Body.GeeASL;
                    dir = Quaternion.AngleAxis((float)cur.DeltaFi, m.VSL.Physics.Up.xzy) * dir;
                    yield return cur;
                }
            }
        }

        class LandingSiteCorrector : LandingSiteOptimizerBase
        {
            readonly BallisticJump m;
            readonly double velocity;

            public LandingSiteCorrector(BallisticJump module, double velocity, float dtol) : base(dtol)
            { 
                m = module;
                this.velocity = velocity;
            }

            public override IEnumerator<LandingTrajectory> GetEnumerator()
            {
                var V = velocity;
                var angle = 0.0;
                var start_offset = Math.Max(m.CorrectionOffset, m.VSL.Torque.NoEngines.TurnTime);
                var targetAlt = m.TargetAltitude;
                LandingTrajectory prev = null, cur = null;
                while(continue_calculation(prev, cur))
                {       
                    prev = cur;
                    var startUT = m.VSL.Physics.UT+start_offset;
                    var vel = m.VesselOrbit.getOrbitalVelocityAtUT(startUT);
                    cur = new LandingTrajectory(m.VSL, 
                                                QuaternionD.AngleAxis(angle, m.VSL.Physics.Up.xzy)*vel.normalized*V - vel, 
                                                startUT, m.CFG.Target, targetAlt);
                    if(Best == null || cur.Quality < Best.Quality) Best = cur;
                    targetAlt = cur.TargetAltitude;
                    V += cur.DeltaR*(1-m.CFG.Target.AngleTo(m.VSL)/Math.PI*0.9)*m.Body.GeeASL;
                    angle += cur.DeltaFi;
                    yield return cur;
                }
            }
        }

		void compute_initial_trajectory()
		{
			MAN.MinDeltaV = 1f;
			var tPos = CFG.Target.RelOrbPos(Body);
			var solver = new LambertSolver(VesselOrbit, tPos+tPos.normalized*LTRJ.FlyOverAlt, VSL.Physics.UT);
			var vel = (VesselOrbit.vel+solver.dV4TransferME());
			if(Vector3d.Dot(vel, VesselOrbit.pos) < 0) vel = -vel;
			var dir = vel.normalized;
			var V = vel.magnitude;
            ComputeTrajectory(new LandingSiteOptimizer(this, V, dir, LTRJ.Dtol));
            stage = Stage.Compute;
            trajectory = null;
		}

		void accelerate()
		{
			CFG.HF.Off();
			clear_nodes(); add_trajectory_node();
			CFG.AT.OnIfNot(Attitude.ManeuverNode);
            fall_correction.Reset();
			stage = Stage.Accelerate;
		}

		protected override void fine_tune_approach()
		{
			double V = VesselOrbit.getOrbitalVelocityAtUT(VSL.Physics.UT+CorrectionOffset).magnitude;
            ComputeTrajectory(new LandingSiteCorrector(this, V, LTRJ.Dtol/2));
            stage = Stage.CorrectTrajectory;
            trajectory = null;
		}

        void VSC_ON()
        {
            CFG.HF.OnIfNot(HFlight.Stop);
            CFG.BlockThrottle = true;
            CFG.AltitudeAboveTerrain = true;
            CFG.VF.OnIfNot(VFlight.AltitudeControl);
        }

		protected override void UpdateState()
		{
			base.UpdateState();
			IsActive &= CFG.AP2[Autopilot2.BallisticJump];
			ControlsActive &= IsActive || (VSL.TargetIsNavPoint || VSL.TargetIsWayPoint || VSL.TargetVessel != null && VSL.TargetVessel.LandedOrSplashed);
		}

		protected override void Update()
		{
			if(!IsActive) { CFG.AP2.OffIfOn(Autopilot2.BallisticJump); return; }
			if(landing) { do_land(); return; }
			switch(stage)
			{
			case Stage.Start:
                VSC_ON();
				if(VSL.LandedOrSplashed || VSL.Altitude.Relative < StartAltitude) 
                    CFG.DesiredAltitude = StartAltitude;
                else CFG.DesiredAltitude = VSL.Altitude.Relative;
                if(!VSL.LandedOrSplashed)
                    compute_initial_trajectory();
				break;
            case Stage.GainAltitude:
                Status("Gaining altitude...");
                VSC_ON();
                if(VSL.Altitude.Relative > CFG.DesiredAltitude-10)
                    compute_initial_trajectory();
                break;
			case Stage.Compute:
				if(!trajectory_computed()) break;
                var obst = obstacle_ahead(trajectory, BJ.StartAltitude);
                if(obst > 0)
				{
                    StartAltitude += (float)obst+BJ.ObstacleOffset;
                    CFG.DesiredAltitude = StartAltitude;
                    stage = Stage.GainAltitude;
				}
                else if(VSL.Altitude.Relative < BJ.StartAltitude-10)
                    stage = Stage.GainAltitude;
				else if(check_initial_trajectory()) accelerate();
				else stage = Stage.Wait;
				break;
			case Stage.Wait:
				if(!string.IsNullOrEmpty(TCAGui.StatusMessage)) break;
				accelerate();
				break;
			case Stage.Accelerate:
                if(!VSL.HasManeuverNode) { CFG.AP2.Off(); break; }
				if(VSL.Controls.AlignmentFactor > 0.9 || !CFG.VSCIsActive)
				{
					CFG.DisableVSC();
                    var dV = VSL.FirstManeuverNode.GetBurnVector(VesselOrbit);
//                        (trajectory.Orbit.GetFrameVelAtUT(VSL.Physics.UT)-
//                              VSL.orbit.GetFrameVelAtUT(VSL.Physics.UT)).xzy;
                    if(VSL.VerticalSpeed.Absolute < 0)
                        fall_correction.Update(-VSL.VerticalSpeed.Absolute);
                    else
                    {
                        fall_correction.IntegralError *= 1-fall_correction.I/10*TimeWarp.fixedDeltaTime;
                        fall_correction.Update(0);
                    }
                    if(!Executor.Execute(dV+fall_correction*VSL.Physics.Up, 10)) 
                        fine_tune_approach();
					Status("white", "Accelerating...");
				}
				break;
			case Stage.CorrectTrajectory:
                warp_to_coundown();
                if(VesselOrbit.ApAhead())
                {
    				if(!trajectory_computed()) break;
    				add_correction_node_if_needed();
    				stage = Stage.Coast;
                }
                else
                {
                    stage = Stage.None;
                    start_landing();
                }
				break;
			case Stage.Coast:
				var ap_ahead = VesselOrbit.ApAhead();
				if(CFG.AP1[Autopilot1.Maneuver]) 
				{ 
					if(ap_ahead)
					{
						Status("Correcting trajectory..."); 
						break; 
					}
				}
				Status("Coasting...");
				if(ap_ahead && !correct_trajectory()) break;
				stage = Stage.None;
				start_landing();
				break;
			}
		}

		public override void Draw()
		{
			#if DEBUG
			DrawDebugLines();
//			if(current != null)//debug
//			{
//				Utils.GLVec(Body.position, current.AtTargetPos.xzy, Color.green);
//				Utils.GLLine(Body.position, current.SurfacePoint.WorldPos(Body), Color.yellow);
//				var brake = current.NewOrbit.getPositionAtUT(current.BrakeStartUT);
//				Utils.GLVec(brake, current.BrakeDeltaV.xzy.normalized*2000, Color.red);
//				Utils.GLVec(brake, (current.NewOrbit.getOrbitalVelocityAtUT(current.BrakeNodeUT)+
//				                      current.BrakeDeltaV).xzy.normalized*2000, 
//				              Color.magenta);
//			}
			#endif
			if(ControlsActive) 
			{	
				if(computing) 
					GUILayout.Label(new GUIContent("Jump To", "Computing maneuver. Push to cancel."), 
					                Styles.inactive_button, GUILayout.ExpandWidth(true));
				else if(Utils.ButtonSwitch("Jump To", CFG.AP2[Autopilot2.BallisticJump],
				                           "Fly to the target using ballistic trajectory.", 
				                           GUILayout.ExpandWidth(true)))
                    VSL.XToggleWithEngines(CFG.AP2, Autopilot2.BallisticJump);
			}
            else if(UI.NAV != null)
            {
                if(GUILayout.Button(new GUIContent("Jump To", "Select target point to jump to"),
                                    Styles.active_button, GUILayout.ExpandWidth(true)))
                    UI.NAV.SetSurfaceTarget();
            }
            else GUILayout.Label(new GUIContent("Jump To", "Target a landed vessel or create a waypoint"),
                                 Styles.inactive_button, GUILayout.ExpandWidth(true));
		}
	}
}

