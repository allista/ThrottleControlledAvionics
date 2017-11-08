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
			[Persistent] public float StartAltitude    = 100f; //m
			[Persistent] public float InclinationF     = 2f;
			[Persistent] public float ObstacleOffset   = 50f;
            [Persistent] public float MinStartAngle    = 10f;

            [Persistent] public PIDf_Controller FallCorrectionPID = new PIDf_Controller(1, 0.5f, 0, 0, float.PositiveInfinity);
		}
		static Config BJ { get { return Globals.Instance.BJ; } }

		public BallisticJump(ModuleTCA tca) : base(tca) {}

        Radar RAD;

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

        protected override Autopilot2 program
        {
            get { return Autopilot2.BallisticJump; }
        }

        protected override string program_name
        {
            get { return "'Jump To' Autopilot"; }
        }

		protected override void Reset()
		{
			base.Reset();
			stage = Stage.None;
			StartAltitude = BJ.StartAltitude;
            Executor.Reset();
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
            var dV = compute_intial_jump_velocity()-VesselOrbit.vel;
            var trj = new LandingTrajectory(VSL, dV, VSL.Physics.UT, CFG.Target, TargetAltitude, false);
            if(trj.TransferTime < ManeuverOffset)
			{
                #if DEBUG
                Log("Too close jump trajectory: {}", trj);
                #endif
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
				Reset();
				if(setup())
				{
					SaveGame("before_jump");
					goto case Multiplexer.Command.Resume;
				}
                Disable();
				return;

			case Multiplexer.Command.Off:
                ReleaseCPS();
				SetTarget();
				Reset();
				break;
			}
		}

        class LandingSiteOptimizer : LandingSiteOptimizerBase
        {
            protected readonly BallisticJump m;
            protected readonly Vector3d direction;
            protected double velocity;
            protected double angle;

            public LandingSiteOptimizer(BallisticJump module, double velocity, Vector3d direction, float dtol) 
                : base(module, dtol)
            { 
                m = module;
                this.velocity = velocity;
                this.direction = direction;
                angle = 0;
//                Utils.Log("BJ: V {}, dir {}", velocity, direction);
            }

            protected virtual double start_offset() { return BJ.StartOffset; }
            protected double startUT { get { return m.VSL.Physics.UT + start_offset(); } }
            protected virtual Vector3d dir { get { return direction; } }
            protected virtual bool ScanDeltaFi { get { return Best.DistanceToTarget > 1000; } }

            IEnumerable<LandingTrajectory> optimize_DeltaV()
            {
                var dV = dR2dV(Best.DeltaR);
//                Utils.Log("dR {}, Angle2Tgt {}, G {}", Best.DeltaR, m.CFG.Target.AngleTo(m.VSL), m.Body.GeeASL);//debug
                var bestV = velocity;
                var dVEnd = Math.Abs(dV)/100;
                LandingTrajectory cur = Best, prev;
                while(Math.Abs(dV) > dVEnd)
                {
                    prev = cur;
                    cur = new LandingTrajectory(m.VSL, 
                                                (QuaternionD.AngleAxis(angle, m.VesselOrbit.getRelativePositionAtUT(startUT)) * dir)*velocity -
                                                m.VesselOrbit.getOrbitalVelocityAtUT(startUT), 
                                                startUT, m.CFG.Target, cur.TargetAltitude);
                    if(cur.DistanceToTarget < Best.DistanceToTarget) 
                    {
                        Best = cur;
                        bestV = velocity;
                    }
                    if(cur.DistanceToTarget > prev.DistanceToTarget) 
                    {
                        dV /= -2.1;
                        velocity = bestV;
                    }
                    velocity += dV;
//                    Utils.Log("D {}, dR {}, dV {}, V {}", cur.DistanceToTarget, cur.DeltaR, dV, velocity);//debug
                    yield return cur;
                }
                velocity = bestV;
            }

            IEnumerable<LandingTrajectory> optimize_DeltaFi()
            {
                var dFi = ScanDeltaFi? 10.0*Math.Sign(Best.DeltaFi) : Best.DeltaFi;
                var fi = 0.0;
                var bestFi = fi;
                var cur = Best;
                var scanned = !ScanDeltaFi;
                while(Math.Abs(dFi) > 1e-3)
                {
                    cur = new LandingTrajectory(m.VSL, 
                                                (QuaternionD.AngleAxis(fi+angle, m.VesselOrbit.getRelativePositionAtUT(startUT)) * dir)*velocity -
                                                m.VesselOrbit.getOrbitalVelocityAtUT(startUT),
                                                startUT, m.CFG.Target, cur.TargetAltitude);
                    if(cur.DistanceToTarget < Best.DistanceToTarget)
                    {
                        Best = cur;
                        bestFi = fi;
                    }
                    else if(scanned || Math.Abs(fi+angle) > 120) 
                    {
                        dFi /= -2.1;
                        fi = bestFi;
                        scanned = true;
                    }
                    fi += dFi;
//                    Utils.Log("D {}, fi {} ({}), dfi {}, DeltaFi {}, trajectory {}", 
//                              cur.DistanceToTarget, fi, fi+angle, dFi, cur.DeltaFi, cur);//debug
                    yield return cur;
                }
                angle += bestFi;
            }

            public override IEnumerator<LandingTrajectory> GetEnumerator()
            {
                Best = new LandingTrajectory(m.VSL, 
                                             dir*velocity-m.VesselOrbit.getOrbitalVelocityAtUT(startUT), 
                                             startUT, m.CFG.Target, m.TargetAltitude);
                LandingTrajectory prev = null;
                while(continue_calculation(prev, Best))
                {       
                    prev = Best;
                    foreach(var t in optimize_DeltaV()) yield return t;
                    foreach(var t in optimize_DeltaFi()) yield return t;
                    Utils.Log("Best so far: {}", Best.DistanceToTarget);
                }
            }
        }

        class LandingSiteCorrector : LandingSiteOptimizer
        {
            public LandingSiteCorrector(BallisticJump module, double velocity, float dtol) 
                : base(module, velocity, Vector3d.zero, dtol) {}

            protected override double start_offset() 
            { return Math.Max(m.CorrectionOffset, m.VSL.Torque.NoEngines.TurnTime); }

            protected override Vector3d dir
            { get { return m.VesselOrbit.getOrbitalVelocityAtUT(startUT).normalized; } }

            protected override bool ScanDeltaFi { get { return false; } }
        }

        Vector3d compute_intial_jump_velocity()
        {
            var tPos = CFG.Target.OrbPos(Body);
            var solver = new LambertSolver(VesselOrbit, tPos+tPos.normalized*LTRJ.FlyOverAlt, VSL.Physics.UT);
            var vel = VesselOrbit.vel +
                solver.dV4TransferME()
                .ClampMagnitudeH(Math.Sqrt(Body.gMagnitudeAtCenter/VesselOrbit.radius));
            if(Vector3d.Dot(vel, VesselOrbit.pos) < 0) vel = -vel;
            //correcto for low trajectories
            var ascention_angle = 90-Utils.Angle2(vel, VesselOrbit.pos);
            if(ascention_angle < BJ.MinStartAngle)
                vel = QuaternionD.AngleAxis(BJ.MinStartAngle-ascention_angle, Vector3d.Cross(vel, VesselOrbit.pos)) * vel;
            return vel;
        }

		void compute_initial_trajectory()
		{
			MAN.MinDeltaV = 1f;
            var vel = compute_intial_jump_velocity();
            var dir = vel.normalized;
            var V = vel.magnitude;
            ComputeTrajectory(new LandingSiteOptimizer(this, V, dir, LTRJ.Dtol));
            stage = Stage.Compute;
            trajectory = null;
		}

		void accelerate()
		{
			CFG.HF.Off();
			clear_nodes();
            fall_correction.Reset();
			stage = Stage.Accelerate;
		}

		protected override void fine_tune_approach()
		{
            update_landing_trajecotry();
			var V = VesselOrbit.getOrbitalVelocityAtUT(VSL.Physics.UT+CorrectionOffset).magnitude;
            ComputeTrajectory(new LandingSiteCorrector(this, V, LTRJ.Dtol/2));
            stage = Stage.CorrectTrajectory;
            trajectory = null;
		}

        void VSC_ON(HFlight HF_program)
        {
            CFG.HF.OnIfNot(HF_program);
            CFG.BlockThrottle = true;
            CFG.AltitudeAboveTerrain = true;
            CFG.VF.OnIfNot(VFlight.AltitudeControl);
        }

		protected override void UpdateState()
		{
			base.UpdateState();
			IsActive &= CFG.AP2[Autopilot2.BallisticJump];
			ControlsActive &= IsActive || VSL.HasTarget;
		}

		protected override void Update()
		{
			if(landing) { do_land(); return; }
			switch(stage)
			{
			case Stage.Start:
                VSC_ON(HFlight.Stop);
				if(VSL.LandedOrSplashed || VSL.Altitude.Relative < StartAltitude) 
                    CFG.DesiredAltitude = StartAltitude;
                else CFG.DesiredAltitude = VSL.Altitude.Relative;
                if(!VSL.LandedOrSplashed)
                    compute_initial_trajectory();
				break;
            case Stage.GainAltitude:
                Status("Gaining altitude...");
                VSC_ON(HFlight.Level);
                if(VSL.Altitude.Relative > CFG.DesiredAltitude-10)
                    compute_initial_trajectory();
                break;
			case Stage.Compute:
                double obstacle;
                if(!trajectory_computed()) break;
                if(find_biggest_obstacle_ahead(BJ.StartAltitude, out obstacle)) break;
                if(obstacle > 0)
				{
                    StartAltitude += (float)obstacle+BJ.ObstacleOffset;
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
                CFG.AT.OnIfNot(Attitude.Custom);
                var dV = (trajectory.Orbit.GetFrameVelAtUT(trajectory.StartUT)-
                          VSL.orbit.GetFrameVelAtUT(trajectory.StartUT)).xzy;
				if(VSL.Controls.AlignmentFactor > 0.9 || !CFG.VSCIsActive)
				{
					CFG.DisableVSC();
                    var dVv = -VSL.Altitude.Absolute;//Vector3d.Dot(dV, VSL.Physics.Up);
                    if(RAD != null)
                    {
                        var dH = VSL.Altitude.Relative - VSL.Altitude.Ahead - VSL.Geometry.H;
                        if(dH < 0) dVv -= dH/RAD.TimeAhead*1.1f;
                    }
                    if(dVv > 0)
                    {
                        fall_correction.I = BJ.FallCorrectionPID.I * Utils.ClampL(100/VSL.Altitude.Relative, 1);
                        fall_correction.Update(-VSL.Altitude.Absolute);
                    }
                    else
                    {
                        fall_correction.IntegralError *= Utils.ClampL(1-fall_correction.I/10*TimeWarp.fixedDeltaTime, 0);
                        fall_correction.Update(0);
                    }
//                    Log("dV {}, dVv {}, correction {}, I {}", dV, dVv, fall_correction.Action, fall_correction.I);//debug
                    if(Executor.Execute(dV+fall_correction*VSL.Physics.Up, 10)) 
                        Status("white", 
                               "Accelerating: <color=yellow><b>{0}</b> m/s</color>", 
                               (dV.magnitude-10).ToString("F1"));
                    else fine_tune_approach();
				}
                else ATC.SetThrustDirW(-dV);
				break;
			case Stage.CorrectTrajectory:
                VSL.Info.Countdown = landing_trajectory.BrakeStartPoint.UT-VSL.Physics.UT-ManeuverOffset;
                if(VSL.Info.Countdown > 0)
                {
                    warp_to_coundown();
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
                if(!coast_to_start())
                    stage = Stage.None;
				break;
			}
		}

        static readonly GUIContent button_content = new GUIContent("Jump To", "Fly to the target using ballistic trajectory.");
		public override void Draw()
		{
			#if DEBUG
//			if(current != null)//debug
//			{
//				Utils.GLVec(Body.position, current.AtTargetPos.xzy, Color.green);
//				Utils.GLLine(Body.position, current.SurfacePoint.WorldPos(Body), Color.yellow);
//				var brake = current.NewOrbit.getPositionAtUT(current.BrakeStartPoint.UT);
//				Utils.GLVec(brake, current.BrakeDeltaV.xzy.normalized*2000, Color.red);
//				Utils.GLVec(brake, (current.NewOrbit.getOrbitalVelocityAtUT(current.BrakeNodeUT)+
//				                      current.BrakeDeltaV).xzy.normalized*2000, 
//				              Color.magenta);
//			}
			#endif
			if(ControlsActive) 
			{	
                if(CFG.AP2[program])
                    GUILayout.Label(button_content, Styles.enabled_button, GUILayout.ExpandWidth(true));
                else if(GUILayout.Button(button_content, Styles.active_button, GUILayout.ExpandWidth(true)))
                    ShowOptions = !ShowOptions;
			}
            else if(UI.NAV != null)
            {
                if(GUILayout.Button(new GUIContent("Jump To", "Select target point to jump to"),
                                    Styles.active_button, GUILayout.ExpandWidth(true)))
                    UI.NAV.SetSurfaceTarget();
            }
            else GUILayout.Label(button_content, Styles.inactive_button, GUILayout.ExpandWidth(true));
		}
	}
}

