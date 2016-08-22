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
			public float StartTangent = 1;

			public override void Init()
			{
				StartTangent = 1/Mathf.Tan(Utils.Clamp(StartInclination, 10, 80)*Mathf.Deg2Rad);
			}
		}
		static Config BJ { get { return Globals.Instance.BJ; } }

		public BallisticJump(ModuleTCA tca) : base(tca) {}

		public enum Stage { None, Start, Compute, Accelerate, CorrectAltitude, CorrectTrajectory, Coast, Wait }
		[Persistent] public Stage stage;
		float StartAltitude;

		public override void Init()
		{
			base.Init();
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
			if(trj.TransferTime < TRJ.ManeuverOffset)
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
//				LogFST("Resuming: stage {}, landing_stage {}, landing {}", stage, landing_stage, landing);//debug
				if(!check_patched_conics()) return;
				NeedRadarWhenMooving();
				if(VSL.HasManeuverNode) 
					CFG.AP1.OnIfNot(Autopilot1.Maneuver);
				if(stage == Stage.None && !landing) 
					stage = Stage.Start;
				break;

			case Multiplexer.Command.On:
				reset();
				if(setup())
					goto case Multiplexer.Command.Resume;
				CFG.AP2.Off();
				return;

			case Multiplexer.Command.Off:
				UnregisterFrom<Radar>();
				reset();
				break;
			}
		}

		protected LandingTrajectory fixed_inclination_orbit(LandingTrajectory old, LandingTrajectory best, 
			ref Vector3d dir, ref double V, float start_offset)
		{
			var StartUT = VSL.Physics.UT+start_offset;
			if(old != null) 
			{
				V += old.DeltaR*(1-CFG.Target.AngleTo(VSL)/Math.PI*0.9)*Body.GeeASL;
				dir = Quaternion.AngleAxis((float)old.DeltaFi, VSL.Physics.Up.xzy) * dir;
			}
			return new LandingTrajectory(VSL, dir*V-VesselOrbit.getOrbitalVelocityAtUT(StartUT), StartUT, 
			                             CFG.Target, old == null? TargetAltitude : old.TargetAltitude);
		}

		protected LandingTrajectory orbit_correction(LandingTrajectory old, LandingTrajectory best, ref double angle, ref double V, float start_offset)
		{
			var StartUT = VSL.Physics.UT+start_offset;
			if(old != null) 
			{
				V += old.DeltaR*(1-CFG.Target.AngleTo(VSL)/Math.PI*0.9)*Body.GeeASL;
				angle += old.DeltaFi;
			}
			var vel = VesselOrbit.getOrbitalVelocityAtUT(StartUT);
			return new LandingTrajectory(VSL, QuaternionD.AngleAxis(angle, VSL.Physics.Up.xzy)*vel.normalized*V - vel, StartUT, 
			                             CFG.Target, old == null? TargetAltitude : old.TargetAltitude);
		}

		#if DEBUG
		public static bool ME_orbit = true;
		#endif
		void compute_initial_trajectory()
		{
			trajectory = null;
			MAN.MinDeltaV = 1f;
			stage = Stage.Compute;
			var tPos = CFG.Target.RelOrbPos(Body);
			var solver = new LambertSolver(VesselOrbit, tPos+tPos.normalized*LTRJ.FlyOverAlt, VSL.Physics.UT);
			var vel = (VesselOrbit.vel+solver.dV4TransferME());
			if(Vector3d.Dot(vel, VesselOrbit.pos) < 0) vel = -vel;
			var dir = vel.normalized;
			var V = vel.magnitude;
			#if DEBUG
			if(!ME_orbit)
				dir = (VSL.Physics.Up+ 
			           Vector3d.Exclude(VSL.Physics.Up, vel).normalized *
			           BJ.StartTangent*(1+CFG.Target.AngleTo(VSL)/Utils.TwoPI)*BJ.InclinationF).normalized.xzy;
			#endif
			setup_calculation((o, b) => fixed_inclination_orbit(o, b, ref dir, ref V, BJ.StartOffset));
		}

		void accelerate()
		{
			CFG.HF.Off();
			clear_nodes(); add_trajectory_node();
			CFG.AT.OnIfNot(Attitude.ManeuverNode);
			stage = Stage.Accelerate;
		}

		protected override void fine_tune_approach()
		{
			trajectory = null;
			stage = Stage.CorrectTrajectory;
			double angle = 0;
			double V = VesselOrbit.getOrbitalVelocityAtUT(VSL.Physics.UT+LTRJ.CorrectionOffset).magnitude;
			setup_calculation((o, b) => orbit_correction(o, b, ref angle, ref V, LTRJ.CorrectionOffset));
		}

		protected override void UpdateState()
		{
			base.UpdateState();
			IsActive &= CFG.AP2[Autopilot2.BallisticJump];
			ControlsActive &= IsActive || (VSL.Target is WayPoint || VSL.TargetVessel != null && VSL.TargetVessel.LandedOrSplashed);
		}

		protected override void Update()
		{
			if(!IsActive) { CFG.AP2.OffIfOn(Autopilot2.BallisticJump); return; }
			if(landing) { do_land(); return; }
			switch(stage)
			{
			case Stage.Start:
				CFG.HF.OnIfNot(HFlight.Stop);
				CFG.AltitudeAboveTerrain = true;
				CFG.VF.OnIfNot(VFlight.AltitudeControl);
				if(VSL.LandedOrSplashed || VSL.Altitude.Relative < StartAltitude) 
					CFG.DesiredAltitude = StartAltitude;
				else CFG.DesiredAltitude = VSL.Altitude.Relative;
				if(VSL.Altitude.Relative > StartAltitude-5)
					compute_initial_trajectory();
				else Status("Gaining initial altitude...");
				break;
			case Stage.Compute:
				if(!trajectory_computed()) break;
				var obst = obstacle_ahead(trajectory);
				if(obst > 0)
				{
					StartAltitude += (float)obst+BJ.ObstacleOffset;
					stage = Stage.Start;
				}
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
					if(!Executor.Execute(VSL.FirstManeuverNode.GetBurnVector(VesselOrbit), 10)) fine_tune_approach();
					Status("white", "Accelerating...");
				}
				break;
			case Stage.CorrectTrajectory:
				if(!trajectory_computed()) break;
				add_correction_node_if_needed();
				stage = Stage.Coast;
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
			Utils.ButtonSwitch("ME", ref ME_orbit, "Use ME orbit instead of a shallow one.", GUILayout.ExpandWidth(false));
			#endif
			if(ControlsActive) 
			{	
				if(computing) 
					GUILayout.Label(new GUIContent("Jump To", "Computing maneuver. Push to cancel."), 
					                Styles.inactive_button, GUILayout.ExpandWidth(false));
				else if(Utils.ButtonSwitch("Jump To", CFG.AP2[Autopilot2.BallisticJump],
				                           "Fly to the target using ballistic trajectory.", 
				                           GUILayout.ExpandWidth(false)))
					CFG.AP2.XToggle(Autopilot2.BallisticJump);
			}
			else GUILayout.Label(new GUIContent("Jump To", "Target a landed vessel or create a waypoint"),
			                     Styles.inactive_button, GUILayout.ExpandWidth(false));
		}
	}
}

