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
		static Config BJ { get { return TCAScenario.Globals.BJ; } }

		public BallisticJump(ModuleTCA tca) : base(tca) {}

		enum Stage { None, Start, Compute, Accelerate, CorrectAltitude, CorrectTrajectory, Coast, Waiting }
		Stage stage;
		float StartAltitude;

		public override void Init()
		{
			base.Init();
			reset();
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
				Status("yellow", "Cannot to perform <b>Ballistic Jump</b> from orbit.\n" +
				       "Use <b>Land at Target</b> instead.");
				return false;
			}
			return true;
		}

		public void BallisticJumpCallback(Multiplexer.Command cmd)
		{
			reset();
			switch(cmd)
			{
			case Multiplexer.Command.Resume:
				RegisterTo<Radar>();
				break;

			case Multiplexer.Command.On:
				if(!setup())
				{
					CFG.AP2.Off();
					return;
				}
				stage = Stage.Start;
				goto case Multiplexer.Command.Resume;

			case Multiplexer.Command.Off:
				UnregisterFrom<Radar>();
				break;
			}
		}

		protected LandingTrajectory fixed_inclination_orbit(LandingTrajectory old, 
			ref Vector3d dir, ref double V, float start_offset)
		{
			var StartUT = VSL.Physics.UT+start_offset;
			if(old != null) 
			{
				V += old.DeltaR*(1-Target.AngleTo(VSL.vessel)/Utils.TwoPI*0.9);
				dir = Quaternion.AngleAxis((float)old.DeltaFi, VSL.Physics.Up.xzy) * dir;
			}
			return new LandingTrajectory(VSL, dir*V-VesselOrbit.getOrbitalVelocityAtUT(StartUT), StartUT, 
			                             Target, old == null? TargetAltitude : old.TargetAltitude);
		}

		protected LandingTrajectory orbit_correction(LandingTrajectory old, ref double angle, ref double V, float start_offset)
		{
			var StartUT = VSL.Physics.UT+start_offset;
			if(old != null) 
			{
				V += old.DeltaR*(1-Target.AngleTo(VSL.vessel)/Utils.TwoPI*0.9);
				angle += old.DeltaFi;
			}
			var vel = VesselOrbit.getOrbitalVelocityAtUT(StartUT);
			LogF("V {}, angle {}, vel {}", V, angle, vel);//debug
			return new LandingTrajectory(VSL, QuaternionD.AngleAxis(angle, VSL.Physics.Up.xzy)*vel.normalized*V - vel, StartUT, 
			                             Target, old == null? TargetAltitude : old.TargetAltitude);
		}

		void compute_initial_trajectory()
		{
			trajectory = null;
			MAN.MinDeltaV = 1f;
			stage = Stage.Compute;
			var dir = (VSL.Physics.Up+ 
			           Vector3d.Exclude(VSL.Physics.Up, VSL.refT.right).normalized *
			           BJ.StartTangent*(1+Target.AngleTo(VSL.vessel)/Utils.TwoPI)*BJ.InclinationF).normalized.xzy;
			var V = Math.Sqrt(VSL.Physics.StG*VSL.Physics.Radial.magnitude)/2;
			setup_calculation(t => fixed_inclination_orbit(t, ref dir, ref V, BJ.StartOffset));
		}

		protected override void start_correction()
		{
			trajectory = null;
			stage = Stage.CorrectTrajectory;
			double angle = 0;
			double V = VesselOrbit.getOrbitalVelocityAtUT(VSL.Physics.UT+LTRJ.CorrectionOffset).magnitude;
			setup_calculation(t => orbit_correction(t, ref angle, ref V, LTRJ.CorrectionOffset));
		}

		protected override void UpdateState()
		{
			base.UpdateState();
			IsActive &= CFG.AP2[Autopilot2.BallisticJump];
			ControlsActive = IsActive || VSL.HasTarget;
		}

		protected override void Update()
		{
			if(!IsActive) return;
			if(CFG.Target != Target) SetTarget(Target);
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
				break;
			case Stage.Compute:
				if(!trajectory_computed()) break;
				clear_nodes(); add_trajectory_node();
				if(trajectory.DistanceToTarget < LTRJ.Dtol)
				{
					var obst = obstacle_ahead(trajectory);
					if(obst > 0)
					{
						StartAltitude += (float)obst+BJ.ObstacleOffset;
						stage = Stage.Start;
					}
					else
					{
						CFG.HF.Off();
						CFG.AT.OnIfNot(Attitude.ManeuverNode);
						stage = Stage.Accelerate;
					}
				}
				else 
				{
					Status("red", "Predicted landing site is too far from the target.\n" +
					       "<i>To proceed, activate maneuver execution manually.</i>");
					stage = Stage.Waiting;
				}
				break;
			case Stage.Waiting:
				if(!CFG.AP1[Autopilot1.Maneuver]) break;
				stage = Stage.Accelerate;
				break;
			case Stage.Accelerate:
				if(VSL.HasManeuverNode && !CFG.AP1[Autopilot1.Maneuver])
					VSL.Info.Countdown = VSL.vessel.patchedConicSolver.maneuverNodes[0].UT-VSL.Physics.UT;
				if(ATC.Aligned || CFG.AP1[Autopilot1.Maneuver]) 
				{
					CFG.DisableVSC();
					CFG.AP1.OnIfNot(Autopilot1.Maneuver);
					if(!CFG.AP1[Autopilot1.Maneuver]) start_correction();
					Status("Accelerating...");
				}
				break;
			case Stage.CorrectTrajectory:
				if(!trajectory_computed()) break;
				if(trajectory.ManeuverDeltaV.magnitude > GLB.THR.MinDeltaV*2)
				{
					clear_nodes(); add_trajectory_node();
					CFG.AP1.OnIfNot(Autopilot1.Maneuver);
				}
				stage = Stage.Coast;
				break;
			case Stage.Coast:
				if(CFG.AP1[Autopilot1.Maneuver]) break;
				if(VesselOrbit.trueAnomaly < 180 && !correct_trajectory()) break;
				stage = Stage.None;
				start_landing();
				break;
			}
		}

		public override void Draw()
		{
			#if DEBUG
			if(current != null)//debug
			{
				GLUtils.GLVec(Body.position, current.AtTargetPos.xzy, Color.green);
				GLUtils.GLLine(Body.position, current.SurfacePoint.WorldPos(Body), Color.yellow);
			}
			#endif
			if(ControlsActive) 
			{	
				if(computing) 
					GUILayout.Label(new GUIContent("Jump To", "Computing maneuver..."), 
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

