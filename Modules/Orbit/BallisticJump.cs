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
//	[CareerPart]
	[RequireModules(typeof(ManeuverAutopilot), 
	                typeof(AttitudeControl),
	                typeof(AltitudeControl),
	                typeof(AutoLander))]
	public class BallisticJump : LandingTrajectoryAutopilot
	{
		public new class Config : TCAModule.ModuleConfig
		{
			[Persistent] public float StartOffset = 10f; //s
			[Persistent] public float StartInclination = 30f; //deg
			[Persistent] public float StartAltitude = 100f; //m
			public float StartTangent = 1;

			public override void Init()
			{
				StartTangent = 1/Mathf.Tan(Utils.Clamp(StartInclination, 10, 80)*Mathf.Deg2Rad);
			}
		}
		static Config BJ { get { return TCAScenario.Globals.BJ; } }

		public BallisticJump(ModuleTCA tca) : base(tca) {}

		AttitudeControl ATC;

		enum Stage { None, Start, Compute, Accelerate, Coast, Waiting }
		Stage stage;
		ManeuverNode Node;

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
			CFG.AP1.Off();
		}

		protected override bool check_target()
		{
			if(!base.check_target()) return false;
			if(VesselOrbit.PeR > Body.Radius)
			{
				Status("Cannot to perform <b>Ballistic Jump</b> from orbit.\n" +
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
				break;
			}
		}

		protected LandingTrajectory fixed_inclination_orbit(LandingTrajectory old, 
			ref Vector3d dir, ref double V, float start_offset)
		{
			var StartUT = VSL.Physics.UT+start_offset;
			if(old != null) 
			{
				if(Math.Abs(old.DeltaFi) < 1 && Math.Abs(old.DeltaR) > Math.Abs(old.DeltaFi))
					V += old.DeltaR;
				else
					dir = Quaternion.AngleAxis((float)old.DeltaFi, VSL.Physics.Up.xzy) * dir;
			}
			return new LandingTrajectory(VSL, dir*V-VesselOrbit.getOrbitalVelocityAtUT(StartUT), StartUT, 
			                             Target, old == null? TargetAltitude : old.TargetAltitude);
		}

		void compute_initial_trajectory()
		{
			trajectory = null;
			MAN.MinDeltaV = 1f;
			stage = Stage.Compute;
			var dir = (VSL.Physics.Up+
				Vector3d.Exclude(VSL.Physics.Up, VSL.refT.right)
				.normalized*BJ.StartTangent).normalized.xzy;
			var V = Math.Sqrt(VSL.Physics.StG*VSL.Physics.Radial.magnitude)/2;
			setup_calculation(t => fixed_inclination_orbit(t, ref dir, ref V, BJ.StartOffset));
		}

		void correct_trajectory()
		{
			trajectory = null;
			stage = Stage.Compute;
			var dir = VesselOrbit.vel.normalized;
			var V = VesselOrbit.orbitalSpeed;
			setup_calculation(t => fixed_inclination_orbit(t, ref dir, ref V, LTRJ.CorrectionOffset));
		}

		protected override void Update()
		{
			if(Target == null || VSL.orbit.referenceBody == null) return;
			if(CFG.Target != Target) SetTarget(Target);
			if(landing) { do_land(); return; }
			switch(stage)
			{
			case Stage.Start:
				CFG.HF.OnIfNot(HFlight.Stop);
				CFG.AltitudeAboveTerrain = true;
				CFG.VF.OnIfNot(VFlight.AltitudeControl);
				if(VSL.LandedOrSplashed || VSL.Altitude.Relative < BJ.StartAltitude) 
					CFG.DesiredAltitude = BJ.StartAltitude;
				else CFG.DesiredAltitude = VSL.Altitude.Relative;
				if(VSL.Altitude.Relative > BJ.StartAltitude/2)
					compute_initial_trajectory();
				break;
			case Stage.Compute:
				if(!trajectory_computed()) break;
				clear_nodes(); add_trajectory_node();
				Node = VSL.vessel.patchedConicSolver.maneuverNodes[0];
				if(trajectory.DistanceToTarget < LTRJ.Dtol)
				{ 
					CFG.HF.Off();
					CFG.AT.OnIfNot(Attitude.ManeuverNode);
					stage = Stage.Accelerate;
				}
				else 
				{
					Status("Predicted landing site is too far from the target.\n" +
					       "To proceed, activate maneuver execution manually.");
					stage = Stage.Waiting;
				}
				break;
			case Stage.Waiting:
				if(!CFG.AP1[Autopilot1.Maneuver]) break;
				stage = Stage.Accelerate;
				break;
			case Stage.Accelerate:
				if(Node == null) { stage = Stage.Coast; break; }
				if(CFG.VSCIsActive)
				{ 
					if(ATC.Aligned) 
					{
						CFG.DisableVSC();
						compute_initial_trajectory();
					}
				}
				else 
				{
					CFG.AP1.OnIfNot(Autopilot1.Maneuver);
					if(!CFG.AP1[Autopilot1.Maneuver]) stage = Stage.Coast;
				}
				break;
			case Stage.Coast:
				if(VesselOrbit.trueAnomaly < 180)
				{
					if(!CorrectionTimer.Check) break;
					trajectory.UpdateOrbit(VesselOrbit);
					if(trajectory.DistanceToTarget >= LTRJ.Dtol)
					{ correct_trajectory(); break; }
					if(Body.atmosphere) break;
				}
				stage = Stage.None;
				start_landing();
				break;
			}
		}

		public override void Draw()
		{
			if(!VSL.HasTarget) 
				GUILayout.Label(new GUIContent("Jump To", "No target."),
				                Styles.grey, GUILayout.ExpandWidth(false));
			else if(computing) 
				GUILayout.Label(new GUIContent("Jump To", "Computing maneuver..."), 
				                Styles.grey, GUILayout.ExpandWidth(false));
			else if(Utils.ButtonSwitch("Jump To", CFG.AP2[Autopilot2.BallisticJump],
			                           "Fly to the target using ballistic trajectory.", 
			                           GUILayout.ExpandWidth(false)))
				CFG.AP2.XToggle(Autopilot2.BallisticJump);
		}
	}
}

