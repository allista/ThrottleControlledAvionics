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
	                typeof(AutoLander))]
	public class BallisticJump : LandingTrajectoryCalculator
	{
		public new class Config : TCAModule.ModuleConfig
		{
			[Persistent] public float StartOffset = 10f; //s
			[Persistent] public float CorrectionOffset = 20f; //s
			[Persistent] public float StartInclination = 30f; //deg
			[Persistent] public float StartAltitude = 100f; //m
			[Persistent] public float CorrectEach = 10f; //sec
			public float StartTangent = 1;

			public override void Init()
			{
				StartTangent = 1/Mathf.Tan(Utils.Clamp(StartInclination, 10, 80)*Mathf.Deg2Rad);
			}
		}
		static Config BJ { get { return TCAScenario.Globals.BJ; } }

		public BallisticJump(ModuleTCA tca) : base(tca) {}

		AttitudeControl ATC;

		enum Stage { None, Accelerate, Coast, Waiting }
		Stage stage;
		double offset;
		double dV;
		Vector3d dVdir;
		bool compute_node;
		Timer CorrectionTimer = new Timer();
		ManeuverNode Node;

		public override void Init()
		{
			base.Init();
			CorrectionTimer.Period = BJ.CorrectEach;
			reset();
			CFG.AP2.AddHandler(this, Autopilot2.BallisticJump);
			#if DEBUG
//			RenderingManager.AddToPostDrawQueue(1, RadarBeam);
			#endif
		}


		#if DEBUG
		public void RadarBeam()
		{
			if(VSL == null || VSL.vessel == null || dVdir.IsZero()) return;
			GLUtils.GLVec(VSL.Physics.wCoM, dVdir.xzy*15, Color.green);
		}

		public override void Reset()
		{
			base.Reset();
//			RenderingManager.RemoveFromPostDrawQueue(1, RadarBeam);
		}
		#endif

		protected override LandingTrajectory compute_next_trajectory(LandingTrajectory old)
		{
			if(old != null) 
			{
				if(Math.Abs(old.DeltaLat) < 1 && Math.Abs(old.DeltaLon) > Math.Abs(old.DeltaLat))
					dV = Utils.ClampL(dV+old.DeltaLon, 0);
				else
					dVdir = Quaternion.AngleAxis((float)old.DeltaLat, VSL.Physics.Up.xzy) * dVdir;
			}
			return new LandingTrajectory(VSL, dVdir*dV-VesselOrbit.vel, VSL.Physics.UT+offset, old == null? TargetAlt : old.TargetAlt);
		}

		protected override void reset()
		{
			base.reset();
			Working = false;
			Target = null;
			stage = Stage.None;
			compute_node = false;
			CFG.AP1.Off();
		}

		public void BallisticJumpCallback(Multiplexer.Command cmd)
		{
			reset();
			switch(cmd)
			{
			case Multiplexer.Command.Resume:
				break;

			case Multiplexer.Command.On:
				if(VSL.Target == null)
				{
					CFG.AP2.Off();
					return;
				}
				if(VesselOrbit.PeR > Body.Radius)
				{
					ThrottleControlledAvionics.StatusMessage = "Unable to perform the <b>Ballistic Jump</b> from orbit.\n" +
						"Use <b>Land at Target</b> instead.";
					CFG.AP2.Off();
					return;
				}
				clear_nodes();
				setup_target();
				goto case Multiplexer.Command.Resume;

			case Multiplexer.Command.Off:
				break;
			}
		}

		void calculate_initial_trajectory()
		{
			trajectory = null;
			compute_node = true;
			stage = Stage.Waiting;
			offset = BJ.StartOffset;
			dVdir = (VSL.Physics.Up+
			         Vector3d.Exclude(VSL.Physics.Up, VSL.refT.right)
			         .normalized*BJ.StartTangent).normalized.xzy;
			dV = Math.Sqrt(VSL.Physics.StG*VSL.Physics.Radial.magnitude)/2;
			MAN.MinDeltaV = 1f;
//			Log("MAN.MinDeltaV {0}", MAN.MinDeltaV);//debug
		}

		void recalculate_trajectory()
		{
			trajectory = null;
			compute_node = true;
			stage = Stage.Waiting;
			offset = BJ.CorrectionOffset;
			dVdir = VesselOrbit.vel.normalized;
			dV = 0;
		}

		protected override void Update()
		{
			if(Target == null || VSL.orbit.referenceBody == null) return;
			if(CFG.Target != Target) SetTarget(Target);
			if(!compute_node && !Working)
			{
				CFG.HF.OnIfNot(HFlight.Stop);
				CFG.AltitudeAboveTerrain = true;
				CFG.VF.OnIfNot(VFlight.AltitudeControl);
				if(VSL.LandedOrSplashed || VSL.Altitude.Relative < BJ.StartAltitude) 
					CFG.DesiredAltitude = BJ.StartAltitude;
				else CFG.DesiredAltitude = VSL.Altitude.Relative;
				if(VSL.Altitude.Relative > BJ.StartAltitude/2)
					calculate_initial_trajectory();
			}
			if(compute_node && trajectory_computed())
			{
				Working = true;
				compute_node = false;
				clear_nodes();
				ManeuverAutopilot.AddNode(VSL, trajectory.ManeuverDeltaV, trajectory.StartUT);
				Node = VSL.vessel.patchedConicSolver.maneuverNodes[0];
				if(trajectory.DistanceToTarget < TRJ.Dtol)
				{ 
					CFG.HF.Off();
					CFG.AT.OnIfNot(Attitude.ManeuverNode);
					stage = Stage.Accelerate;
				}
				else 
				{
					ThrottleControlledAvionics.StatusMessage = "Predicted landing site is too far from the target.\n" +
						"To proceed, activate maneuver execution manually.";
					stage = Stage.Waiting;
				}
				return;
			}
			if(Working) 
			{ 
				if(landing) { do_land(); return; }
				switch(stage)
				{
				case Stage.Waiting:
					if(!CFG.AP1[Autopilot1.Maneuver]) break;
					stage = Stage.Accelerate;
					break;
				case Stage.Accelerate:
					if(Node == null) { stage = Stage.Coast; break; }
					if(CFG.VSCIsActive)
					{ 
						if(ATC.AttitudeFactor > 0) 
						{
							CFG.DisableVSC();
							calculate_initial_trajectory();
						}
					}
					else 
					{
						CFG.AP1.OnIfNot(Autopilot1.Maneuver);
						if(!CFG.AP1[Autopilot1.Maneuver]) stage = Stage.Coast;
					}
					break;
				case Stage.Coast:
					if(VesselOrbit.trueAnomaly > 180)
					{
						stage = Stage.None;
						start_landing();
						break;
					}
					if(!CorrectionTimer.Check) break;
					CorrectionTimer.Reset();
					trajectory.UpdateOrbit(VesselOrbit);
					trajectory.UpdateDistance(Target);
					if(trajectory.DistanceToTarget >= TRJ.Dtol)
						recalculate_trajectory();
					break;
				}
			}
		}

		public override void Draw()
		{
			if(compute_node) GUILayout.Label("Computing...", Styles.grey_button, GUILayout.ExpandWidth(false));
			else if(Utils.ButtonSwitch("Jump to Target", CFG.AP2[Autopilot2.BallisticJump],
			                           "Fly to the target using ballistic trajectory.", 
			                           GUILayout.ExpandWidth(false)))
				CFG.AP2.XToggle(Autopilot2.BallisticJump);
		}
	}
}

