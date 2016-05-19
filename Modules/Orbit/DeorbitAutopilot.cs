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
	public class DeorbitAutopilot : LandingTrajectoryAutopilot
	{
		public new class Config : TCAModule.ModuleConfig
		{
			[Persistent] public float StartOffset = 60f;   //s
			[Persistent] public float StartPeR    = 0.49f; //of planet radius
		}
		static Config DEO { get { return TCAScenario.Globals.DEO; } }

		public DeorbitAutopilot(ModuleTCA tca) : base(tca) {}

		public override void Init()
		{
			base.Init();
			CFG.AP2.AddHandler(this, Autopilot2.Deorbit);
		}

		enum Stage { None, Compute, Deorbit, Correct, Coast, Wait }
		Stage stage;
		double current_PeR;

		protected LandingTrajectory fixed_PeR_orbit(LandingTrajectory old, LandingTrajectory best, ref Vector3d NodeDeltaV, double PeR)
		{
			double StartUT;
			double targetAlt;
			if(old != null) 
			{
				double dLatLon;
				double dLonLat;
				StartUT = old.StartUT;
				targetAlt = old.TargetAltitude;
				if(Math.Abs(VesselOrbit.inclination) <= 45) 
				{ dLonLat = old.DeltaLon; dLatLon = old.DeltaLat; }
				else { dLonLat = old.DeltaLat; dLatLon = old.DeltaLon; }
				if(Math.Abs(dLonLat) > Math.Abs(dLatLon))
					StartUT = AngleDelta2StartUT(old, dLonLat, DEO.StartOffset, VesselOrbit.period, VesselOrbit.period);
				else NodeDeltaV += PlaneCorrection(old);
				Log("\nStartUT change: {0}\ndV {1}", StartUT-old.StartUT, NodeDeltaV);//debug
			}
			else 
			{
				StartUT = VSL.Physics.UT+DEO.StartOffset;
				targetAlt = TargetAltitude;
			}
			return new LandingTrajectory(VSL, dV4Pe(VesselOrbit, Body.Radius*PeR, StartUT, Node2OrbitDeltaV(StartUT, NodeDeltaV)), 
			                             StartUT, Target, targetAlt);
		}

		protected LandingTrajectory horizontal_correction(LandingTrajectory old, LandingTrajectory best, ref Vector3d NodeDeltaV, double start_offset)
		{
			var StartUT = VSL.Physics.UT+start_offset;
			if(old != null) 
			{
				if(Math.Abs(old.DeltaR) > Math.Abs(old.DeltaFi)) 
					NodeDeltaV += new Vector3d(0,0,old.DeltaR);
				else 
					NodeDeltaV += PlaneCorrection(old);
				Log("\ndV: {0}", NodeDeltaV);//debug
			}
			return new LandingTrajectory(VSL, Node2OrbitDeltaV(StartUT, NodeDeltaV), 
			                             StartUT, Target, old == null? TargetAltitude : old.TargetAltitude);
		}

		void compute_landing_trajectory()
		{
			trajectory = null;
			stage = Stage.Compute;
			var NodeDeltaV = Vector3d.zero;
			setup_calculation((o, b) => fixed_PeR_orbit(o, b, ref NodeDeltaV, current_PeR));
		}

		protected override void start_correction()
		{
			trajectory = null;
			stage = Stage.Correct;
			CorrectionTimer.Reset();
			var NodeDeltaV = Vector3d.zero;
			setup_calculation((o, b) => horizontal_correction(o, b, ref NodeDeltaV, LTRJ.CorrectionOffset));
		}

		protected override void reset()
		{
			base.reset();
			stage = Stage.None;
			current_PeR = DEO.StartPeR;
			CFG.AP1.Off();
		}

		public void DeorbitCallback(Multiplexer.Command cmd)
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
				if(VesselOrbit.PeR < Body.Radius)
				{
					Status("<color=red>Already deorbiting.</color> Trying to correct course and land.");
					start_correction();
				}
				else compute_landing_trajectory();
				goto case Multiplexer.Command.Resume;

			case Multiplexer.Command.Off:
				UnregisterFrom<Radar>();
				break;
			}
		}

		protected override void UpdateState()
		{
			base.UpdateState();
			IsActive &= CFG.AP2[Autopilot2.Deorbit];
			var tVSL = VSL.TargetVessel;
			ControlsActive = IsActive || !VSL.LandedOrSplashed && (tVSL != null && tVSL.LandedOrSplashed || VSL.Target is WayPoint);
		}

		protected override void Update()
		{
			if(!IsActive) return;
			if(CFG.Target != Target) SetTarget(Target);
			if(landing) { do_land(); return; }
			switch(stage)
			{
			case Stage.Compute:
				if(!trajectory_computed()) break;
				if(trajectory.DistanceToTarget < LTRJ.Dtol || current_PeR >= 1)
				{
					clear_nodes(); add_trajectory_node();
					CorrectionTimer.Reset();
					if(trajectory.DistanceToTarget < LTRJ.Dtol) 
					{ CFG.AP1.On(Autopilot1.Maneuver); stage = Stage.Deorbit; }
					else 
					{
						Status("red", "Predicted landing site is too far from the target.\n" +
						       "<i>To proceed, activate maneuver execution manually.</i>");
						stage = Stage.Wait;
					}
				}
				else 
				{
					current_PeR += 0.1;
					if(current_PeR < 1) compute_landing_trajectory();
				}
				break;
			case Stage.Wait:
				if(!CFG.AP1[Autopilot1.Maneuver]) break;
				stage = Stage.Deorbit;
				break;
			case Stage.Deorbit:
				if(CFG.AP1[Autopilot1.Maneuver]) break;
				start_correction();
				break;
			case Stage.Correct:
				if(!trajectory_computed()) break;
				clear_nodes(); add_trajectory_node();
				CFG.AP1.OnIfNot(Autopilot1.Maneuver);
				stage = Stage.Coast; 
				break;
			case Stage.Coast:
				if(CFG.AP1[Autopilot1.Maneuver]) break;
				VSL.Info.Countdown = trajectory.BrakeStartUT-VSL.Physics.UT-DEO.StartOffset;
				if(VSL.Info.Countdown > 0 && !correct_trajectory()) break;
				stage = Stage.None;
				start_landing();
				break;
			}
		}

		public override void Draw()
		{
			if(ControlsActive)
			{
				if(computing) GUILayout.Label("Computing...", Styles.inactive_button, GUILayout.ExpandWidth(false));
				else if(Utils.ButtonSwitch("Land", CFG.AP2[Autopilot2.Deorbit],
				                           "Compute and perform a deorbit maneuver, then land near the target.", 
				                           GUILayout.ExpandWidth(false)))
					CFG.AP2.XToggle(Autopilot2.Deorbit);
			}
			else GUILayout.Label(new GUIContent("Land", "Compute and perform a deorbit maneuver, then land near the target."), 
			                     Styles.inactive_button, GUILayout.ExpandWidth(false));
		}
	}
}

