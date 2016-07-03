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
	public class DeorbitAutopilot : LandingTrajectoryAutopilot
	{
		public new class Config : TCAModule.ModuleConfig
		{
			[Persistent] public float StartEcc     = 0.5f;
			[Persistent] public int   EccSteps     = 10;
			[Persistent] public float MaxEcc       = 0.8f; 
			[Persistent] public float AngularDragF = 0.5f;

			public float dEcc;

			public override void Init()
			{
				base.Init();
				dEcc = StartEcc/EccSteps;
			}
		}
		static Config DEO { get { return Globals.Instance.DEO; } }

		public DeorbitAutopilot(ModuleTCA tca) : base(tca) {}

		public override void Init()
		{
			base.Init();
			CFG.AP2.AddHandler(this, Autopilot2.Deorbit);
		}

		public enum Stage { None, Compute, Deorbit, Correct, Coast, Wait }
		[Persistent] public Stage stage;
		double currentEcc;

		protected LandingTrajectory fixed_Ecc_orbit(LandingTrajectory old, LandingTrajectory best, ref Vector3d NodeDeltaV, double ecc)
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
					StartUT = AngleDelta2StartUT(old, dLonLat, TRJ.ManeuverOffset, VesselOrbit.period, VesselOrbit.period);
				else NodeDeltaV += PlaneCorrection(old);
			}
			else 
			{
				StartUT = VSL.Physics.UT+TRJ.ManeuverOffset;
				targetAlt = TargetAltitude;
			}
			return new LandingTrajectory(VSL, dV4Ecc(VesselOrbit, ecc, StartUT, Body.Radius*0.9, Node2OrbitDeltaV(StartUT, NodeDeltaV)), 
			                             StartUT, CFG.Target, targetAlt);
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
			}
			return new LandingTrajectory(VSL, Node2OrbitDeltaV(StartUT, NodeDeltaV), 
			                             StartUT, CFG.Target, old == null? TargetAltitude : old.TargetAltitude);
		}

		void compute_landing_trajectory()
		{
			trajectory = null;
			MAN.MinDeltaV = 1;
			Dtol = LTRJ.Dtol;
			var NodeDeltaV = Vector3d.zero;
			setup_calculation((o, b) => fixed_Ecc_orbit(o, b, ref NodeDeltaV, currentEcc));
			stage = Stage.Compute;
		}

		protected override void fine_tune_approach()
		{
			trajectory = null;
			Dtol = LTRJ.Dtol/2;
			CorrectionTimer.Reset();
			var NodeDeltaV = Vector3d.zero;
			setup_calculation((o, b) => horizontal_correction(o, b, ref NodeDeltaV, Mathf.Max(LTRJ.CorrectionOffset, VSL.Torque.NoEngines.TurnTime)));
			stage = Stage.Correct;
		}

		protected override void reset()
		{
			base.reset();
			stage = Stage.None;
			currentEcc = DEO.StartEcc;
			CFG.AP1.Off();
		}

		public void DeorbitCallback(Multiplexer.Command cmd)
		{
			switch(cmd)
			{
			case Multiplexer.Command.Resume:
//				Utils.Log("Resuming: stage {}, landing_stage {}, landing {}", stage, landing_stage, landing);//debug
				NeedRadarWhenMooving();
				if(stage == Stage.None && !landing) 
					goto case Multiplexer.Command.On;
				else if(VSL.HasManeuverNode) 
					CFG.AP1.OnIfNot(Autopilot1.Maneuver);
				break;

			case Multiplexer.Command.On:
				reset();
				if(!setup()) 
				{
					CFG.AP2.Off();
					return;
				}
				if(VesselOrbit.PeR < Body.Radius)
				{
					Status("red", "Already deorbiting. Trying to correct course and land.");
					fine_tune_approach();
				}
				else 
				{
					currentEcc = DEO.StartEcc;
					if(Body.atmosphere) currentEcc = 
						Utils.ClampH(currentEcc*(2.1-Utils.ClampH(VSL.Torque.MaxPossible.AngularDragResistance/Body.atmDensityASL*DEO.AngularDragF, 1)), DEO.MaxEcc);
					compute_landing_trajectory();
				}
				goto case Multiplexer.Command.Resume;

			case Multiplexer.Command.Off:
				UnregisterFrom<Radar>();
				reset();
				break;
			}
		}

		protected override void UpdateState()
		{
			base.UpdateState();
			IsActive &= CFG.AP2[Autopilot2.Deorbit];
			ControlsActive = IsActive || 
				!VSL.LandedOrSplashed && (VSL.Target is WayPoint || VSL.TargetVessel != null && VSL.TargetVessel.LandedOrSplashed);
		}

		protected override void Update()
		{
			if(!IsActive) { CFG.AP2.OffIfOn(Autopilot2.Deorbit); return; }
			if(landing) { do_land(); return; }
			switch(stage)
			{
			case Stage.Compute:
				if(!trajectory_computed()) break;
				if(trajectory.DistanceToTarget < LTRJ.Dtol || currentEcc < 1e-10)
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
					currentEcc -= DEO.dEcc;
					if(currentEcc > 1e-10) 
						compute_landing_trajectory();
				}
				break;
			case Stage.Wait:
				if(!CFG.AP1[Autopilot1.Maneuver]) break;
				stage = Stage.Deorbit;
				break;
			case Stage.Deorbit:
				Status("Executing deorbit burn...");
				if(CFG.AP1[Autopilot1.Maneuver]) break;
				fine_tune_approach();
				break;
			case Stage.Correct:
				if(!trajectory_computed()) break;
				add_correction_node_if_needed();
				stage = Stage.Coast; 
				break;
			case Stage.Coast:
				if(CFG.AP1[Autopilot1.Maneuver]) 
				{ Status("Correcting trajectory..."); break; }
				Status("Coasting...");
				update_trajectory();
				VSL.Info.Countdown = trajectory.BrakeStartUT-VSL.Physics.UT-TRJ.ManeuverOffset;
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
				if(computing) 
				{
					if(GUILayout.Button(new GUIContent("Land", "Computing trajectory. Push to cancel."), 
					                                   Styles.inactive_button, GUILayout.ExpandWidth(false)))
						CFG.AP2.XOff();
				}
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

