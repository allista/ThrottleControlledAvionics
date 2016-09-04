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


		Vector3d PlaneCorrection(TargetedTrajectoryBase old)
		{
			var angle = old.DeltaFi;
			angle *= Math.Sin(old.TransferTime/old.Orbit.period*2*Math.PI);
			var rot = QuaternionD.AngleAxis(angle, old.StartPos);
			return Orbit2NodeDeltaV((rot*old.StartVel)-old.StartVel, old.StartUT);
		}

		protected LandingTrajectory fixed_Ecc_orbit(LandingTrajectory old, LandingTrajectory best, ref Vector3d NodeDeltaV, double ecc)
		{
			double StartUT;
			double targetAlt;
			if(old != null) 
			{
				targetAlt = old.TargetAltitude;
				StartUT = AngleDelta2StartUT(old, Math.Abs(VesselOrbit.inclination) <= 45 ? old.DeltaLon : old.DeltaLat, 
				                             TRJ.ManeuverOffset, VesselOrbit.period, VesselOrbit.period);
				NodeDeltaV += PlaneCorrection(old);
				if(old.BrakeEndDeltaAlt < LTRJ.FlyOverAlt) //correct fly-over altitude
					NodeDeltaV += new Vector3d((LTRJ.FlyOverAlt-old.BrakeEndDeltaAlt)/100, 0, 0);
			}
			else 
			{
				StartUT = VSL.Physics.UT+TRJ.ManeuverOffset;
				targetAlt = TargetAltitude;
			}
			return new LandingTrajectory(VSL, 
			                             dV4Ecc(VesselOrbit, ecc, StartUT, Body.Radius*0.9)+
			                             Node2OrbitDeltaV(NodeDeltaV, StartUT), 
			                             StartUT, CFG.Target, targetAlt);
		}

		protected LandingTrajectory horizontal_correction(LandingTrajectory old, LandingTrajectory best, ref Vector3d NodeDeltaV, double start_offset)
		{
			var StartUT = VSL.Physics.UT+start_offset;
			if(old != null) 
			{
				if(Math.Abs(old.DeltaR) > Math.Abs(old.DeltaFi)) 
					NodeDeltaV += new Vector3d(0, 0, old.DeltaR *
					                           Utils.ClampH(old.Orbit.period/16/old.TimeToSurface, 1));
				else 
					NodeDeltaV += PlaneCorrection(old);
			}
			return new LandingTrajectory(VSL, Node2OrbitDeltaV(NodeDeltaV, StartUT), 
			                             StartUT, CFG.Target, old == null? TargetAltitude : old.TargetAltitude);
		}

		void compute_landing_trajectory()
		{
			//FIXME: sometimes the resulting trajectory still fails to account for CB rotation
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
				if(!check_patched_conics()) return;
				NeedRadarWhenMooving();
				if(stage == Stage.None && !landing) 
					goto case Multiplexer.Command.On;
				else if(VSL.HasManeuverNode) 
					CFG.AP1.OnIfNot(Autopilot1.Maneuver);
				break;

			case Multiplexer.Command.On:
				reset();
				if(!check_patched_conics()) return;
				if(!setup()) { CFG.AP2.Off(); return; }
				if(VesselOrbit.PeR < Body.Radius)
				{
					Status("red", "Already deorbiting. Trying to correct course and land.");
					fine_tune_approach();
				}
				else 
				{
//					Log("Calculating initial orbit eccentricity...");//debug
					var tPos = CFG.Target.RelOrbPos(Body);
					var UT = VSL.Physics.UT +
						AngleDelta(VesselOrbit, tPos, VSL.Physics.UT)/360*VesselOrbit.period;
					var vPos = VesselOrbit.getRelativePositionAtUT(UT);
					var dir = Vector3d.Exclude(vPos, tPos-vPos);
					var ini_orb = CircularOrbit(dir, UT);
					var ini_dV = ini_orb.getOrbitalVelocityAtUT(UT)-VesselOrbit.getOrbitalVelocityAtUT(UT) + dV4Pe(ini_orb, Body.Radius*0.9, UT);
					var trj = new LandingTrajectory(VSL, ini_dV, UT, CFG.Target, TargetAltitude);
					var dV = 10.0;
					dir = -dir.normalized;
					while(trj.Orbit.eccentricity < DEO.StartEcc)
					{
//						Log("\ndV: {}m/s\nini trj:\n{}", dV, trj);//debug
						if(trj.DeltaR > 0) break;
						trj = new LandingTrajectory(VSL, ini_dV+dir*dV, UT, CFG.Target, trj.TargetAltitude);
						dV += 10;
					}
					if(trj.Orbit.eccentricity > DEO.StartEcc)
					{
						currentEcc = DEO.StartEcc;
						if(Body.atmosphere) currentEcc = 
							Utils.ClampH(currentEcc*(2.1-Utils.ClampH(VSL.Torque.MaxPossible.AngularDragResistance/Body.atmDensityASL*DEO.AngularDragF, 1)), DEO.MaxEcc);
					}
					else currentEcc = Utils.ClampL(trj.Orbit.eccentricity - DEO.dEcc, DEO.dEcc);
//					Log("currentEcc: {}, dEcc {}", currentEcc, DEO.dEcc);//debug
					if(Globals.Instance.AutosaveBeforeLanding)
						Utils.SaveGame(VSL.vessel.vesselName.Replace(" ", "_")+"-before_landing");
					compute_landing_trajectory();
				}
				goto case Multiplexer.Command.Resume;

			case Multiplexer.Command.Off:
				UnregisterFrom<Radar>();
				reset();
				break;
			}
		}

		void deorbit()
		{
			CorrectionTimer.Reset();
			clear_nodes(); add_trajectory_node();
			CFG.AP1.On(Autopilot1.Maneuver); 
			stage = Stage.Deorbit; 
		}

		protected override void UpdateState()
		{
			base.UpdateState();
			IsActive &= CFG.AP2[Autopilot2.Deorbit];
			ControlsActive &= IsActive || 
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
					if(check_initial_trajectory()) deorbit();
					else stage = Stage.Wait;
				}
				else 
				{
					currentEcc -= DEO.dEcc;
					if(currentEcc > 1e-10) 
						compute_landing_trajectory();
				}
				break;
			case Stage.Wait:
				VSL.Info.CustomMarkersWP.Add(trajectory.SurfacePoint);
				if(!string.IsNullOrEmpty(TCAGui.StatusMessage)) break;
				deorbit();
				break;
			case Stage.Deorbit:
				Status("Executing deorbit burn...");
				VSL.Info.CustomMarkersWP.Add(trajectory.SurfacePoint);
				if(CFG.AP1[Autopilot1.Maneuver]) break;
				fine_tune_approach();
				break;
			case Stage.Correct:
				if(!trajectory_computed()) break;
				add_correction_node_if_needed();
				stage = Stage.Coast; 
				break;
			case Stage.Coast:
				update_trajectory();
				VSL.Info.Countdown = trajectory.BrakeStartUT-VSL.Physics.UT-TRJ.ManeuverOffset;
				if(CFG.AP1[Autopilot1.Maneuver]) 
				{ 
					if(VSL.Info.Countdown > 0)
					{
						Status("Correcting trajectory..."); 
						break; 
					}
				}
				Status("Coasting...");
				if(VSL.Info.Countdown > 0 && !correct_trajectory()) break;
				CFG.AP1.OffIfOn(Autopilot1.Maneuver);
				stage = Stage.None;
				start_landing();
				break;
			}
		}

		public override void Draw()
		{
			#if DEBUG
			DrawDebugLines();
			#endif
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

