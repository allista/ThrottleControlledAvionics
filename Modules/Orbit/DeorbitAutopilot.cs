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
	public class DeorbitAutopilot : LandingTrajectoryAutopilot
	{
		public new class Config : TCAModule.ModuleConfig
		{
			[Persistent] public float MinLandingAngle    = 20f;
			[Persistent] public float MaxDynPressure     = 7f;
			[Persistent] public int   EccSteps           = 10;
		}
		static Config DEO { get { return Globals.Instance.DEO; } }

		public DeorbitAutopilot(ModuleTCA tca) : base(tca) {}

		public override void Init()
		{
			base.Init();
			CFG.AP2.AddHandler(this, Autopilot2.Deorbit);
		}

		public enum Stage { None, Precalculate, Compute, Deorbit, Correct, Coast, Wait }
		[Persistent] public Stage stage;
		double currentEcc;
		double dEcc;


		Vector3d PlaneCorrection(TargetedTrajectoryBase old)
		{
			var angle = old.DeltaFi;
			angle *= Math.Sin(old.TransferTime/old.Orbit.period*2*Math.PI);
			angle *= Math.Sign(Utils.ProjectionAngle(old.StartPos, old.AtTargetPos, old.StartVel));
			var rot = QuaternionD.AngleAxis(angle, old.StartPos);
			return Orbit2NodeDeltaV((rot*old.StartVel)-old.StartVel, old.StartUT);
		}

        double ProgradeCorrection(LandingTrajectory old)
        {
            return old.DeltaR *
                Utils.ClampH(old.Orbit.period/old.TimeToTarget*Body.GeeASL, 1);
        }

		protected LandingTrajectory fixed_Ecc_orbit(LandingTrajectory old, LandingTrajectory best, ref Vector3d NodeDeltaV, double ecc)
		{
			double StartUT;
			double targetAlt;
			if(old != null) 
			{
				targetAlt = old.TargetAltitude;
                var deltaAngle = Math.Abs(90-VesselOrbit.inclination) > 45 ? old.DeltaLon : old.DeltaLat;
                var inv_target_lat = 90-Math.Abs(Coordinates.NormalizeLatitude(old.Target.Pos.Lat));
                var target_at_pole = inv_target_lat < 5;
                deltaAngle *= inv_target_lat/90;
                //if target is at a pole, just use its longitude as start UT
                if(target_at_pole)
                    StartUT = VSL.Physics.UT+
                        Math.Max(Utils.ClampAngle(Utils.AngleDelta(old.Target.Pos.Lon, VSL.vessel.longitude))/360*VesselOrbit.period,
                                 ManeuverOffset);
                //otherwise correct start UT according to lat-lon delta between target and landing site
                else 
                    StartUT = AngleDelta2StartUT(old, deltaAngle, 
                                                 ManeuverOffset, VesselOrbit.period, VesselOrbit.period);
                //if start UT is good enough, correct orbital plane and landing site
                if(target_at_pole || Math.Abs(deltaAngle) < 1)
                {
                    NodeDeltaV += PlaneCorrection(old);
                    //correct fly-over altitude if needed
    				if(old.BrakeEndDeltaAlt < LTRJ.FlyOverAlt)
                        NodeDeltaV += new Vector3d((LTRJ.FlyOverAlt-old.BrakeEndDeltaAlt)*Body.GeeASL/100, 0, 0);
                    //if close enough, start tuning deltaR
                    if(Math.Abs(old.DeltaFi) < 1)
                        NodeDeltaV += Orbit2NodeDeltaV(old.StartVel.normalized * ProgradeCorrection(old), old.StartUT);
                }
//                Log("target_at_pole {}, inv_target_lat {}/{}/{}, deltaAngle {}, deltaFi {}, vsl-target-deltaLon {}, StartT {}", 
//                    target_at_pole, inv_target_lat, Coordinates.NormalizeLatitude(old.Target.Pos.Lat), old.Target.Pos.Lat,
//                    deltaAngle, old.DeltaFi, 
//                    Utils.ClampAngle(Utils.AngleDelta(old.Target.Pos.Lon, VSL.vessel.longitude)),
//                    StartUT-VSL.Physics.UT);//debug
			}
			else 
			{
				StartUT = VSL.Physics.UT+ManeuverOffset;
				targetAlt = TargetAltitude;
			}
			return new LandingTrajectory(VSL, 
			                             dV4Ecc(VesselOrbit, ecc, StartUT, Body.Radius+targetAlt-10)+
			                             Node2OrbitDeltaV(NodeDeltaV, StartUT), 
                                         StartUT, CFG.Target, targetAlt);
		}

		protected LandingTrajectory horizontal_correction(LandingTrajectory old, LandingTrajectory best, ref Vector3d NodeDeltaV, double start_offset)
		{
			var StartUT = VSL.Physics.UT+start_offset;
			if(old != null) 
			{
				if(Math.Abs(old.DeltaR) > Math.Abs(old.DeltaFi)) 
                    NodeDeltaV += new Vector3d(0, 0, ProgradeCorrection(old));
				else 
					NodeDeltaV += PlaneCorrection(old);
			}
			return new LandingTrajectory(VSL, Node2OrbitDeltaV(NodeDeltaV, StartUT), 
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
			setup_calculation((o, b) => horizontal_correction(o, b, ref NodeDeltaV, Math.Max(CorrectionOffset, VSL.Torque.NoEngines.TurnTime)));
			stage = Stage.Correct;
		}

		protected override void reset()
		{
			base.reset();
			stage = Stage.None;
			CFG.AP1.Off();
		}

		IEnumerator<YieldInstruction> eccentricity_calculator = null;
		IEnumerator<YieldInstruction> compute_initial_eccentricity()
		{
//			Log("Calculating initial orbit eccentricity...");//debug
			var tPos = CFG.Target.RelOrbPos(Body);
			var UT = VSL.Physics.UT +
				AngleDelta(VesselOrbit, tPos, VSL.Physics.UT)/360*VesselOrbit.period;
			var vPos = VesselOrbit.getRelativePositionAtUT(UT);
			var vVel = VesselOrbit.getOrbitalVelocityAtUT(UT);
			var dir = Vector3d.Exclude(vPos, vVel);
            var ini_dV = dV4Pe(VesselOrbit, (Body.Radius+TargetAltitude)*0.999, UT);
			var trj = new LandingTrajectory(VSL, ini_dV, UT, CFG.Target, TargetAltitude);
			var atmo_curve = trj.GetAtmosphericCurve(5);
			var maxV = vVel.magnitude;
			var minV = 0.0;
			var dV = 0.0;
			dir = -dir.normalized;
			var in_plane = Math.Abs(90-Vector3.Angle(tPos, VesselOrbit.GetOrbitNormal())) < 5;
//			Log("in plane {}, ini dV {}\nini trj:\n{}", in_plane, ini_dV, trj);//debug
			yield return null;
			while(maxV-minV > 1)
			{
				dV = (maxV+minV)/2;
				trj = new LandingTrajectory(VSL, ini_dV+dir*dV, UT, CFG.Target, trj.TargetAltitude);
				atmo_curve = trj.GetAtmosphericCurve(5);
//				Log("dV: {} : {} : {} m/s\ntrj:\n{}", minV, dV, maxV, trj);//debug
				if(!trj.NotEnoughFuel && (in_plane || trj.DeltaR < 0) &&
				   (atmo_curve != null &&
				    (will_overheat(atmo_curve) ||
				     atmo_curve[atmo_curve.Count-1].DynamicPressure > DEO.MaxDynPressure) ||
				    !Body.atmosphere && trj.LandingAngle < DEO.MinLandingAngle))
					minV = dV;
				else maxV = dV;
				yield return null;
			}
			currentEcc = trj.Orbit.eccentricity;
			dEcc = currentEcc/DEO.EccSteps;
//			Log("currentEcc: {}, dEcc {}", currentEcc, dEcc);//debug
		}

		public void DeorbitCallback(Multiplexer.Command cmd)
		{
			switch(cmd)
			{
			case Multiplexer.Command.Resume:
				if(!check_patched_conics()) return;
				UseTarget();
				NeedRadarWhenMooving();
				if(trajectory == null) update_trajectory();
				if(stage == Stage.None && !landing) 
					goto case Multiplexer.Command.On;
				if(VSL.HasManeuverNode)
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
					eccentricity_calculator = compute_initial_eccentricity();
					stage = Stage.Precalculate;
				}
				goto case Multiplexer.Command.Resume;

			case Multiplexer.Command.Off:
				UnregisterFrom<Radar>();
                StopUsingTarget();
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
				!VSL.LandedOrSplashed && (VSL.TargetIsWayPoint || VSL.TargetVessel != null && VSL.TargetVessel.LandedOrSplashed);
		}

		protected override void Update()
		{
			if(!IsActive) { CFG.AP2.OffIfOn(Autopilot2.Deorbit); return; }
			if(landing) { do_land(); return; }
			switch(stage)
			{
			case Stage.Precalculate:
				Status("Computing trajectory...");
				if(eccentricity_calculator != null &&
				   eccentricity_calculator.MoveNext()) break;
				eccentricity_calculator = null;
				compute_landing_trajectory();
				break;
			case Stage.Compute:
				if(!trajectory_computed()) break;
				if(trajectory.DistanceToTarget < LTRJ.Dtol || currentEcc < 1e-10)
				{
					if(check_initial_trajectory()) 
					{
						SaveGame("before_landing");
						deorbit();
					}
					else 
                    {
                        stage = Stage.Wait;
                        #if DEBUG
                        PauseMenu.Display();
                        #endif
                    }
				}
				else 
				{
                    currentEcc -= dEcc;
                    if(currentEcc > 1e-10 && trajectory.Orbit.PeR < Body.Radius) 
						compute_landing_trajectory();
                    #if DEBUG
                    setp_by_step_computation = true;
                    #endif
				}
				break;
			case Stage.Wait:
				VSL.Info.CustomMarkersWP.Add(trajectory.SurfacePoint);
				if(!string.IsNullOrEmpty(TCAGui.StatusMessage)) break;
				SaveGame("before_landing");
				deorbit();
				break;
			case Stage.Deorbit:
				Status("Executing deorbit burn...");
				VSL.Info.CustomMarkersWP.Add(trajectory.SurfacePoint);
				if(CFG.AP1[Autopilot1.Maneuver]) break;
				#if DEBUG
				PauseMenu.Display();
				#endif
				fine_tune_approach();
				break;
			case Stage.Correct:
				if(!trajectory_computed()) break;
				if(!will_overheat(trajectory.GetAtmosphericCurve(5)))
					add_correction_node_if_needed();
				stage = Stage.Coast; 
				break;
			case Stage.Coast:
				update_trajectory();
				VSL.Info.Countdown = trajectory.BrakeStartUT-VSL.Physics.UT-ManeuverOffset;
				if(CFG.AP1[Autopilot1.Maneuver]) 
				{ 
					if(VSL.Info.Countdown > 0 ||
					   trajectory.BrakeStartUT-Math.Max(MAN.NodeUT, VSL.Physics.UT)-VSL.Info.TTB -
                       VSL.Torque.NoEngines.RotationTime2Phase(Vector3.Angle(VesselOrbit.vel, MAN.NodeDeltaV)) > CorrectionOffset)
					{
						Status("Correcting trajectory..."); 
						break; 
					}
				}
				Status("Coasting...");
				WRP.NoDewarpOffset = true;
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
					                    Styles.inactive_button, GUILayout.ExpandWidth(true)))
						CFG.AP2.XOff();
				}
				else if(Utils.ButtonSwitch("Land", CFG.AP2[Autopilot2.Deorbit],
				                           "Compute and perform a deorbit maneuver, then land near the target.", 
				                           GUILayout.ExpandWidth(true)))
					CFG.AP2.XToggle(Autopilot2.Deorbit);
			}
			else GUILayout.Label(new GUIContent("Land", "Compute and perform a deorbit maneuver, then land near the target."), 
			                     Styles.inactive_button, GUILayout.ExpandWidth(true));
		}

        #if DEBUG
        RealTimer delay = new RealTimer(5);
        public void Test(System.Random rnd = null)
        {
            VSL.Info.AddCustopWaypoint(new Coordinates(0,0,0),   "Zero");
            VSL.Info.AddCustopWaypoint(new Coordinates(90,0,0),  "North");
            VSL.Info.AddCustopWaypoint(new Coordinates(-90,0,0), "South");
            VSL.Info.AddCustopWaypoint(new Coordinates(0,90,0),  "90 deg");
            VSL.Info.AddCustopWaypoint(new Coordinates(0,180,0), "180 deg");
            VSL.Info.AddCustopWaypoint(new Coordinates(0,270,0), "270 deg");
            if(CFG.Target == null)
            {
                var c = Coordinates.SurfacePoint(rnd.NextDouble()*180-90, //30+60, //5+85, 
                                                 rnd.NextDouble()*360, 
                                                 Body);
                SetTarget(new WayPoint(c));
                CFG.AP2.XOnIfNot(Autopilot2.Deorbit);
            }
            if(CFG.AP2[Autopilot2.Deorbit] &&
               !CFG.AP1[Autopilot1.Maneuver]) 
            {
                delay.Reset();
                return;
            }
            CFG.AP2.XOff();
            if(!delay.TimePassed) return;
            CFG.Target = null;
        }
        #endif
	}
}

