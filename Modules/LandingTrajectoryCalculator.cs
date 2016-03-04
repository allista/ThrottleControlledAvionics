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

namespace ThrottleControlledAvionics
{
	public abstract class LandingTrajectoryCalculator : TCAModule
	{
		public class Config : TCAModule.ModuleConfig
		{
			[Persistent] public float dVtol              = 0.01f; //m/s
			[Persistent] public float Dtol               = 1000f; //m
			[Persistent] public float FlyOverAlt         = 1000;  //m
			[Persistent] public float ApproachAlt        = 250;   //m
			[Persistent] public float BrakeEndSpeed      = 100;   //m/s
			[Persistent] public int   MaxIterations      = 1000;
			[Persistent] public int   PerFrameIterations = 10;
		}
		protected static Config TRJ { get { return TCAScenario.Globals.TRJ; } }

		protected LandingTrajectoryCalculator(ModuleTCA tca) : base(tca) {}

		protected ManeuverAutopilot MAN;

		protected Orbit VesselOrbit { get { return VSL.vessel.orbitDriver.orbit; } }
		protected CelestialBody Body { get { return VSL.vessel.orbitDriver.orbit.referenceBody; } }

		enum LandingStage { None, Start, Decelerate, Approach, Land }
		LandingStage landing_stage;

		protected WayPoint Target;
		protected double TargetAlt { get { return Target.SurfaceAlt(Body)+TRJ.FlyOverAlt; } }

		public static Orbit NewOrbit(Orbit old, Vector3d dV, double UT)
		{
			var obt = new Orbit();
			var pos = old.getRelativePositionAtUT(UT);
			var vel = old.getOrbitalVelocityAtUT(UT)+dV;
			obt.UpdateFromStateVectors(pos, vel, old.referenceBody, UT);
			return obt;
		}

		public static Orbit CopyOrbit(Orbit o)
		{
			return new Orbit(o.inclination, o.eccentricity, o.semiMajorAxis, o.LAN, 
			                 o.argumentOfPeriapsis, o.meanAnomalyAtEpoch, o.epoch, o.referenceBody);
		}

		public static Orbit LandedOrbit(VesselWrapper vsl)
		{
			var obt = new Orbit();
			var pos = vsl.vessel.GetWorldPos3D();
			obt.UpdateFromStateVectors(pos.xzy, 
			                           vsl.mainBody.getRFrmVel(pos).xzy,
			                           vsl.mainBody, vsl.Physics.UT);
			return obt;
		}

		public static Vector3d dV4Pe(Orbit old, double PeR, double UT, Vector3d add_dV = default(Vector3d))
		{
			var oldPeR = old.PeR;
			var vel = old.getOrbitalVelocityAtUT(UT);
			var up = oldPeR < PeR;
			var pg = up? 1 : -1;
			var velN = vel.normalized * pg;
			var min_dV = 0.0;
			var max_dV = 0.0;
			if(up)
			{
				max_dV = 1;
				while(NewOrbit(old, velN*max_dV, UT).PeR < PeR)
				{ max_dV *= 2; if(max_dV > 100000) break; }
			}
			else max_dV = vel.magnitude+add_dV.magnitude;
			while(max_dV-min_dV > TRJ.dVtol)
			{
				var dV = (max_dV+min_dV)/2;
				var nPe = NewOrbit(old, velN*dV+add_dV, UT).PeR;
				if(up && nPe > PeR || !up && nPe < PeR) max_dV = dV;
				else min_dV = dV;
			}
			return (max_dV+min_dV)/2*velN+add_dV;
		}

		protected bool landing { get { return landing_stage != LandingStage.None; } }

		protected void start_landing()
		{
			if(trajectory != null) trajectory.UpdateOrbit(VesselOrbit);
			else trajectory = new LandingTrajectory(VSL, Vector3d.zero, VSL.Physics.UT, TargetAlt);
			ManeuverAutopilot.AddNode(VSL, trajectory.BrakeDeltaV-trajectory.BrakeDeltaV.normalized*TRJ.BrakeEndSpeed, 
			                              trajectory.BrakeStartUT);
			landing_stage = LandingStage.Decelerate;
			CFG.AP1.On(Autopilot1.Maneuver);
			MAN.MinDeltaV = 1f;
		}

		protected bool do_land()
		{
			switch(landing_stage)
			{
			case LandingStage.Decelerate:
				if(VSL.Altitude.Relative < TRJ.FlyOverAlt)
					CFG.AP1.Off();
				if(CFG.AP1[Autopilot1.Maneuver]) break;
				CFG.AltitudeAboveTerrain = true;
				CFG.BlockThrottle = true;
				CFG.VF.On(VFlight.AltitudeControl);
				CFG.HF.On(HFlight.Level);
				CFG.DesiredAltitude = TRJ.ApproachAlt;
				landing_stage = LandingStage.Approach;
				break;
			case LandingStage.Approach:
				CFG.Nav.OnIfNot(Navigation.GoToTarget);
				if(CFG.Nav[Navigation.GoToTarget]) break;
				CFG.AP1.OnIfNot(Autopilot1.Land);
				landing_stage = LandingStage.Land;
				break;
			case LandingStage.Land:
				if(CFG.AP1[Autopilot1.Land]) break;
				CFG.AP2.Off();
				return true;
			}
			return false;
		}

		protected void clear_nodes()
		{
			VSL.vessel.patchedConicSolver.maneuverNodes.ForEach(n => n.RemoveSelf());
			VSL.vessel.patchedConicSolver.maneuverNodes.Clear();
			VSL.vessel.patchedConicSolver.flightPlan.Clear();
		}

		protected void setup_target()
		{
			Target = Target2WP();
			Target.UpdateCoordinates(Body);
			SetTarget(Target);
		}

		protected virtual void bootstrap_computation() {}
		protected abstract LandingTrajectory compute_next_trajectory(LandingTrajectory old);

		IEnumerator<LandingTrajectory> compute_landing_trajectory()
		{
//			Log("\nComputing trajectory for Target:\n{0}", Target);//debug
			LandingTrajectory site = null;
			LandingTrajectory best = null;
			var maxI = TRJ.MaxIterations;
			var frameI = TRJ.PerFrameIterations;
			bootstrap_computation();
			do {
				site = compute_next_trajectory(site);
				site.UpdateDistance(Target);
				if(best == null || site < best) best = site;
				frameI--;
				maxI--;
//				Log("Target: {0}\n{1}", Target, site);//debug
				if(frameI < 0)
				{
					yield return null;
					frameI = TRJ.PerFrameIterations;
				}
			} while(best.DistanceToTarget > TRJ.Dtol && maxI > 0);
			yield return best;
		}

		protected LandingTrajectory trajectory;
		IEnumerator<LandingTrajectory> trajectory_calculator;
		protected bool trajectory_computed()
		{
			if(trajectory != null) return true;
			if(trajectory_calculator == null)
				trajectory_calculator = compute_landing_trajectory();
			if(trajectory_calculator.MoveNext())
				trajectory = trajectory_calculator.Current;
			else trajectory_calculator = null;
			return trajectory != null;
		}

		protected virtual void reset()
		{
			Working = false;
			Target = null;
			trajectory = null;
			trajectory_calculator = null;
			landing_stage = LandingStage.None;
		}
	}
}

