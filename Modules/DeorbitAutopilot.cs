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

namespace ThrottleControlledAvionics
{
//	[CareerPart]
	[RequireModules(typeof(ManeuverAutopilot))]
	[OptionalModules(typeof(PointNavigator))]
	public class DeorbitAutopilot : TCAModule
	{
		public class Config : TCAModule.ModuleConfig
		{
			[Persistent] public float StartOffset        = 60f;   //s
			[Persistent] public float StartPeR           = 0.5f;  //of planet radius
			[Persistent] public float dVtol              = 0.01f; //m/s
			[Persistent] public float Dtol               = 1000f; //m
			[Persistent] public float FlyOverAlt         = 1000;  //m
			[Persistent] public int   MaxIterations      = 100;
			[Persistent] public int   PerFrameIterations = 10;

		}
		static Config DEO { get { return TCAScenario.Globals.DEO; } }

		public DeorbitAutopilot(ModuleTCA tca) : base(tca) {}

		Orbit VesselOrbit { get { return VSL.vessel.orbitDriver.orbit; } }
		CelestialBody Body { get { return VSL.vessel.orbitDriver.orbit.referenceBody; } }

		public static Orbit NewOrbit(Orbit old, Vector3d dV, double UT)
		{
			var obt = new Orbit();
			var pos = old.getRelativePositionAtUT(UT);
			var vel = old.getOrbitalVelocityAtUT(UT)+dV;
			obt.UpdateFromStateVectors(pos, vel, old.referenceBody, UT);
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
			while(max_dV-min_dV > DEO.dVtol)
			{
				var dV = (max_dV+min_dV)/2;
				var nPe = NewOrbit(old, velN*dV+add_dV, UT).PeR;
				if(up && nPe > PeR || !up && nPe < PeR) max_dV = dV;
				else min_dV = dV;
			}
			return (max_dV+min_dV)/2*velN+add_dV;
		}

		public static Orbit NewPeR(Orbit old, double PeR, double UT)
		{
			var dV = dV4Pe(old, PeR, UT);
			return NewOrbit(old, dV, UT);
		}

		public static void AddNode(VesselWrapper VSL, Vector3d dV, double UT)
		{
			var node = VSL.vessel.patchedConicSolver.AddManeuverNode(UT);
			var norm = VSL.orbit.GetOrbitNormal().normalized;
			var prograde = VSL.orbit.getOrbitalVelocityAtUT(UT).normalized;
			var radial = Vector3d.Cross(prograde, norm).normalized;
			node.DeltaV = new Vector3d(Vector3d.Dot(dV, radial),
			                           Vector3d.Dot(dV, norm),
			                           Vector3d.Dot(dV, prograde));
			VSL.vessel.patchedConicSolver.UpdateFlightPlan();
		}

		protected override void UpdateState()
		{
			base.UpdateState();
			IsActive = VSL.InOrbit && VSL.HasTarget;
		}

		IEnumerator<LandingTrajectory> compute_landing_trajectory(WayPoint target, double PeR)
		{
			var targetAlt = target.SurfaceAlt(Body)+DEO.FlyOverAlt;
			var StartUT = VSL.Physics.UT+DEO.StartOffset;
			LandingTrajectory site = null;
			LandingTrajectory best = null;
			Vector3d dVn = Vector3d.zero;
			var maxI = DEO.MaxIterations;
			var frameI = DEO.PerFrameIterations;
			do {
				if(site != null) 
				{
					if(Math.Abs(site.DeltaLon) > Math.Abs(site.LandingTargetInclination))
					{
						StartUT = site.StartUT+site.DeltaLon/360*VesselOrbit.period;
						if(StartUT-VSL.Physics.UT < DEO.StartOffset) StartUT += VesselOrbit.period;
						if(site.ManeuverDuration > StartUT-VSL.Physics.UT) StartUT = VSL.Physics.UT+site.ManeuverDuration+DEO.StartOffset;
					}
					else dVn += VesselOrbit.GetOrbitNormal().normalized*
							VesselOrbit.vel.magnitude*Math.Sin(site.LandingTargetInclination*Mathf.Deg2Rad/2);
				}
				site = new LandingTrajectory(VSL, dV4Pe(VesselOrbit, Body.Radius*PeR, StartUT, dVn), StartUT, targetAlt);
				var targetPos = site.CBRotation * Body.GetRelSurfacePosition(target.Lat, target.Lon, 0);;
				site.UpdateDeltaLon(target.Lon);
				site.DistanceToTarget = target.AngleTo(site.SurfacePoint)*Body.Radius;
				site.LandingTargetInclination = 90-Vector3d.Angle(site.LandingOrbit.GetOrbitNormal().xzy, targetPos);
				if(best == null || site < best) best = site;
				frameI--;
				maxI--;
				if(frameI < 0)
				{
					yield return null;
					frameI = DEO.PerFrameIterations;
				}
			} while(best.DistanceToTarget > DEO.Dtol && maxI > 0);
			yield return best;
		}

		double CurrentPeR;
		bool compute_node;
		LandingTrajectory trajectory;
		IEnumerator<LandingTrajectory> trajectory_calculator;
		bool trajectory_computed(WayPoint target, double PeR)
		{
			if(trajectory != null) return true;
			if(trajectory_calculator == null)
				trajectory_calculator = compute_landing_trajectory(target, PeR);
			if(trajectory_calculator.MoveNext())
				trajectory = trajectory_calculator.Current;
			else trajectory_calculator = null;
			return trajectory != null;
		}

		void reset()
		{
			trajectory = null;
			trajectory_calculator = null;
			CurrentPeR = DEO.StartPeR;
		}

		void start()
		{
			reset();
			var target = Target2WP();
			if(CFG.Target != target) 
			{
				target.UpdateCoordinates(Body);
				CFG.Target = target;
			}
			compute_node = true;
		}

		protected override void Update()
		{
			if(!IsActive || CFG.Target == null || VSL.orbit.referenceBody == null) return;
			if(!compute_node) return;
			if(trajectory_computed(CFG.Target, CurrentPeR))
			{
				if(trajectory.DistanceToTarget < DEO.Dtol || CurrentPeR >= 1)
				{
					compute_node = false;
					AddNode(VSL, trajectory.ManeuverDeltaV, trajectory.StartUT);
					CFG.ShowWaypoints = true;
					CFG.GUIVisible = true;
					MapView.EnterMapView();
					ThrottleControlledAvionics.DebugMessage = string.Format("{0}", trajectory);//debug
				}
				else 
				{
					CurrentPeR += 0.1;
					if(CurrentPeR < 1) trajectory = null;
				}
			}
		}

		public override void Draw()
		{
			if(compute_node) GUILayout.Label("Computing...", Styles.grey_button, GUILayout.ExpandWidth(false));
			else if(GUILayout.Button(new GUIContent("Deorbit", "Compute deorbit maneuver to land at target."), 
			                    Styles.yellow_button, 
			                    GUILayout.ExpandWidth(false)))
				start(); //TODO: convert to a standard CFG routine
		}
	}


	public class LandingTrajectory
	{
		//for iterative search
		public readonly VesselWrapper VSL;
		public readonly Orbit OrigOrbit;
		public readonly Vector3d ManeuverDeltaV;
		public readonly double StartUT;
		public readonly double TimeToStart;
		public readonly double ManeuverDuration;
		public readonly double TargetAlt;

		public Orbit LandingOrbit { get; private set; }
		public double FreeFallTime { get; private set; }
		public double AtSurfaceUT { get; private set; }
		public double TrueAnomaly { get; private set; }
		public Quaternion CBRotation { get; private set; }
		public WayPoint SurfacePoint { get; private set; }

		public double DistanceToTarget = -1;
		public double DeltaLon { get; private set; }
		public double LandingTargetInclination = 90;

		//for choosen trajectory
		public Vector3d BrakeDeltaV { get; private set; }
		public double BrakeDuration { get; private set; }
		public double TimeToSurface { get { return AtSurfaceUT-VSL.Physics.UT; } }

		public LandingTrajectory(VesselWrapper vsl, Vector3d dV, double startUT, double alt = 0)
		{
			VSL = vsl;
			TargetAlt = alt;
			ManeuverDeltaV = dV;
			ManeuverDuration = ManeuverAutopilot.TTB(VSL, (float)ManeuverDeltaV.magnitude, 1);
			StartUT = startUT;
			TimeToStart = startUT-VSL.Physics.UT;
			OrigOrbit = VSL.vessel.orbitDriver.orbit;
			update_from_orbit(DeorbitAutopilot.NewOrbit(OrigOrbit, ManeuverDeltaV, StartUT), StartUT);
		}

		void update_from_orbit(Orbit orb, double UT)
		{
			LandingOrbit = orb;
			var tA0 = LandingOrbit.TrueAnomalyAtUT(UT);
			TrueAnomaly = LandingOrbit.TrueAnomalyAtRadius(LandingOrbit.referenceBody.Radius+TargetAlt);
			if(tA0 > TrueAnomaly) TrueAnomaly = Utils.TwoPI-TrueAnomaly;
			FreeFallTime = LandingOrbit.GetDTforTrueAnomaly(TrueAnomaly, LandingOrbit.period/2);
			AtSurfaceUT = UT+FreeFallTime;
			CBRotation = Quaternion.AngleAxis((float)(TimeToSurface/orb.referenceBody.rotationPeriod*360), 
			                                  orb.referenceBody.angularVelocity);
			var pos = (CBRotation.Inverse()*orb.getRelativePositionAtUT(AtSurfaceUT).xzy)+orb.referenceBody.position;
			SurfacePoint = new WayPoint(orb.referenceBody.GetLatitude(pos), orb.referenceBody.GetLongitude(pos));
		}

		public Vector3d GetRelSurfacePosition()
		{ return LandingOrbit.referenceBody.GetRelSurfacePosition(SurfacePoint.Lat, SurfacePoint.Lon, 0); }

		public Vector3d GetOrbitVelocityAtSurface()
		{ return LandingOrbit.getOrbitalVelocityAtUT(AtSurfaceUT); }

		public void UpdateDeltaLon(double Lon)
		{
			var dLon = Utils.CenterAngle(Lon)-Utils.CenterAngle(SurfacePoint.Lon);
			DeltaLon = Math.Abs(dLon) > 180? Math.Sign(dLon)*(360-Math.Abs(dLon)) : dLon;
		}

		public void UpdateOrbit(Orbit current)
		{
			update_from_orbit(current, VSL.Physics.UT);
			BrakeDeltaV = LandingOrbit.getOrbitalVelocityAtUT(AtSurfaceUT);
			BrakeDuration = ManeuverAutopilot.TTB(VSL, (float)BrakeDeltaV.magnitude, 1);
		}

		public static bool operator <(LandingTrajectory s1, LandingTrajectory s2)
		{ 
			return s2.DistanceToTarget < 0 || 
				s1.DistanceToTarget > 0 && s1.DistanceToTarget < s2.DistanceToTarget;
		}

		public static bool operator >(LandingTrajectory s1, LandingTrajectory s2)
		{ 
			return s1.DistanceToTarget < 0 || 
				s2.DistanceToTarget > 0 && s1.DistanceToTarget > s2.DistanceToTarget;
		}

		public override string ToString()
		{
			return string.Format("LandingSite:\nTrueAnomaly: {0} deg,\nCoordinates: {1},\n" +
		                         "DeltaV: {2}\n"+
		                         "Node in: {3} s, TTB: {4} s, TimeToSurface: {5} s\n" +
		                         "Distance to Target: {6} m, AngleToTarget: {7} deg, Delta Lon: {8} deg",
		                         TrueAnomaly*Mathf.Rad2Deg, SurfacePoint, ManeuverDeltaV, 
		                         TimeToStart, ManeuverDuration, FreeFallTime,
		                         DistanceToTarget, 
		                         LandingTargetInclination, DeltaLon);
		}
	}
}

