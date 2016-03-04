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
	public class LandingTrajectory
	{
		//for iterative search
		public readonly VesselWrapper VSL;
		public readonly CelestialBody Body;
		public readonly Orbit OrigOrbit;
		public readonly Vector3d ManeuverDeltaV;
		public readonly double TimeToStart;
		public readonly double ManeuverDuration;
		public readonly double TargetAlt;

		public Orbit LandingOrbit { get; private set; }
		public double StartUT { get; private set; }
		public double FreeFallTime { get; private set; }
		public double AtSurfaceUT { get; private set; }
		public double TrueAnomaly { get; private set; }
		public Quaternion CBRotation { get; private set; }
		public WayPoint SurfacePoint { get; private set; }
		public Vector3d RelSurfacePosition { get; private set; }

		public double DistanceToTarget = -1;
		public double DeltaLon { get; private set; }
		public double DeltaLat = 90;

		//for choosen trajectory
		public Vector3d BrakeDeltaV { get; private set; }
		public double BrakeDuration { get; private set; }
		public double BrakeStartUT { get; private set; }
		public double TimeToSurface { get { return AtSurfaceUT-VSL.Physics.UT; } }

		public LandingTrajectory(VesselWrapper vsl, Vector3d dV, double startUT, double alt = 0)
		{
			VSL = vsl;
			TargetAlt = alt;
			ManeuverDeltaV = dV;
			ManeuverDuration = ManeuverAutopilot.TTB(VSL, (float)ManeuverDeltaV.magnitude, 1);
			StartUT = startUT;
			TimeToStart = startUT-VSL.Physics.UT;
			Body = VSL.vessel.orbitDriver.orbit.referenceBody;
			OrigOrbit = VSL.vessel.orbitDriver.orbit;
			update_from_orbit(LandingTrajectoryCalculator.NewOrbit(OrigOrbit, ManeuverDeltaV, StartUT));
		}

		void update_from_orbit(Orbit orb)
		{
			LandingOrbit = orb;
//			DebugUtils.logOrbit("Orig", OrigOrbit);//debug
//			DebugUtils.logOrbit("Land", LandingOrbit);//debug
			var tA0 = OrigOrbit.radius > Body.Radius+TargetAlt? 
				LandingOrbit.trueAnomaly*Mathf.Deg2Rad : Math.PI;
			TrueAnomaly = LandingOrbit.TrueAnomalyAtRadius(Body.Radius+TargetAlt);
			if(Utils.RadDelta(tA0, TrueAnomaly) < 0) TrueAnomaly = Utils.TwoPI-TrueAnomaly;
			FreeFallTime = LandingOrbit.GetDTforTrueAnomaly(TrueAnomaly, LandingOrbit.period/2);
			AtSurfaceUT = StartUT+FreeFallTime;
			CBRotation = Quaternion.AngleAxis((float)(TimeToSurface/Body.rotationPeriod*360), 
			                                  Body.angularVelocity);
			RelSurfacePosition = CBRotation.Inverse()*orb.getRelativePositionAtUT(AtSurfaceUT).xzy;
			SurfacePoint = new WayPoint(RelSurfacePosition+Body.position, Body);
		}

		public Vector3d GetOrbitVelocityAtSurface()
		{ return LandingOrbit.getOrbitalVelocityAtUT(AtSurfaceUT); }

		public void UpdateOrbit(Orbit current)
		{
			StartUT = VSL.Physics.UT;
			update_from_orbit(current);
			BrakeDeltaV = -LandingOrbit.getOrbitalVelocityAtUT(AtSurfaceUT);
			BrakeDuration = ManeuverAutopilot.TTB(VSL, (float)BrakeDeltaV.magnitude, 1);
			BrakeStartUT = AtSurfaceUT-BrakeDuration;
		}

		public void UpdateDistance(WayPoint target)
		{
			DistanceToTarget = target.AngleTo(SurfacePoint)*Body.Radius;
			DeltaLon = Utils.AngleDelta(SurfacePoint.Lon, target.Lon)*
				Math.Sign(Utils.AngleDelta(Utils.ClampAngle(VSL.vessel.longitude), SurfacePoint.Lon));
			DeltaLat = 90-Vector3d.Angle(LandingOrbit.GetOrbitNormal().xzy, 
			                             CBRotation * Body.GetRelSurfacePosition(target.Lat, target.Lon, 0));
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
			                     "Node in: {3} s, TTB: {4} s, FreeFallTime: {5} s\n" +
			                     "Distance to Target: {6} m, Delta Lat: {7} deg, Delta Lon: {8} deg",
			                     TrueAnomaly*Mathf.Rad2Deg, SurfacePoint, ManeuverDeltaV, 
			                     TimeToStart, ManeuverDuration, FreeFallTime,
			                     DistanceToTarget, 
			                     DeltaLat, DeltaLon);
		}
	}
}

