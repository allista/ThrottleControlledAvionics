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
			DeltaLon = Math.Abs(dLon) > 180? -Math.Sign(dLon)*(360-Math.Abs(dLon)) : dLon;
		}

		public void UpdateOrbit(Orbit current)
		{
			update_from_orbit(current, VSL.Physics.UT);
			BrakeDeltaV = -LandingOrbit.getOrbitalVelocityAtUT(AtSurfaceUT);
			BrakeDuration = ManeuverAutopilot.TTB(VSL, (float)BrakeDeltaV.magnitude, 1);
			BrakeStartUT = AtSurfaceUT-BrakeDuration;
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
			                     "Distance to Target: {6} m, Target Inclination: {7} deg, Delta Lon: {8} deg",
			                     TrueAnomaly*Mathf.Rad2Deg, SurfacePoint, ManeuverDeltaV, 
			                     TimeToStart, ManeuverDuration, FreeFallTime,
			                     DistanceToTarget, 
			                     LandingTargetInclination, DeltaLon);
		}
	}
}

