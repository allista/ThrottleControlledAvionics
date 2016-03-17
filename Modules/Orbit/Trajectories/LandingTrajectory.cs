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
	public class LandingTrajectory : TargetedTrajectory<WayPoint>
	{
		public readonly double TargetAltitude;
		public Quaternion AtSurfaceRotation { get; private set; }
		public Quaternion AtStartRotation { get; private set; }
		public double SurfacePointTA { get; private set; }
		public WayPoint SurfacePoint { get; private set; }

		public double VslStartLat { get; private set; }
		public double VslStartLon { get; private set; }

		public double DeltaLat { get; private set; }
		public double DeltaLon { get; private set; }

		//distance in radial coordinates with the center at the vessel
		/// <summary>
		/// Radial difference between the target and the landing site in degrees.
		/// </summary>
		public double DeltaR { get; private set; } = 180;

		public Vector3d BrakeDeltaV { get; private set; }
		public double BrakeDuration { get; private set; }
		public double BrakeStartUT { get; private set; }
		public double TimeToSurface { get { return AtTargetUT-VSL.Physics.UT; } }

		public LandingTrajectory(VesselWrapper vsl, Vector3d dV, double startUT, 
			WayPoint target, double target_altitude = 0)
			: base(vsl, dV, startUT, target)
		{
			TargetAltitude = target_altitude;
			update();
		}

		Quaternion body_rotation_at_dT(double dT)
		{ return Quaternion.AngleAxis((float)(dT/Body.rotationPeriod*360), Body.angularVelocity); }

		void update()
		{
			//calculate the position of a landing site
			var tA0 = OrigOrbit.radius > Body.Radius+TargetAltitude? 
				NewOrbit.trueAnomaly*Mathf.Deg2Rad : Math.PI;
			SurfacePointTA = NewOrbit.TrueAnomalyAtRadius(Body.Radius+TargetAltitude);
			if(Utils.RadDelta(tA0, SurfacePointTA) < 0) SurfacePointTA = Utils.TwoPI-SurfacePointTA;
			TimeToTarget = NewOrbit.GetDTforTrueAnomaly(SurfacePointTA, NewOrbit.period/2);
			AtTargetUT   = StartUT+TimeToTarget;
			AtSurfaceRotation = body_rotation_at_dT(TimeToSurface);
			AtStartRotation   = body_rotation_at_dT(TimeToStart);
			AtTargetPos  = NewOrbit.getRelativePositionAtUT(AtTargetUT);
			AtTargetVel  = NewOrbit.getOrbitalVelocityAtUT(AtTargetUT);
			SurfacePoint = new WayPoint(AtSurfaceRotation.Inverse()*AtTargetPos.xzy+Body.position, Body);
			//brake maneuver
			BrakeDeltaV   = -AtTargetVel;
			BrakeDuration = ManeuverAutopilot.TTB(VSL, (float)BrakeDeltaV.magnitude, 1);
			BrakeStartUT  = AtTargetUT-BrakeDuration;
			//compute vessel coordinates at maneuver start
			if(VSL.LandedOrSplashed)
			{
				VslStartLat = Utils.ClampAngle(VSL.vessel.latitude);
				VslStartLon = Utils.ClampAngle(VSL.vessel.longitude);
			}
			else
			{
				var start_pos = AtStartRotation.Inverse()*StartPos.xzy + Body.position;
				VslStartLat = Utils.ClampAngle(Body.GetLatitude(start_pos));
				VslStartLon = Utils.ClampAngle(Body.GetLongitude(start_pos));
			}
			//compute distance to target
			DistanceToTarget = Target.AngleTo(SurfacePoint)*Body.Radius;
			//compute distance in lat-lon coordinates
			DeltaLat = Utils.AngleDelta(SurfacePoint.Lat, Target.Lat)*
				Math.Sign(Utils.AngleDelta(Utils.ClampAngle(VslStartLat), SurfacePoint.Lat));
			DeltaLon = Utils.AngleDelta(SurfacePoint.Lon, Target.Lon)*
				Math.Sign(Utils.AngleDelta(Utils.ClampAngle(VslStartLon), SurfacePoint.Lon));
			//compute distance in radial coordinates
			DeltaFi = 90-Vector3d.Angle(NewOrbit.GetOrbitNormal().xzy, 
			                            AtSurfaceRotation * Body.GetRelSurfacePosition(Target.Lat, Target.Lon, TargetAltitude));
			DeltaR = Utils.RadDelta(SurfacePoint.AngleTo(VslStartLat, VslStartLon), Target.AngleTo(VslStartLat, VslStartLon))*Mathf.Rad2Deg;

//			Utils.Log("Vessel: {0}\nAngleTo Target: {1}, AngleTo SurfacePoint: {2}", 
//			          new Coordinates(VslStartLat, VslStartLon),
//			          Target.AngleTo(VslStartLat, VslStartLon)*Mathf.Rad2Deg, 
//			          SurfacePoint.AngleTo(VslStartLat, VslStartLon)*Mathf.Rad2Deg);//debug
			Utils.Log("{0}", this);//debug
		}

		public Vector3d GetOrbitVelocityAtSurface()
		{ return NewOrbit.getOrbitalVelocityAtUT(AtTargetUT); }

		public override void UpdateOrbit(Orbit current)
		{
			base.UpdateOrbit(current);
			update();
		}

		public override string ToString()
		{
			return base.ToString()+
				string.Format("\nTrueAnomaly of SurfacePoint: {0}deg,\n" +
				              "Coordinates: {1},\n" +
				              "FreeFallTime: {2} s\n" +
		                      "Delta R: {3} deg\n" +
				              "Delta Lat: {4} deg, Delta Lon: {5} deg\n" +
				              "Brake DeltaV: {6}\n" +
				              "Brake Duration: {7}, Time to Brake: {6}",
		                      SurfacePointTA*Mathf.Rad2Deg, SurfacePoint,
				              TimeToTarget, DeltaR, DeltaLat, DeltaLon,
				              BrakeDeltaV, BrakeDuration, BrakeStartUT-VSL.Physics.UT);
		}
	}
}

