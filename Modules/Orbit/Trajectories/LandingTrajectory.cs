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
		public QuaternionD AtSurfaceRotation { get; private set; }
		public QuaternionD AtStartRotation { get; private set; }
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

		Vector3d brake_delta_v;
		public override Vector3d BrakeDeltaV { get { return brake_delta_v; } }
		public double BrakeDuration { get; private set; }
		public double BrakeStartUT { get; private set; }
		public double BrakeNodeUT { get; private set; }
		public double TimeToSurface { get { return AtTargetUT-VSL.Physics.UT; } }

		public LandingTrajectory(VesselWrapper vsl, Vector3d dV, double startUT, 
		                         WayPoint target, double target_altitude = 0, bool with_brake = true)
			: base(vsl, dV, startUT, target)
		{
			TargetAltitude = target_altitude;
			update(with_brake);
		}

		QuaternionD body_rotation_at_dT(double dT)
		{ return QuaternionD.AngleAxis((dT/Body.rotationPeriod*360), Body.angularVelocity.normalized); }

		void update_from_orbit(Orbit orb, double UT)
		{
			//calculate the position of a landing site
			AtTargetUT   = TrajectoryCalculator.NearestRadiusUT(orb, Body.Radius+TargetAltitude, UT);
			TimeToTarget = AtTargetUT-UT;
			AtSurfaceRotation = body_rotation_at_dT(TimeToSurface);
			AtStartRotation = body_rotation_at_dT(TimeToStart);
			AtTargetPos = orb.getRelativePositionAtUT(AtTargetUT);
			AtTargetVel = orb.getOrbitalVelocityAtUT(AtTargetUT);
			SurfacePoint = new WayPoint(QuaternionD.Inverse(AtSurfaceRotation)*AtTargetPos.xzy+Body.position, Body);
		}

		void update(bool with_brake = true)
		{
			update_from_orbit(NewOrbit, StartUT);
			//correct for brake maneuver
			if(with_brake)
			{
				BrakeDuration = VSL.Engines.TTB((float)AtTargetVel.magnitude);
				BrakeStartUT  = AtTargetUT-BrakeDuration;
				BrakeNodeUT   = AtTargetUT-BrakeDuration/2;
				brake_delta_v = -(Vector3d.Exclude(NewOrbit.getRelativePositionAtUT(BrakeNodeUT),
				                                   NewOrbit.getOrbitalVelocityAtUT(BrakeNodeUT))*0.9+
				                  Vector3d.Cross(Body.angularVelocity.xzy, AtTargetPos));
				update_from_orbit(TrajectoryCalculator.NewOrbit(NewOrbit, BrakeDeltaV, BrakeNodeUT), BrakeNodeUT);
			}
			else
			{
				brake_delta_v = -(AtTargetVel+
				                  Vector3d.Cross(Body.angularVelocity.xzy, AtTargetPos));
				BrakeDuration = VSL.Engines.TTB((float)BrakeDeltaV.magnitude);
				BrakeStartUT  = AtTargetUT-BrakeDuration;
				BrakeNodeUT   = AtTargetUT-BrakeDuration/2;
			}
			//compute vessel coordinates at maneuver start
			if(VSL.LandedOrSplashed)
			{
				VslStartLat = Utils.ClampAngle(VSL.vessel.latitude);
				VslStartLon = Utils.ClampAngle(VSL.vessel.longitude);
			}
			else
			{
				var start_pos = QuaternionD.Inverse(AtStartRotation)*StartPos.xzy + Body.position;
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
			DeltaFi = 90-Vector3d.Angle(Vector3d.Cross(StartPos, AtTargetPos).xzy,
			                            AtSurfaceRotation * Body.GetRelSurfacePosition(Target.Lat, Target.Lon, TargetAltitude));
			DeltaR = Utils.RadDelta(SurfacePoint.AngleTo(VslStartLat, VslStartLon), Target.AngleTo(VslStartLat, VslStartLon))*Mathf.Rad2Deg;

//			Utils.Log("{0}", this);//debug
		}

		public Vector3d GetOrbitVelocityAtSurface()
		{ return NewOrbit.getOrbitalVelocityAtUT(AtTargetUT); }

		public override void UpdateOrbit(Orbit current)
		{
			base.UpdateOrbit(current);
			update();
		}

		public void UpdateOrbit(Orbit current, bool with_brake)
		{
			base.UpdateOrbit(current);
			update(with_brake);
		}

		public override string ToString()
		{
			return base.ToString()+
				Utils.Format("\nCoordinates: {},\n" +
				             "TimeToSurface: {}\n" +
		                     "Delta R: {} deg\n" +
				             "Delta Lat: {} deg, Delta Lon: {} deg\n" +
				             "Brake DeltaV: {}\n" +
				             "Brake Duration: {}, Time to Brake: {}",
				             SurfacePoint, TimeToSurface,
				             DeltaR, DeltaLat, DeltaLon,
				             BrakeDeltaV, BrakeDuration, BrakeStartUT-VSL.Physics.UT);
		}
	}
}

