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
//	[CareerPart]
	[RequireModules(typeof(ManeuverAutopilot))]
	public class DeorbitAutopilot : TCAModule
	{
		public class Config : TCAModule.ModuleConfig
		{
			[Persistent] public float StartOffset = 60f;   //s
			[Persistent] public float PeR         = 0.9f;  //of planet radius
			[Persistent] public float dVtol       = 0.01f; //m/s
			[Persistent] public float Dtol        = 100f;  //m
		}
		static Config DEO { get { return TCAScenario.Globals.DEO; } }

		public DeorbitAutopilot(ModuleTCA tca) : base(tca) {}

		Orbit O { get { return VSL.vessel.orbitDriver.orbit; } }
		CelestialBody Body { get { return VSL.vessel.orbitDriver.orbit.referenceBody; } }

		ManeuverAutopilot MAN;

		double Start = -1f;
//		double dVr = 0f;
//		double dVn = 0f;

		static Orbit new_orbit(Orbit old, Vector3d dV, double UT)
		{
			var obt = new Orbit();
			var pos = old.getRelativePositionAtUT(UT);
			var vel = old.getOrbitalVelocityAtUT(UT)+dV;
			obt.UpdateFromStateVectors(pos, vel, old.referenceBody, UT);
			return obt;
		}

		static Vector3d dV4Pe(Orbit old, double PeR, double UT)
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
				while(new_orbit(old, velN*max_dV, UT).PeR < PeR)
				{ max_dV *= 2; if(max_dV > 100000) break; }
			}
			else max_dV = vel.magnitude;
			while(max_dV-min_dV > DEO.dVtol)
			{
				var dV = (max_dV+min_dV)/2;
				var nPe = new_orbit(old, velN*dV, UT).PeR;
				if(up && nPe > PeR || !up && nPe < PeR) max_dV = dV;
				else min_dV = dV;
			}
			return (max_dV+min_dV)/2*velN;
		}

		static Orbit new_PeR(Orbit old, double PeR, double UT)
		{
			var dV = dV4Pe(old, PeR, UT);
			return new_orbit(old, dV, UT);
		}

		protected override void UpdateState()
		{
			base.UpdateState();
			IsActive = VSL.InOrbit;
		}

		public override void OnEnable(bool enabled)
		{
			base.OnEnable(enabled);
			if(enabled) Start = -1;
		}

		public class LandingSite
		{
			public readonly Orbit orbit;
			public double TrueAnomaly { get; private set; }
			public readonly double StartUT;
			public readonly double TimeToSurface;
			public readonly double AtSurfaceUT;
			public readonly WayPoint SurfacePoint;

			public LandingSite(Orbit landing_trajectory, double startUT)
			{
				orbit = landing_trajectory;
				StartUT = startUT;
				var tA0 = orbit.TrueAnomalyAtUT(startUT);
				TrueAnomaly = orbit.TrueAnomalyAtRadius(orbit.referenceBody.Radius);
				if(tA0 > TrueAnomaly) TrueAnomaly = TwoPi-TrueAnomaly;
				SurfacePoint = MakeWaypoint(orbit, TrueAnomaly, "Landing Site");
				TimeToSurface = orbit.GetDTforTrueAnomaly(TrueAnomaly, orbit.period/2);
				AtSurfaceUT = StartUT+TimeToSurface;
			}

			public Vector3d GetRelSurfacePosition()
			{ return orbit.referenceBody.GetRelSurfacePosition(SurfacePoint.Lat, SurfacePoint.Lon, 0); }

			public Vector3d GetOrbitVelocityAtSurface()
			{ return orbit.getOrbitalVelocityAtUT(StartUT+TimeToSurface); }

			public override string ToString()
			{
				return string.Format("LandingSite:\nTrueAnomaly: {0} deg,\nCoordinates: {1},\nTimeToSurface: {2} s", 
				                     TrueAnomaly*Mathf.Rad2Deg, SurfacePoint, TimeToSurface);
			}
		}

		public const double TwoPi = 6.2831853;

		protected override void Update()
		{
			if(!IsActive || VSL.orbit.referenceBody == null) return;
			double dist = DEO.Dtol*10;
			double old_dist = dist;
			double rate = 0.1;
			if(Start < 0) 
			{
				Start = VSL.Physics.UT+DEO.StartOffset;
				var PeR = DEO.PeR;
				var target = MakeWaypoint(VSL.orbit, VSL.orbit.trueAnomaly+Mathf.PI*1.5, "Target");
				CFG.Waypoints.Enqueue(target);
				LandingSite site = null;
				var maxI = 100;
				do {
					if(site != null) 
					{
						if(Math.Abs(old_dist) < Math.Abs(dist)) rate /=2;
						Start = site.StartUT+dist/TwoPi*Body.rotationPeriod*rate;
						if(Start-VSL.Physics.UT < DEO.StartOffset) 
							Start = site.StartUT+Body.rotationPeriod/2;
						Log("Changing start time: {0} s", Start-site.StartUT);
					}
					site = new LandingSite(new_PeR(VSL.orbit, VSL.mainBody.Radius*PeR, Start), Start);
					var targetPos0  = Body.GetRelSurfacePosition(target.Lat, target.Lon, 0);
					var targetPos1  = Quaternion.AngleAxis((float)((site.AtSurfaceUT-VSL.Physics.UT)/
					                                               VSL.mainBody.rotationPeriod * Mathf.PI*2f), 
					                                       Body.angularVelocity) * targetPos0;
					old_dist = dist;
					dist = target.AngleTo(site.SurfacePoint);
					dist *= Math.Sign(Vector3d.Dot(targetPos1-site.GetRelSurfacePosition(), site.GetOrbitVelocityAtSurface()));
					Log("{0}\ndistance: {1} m", site, dist*Body.Radius);
					maxI--;
				} while(Math.Abs(dist*Body.Radius) > DEO.Dtol && maxI > 0);
			}
		}

		public static WayPoint MakeWaypoint(Orbit orb, double tA, string name = "")
		{
			var pos = orb.getPositionFromTrueAnomaly(tA);
			var lat = orb.referenceBody.GetLatitude(pos);
			var lon = orb.referenceBody.GetLongitude(pos);
			var wp = new WayPoint(lat, lon);
			if(!string.IsNullOrEmpty(name)) wp.Name = name;
			return wp;
		}

		void addNode(Vector3d dV, double UT)
		{
			var node = VSL.vessel.patchedConicSolver.AddManeuverNode(UT);
			var norm = O.GetOrbitNormal();
			var radial = Vector3d.Cross(O.getOrbitalVelocityAtUT(UT), norm).normalized;
			node.DeltaV = new Vector3d(Vector3d.Dot(dV, radial),
			                           Vector3d.Dot(dV, norm),
			                           Vector3d.Dot(dV, O.getOrbitalVelocityAtUT(UT).normalized));
			VSL.vessel.patchedConicSolver.UpdateFlightPlan();
		}

//		void deorbit()
//		{
//			if(Start < 0) 
//			{
//				var dV = 0.0;
//				var ttb = 0f;
//				var offset = DEO.StartOffset;
//				do {
//					if(ttb > 0) offset = ttb+DEO.StartOffset;
//					dV = dV4Pe(VSL.orbit, VSL.mainBody.Radius*DEO.PeR, VSL.Physics.UT+offset);
//					ttb = MAN.TTB((float)Math.Abs(dV), 1)/2;
//				} while(ttb > offset);
//
//				Start = VSL.Physics.UT+offset;
//				var node = VSL.vessel.patchedConicSolver.AddManeuverNode(Start);
//				node.DeltaV = new Vector3d(0,0,dV);
//				VSL.vessel.patchedConicSolver.UpdateFlightPlan();
//			}
//		}
	}
}

