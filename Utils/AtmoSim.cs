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
	public class AtmoSim
	{
		const double DeltaTime = 0.5;
		static Globals GLB { get { return Globals.Instance; } }
		static double Cd;

		readonly CelestialBody Body;
		readonly VesselWrapper VSL;

		/// <summary>
		/// Initializes a new instance of the <see cref="ThrottleControlledAvionics.AtmoSim"/> class.
		/// </summary>
		/// <param name="body">Planetary body.</param>
		/// <param name="vsl">VesselWrapper.</param>
		public AtmoSim(CelestialBody body, VesselWrapper vsl)
		{
			Body = body;
			VSL = vsl;
			//0.0005 converts dynamic pressure to kPa and divides area by 2: Drag = dP * Cd * S/2.
			Cd = 0.0005 * Globals.Instance.ORB.DragK * PhysicsGlobals.DragCubeMultiplier * PhysicsGlobals.DragMultiplier;
		}

		double drag(double s, double h, double v)
		{ 
			if(h > Body.atmosphereDepth) return 0;
			var atm = Body.AtmoParamsAtAltitude(h);
			var v2 = v*v;
			var dP = atm.Rho * v2;
			var mach = v/atm.Mach1;
			var Cd = AtmoSim.Cd *
			    PhysicsGlobals.DragCurveMultiplier.Evaluate((float)mach) *
				PhysicsGlobals.DragCurvePseudoReynolds.Evaluate((float)(atm.Rho*Math.Abs(v)));
			return dP * Cd * s;
		}

		double StG(double h) 
		{ 
			var r = Body.Radius+h;
			return Body.gMagnitudeAtCenter/r/r; 
		}

		double G(double h, double hv) { return StG(h)-hv*hv/(Body.Radius+h); }

		double freeclimb_altitude(double m, double s, double h, double v, double hv, double dt, out double t)
		{
			t = 0;
			var H = h;
			while(v > 0)
			{
				H += v*dt;
				v -= (G(H, hv) + drag(s, H, v)/m)*dt;
				t += dt;
			}
			return H;
		}

		public double FreeFallTime(out double terminal_velocity)
		{
			var t = 0.0;
			var v = (double)VSL.VerticalSpeed.Absolute;
			var h = (double)VSL.Altitude.Relative;
			var th = (double)VSL.Altitude.TerrainAltitude;
			var m = (double)VSL.Physics.M;
			var s = VSL.Geometry.AreaInDirection(VSL.Physics.UpL);
			var dt = v < 0? Math.Abs(h/v/10) : 1;
			if(dt > DeltaTime) dt = DeltaTime;
			while(h > VSL.Geometry.H)
			{
				h += v*dt;
				var ah = h+th;
				v = Utils.ClampH(v-(StG(ah) - drag(s, ah, v)/m)*dt, -0.1);
				t += dt;
				dt = Math.Max(Math.Min(dt, (VSL.Geometry.H-h)/v*0.9), 0.01);
//				Utils.Log("h {}, v {}, t {}", h, v, t);//debug
			}
			terminal_velocity = Math.Abs(v);
			return t;
		}

		public double FromSurfaceTTA(double ApA, double alpha, double gturn_curve, double surface_vel)
		{
			var t = 0.0;
			var v = new Vector3d(0, surface_vel);
			var h = (double)VSL.Altitude.Absolute;
			var m = (double)VSL.Physics.M;
			var eStats = VSL.Engines.NoActiveEngines? 
				VSL.Engines.GetNearestEnginedStageStats() :
				VSL.Engines.GetEnginesStats(VSL.Engines.Active);
			var mT = eStats.Thrust;
			var mflow = eStats.MassFlow;
			var mTm = mT.magnitude*Mathfx.Lerp(0.6, 0.95, Utils.ClampH(
				VSL.Torque.AngularAcceleration(eStats.TorqueLimits.Max+VSL.Torque.RCSLimits.Max+VSL.Torque.WheelsLimits.Max).magnitude, 1));
			var s = VSL.Geometry.AreaInDirection(mT);
			var R = Body.Radius;
			var thrust = true;
			var throttle = 1.0;
//			var alpha0 = alpha; //debug
			while(v.x >= 0)
			{
				var atmF = Utils.Clamp(h/Body.atmosphereDepth, 0, 1);
				if(thrust)
				{
					double apaT;
					var apa = freeclimb_altitude(m, s, h, v.x, v.y, DeltaTime*4, out apaT);
					var dapa = (ApA-apa)*gturn_curve;
					var arc = (alpha-v.y*apaT/(R+(h+apa)/2))*(R+apa);
					var vv = Utils.ClampL(dapa, 0);
					var hv = Utils.ClampL(arc-dapa, 0)*Utils.Clamp((h-VSL.Altitude.Absolute)/GLB.ORB.GTurnOffset, 0, 1);
					if(h < Body.atmosphereDepth) hv *= Math.Sqrt(atmF);
					var angle = Math.Atan2(vv, hv);
					throttle = ThrottleControl.NextThrottle((float)(Math.Sqrt(vv*vv+hv*hv)*Globals.Instance.ORB.Dist2VelF*VSL.Physics.StG/Utils.G0), 
					                                        (float)throttle, (float)m, (float)mTm, 0);
					var v_throttle = Math.Sin(angle);
					v.x += (mTm*v_throttle*throttle/m-G(h, v.y))*DeltaTime;
					var h_throttle = 0.0;
					if(arc > 0)
					{
						h_throttle = Math.Cos(angle) ;
						v.y += (mTm*h_throttle*throttle/m)*DeltaTime;
					}
					thrust = ApA-apa > GLB.ORB.Dtol && arc > GLB.ORB.Dtol;
					if(!CheatOptions.InfinitePropellant)
					{
						var dm = mflow*(h_throttle+v_throttle)*throttle*DeltaTime;
						if(m < dm) { thrust = false; continue; }
						m -= dm;
					}
//					Utils.Log("apaT {}, dapa {}, arc {}, throttle {}, h-thr {}, v-thr {}", apaT, dapa, arc, throttle);//debug
				}
				else v.x -= G(h, v.y)*DeltaTime;
				if(h < Body.atmosphereDepth)
				{
					var y = v.y-surface_vel*(1-atmF);
					var vm = Math.Sqrt(v.x*v.x+y*y);
					var D = drag(s, h, vm)/m*DeltaTime/vm;
					v.x -= v.x*D;
					v.y -= y*D;
				}
				alpha -= v.y*DeltaTime/Body.Radius;
				h += v.x*DeltaTime;
				t += DeltaTime;
//				Utils.Log("v.v {}, v.h {}, drag {}, h {}, hmove {}", v.x, v.y, drag(s, h, vm)/m, h, hmove);//debug
//				DebugUtils.CSV("LambertSolver", t, v.x, v.y, (alpha0-alpha)*(R+h), h, m, mTm*throttle, throttle, Math.Atan2(v.x, v.y)*Mathf.Rad2Deg);//debug
			}
			return t-DeltaTime/2;
		}

		public static double FromSurfaceTTA(VesselWrapper VSL, double ApA, double alpha, double gturn_curve, double surface_vel)
		{
			var sim = new AtmoSim(VSL.Body, VSL);
			return sim.FromSurfaceTTA(ApA, alpha, gturn_curve, surface_vel);
		}

	}
}

