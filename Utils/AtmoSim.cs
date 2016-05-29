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

namespace ThrottleControlledAvionics
{
	public class AtmoSim
	{
		const double DeltaTime = 0.5;
		const double Cd = 0.0006;

		readonly CelestialBody Body;
		readonly VesselWrapper VSL;

		/// <summary>
		/// Initializes a new instance of the <see cref="ThrottleControlledAvionics.ToOrbitSim"/> class.
		/// </summary>
		/// <param name="body">Planetary body.</param>
		/// <param name="vsl">VesselWrapper.</param>
		public AtmoSim(CelestialBody body, VesselWrapper vsl)
		{
			Body = body;
			VSL = vsl;
		}

		double atm_density(double h)
		{
			if(!Body.atmosphere) return 0;
			var P = Body.GetPressure(h);
			var T = Body.GetTemperature(h);
			return Body.GetDensity(P, T);
		}

		double drag(double s, double h, double v)
		{ return atm_density(h) * v*v * Cd * s/2; }

		double StG(double h) 
		{ 
			var r = Body.Radius+h;
			return Body.gMagnitudeAtCenter/r/r; 
		}

		double freeclimb_altitude(double m, double s, double h, double v, double dt)
		{
			var H = h;
			while(v > 0)
			{
				H += v*dt;
				v -= (StG(H) + drag(s, H, v)/m)*dt;
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
				v -= (StG(ah) - drag(s, ah, v)/m)*dt;
				t += dt;
//				Utils.LogF("h {}, v {}, t {}", h, v, t);//debug
			}
			terminal_velocity = Math.Abs(v);
			return t-dt/2;
		}

		public double FromSurfaceTTA(double ApA, double slope, double gturn_curve)
		{
			var t = 0.0;
			var v = Vector3d.zero;
			var h = (double)VSL.Altitude.Absolute;
			var m = (double)VSL.Physics.M;
			var mT = VSL.Engines.MaxThrust+VSL.Engines.ManualThrust;
			var mflow = VSL.Engines.MaxMassFlow+VSL.Engines.ManualMassFlow;
			if(VSL.Engines.NumActive.Equals(0))
				mT = VSL.Engines.GetNearestEnginedStageMaxThrust(out mflow);
			var mTm = mT.magnitude;
			var s = VSL.Geometry.AreaInDirection(mT);
			var hmove = slope*ApA;
			var thrust = true;
			while(v.x >= 0)
			{
				if(thrust)
				{
					if(!CheatOptions.InfiniteFuel)
					{
						var dm = mflow*DeltaTime;
						if(m < dm) { thrust = false; continue; }
						m -= dm;
					}
					var apa = freeclimb_altitude(m, s, h, v.x, DeltaTime*4);
					var dapa = ApA-apa;
					var vv = Utils.ClampL(dapa, 0);
					var hv = Utils.ClampL(hmove-dapa*gturn_curve, 0)*Utils.Clamp((apa-h)/100, 0, 1);
					if(Body.atmosphere) hv *= Math.Sqrt(Utils.Clamp(VSL.Altitude.Absolute/Body.atmosphereDepth, 0, 1));
					var angle = Math.Atan2(vv, hv);
					v.x += (mTm*Math.Sin(angle)/m-StG(h))*DeltaTime;
					v.y += (mTm*Math.Cos(angle)/m)*DeltaTime;
					thrust = ApA-apa > 1;
//					Utils.LogF("v.v {}, v.h {}, dapa {}, hmove {}",
//					           v.x, v.y, dapa, hmove);//debug
				}
				else v.x -= StG(h)*DeltaTime;
				var vm = v.magnitude;
				v *= 1-drag(s, h, vm)/m/vm*DeltaTime;
//				Utils.LogF("v.v {}, v.h {}, drag {}, h {}, hmove {}", 
//				           v.x, v.y, drag(s, h, vm)/m, h, hmove);//debug
				hmove -= v.y*DeltaTime;
				h += v.x*DeltaTime;
				t += DeltaTime;
			}
			return t-DeltaTime/2;
		}

		public static double FromSurfaceTTA(VesselWrapper VSL, double ApA, double slope, double gturn_curve)
		{
			var sim = new AtmoSim(VSL.mainBody, VSL);
			return sim.FromSurfaceTTA(ApA, slope, gturn_curve);
		}

	}
}

