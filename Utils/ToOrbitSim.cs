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
	public class ToOrbitSim
	{
		const double dt = 0.5;
		const double Cd = 0.0006;

		readonly CelestialBody Body;
		readonly VesselWrapper VSL;

		/// <summary>
		/// Initializes a new instance of the <see cref="ThrottleControlledAvionics.ToOrbitSim"/> class.
		/// </summary>
		/// <param name="body">Planetary body.</param>
		/// <param name="vsl">VesselWrapper.</param>
		public ToOrbitSim(CelestialBody body, VesselWrapper vsl)
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

		double freefall(double m, double s, double h, double v, double dt)
		{
			var H = h;
			while(v > 0)
			{
				H += v*dt;
				v -= (StG(H) + drag(s, H, v)/m)*dt;
			}
			return H;
		}

		public double FromSurfaceTTA(double ApA, double ApA2SmA, double gturn_curve)
		{
			var t = 0.0;
			var v = 0.0;
			var h = (double)VSL.Altitude.Absolute;
			var m = (double)VSL.Physics.M;
			var mT = VSL.Engines.MaxThrust;
			var mTm = VSL.Engines.MaxThrustM;
			var mflow = VSL.Engines.MaxMassFlow;
			if(VSL.Engines.NumActive.Equals(0))
			{
				mT = VSL.Engines.NextStageMaxThrust(out mflow);
				mTm = mT.magnitude;
			}
			var s = VSL.Geometry.AreaInDirection(mT);
			var hmove = ApA2SmA*ApA;
			var thrust = true;
			while(v >= 0)
			{
				if(thrust)
				{
					if(!CheatOptions.InfiniteFuel)
					{
						var dm = mflow*dt;
						if(m < dm) { thrust = false; continue; }
						m -= dm;
					}
					var apa = freefall(m, s, h, v, dt*4);
					var dapa = ApA-apa;
					var vv = dapa/Utils.ClampL(v, 1);
					var hv = hmove/Utils.ClampL(dapa, gturn_curve)*Utils.Clamp((apa-h)/100, 0, 1)*gturn_curve;
					if(Body.atmosphere) hv *= Math.Sqrt(Utils.Clamp(VSL.Altitude.Absolute/Body.atmosphereDepth, 0, 1));
					hmove -= hv*dt;
					var T = mTm*Math.Sin(Math.Atan2(vv, hv));
					v += (T/m-StG(h))*dt;
					thrust = ApA-apa > 1;
				}
				else v -= StG(h)*dt;
				v -= drag(s, h, v)/m*dt;
				h += v*dt;
				t += dt;
			}
			return t-dt/2;
		}

		public static double FromSurfaceTTA(VesselWrapper VSL, double ApA, double ApA2SmA, double gturn_curve)
		{
			var sim = new ToOrbitSim(VSL.mainBody, VSL);
			return sim.FromSurfaceTTA(ApA, ApA2SmA, gturn_curve);
		}

	}
}

