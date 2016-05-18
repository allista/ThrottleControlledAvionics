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
	public class LambertSolver
	{
		Orbit orbit;
		CelestialBody body;
		double StartUT;
		double mu; //standard grav. paramenter of the reference body

		Vector3d r1; //start position
		Vector3d c;  //r2-r1
		//magnitudes
		double   cm;
		double   m; //r1m+r2m+cm
		double   n; //r1m+r2m-cm
		//transfer parameters
		double   sigma; //angle parameter
		double   tau;   //normalized transfer time
		double   tauME; //normalized Minimum Energy transfer time
		//utility
		double sigma2;
		double m3;

		public LambertSolver(Orbit orb, Vector3d destination, double UT)
		{
			orbit = orb;
			body = orbit.referenceBody;
			StartUT = UT;
			mu = body.gravParameter;

			r1 = orb.getRelativePositionAtUT(UT);
			var h  = Vector3d.Cross(r1, destination);
			if(h.sqrMagnitude < 0.01) h = orb.GetOrbitNormal();
			c  = destination-r1;

			cm  = c.magnitude;
			var r1m = r1.magnitude;
			var r2m = destination.magnitude;
			var rrm = r1m+r2m;
			m  = rrm+cm;
			n  = rrm-cm;
			m3 = m*m*m;

			var transfer_angle = Vector3d.Angle(r1, destination)*Mathf.Deg2Rad;
			if(h.z < 0) transfer_angle = Utils.TwoPI-transfer_angle;
			sigma = Math.Sqrt(n/m);
			if(transfer_angle > Math.PI) sigma = -sigma;
			sigma2 = sigma*sigma;

			tauME = Math.Acos(sigma)+sigma*Math.Sqrt(1-sigma2);
		}

		/// <summary>
		/// Computes the next approximation of the polynomial root using Laguerre method.
		/// </summary>
		/// <returns>Aapproximation of the root x of the polynomial f.</returns>
		/// <param name="x">The previous x approximation.</param>
		/// <param name="f">F(x).</param>
		/// <param name="f1">F'(x).</param>
		/// <param name="f2">F''(x).</param>
		/// <param name="N">Polynomial rank.</param>
		static double Laguerre_next(double x, double f, double f1, double f2, int N)
		{
			var G  = f1/f;
			var G2 = G*G;
			var H  = G2 - f2/f;
			var s2 = (N-1)*(N*H-G2);
			if(double.IsNaN(s2)) return double.NaN;
			var s  = Math.Sqrt(s2);
			var Gs = G+s;
			var G_s = G-s;
			return x - N/(Math.Abs(Gs) > Math.Abs(G_s)? Gs : G_s);
		}

		/// <summary>
		/// Calculates the ME transfer orbit from a given orbit and UT to the destination radius-vector.
		/// The calculation procedure is derived form the "The Superior Lambert Algorithm" paper by Gim J. Der.
		/// It uses Sun's analytical solution to the general Lambert problem, not the iterative solver for arbitrary transfer time itself.
		/// </summary>
		/// <returns>The DeltaVee for the maneuver.</returns>
		/// <param name="transfer_time">Returned value of the transfer time in seconds.</param>
		public Vector3d dV4TransferME(out double transfer_time)
		{
			var y = Math.Sign(sigma)*Math.Sqrt(1-sigma2);
			var v = Math.Sqrt(mu)*y/Math.Sqrt(n);
			transfer_time = tauME/4/Math.Sqrt(mu/m3);
			return (r1.normalized + c/cm)*v - orbit.getOrbitalVelocityAtUT(StartUT);
		}

		/// <summary>
		/// Calculates a transfer orbit from a given orbit and UT to the destination radius-vector in a given transfer time.
		/// The calculation procedure is derived form the "The Superior Lambert Algorithm" paper by Gim J. Der.
		/// It uses Sun's analytical solution to the general Lambert problem and the Laguerre iterative root finder.
		/// </summary>
		/// <returns>The DeltaVee for the maneuver.</returns>
		/// <param name="transfer_time">Transfer time.</param>
		/// <param name="tol">Error tolerance.</param>
		public Vector3d dV4Transfer(double transfer_time, double tol = 1e-6)
		{
			tau = 4 * transfer_time * Math.Sqrt(mu/(m*m*m));
			if(Math.Abs(tau-tauME) < tol) return dV4TransferME(out transfer_time);
			var tauP = 2/3*(1-sigma2*sigma);
			if(tau <= tauP) //TODO: implement parabolic and hyperbolic transfers
			{
				Utils.LogF("dV4Transfer: PANIC! Non-elliptic transfer orbit detected!");
				return Vector3d.zero;
			}
			var x0 = 0.0;
			var x1 = tau < tauME? 0.1 : -0.1;
			while(Math.Abs(x1-x0) > tol)
			{
				x0 = x1;
				x1 = next_elliptic(x1, 2);
			}
			var sqrt_mu = Math.Sqrt(mu);
			var sqrt_m  = Math.Sqrt(m);
			var sqrt_n  = Math.Sqrt(n);
			var y1 = _y(x1);
			var vr = sqrt_mu * (y1/sqrt_n - x1/sqrt_m);
			var vc = sqrt_mu * (y1/sqrt_n + x1/sqrt_m);

			Utils.LogF("\nr1: {}\n" +
			           "c: {}\n" +
			           "sigma: {}\n" +
			           "tau: {}\n" +
			           "tauME: {}\n" +
			           "x {}, y {}\n" +
			           "vc {}, vr {}\n" +
			           "v: {}",
			           r1, c,
			           sigma, tau, tauME,
			           x1, y1,
			           vc, vr,
			           r1.normalized*vr + c/cm*vc
			          );//debug

			return r1.normalized*vr + c/cm*vc - orbit.getOrbitalVelocityAtUT(StartUT);
		}

		double _y(double x) { return Math.Sign(sigma)*Math.Sqrt(1-sigma2*(1-x*x)); }

		double next_elliptic(double x, int N)
		{
			var y = _y(x);
			var x2 = x*x;
			var y2 = y*y;
			var sqrt_one_x2 = Math.Sqrt(1 - x2);
			var sqrt_one_y2 = Math.Sqrt(1 - y2);
			var acot_x = Utils.Acot(x / sqrt_one_x2);
			var acot_y = Utils.Acot(y / sqrt_one_y2);
			var x2_one = x2-1;
			var y2_one = y2-1;

			var f = (((acot_x-x*sqrt_one_x2) -
			          (acot_y-y*sqrt_one_y2))
					 /(sqrt_one_x2 * sqrt_one_x2 * sqrt_one_x2) -tau);

			var f1 = ((3*x*acot_x+
			           (-x2-2)*sqrt_one_x2+
			           3*(y*sqrt_one_y2-acot_y)*x)
			          /Math.Pow(sqrt_one_x2, 5));

			var f2 = (-(3*Math.Sqrt(-x2_one)*(4*x2+1)*
			            (Utils.Acot(x*Math.Sqrt(-x2_one)/x2_one)-
			             Utils.Acot(y*Math.Sqrt(-y2_one)/y2_one)-
			             y*Math.Sqrt(-y2_one))-
			            x*x2_one*(2*x2+13))
			          /Math.Pow(x2_one, 4));

			return Laguerre_next(x, f, f1, f2, N);
		}
	}
}

