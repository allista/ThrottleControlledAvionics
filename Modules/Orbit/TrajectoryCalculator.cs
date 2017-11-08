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
using AT_Utils;

namespace ThrottleControlledAvionics
{
	public abstract class TrajectoryCalculator : TCAModule
	{
		public class Config : TCAModule.ModuleConfig
		{
			[Persistent] public float dVtol              = 0.01f; //m/s
			[Persistent] public int   PerFrameIterations = 10;
			[Persistent] public float ManeuverOffset     = 60f;    //s
            [Persistent] public float CorrectionOffset   = 20f;    //s
		}
		protected static Config TRJ { get { return Globals.Instance.TRJ; } }

		protected TrajectoryCalculator(ModuleTCA tca) : base(tca) {}
		//multiple inheritance or some sort of mixins or property extensions would be great here =/
        public Orbit VesselOrbit { get { return VSL.vessel.orbitDriver.orbit; } }
        public CelestialBody Body { get { return VSL.vessel.orbitDriver.orbit.referenceBody; } }
        public Vector3d SurfaceVel {get { return Vector3d.Cross(-Body.zUpAngularVelocity, VesselOrbit.pos); } }
        public double ManeuverOffset { get { return Math.Max(TRJ.ManeuverOffset, VSL.Torque.MaxCurrent.TurnTime); } }
        public double CorrectionOffset { get { return Math.Max(TRJ.CorrectionOffset, VSL.Torque.MaxCurrent.TurnTime); } }
        public double MinPeR { get { return VesselOrbit.MinPeR(); } }

		public static Orbit NextOrbit(Orbit orb, double UT)
		{
			while(orb != null && 
                  orb.nextPatch != null && 
			      orb.nextPatch.referenceBody != null &&
                  orb.patchEndTransition != Orbit.PatchTransitionType.FINAL &&
			      orb.EndUT < UT)
				orb = orb.nextPatch;
			return orb;
		}

		protected Orbit NextOrbit(double UT)
		{ return NextOrbit(VesselOrbit, UT); }

        public static Orbit LastOrbit(Orbit orb)
        {
            while(orb != null && 
                  orb.nextPatch != null && 
                  orb.nextPatch.referenceBody != null &&
                  orb.patchEndTransition != Orbit.PatchTransitionType.FINAL)
                orb = orb.nextPatch;
            return orb;
        }

        public static bool DiscontiniousOrbit(Orbit o)
        {
            return o.EndUT > 0 && !double.IsInfinity(o.EndUT) &&
                (o.patchEndTransition == Orbit.PatchTransitionType.ESCAPE ||
                 o.patchEndTransition == Orbit.PatchTransitionType.IMPACT);
        }

		protected Vector3d hV(double UT) { return VesselOrbit.hV(UT); }

		protected bool LiftoffPossible
		{
			get
			{
				if(VSL.Engines.NumActive > 0 && VSL.OnPlanet && VSL.OnPlanetParams.MaxTWR <= 1)
				{
					Status("red", "TWR < 1, impossible to achive orbit");
                    Disable();
					return false;
				}
				return true;
			}
		}

        public static Orbit NewOrbit(CelestialBody body, Vector3d pos, Vector3d vel, double UT)
        {
            var obt = new Orbit();
            obt.UpdateFromStateVectors(pos, vel, body, UT);
            obt.Init();
            if(obt.eccentricity < 0.01)
            {
                var T   = UT;
                var v   = obt.getOrbitalVelocityAtUT(UT);
                var D   = (vel-v).sqrMagnitude;
                var Dot = Vector3d.Dot(vel, v);
                var dT  = obt.period/10;
                while(D > 1e-4 && Math.Abs(dT) > 0.01)
                {
                    T += dT;
                    v = obt.getOrbitalVelocityAtUT(T);
                    var dot = Vector3d.Dot(vel, v);
                    if(dot > 0)
                    {
                        D = (vel-v).sqrMagnitude;
                        if(dot < Dot) dT /= -2;
                        Dot = dot;
                    }
                }
                if(!T.Equals(UT))
                {
                    var dP = (T-UT)/obt.period;
                    obt.LAN = (obt.LAN-dP*360)%360;
                    obt.argumentOfPeriapsis = (obt.argumentOfPeriapsis-dP*360)%360;
                    obt.meanAnomalyAtEpoch = (obt.meanAnomaly+dP*Utils.TwoPI)%Utils.TwoPI;
                    obt.eccentricAnomaly = obt.solveEccentricAnomaly(obt.meanAnomalyAtEpoch, obt.eccentricity, 1e-7, 8);
                    obt.trueAnomaly = obt.GetTrueAnomaly(obt.eccentricAnomaly);
                    obt.Init();
                }
            }
            obt.StartUT = UT;
            obt.EndUT = UT+obt.period;
            obt.patchEndTransition = Orbit.PatchTransitionType.FINAL;
            obt.patchStartTransition = Orbit.PatchTransitionType.MANEUVER;
            return obt;
        }

		public static Orbit NewOrbit(Orbit old, Vector3d dV, double UT)
		{
			var pos = old.getRelativePositionAtUT(UT);
			var vel = old.getOrbitalVelocityAtUT(UT)+dV;
            return NewOrbit(old.referenceBody, pos, vel, UT);
		}

		public static Orbit CopyOrbit(Orbit o)
		{
			return new Orbit(o.inclination, o.eccentricity, o.semiMajorAxis, o.LAN, 
				o.argumentOfPeriapsis, o.meanAnomalyAtEpoch, o.epoch, o.referenceBody);
		}

		public static Vector3d dV4C(Orbit old, Vector3d dir, double UT)
		{
			var V = Math.Sqrt(old.referenceBody.gMagnitudeAtCenter/old.getRelativePositionAtUT(UT).magnitude);
			return dir.normalized*V-old.getOrbitalVelocityAtUT(UT);
		}

		protected Orbit CircularOrbit(Vector3d dir, double UT)
		{ return NewOrbit(VesselOrbit, dV4C(VesselOrbit, dir, UT), UT); }

		protected Orbit CircularOrbit(double UT) { return CircularOrbit(hV(UT), UT); }

        public static Orbit CircularOrbit(CelestialBody body, Vector3d pos, Vector3d dir, double UT)
        {
            var vel = dir.normalized*Math.Sqrt(body.gMagnitudeAtCenter/pos.magnitude);
            return NewOrbit(body, pos, vel, UT);
        }

		public static Vector3d dV4Pe(Orbit old, double R, double UT, Vector3d add_dV = default(Vector3d))
		{
			var up = old.PeR < R;
			var pos = old.getRelativePositionAtUT(UT);
            var vel = old.getOrbitalVelocityAtUT(UT);
			var hvel = Vector3d.Exclude(pos, vel);
            var dir = up? hvel.normalized : -hvel.normalized;
			var min_dV = 0.0;
			var max_dV = 0.0;
			if(up)
			{
				max_dV = 10;
				var max_PeR = pos.magnitude;
				if(R > max_PeR) R = max_PeR;
				while(max_dV < 100000)
				{ 
                    var orb = NewOrbit(old.referenceBody, pos, vel+dir*max_dV, UT);
					if(orb.eccentricity >= 1 || orb.PeR > R) break;
					max_dV *= 2;
				}
			}
            else max_dV = hvel.magnitude+add_dV.magnitude;
			while(max_dV-min_dV > TRJ.dVtol)
			{
				var dV = (max_dV+min_dV)/2;
                var orb = NewOrbit(old.referenceBody, pos, vel+dir*dV+add_dV, UT);
				if(up && (orb.eccentricity >= 1 || orb.PeR > R) || 
				   !up && orb.PeR < R) 
					max_dV = dV;
				else min_dV = dV;
			}
			return (max_dV+min_dV)/2*dir+add_dV;
		}

		public static Vector3d dV4Ap(Orbit old, double R, double UT, Vector3d add_dV = default(Vector3d))
		{
			var up = old.ApR < R;
            var pos = old.getRelativePositionAtUT(UT);
            var vel = old.getOrbitalVelocityAtUT(UT);
            var hvel = Vector3d.Exclude(pos, vel);
            var dir = up? hvel.normalized : -hvel.normalized;
			var min_dV = 0.0;
			var max_dV = 0.0;
			if(up)
			{
				max_dV = 10;
				while(max_dV < 100000)
				{ 
                    var orb = NewOrbit(old.referenceBody, pos, vel+dir*max_dV, UT);
					if(orb.eccentricity >= 1 || orb.ApR > R) break;
					max_dV *= 2;
				}
			}
			else 
			{
				var min_ApR = old.getRelativePositionAtUT(UT).magnitude;
				if(R < min_ApR) R = min_ApR;
				max_dV = hvel.magnitude+add_dV.magnitude;
			}
			while(max_dV-min_dV > TRJ.dVtol)
			{
				var dV = (max_dV+min_dV)/2;
                var orb = NewOrbit(old.referenceBody, pos, vel+dir*dV+add_dV, UT);
				if(up && (orb.eccentricity >= 1 || orb.ApR > R) ||
				   !up && orb.ApR < R) 
					max_dV = dV;
				else min_dV = dV;
			}
			return (max_dV+min_dV)/2*dir+add_dV;
		}

		public static Vector3d dV4R(Orbit old, double R, double UT, double TargetUT, Vector3d add_dV = default(Vector3d))
		{
            var pos = old.getRelativePositionAtUT(UT);
            var vel = old.getOrbitalVelocityAtUT(UT);
			var oldR = old.getRelativePositionAtUT(TargetUT);
			var up  = oldR.magnitude < R;
            var dir = old.hV(UT).normalized;
            if(!up) dir = -dir;
			var min_dV = 0.0;
			var max_dV = 0.0;
			if(up)
			{
				max_dV = 1;
                while(NewOrbit(old.referenceBody, pos, vel+dir*max_dV, UT)
				      .getRelativePositionAtUT(TargetUT)
				      .magnitude < R)
				{ max_dV *= 2; if(max_dV > 100000) break; }
			}
			else max_dV = old.getOrbitalVelocityAtUT(UT).magnitude+add_dV.magnitude;
			while(max_dV-min_dV > TRJ.dVtol)
			{
				var dV = (max_dV+min_dV)/2;
                var nR = NewOrbit(old.referenceBody, pos, vel+dir*dV+add_dV, UT)
					.getRelativePositionAtUT(TargetUT)
					.magnitude;
				if(up && nR > R || !up && nR < R) max_dV = dV;
				else min_dV = dV;
			}
			return (max_dV+min_dV)/2*dir+add_dV;
		}

		public static Vector3d dV4Ecc(Orbit old, double ecc, double UT, double maxR = -1)
		{
			var up = old.eccentricity > ecc;
            var pos = old.getRelativePositionAtUT(UT);
            var vel = old.getOrbitalVelocityAtUT(UT);
			var dir = vel;
			var min_dV = 0.0;
			var max_dV = up? dV4C(old, dir, UT).magnitude : dir.magnitude;
			if(!up) dir = -dir;
			dir.Normalize();
			while(max_dV-min_dV > TRJ.dVtol)
			{
				var dV = (max_dV+min_dV)/2;
                var orb = NewOrbit(old.referenceBody, pos, vel+dir*dV, UT);
				if( up && (orb.eccentricity < ecc || maxR > 0 && orb.PeR > maxR) || 
				   !up && orb.eccentricity > ecc) 
					max_dV = dV;
				else min_dV = dV;
			}
			return (max_dV+min_dV)/2*dir;
		}

		protected double slope2rad(double slope, double ApR)
		{
			var body_rot = Body.angularV*Body.Radius*Math.Sqrt(2/VSL.Physics.StG/(ApR-Body.Radius));
			return body_rot >= slope ? Utils.HalfPI : Math.Atan2(2, slope - body_rot);
		}

		protected Orbit AscendingOrbit(double ApR, Vector3d hVdir, double angle)
		{
			var LaunchRad = Utils.Clamp(angle*Mathf.Deg2Rad, 0, Utils.HalfPI);
			var velN = (Math.Sin(LaunchRad)*VesselOrbit.pos.normalized + Math.Cos(LaunchRad)*hVdir).normalized;
			var vel = Math.Sqrt(2*VSL.Physics.StG*(ApR-Body.Radius)) / Math.Sin(LaunchRad);
			var v   = 0.0;
			while(vel-v > TRJ.dVtol)
			{
				var V = (v+vel)/2;
				var o = NewOrbit(VesselOrbit, velN*V-VesselOrbit.vel, VSL.Physics.UT);
				if(o.ApR > ApR) vel = V;
				else v = V;
			} vel = (v+vel)/2;
			return NewOrbit(VesselOrbit, velN*vel-VesselOrbit.vel, VSL.Physics.UT);
		}

		/// <summary>
		/// Resonances of two orbits in seconds
		/// </summary>
		public static double ResonanceS(Orbit a, Orbit b)
		{ return a.period*b.period/(b.period - a.period); }

		/// <summary>
		/// Resonance of two orbits in 1/a.period units.
		/// </summary>
		public static double ResonanceA(Orbit a, Orbit b)
		{ return b.period/(b.period - a.period); }

		/// <summary>
		/// Resonance of two orbits in 1/b.period units.
		/// </summary>
		public static double ResonanceB(Orbit a, Orbit b)
		{ return a.period/(b.period - a.period); }

		public static double AngleDelta(Orbit a, Vector3d posB)
		{
			var tanA = Vector3d.Cross(a.GetOrbitNormal(), a.pos);
				return Utils.ProjectionAngle(a.pos, posB, tanA);
		}

		public static double AngleDelta(Orbit a, Vector3d posB, double UT)
		{
			var posA = a.getRelativePositionAtUT(UT);
			var tanA = Vector3d.Cross(a.GetOrbitNormal(), posA);
//			DebugUtils.Log("\nposA {}\ntanA {}\nposB {}", posA, tanA, posB);//debug
			return Utils.ProjectionAngle(posA, posB, tanA);
		}

		public static double AngleDelta(Orbit a, Orbit b, double UT)
		{ return AngleDelta(a, b.getRelativePositionAtUT(UT), UT); }

		public static double TimeToResonance(Orbit a, Orbit b, double UT, out double resonance, out double alpha)
		{
			alpha = AngleDelta(a, b, UT)/360;
			resonance = ResonanceA(a, b);
//			DebugUtils.Log("\nUT {}\nalpha {}\nresonance {}", UT, alpha, resonance);//debug
			var TTR = alpha*resonance;
			return TTR > 0? TTR : TTR+Math.Abs(resonance);
		}

		public static double TimeToResonance(Orbit a, Orbit b, double UT)
		{ double resonance, alpha; return TimeToResonance(a, b, UT, out resonance, out alpha); }

		public static QuaternionD BodyRotationAtdT(CelestialBody Body, double dT)
		{ 
			var angle = -(dT/Body.rotationPeriod*360 % 360.0);
			return QuaternionD.AngleAxis(angle, Body.zUpAngularVelocity.normalized); 
		}

		public static double RelativeInclination(Orbit orb, Vector3d srf_pos)
		{ return 90-Utils.Angle2(orb.GetOrbitNormal(), srf_pos); }

		public static double RelativeInclinationAtResonance(Orbit orb, Vector3d srf_pos, double UT, out double ttr)
		{
			ttr = Utils.ClampedProjectionAngle(orb.getRelativePositionAtUT(UT), srf_pos, 
			                                   orb.getOrbitalVelocityAtUT(UT))
				/360*orb.period;
			return RelativeInclination(orb, BodyRotationAtdT(orb.referenceBody, ttr)*srf_pos);
		}

		public static Vector3d dV4T(Orbit old, double T, double UT)
		{
			var body = old.referenceBody;
			var vel = old.getOrbitalVelocityAtUT(UT);
			var pos = old.getRelativePositionAtUT(UT);
			var R   = pos.magnitude;
			var sma = Math.Pow(body.gravParameter*T*T/Utils.TwoPI/Utils.TwoPI, 1/3.0);
			return sma <= R/2? -vel : 
				Vector3d.Exclude(pos, vel).normalized * 
				Math.Sqrt((2/R - 1/sma)*body.gravParameter) - vel;
		}

        public static Vector3d dV4T2(Orbit old, double T, double UT)
        {
            var up = old.period < T;
            var vel = old.getOrbitalVelocityAtUT(UT);
            var velM = vel.magnitude;
            var dir = vel/velM;
            var maxV = up? 1.0 :  0.0;
            var minV = up? 0.0 : -velM;
            if(up)
            {
                var t = old.period;
                while(t < T)
                {
                    maxV *= 2;
                    t = NewOrbit(old, dir*maxV, UT).period;
                }
            }
            while(maxV-minV > TRJ.dVtol)
            {
                var v = (maxV+minV)/2;
                var t = NewOrbit(old, dir*v, UT).period;
//                Utils.Log("{} : {} : {} = {}/{}", minV, v, maxV, t, T);//debug
                if(t > T) maxV = v;
                else minV = v;
            }
            return dir*(maxV+minV)/2;
        }

		public static Vector3d dV4Resonance(Orbit old, Orbit target, double TTR, double alpha, double UT)
		{ 
			if(alpha < 0) 
			{ 
				var minTTR = -alpha*target.period/old.period*1.1;
				if(TTR < minTTR) TTR = minTTR;
			}
			return dV4T2(old, target.period/(1+alpha*target.period/old.period/TTR), UT);
		}

		/// <summary>
		/// Computes maneuver dV for resonance orbit
		/// </summary>
		/// <param name="old">Starting orbit.</param>
		/// <param name="target">Target orbit.</param>
		/// <param name="max_TTR">maximum TimeToResonance in 1/old.period units.</param>
		/// <param name="max_dV">maximum allowed dV.</param>
		/// <param name="min_PeR">minimum allowed PeR.</param>
		/// <param name="UT">Starting UT.</param>
		public static Vector3d dV4TTR(Orbit old, Orbit target, double max_TTR, double max_dV, double min_PeR, double UT)
		{
            double alpha, resonance;
            var TTR = TimeToResonance(old, target, UT, out resonance, out alpha);
            if(TTR < max_TTR) return Vector3d.zero;
            TTR = Math.Max(max_TTR/2, 0.75);
            var dir = old.getOrbitalVelocityAtUT(UT).normalized;
            //check lower orbits
            var minV = -max_dV;
            var maxV = 0.0;
            var lowTTR = double.MaxValue;
            double lowV;
            while(maxV-minV > TRJ.dVtol)
            {
                lowV = (maxV+minV)/2;
                var o = NewOrbit(old, dir*lowV, UT);
                lowTTR = TimeToResonance(o, target, UT, out resonance, out alpha);
                if(lowTTR > TTR && o.PeR > min_PeR) maxV = lowV;
                else minV = lowV;
            }
            lowV = (maxV+minV)/2;
            //check higher orbits
            minV = 0;
            maxV = max_dV;
            var highTTR = double.MaxValue;
            double highV;
            while(maxV-minV > TRJ.dVtol)
            {
                highV = (maxV+minV)/2;
                var o = NewOrbit(old, dir*highV, UT);
                highTTR = TimeToResonance(o, target, UT, out resonance, out alpha);
                if(highTTR > TTR) minV = highV;
                else maxV = highV;
            }
            highV = (maxV+minV)/2;
            //choose the best maneuver
            if(lowTTR < max_TTR && highTTR < max_TTR)
                return Math.Abs(lowV) < highV? dir*lowV : dir*highV;
            if(lowTTR < max_TTR) 
                return dir*lowV;
            if(highTTR < max_TTR)
                return dir*highV;
            return lowTTR < highTTR? dir*lowV : dir*highV;
		}

        public static Vector3d RelativePosAtUT(CelestialBody referenceBody, Orbit obt, double UT)
        { 
            obt = NextOrbit(obt, UT);
            return referenceBody == obt.referenceBody? 
                obt.getRelativePositionAtUT(UT) :
                (obt.getTruePositionAtUT(UT) - referenceBody.getTruePositionAtUT(UT)).xzy;
        }

		public static double SqrDistAtUT(Orbit a, Orbit b, double UT)
        { 
            a = NextOrbit(a, UT); b = NextOrbit(b, UT);
            return a.referenceBody == b.referenceBody ? 
                (a.getRelativePositionAtUT(UT) - b.getRelativePositionAtUT(UT)).sqrMagnitude : 
                (a.getTruePositionAtUT(UT) - b.getTruePositionAtUT(UT)).sqrMagnitude;
        }

		public static double ClosestApproach(Orbit a, Orbit t, double fromUT, double minDist, out double ApproachUT)
		{
            var minUT = fromUT;
            var toUT = Math.Max(a.GetEndUT(), t.GetEndUT());
//            Utils.Log("a.endUT {}, t.endUT {}, toUT {}", a.GetEndUT(), t.GetEndUT(), toUT);//debug
            if(!double.IsInfinity(toUT))
            {
                var dT = (toUT-fromUT)/10;
                var minD  = double.MaxValue;
                var UT = fromUT;
                while(UT <= toUT)
                {
                    var d = SqrDistAtUT(a, t, UT);
//                    Utils.Log("Scan: d {} < minD {}, UT {}, minUT {}, dT {}", d, minD, UT, minUT, dT);//debug
                    if(d < minD) { minD = d; minUT = UT; }
                    UT += dT;
                }
            }
            return NearestApproach(a, t, minUT, fromUT, toUT, minDist, out ApproachUT);
		}

        public static double NearestApproach(Orbit a, Orbit t, double fromUT, double minDist, out double ApproachUT)
        { return NearestApproach(a, t, fromUT, fromUT, fromUT+a.GetEndUT(), minDist, out ApproachUT); }

		public static double NearestApproach(Orbit a, Orbit t, double startUT, double fromUT, double toUT, double minDist, out double ApproachUT)
		{
			double UT = startUT;
			double dT = (toUT-fromUT)/10;
			bool dir = dT > 0;
			double minD  = double.MaxValue;
			double minUT = UT;
			minDist *= minDist;
//            Utils.Log("fromUT {}, toUT {}, startUT {}", fromUT, toUT, startUT);//debug
            //search nearest point
			while(Math.Abs(dT) > 0.01)
			{
				var d = SqrDistAtUT(a, t, UT);
                if(d < minD) { minD = d; minUT = UT; }
//                Utils.Log("Search: d {} < minD {}, UT {}, minUT {}, dT {}", d, minD, UT, minUT, dT);//debug
                UT += dT;
                if(d > minD || 
				   (dir? UT < fromUT : UT > fromUT) ||
				   (dir? UT > toUT  : UT < toUT))
                {
					dT /= -2.1;
                    UT = Utils.Clamp(minUT+dT, fromUT, toUT);
                }
			}
            //if it's too near, find the border of the minDist using binary search
            if(minD < minDist)
            {
                toUT = minUT;
                while(toUT-fromUT > 0.01)
                {
                    minUT = fromUT+(toUT-fromUT)/2;
                    minD = SqrDistAtUT(a, t, minUT)-minDist;
                    if(minD > 0) fromUT = minUT;
                    else toUT = minUT;
                }
//                Utils.Log("Result: minD {}, threshold {}", Math.Sqrt(SqrDistAtUT(a, t, minUT)), Math.Sqrt(minUT));//debug
                minD += minDist;
            }
            ApproachUT = minUT;
//            Utils.Log("Result: minD {}, minUT {}", minD, minUT);//debug
            return Math.Sqrt(minD);
		}

		public static double NearestRadiusUT(Orbit orb, double radius, double StartUT, bool descending = true)
		{
			radius *= radius;
			var StopUT = StartUT;
			var dT = orb.period/10;
			var below = orb.getRelativePositionAtUT(StartUT).sqrMagnitude < radius;
			if(below)
			{
				while(StopUT-StartUT < orb.timeToAp) 
				{ 
					if(orb.getRelativePositionAtUT(StopUT).sqrMagnitude > radius) break;
					StopUT += dT;
				}
			}
			if(!below || descending)
			{
				while(StopUT-StartUT < orb.period) 
				{ 
					if(orb.getRelativePositionAtUT(StopUT).sqrMagnitude < radius) break;
					StopUT += dT;
				}
			}
			StartUT = Math.Max(StartUT, StopUT-dT);
			while(StopUT-StartUT > 0.01)
			{
				var UT = StartUT+(StopUT-StartUT)/2;
				if(orb.getRelativePositionAtUT(UT).sqrMagnitude > radius) StartUT = UT;
				else StopUT = UT;
			}
			return StartUT+(StopUT-StartUT)/2;
		}

		public static double FlyAboveUT(Orbit orb, Vector3d pos, double StartUT)
		{
			var dT = orb.period/10;
            var startUT = StartUT;
            var StopUT = startUT;
            while(StopUT-startUT < orb.period)
			{
				if(Utils.ProjectionAngle(orb.getRelativePositionAtUT(StopUT), 
                                         BodyRotationAtdT(orb.referenceBody, StopUT-StartUT)*pos, 
				                         orb.getOrbitalVelocityAtUT(StopUT)) < 0) break;
				StopUT += dT;
			}
            startUT = Math.Max(startUT, StopUT-dT);
            while(StopUT-startUT > 0.01)
			{
                var UT = startUT+(StopUT-startUT)/2;
				if(Utils.ProjectionAngle(orb.getRelativePositionAtUT(UT), 
                                         BodyRotationAtdT(orb.referenceBody, UT-StartUT)*pos, 
				                         orb.getOrbitalVelocityAtUT(UT)) > 0) 
                    startUT = UT;
				else StopUT = UT;
			}
            return startUT+(StopUT-startUT)/2;
		}

		protected double NextStartUT(BaseTrajectory old, double dUT, double offset, double forward_step)
		{
			var StartUT = old.StartUT+dUT;
			if(StartUT-VSL.Physics.UT-old.ManeuverDuration < offset) 
				StartUT += forward_step;
			return StartUT;
		}

		protected double AngleDelta2StartUT(BaseTrajectory old, double angle, double offset, double forward_step, double period)
		{ return NextStartUT(old, angle/360*period, offset, forward_step); }

        //deprecated
		protected Vector3d OptimizeManeuver(Func<double, Vector3d> next_dV, ref double StartUT, double offset)
		{
			Vector3d dV;
			double TTB;
			double TimeToStart = 0;
			int maxI = TRJ.PerFrameIterations;
			do {
				if(TimeToStart > 0 && TimeToStart < offset)
					StartUT += offset-TimeToStart+1;
				dV = next_dV(StartUT);
				TTB = VSL.Engines.TTB((float)dV.magnitude);
				TimeToStart = StartUT-VSL.Physics.UT-TTB/2;
			} while(maxI-- > 0 && TimeToStart < offset);
			return dV;
		}

		protected void clear_nodes()
		{
			if(VSL.vessel.patchedConicSolver == null) return;
			VSL.vessel.patchedConicSolver.maneuverNodes.ForEach(n => n.RemoveSelf());
			VSL.vessel.patchedConicSolver.maneuverNodes.Clear();
			VSL.vessel.patchedConicSolver.flightPlan.Clear();
		}


		protected bool check_patched_conics()
		{
			if(!TCAScenario.HavePatchedConics)
			{
				Status("yellow", "WARNING: maneuver nodes are not yet available. Upgrade the Tracking Station.");
                Disable();
				return false;
			}
			return true;
		}

        public override void Disable()
        {
            CFG.AP2.Off();
        }

		protected override void UpdateState()
		{
			base.UpdateState();
			IsActive &= TCAScenario.HavePatchedConics;
			ControlsActive &= TCAScenario.HavePatchedConics;
		}

		#if DEBUG
		public static bool setp_by_step_computation;
		#endif
	}

	public abstract class TrajectoryCalculator<T> : TrajectoryCalculator where T : BaseTrajectory
	{
        protected interface TrajectoryOptimizer : IEnumerable<T>
        {
            T Best { get; }
            string Status { get; }
        }

        #if DEBUG
        protected LandingTrajectory current_landing_trajectory;
        IEnumerator<T> create_trajecory_calculator(TrajectoryOptimizer optimizer)
        {
            yield return null;
            var I = 0;
            T t = null;
            var frameI = setp_by_step_computation? 1 : TRJ.PerFrameIterations;
            var ioptimizer = optimizer.GetEnumerator();
            Status("white", "{0}\nPush to continue", optimizer.Status);
            while(true)
            {
                current_landing_trajectory = t as LandingTrajectory;
                if(current_landing_trajectory != null) 
                    VSL.Info.CustomMarkersWP.Add(current_landing_trajectory.SurfacePoint);
                if(setp_by_step_computation && !string.IsNullOrEmpty(TCAGui.StatusMessage))
                { yield return t; continue; }
                if(!ioptimizer.MoveNext()) break;
                t = ioptimizer.Current;
                frameI--; I++;
                if(t == null) 
                {
                    Status("white", "{0}\nPush to continue", optimizer.Status);
                    yield return t;
                    continue;
                }
                clear_nodes();
                if(frameI <= 0)
                {
                    
                    ManeuverAutopilot.AddNodeRaw(VSL, t.NodeDeltaV, t.StartUT);
                    if(setp_by_step_computation) 
                    {
                        Log("Trajectory #{}\n{}", I, t);
                        Status("white", "{0}\nPush to continue", optimizer.Status);
                    }
                    else Status(optimizer.Status);
                    yield return t;
                    frameI = setp_by_step_computation? 1 : TRJ.PerFrameIterations;
                }
            }
            clear_nodes();
            trajectory = optimizer.Best;
            current_landing_trajectory = null;
            Log("Best trajectory:\n{}", trajectory);
        }
        #else
        IEnumerator<T> create_trajecory_calculator(TrajectoryOptimizer optimizer)
        {
            yield return null;
            var frameI = TRJ.PerFrameIterations;
            foreach(var t in optimizer)
            {
                frameI--;
                if(frameI <= 0)
                {
                    Status(optimizer.Status);
                    frameI = TRJ.PerFrameIterations;
                    yield return t;
                }
            }
            trajectory = optimizer.Best;
        }
        #endif

        protected void ComputeTrajectory(TrajectoryOptimizer optimizer)
        { trajectory_calculator = create_trajecory_calculator(optimizer); }

		protected TrajectoryCalculator(ModuleTCA tca) : base(tca) {}

		protected void add_node_abs(Vector3d dV, double UT) 
		{ ManeuverAutopilot.AddNode(VSL, dV, UT); }

        protected void add_node_rel(Vector3d dV, double UT) 
        { ManeuverAutopilot.AddNodeRaw(VSL, dV, UT); }

		protected void add_trajectory_node_rel()
		{ ManeuverAutopilot.AddNodeRaw(VSL, trajectory.NodeDeltaV, trajectory.StartUT); }

        protected void add_trajectory_node_abs()
        { ManeuverAutopilot.AddNode(VSL, trajectory.ManeuverDeltaV, trajectory.StartUT); }

		protected override void Reset()
		{
			base.Reset();
			trajectory = null;
			trajectory_calculator = null;
            #if DEBUG
            current_landing_trajectory = null;
            #endif
		}

		protected T trajectory;
		IEnumerator<T> trajectory_calculator;
		protected bool computing { get { return trajectory_calculator != null; } }
		protected virtual bool trajectory_computed()
		{
			if(trajectory != null) return true;
            if(trajectory_calculator == null)
            {
                Log("ERROR: trajectory_computed is called while trajectory_calculator is null. This should never happen.");
                return false;
            }
            if(trajectory_calculator.MoveNext()) return false;
            trajectory_calculator = null;
            if(trajectory == null) 
                update_trajectory();
            ClearStatus();
            return true;
		}

		protected abstract T CurrentTrajectory { get; }
		protected virtual void update_trajectory()
		{
			if(trajectory == null) trajectory = CurrentTrajectory;
			else trajectory.UpdateOrbit(VesselOrbit);
		}
	}

	public abstract class TargetedTrajectoryCalculator<T> : TrajectoryCalculator<T>  where T : TargetedTrajectory
	{
		protected TargetedTrajectoryCalculator(ModuleTCA tca) : base(tca) {}

        protected Orbit TargetOrbit { get { return CFG.Target.GetOrbit(); } }
        protected Vessel TargetVessel { get { return CFG.Target.GetVessel(); } }
        protected bool TargetLoaded { get { return TargetVessel != null && TargetVessel.loaded; } }

		protected ManeuverAutopilot MAN;

		protected Timer CorrectionTimer = new Timer();

        protected void add_target_node()
        {
            var dV = trajectory.BrakeDeltaV.magnitude;
            ManeuverAutopilot.AddNode(VSL, trajectory.BrakeDeltaV, 
                                      trajectory.AtTargetUT
                                      -MatchVelocityAutopilot.BrakingNodeCorrection((float)dV, VSL));
        }

		protected virtual bool check_target()
		{
            return CFG.Target;
		}

		protected virtual void setup_target()
		{
            if(VSL.HasTarget)
                SetTarget(VSL.TargetAsWP);
            else if(CFG.Target)
            {
                CFG.Target.UpdateCoordinates(Body);
                VSL.Target = CFG.Target.GetTarget();
            }
		}

		protected bool setup()
		{
			if(VSL.Engines.NoActiveEngines)
			{
				Status("yellow", "No engines are active, unable to calculate trajectory.\n" +
				       "Please, activate ship's engines and try again.");
				return false;
			}
			if(!VSL.Engines.HaveThrusters)
			{
				Status("yellow", "There are only Maneuver/Manual engines in current profile.\n" +
				       "Please, change engines profile.");
				return false;
			}
			setup_target();
			if(check_target())
			{
				clear_nodes();
				return true;
			}
			return false;
		}

		protected virtual void fine_tune_approach() {}

		protected override void UpdateState()
		{
			base.UpdateState();
            IsActive &= CFG.Target && VSL.orbit != null && VSL.orbit.referenceBody != null;
		}
	}
}

