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
			[Persistent] public float dV4dRf             = 1e-4f;
			[Persistent] public float MinPeA             = 10000f; //m
			[Persistent] public int   MaxIterations      = 1000;
			[Persistent] public int   PerFrameIterations = 10;
			[Persistent] public float ManeuverOffset     = 60f;    //s
		}
		protected static Config TRJ { get { return Globals.Instance.TRJ; } }

		protected TrajectoryCalculator(ModuleTCA tca) : base(tca) {}
		//multiple inheritance or some sort of mixins or property extensions would be great here =/
		protected Orbit VesselOrbit { get { return VSL.vessel.orbitDriver.orbit; } }
		protected CelestialBody Body { get { return VSL.vessel.orbitDriver.orbit.referenceBody; } }
		protected Vector3d SurfaceVel {get { return Vector3d.Cross(-Body.zUpAngularVelocity, VesselOrbit.pos); } }
		protected double MinPeR { get { return Body.atmosphere? Body.Radius+Body.atmosphereDepth+1000 : Body.Radius+TRJ.MinPeA; } }

		protected Vector3d hV(double UT) { return VesselOrbit.hV(UT); }

		protected bool LiftoffPossible
		{
			get
			{
				if(VSL.Engines.NumActive > 0 && VSL.OnPlanet && VSL.OnPlanetParams.MaxTWR <= 1)
				{
					Status("red", "TWR < 1, impossible to achive orbit");
					CFG.AP2.Off();
					return false;
				}
				return true;
			}
		}

		public static Orbit NewOrbit(Orbit old, Vector3d dV, double UT)
		{
			var obt = new Orbit();
			var pos = old.getRelativePositionAtUT(UT);
			var vel = old.getOrbitalVelocityAtUT(UT)+dV;
			obt.UpdateFromStateVectors(pos, vel, old.referenceBody, UT);
			if(obt.eccentricity < 0.01) 
			{
				Func<double,double> dist = t => (vel-obt.getOrbitalVelocityAtObT(t)).sqrMagnitude;
				var T  = obt.ObT;
				var dT = obt.period/10;
				var D  = dist(T);
				while(D > 1e-5)
				{
					T += dT;
					if(T < 0) T += obt.period;
					else if(T > obt.period) T -= obt.period;
					var d = dist(T);
					if(d > D) dT /= -2.1;
					D = d;
				}
				if(!T.Equals(obt.ObT))
				{
					var dP = (T-obt.ObT)/obt.period;
					obt.argumentOfPeriapsis = (obt.argumentOfPeriapsis-dP*360)%360;
					obt.meanAnomaly = (obt.meanAnomaly+dP*Utils.TwoPI)%Utils.TwoPI;
					obt.meanAnomalyAtEpoch = obt.meanAnomaly;
					obt.orbitPercent = obt.meanAnomaly/Utils.TwoPI;
					obt.eccentricAnomaly = obt.solveEccentricAnomaly(obt.meanAnomaly, obt.eccentricity, 1e-7, 8);
					obt.trueAnomaly = obt.getTrueAnomaly(obt.eccentricAnomaly)/Math.PI*180;
					obt.ObT = obt.ObTAtEpoch = T;
				}
			}
			return obt;
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

		public static Vector3d dV4Pe(Orbit old, double R, double UT, Vector3d add_dV = default(Vector3d))
		{
			var up     = old.PeR < R;
			var pos    = old.getRelativePositionAtUT(UT);
			var vel    = Vector3d.Exclude(pos, old.getOrbitalVelocityAtUT(UT));
			var dVdir  = vel.normalized * (up? 1 : -1);
			var min_dV = 0.0;
			var max_dV = 0.0;
			if(up)
			{
				max_dV = 1;
				var max_PeR = pos.magnitude;
				if(R > max_PeR) R = max_PeR;
				while(NewOrbit(old, dVdir*max_dV, UT).PeR < R)
				{ max_dV *= 2; if(max_dV > 100000) break; }
			}
			else max_dV = vel.magnitude+add_dV.magnitude;
			while(max_dV-min_dV > TRJ.dVtol)
			{
				var dV = (max_dV+min_dV)/2;
				var nR = NewOrbit(old, dVdir*dV+add_dV, UT).PeR;
				if(up && nR > R || !up && nR < R) max_dV = dV;
				else min_dV = dV;
			}
			return (max_dV+min_dV)/2*dVdir+add_dV;
		}

		public static Vector3d dV4Ap(Orbit old, double R, double UT, Vector3d add_dV = default(Vector3d))
		{
			var up     = old.ApR < R;
			var vel    = old.hV(UT);
			var dVdir  = vel.normalized * (up? 1 : -1);
			var min_dV = 0.0;
			var max_dV = 0.0;
			if(up)
			{
				max_dV = 1;
				while(NewOrbit(old, dVdir*max_dV, UT).ApR < R)
				{ max_dV *= 2; if(max_dV > 100000) break; }
			}
			else 
			{
				var min_ApR = old.getRelativePositionAtUT(UT).magnitude;
				if(R < min_ApR) R = min_ApR;
				max_dV = vel.magnitude+add_dV.magnitude;
			}
			while(max_dV-min_dV > TRJ.dVtol)
			{
				var dV = (max_dV+min_dV)/2;
				var nR = NewOrbit(old, dVdir*dV+add_dV, UT).ApR;
				if(up && nR > R || !up && nR < R) max_dV = dV;
				else min_dV = dV;
			}
			return (max_dV+min_dV)/2*dVdir+add_dV;
		}

		public static Vector3d dV4R(Orbit old, double R, double UT, double TargetUT, Vector3d add_dV = default(Vector3d))
		{
			var oldR   = old.getRelativePositionAtUT(TargetUT);
			var up     = oldR.magnitude < R;
			var dVdir  = old.hV(UT).normalized * (up? 1 : -1);
			var min_dV = 0.0;
			var max_dV = 0.0;
			if(up)
			{
				max_dV = 1;
				while(NewOrbit(old, dVdir*max_dV, UT)
				      .getRelativePositionAtUT(TargetUT)
				      .magnitude < R)
				{ max_dV *= 2; if(max_dV > 100000) break; }
			}
			else max_dV = old.getOrbitalVelocityAtUT(UT).magnitude+add_dV.magnitude;
			while(max_dV-min_dV > TRJ.dVtol)
			{
				var dV = (max_dV+min_dV)/2;
				var nR = NewOrbit(old, dVdir*dV+add_dV, UT)
					.getRelativePositionAtUT(TargetUT)
					.magnitude;
				if(up && nR > R || !up && nR < R) max_dV = dV;
				else min_dV = dV;
			}
			return (max_dV+min_dV)/2*dVdir+add_dV;
		}

		public static Vector3d dV4Ecc(Orbit old, double ecc, double UT, double maxR = -1)
		{
			var up = old.eccentricity > ecc;
			var dir = old.getOrbitalVelocityAtUT(UT);
			var min_dV = 0.0;
			var max_dV = up? dV4C(old, dir, UT).magnitude : dir.magnitude;
			if(!up) dir *= -1;
			dir.Normalize();
			while(max_dV-min_dV > TRJ.dVtol)
			{
				var dV = (max_dV+min_dV)/2;
				var orb = NewOrbit(old, dir*dV, UT);
				if( up && (orb.eccentricity < ecc || maxR > 0 && orb.PeR > maxR) || 
				   !up && orb.eccentricity > ecc) 
					max_dV = dV;
				else min_dV = dV;
			}
			return (max_dV+min_dV)/2*dir;
		}

		protected Orbit AscendingOrbit(double ApR, Vector3d hVdir, double slope)
		{
			var LaunchRad = Utils.ClampH(Math.Atan(1/(Body.Radius*slope/(2*ApR) - 
			                                          Body.angularV/Math.Sqrt(2*VSL.Physics.StG*(ApR-Body.Radius)))), 
			                             Utils.HalfPI);
			var velN = (Math.Sin(LaunchRad)*VesselOrbit.pos.normalized + Math.Cos(LaunchRad)*hVdir).normalized;
			var vel = Math.Sqrt(2*VSL.Physics.G*(ApR-Body.Radius)) / Math.Sin(LaunchRad);
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
		{ return 90-Vector3d.Angle(orb.GetOrbitNormal(), srf_pos); }

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

		public static Vector3d dV4Resonance(Orbit old, Orbit target, double TTR, double alpha, double UT)
		{ 
			if(alpha < 0) 
			{ 
				var minTTR = -alpha*target.period/old.period*1.1;
				if(TTR < minTTR) TTR = minTTR;
			}
			return dV4T(old, target.period/(1+alpha*target.period/old.period/TTR), UT);
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
			double min_dV;
			Vector3d dV, dVdir;
			double alpha, resonance;
			var TTR = TimeToResonance(old, target, UT, out resonance, out alpha);
//			Utils.Log("\nTTR {}, alpha {}, resonance {}", TTR, alpha, resonance);//debug
			if(TTR > max_TTR) dV = dV4Resonance(old, target, Math.Max(max_TTR/2, 0.75), alpha, UT);
			else return Vector3d.zero;
			min_dV = dV.magnitude;
			dVdir  = dV/min_dV;
			if(min_dV > max_dV) min_dV = max_dV;
			if(NewOrbit(old, dV, UT).PeR > min_PeR) return dVdir*min_dV;
			max_dV = min_dV;
			min_dV = 0;
			//tune orbit for maximum dV but PeR above the min_PeR
			while(max_dV-min_dV > TRJ.dVtol)
			{
				var dVm  = (max_dV+min_dV)/2;
				var orb  = NewOrbit(old, dVdir*dVm, UT);
				if(orb.PeR > min_PeR) min_dV = dVm;
				else max_dV = dVm;
			}
			return dVdir*(max_dV+min_dV)/2;
		}

		public static double SqrDistAtUT(Orbit a, Orbit b, double UT)
		{ return (a.getRelativePositionAtUT(UT)-b.getRelativePositionAtUT(UT)).sqrMagnitude; }

		public static double ClosestApproach(Orbit a, Orbit t, double StartUT, out double ApproachUT)
		{
			double UT1;
			double D1 = NearestApproach(a, t, StartUT, StartUT+a.period, out UT1);
			double UT2;
			double D2 = NearestApproach(a, t, StartUT+a.period, StartUT, out UT2);
			if(D1 <= D2 || Math.Abs(D1-D2) < GLB.REN.Dtol) { ApproachUT = UT1; return D1; }
			ApproachUT = UT2; return D2;
		}

		public static double NearestApproach(Orbit a, Orbit t, double StartUT, out double ApproachUT)
		{ return NearestApproach(a, t, StartUT, StartUT+a.period, out ApproachUT); }

		public static double NearestApproach(Orbit a, Orbit t, double StartUT, double StopUT, out double ApproachUT)
		{
			double UT = StartUT;
			double dT = (StopUT-StartUT)/10;
			bool dir = dT > 0;
			double lastD = double.MaxValue;
			double minD  = double.MaxValue;
			double minUT = UT;
			while(Math.Abs(dT) > 0.01)
			{
				var d = SqrDistAtUT(a, t, UT);
				if(d < minD) { minD = d; minUT = UT; }
				if(d > lastD || 
				   (dir? UT+dT < StartUT : UT+dT > StartUT) ||
				   (dir? UT+dT > StopUT  : UT+dT < StopUT))
					dT /= -2.1;
				lastD = d;
				UT += dT;
			}
			ApproachUT = minUT; return Math.Sqrt(minD);
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
			var ini_error = Utils.ClampedProjectionAngle(orb.getRelativePositionAtUT(StartUT), pos, 
			                                      		 orb.getOrbitalVelocityAtUT(StartUT))/360*orb.period;
			var dT = orb.period/10; 
			StartUT += Utils.ClampL(ini_error-dT, 0);
			var StopUT = StartUT;
			while(StopUT-StartUT < orb.period)
			{
				if(Utils.ProjectionAngle(orb.getRelativePositionAtUT(StopUT), pos, 
				                         orb.getOrbitalVelocityAtUT(StopUT)) < 0) break;
				StopUT += dT;
			}
			StartUT = Math.Max(StartUT, StopUT-dT);
			while(StopUT-StartUT > 0.01)
			{
				var UT = StartUT+(StopUT-StartUT)/2;
				if(Utils.ProjectionAngle(orb.getRelativePositionAtUT(UT), pos, 
				                         orb.getOrbitalVelocityAtUT(UT)) > 0) 
					StartUT = UT;
				else StopUT = UT;
			}
			return StartUT+(StopUT-StartUT)/2;
		}

		//Node: radial, normal, prograde
		protected Vector3d Orbit2NodeDeltaV(Vector3d OrbitDeltaV, double StartUT)
		{
			var norm = VesselOrbit.GetOrbitNormal().normalized;
			var prograde = hV(StartUT).normalized;
			var radial = Vector3d.Cross(prograde, norm).normalized;
			return new Vector3d(Vector3d.Dot(OrbitDeltaV, radial),
			                    Vector3d.Dot(OrbitDeltaV, norm),
			                    Vector3d.Dot(OrbitDeltaV, prograde));
		}

		protected Vector3d Node2OrbitDeltaV(Vector3d NodeDeltaV, double StartUT)
		{ 
			var norm = VesselOrbit.GetOrbitNormal().normalized;
			var prograde = hV(StartUT).normalized;
			var radial = Vector3d.Cross(prograde, norm).normalized;
			return radial*NodeDeltaV.x + norm*NodeDeltaV.y + prograde*NodeDeltaV.z;
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
			VSL.vessel.patchedConicSolver.maneuverNodes.ForEach(n => n.RemoveSelf());
			VSL.vessel.patchedConicSolver.maneuverNodes.Clear();
			VSL.vessel.patchedConicSolver.flightPlan.Clear();
		}


		protected bool check_patched_conics()
		{
			if(!TCAScenario.HavePatchedConics)
			{
				Status("yellow", "WARNING: maneuver nodes are not yet available. Upgrade the Tracking Station.");
				CFG.AP2.Off(); 
				return false;
			}
			return true;
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
		protected TrajectoryCalculator(ModuleTCA tca) : base(tca) {}

		public delegate T NextTrajectory(T old, T best);
		public delegate bool TrajectoryPredicate(T first, T second);

		protected TimeWarpControl WRP;

		protected void add_node(Vector3d dV, double UT) 
		{ ManeuverAutopilot.AddNode(VSL, dV, UT); }

		protected void add_trajectory_node()
		{ ManeuverAutopilot.AddNode(VSL, trajectory.ManeuverDeltaV, trajectory.StartUT); }

		protected override void reset()
		{
			base.reset();
			trajectory = null;
			trajectory_calculator = null;
		}

		#if DEBUG
		IEnumerator<T> compute_trajectory()
		{
			T current = null;
			T best = null;
			var maxI = TRJ.MaxIterations;
			var frameI = setp_by_step_computation? 1 : TRJ.PerFrameIterations;
			do {
				var lt = current as LandingTrajectory;
				if(lt != null) VSL.Info.CustomMarkersWP.Add(lt.SurfacePoint);
				if(setp_by_step_computation && !string.IsNullOrEmpty(TCAGui.StatusMessage))
				{ yield return null; continue; }
				clear_nodes();
				current = next_trajectory(current, best);
				if(best == null || better_predicate(current, best)) 
					best = current;
				frameI--; maxI--;
				if(frameI <= 0)
				{
					add_node(current.ManeuverDeltaV, current.StartUT);
					if(setp_by_step_computation) 
					{
						Log("Trajectory #{}\n{}", TRJ.MaxIterations-maxI, current);
						Status("Push to continue");
					}
					else Status("Computing trajectory...");
					yield return null;
					frameI = setp_by_step_computation? 1 : TRJ.PerFrameIterations;
				}
			} while(current == null || best == null || end_predicate(current, best) && maxI > 0);
			Log("Best trajectory:\n{}", best);
			clear_nodes();
			yield return best;
		}
		#else
		IEnumerator<T> compute_trajectory()
		{
			T current = null;
			T best = null;
			var maxI = TRJ.MaxIterations;
			var frameI = TRJ.PerFrameIterations;
			do {
				current = next_trajectory(current, best);
				if(best == null || better_predicate(current, best)) 
					best = current;
				frameI--; maxI--;
				if(frameI <= 0)
				{
					yield return null;
					frameI = TRJ.PerFrameIterations;
				}
			} while(end_predicate(current, best) && maxI > 0);
			yield return best;
		}
		#endif

		protected T trajectory;
		protected NextTrajectory next_trajectory;
		protected TrajectoryPredicate end_predicate;
		protected TrajectoryPredicate better_predicate;
		IEnumerator<T> trajectory_calculator;
		protected bool computing { get { return trajectory_calculator != null; } }
		protected bool trajectory_computed()
		{
			if(trajectory != null) return true;
			#if !DEBUG
			Status("Computing trajectory...");	
			#endif
			if(trajectory_calculator == null)
				trajectory_calculator = compute_trajectory();
			if(trajectory_calculator.MoveNext())
				trajectory = trajectory_calculator.Current;
			if(trajectory != null)
			{ 
				trajectory_calculator = null;
				return true;
			}
			return false;
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

		protected ManeuverAutopilot MAN;

		protected double Dtol;
		protected Timer CorrectionTimer = new Timer();

		protected bool target_is_far(T cur, T best)
		{ 
			return best.DistanceToTarget > Dtol && 
				(cur == best || Math.Abs(cur.DistanceToTarget-best.DistanceToTarget)/Dtol > 0.1); //convergence
		}

		protected void add_target_node()
		{
			var dV = trajectory.BrakeDeltaV;
			ManeuverAutopilot.AddNode(VSL, dV, 
			                          trajectory.AtTargetUT
			                          -MatchVelocityAutopilot.BrakingNodeCorrection((float)dV.magnitude, VSL));
		}

		protected virtual bool trajectory_is_better(T first, T second)
		{
			return second.DistanceToTarget < 0 || first.DistanceToTarget >= 0 && 
				(first.DistanceToTarget+first.ManeuverDeltaV.magnitude+first.BrakeDeltaV.magnitude < 
				 second.DistanceToTarget+second.ManeuverDeltaV.magnitude+second.BrakeDeltaV.magnitude);
		}

		protected virtual void setup_calculation(NextTrajectory next)
		{
			next_trajectory = next;
			end_predicate = target_is_far;
			better_predicate = trajectory_is_better;
		}

		protected virtual bool check_target()
		{
			if(CFG.Target == null) return false;
			var orb = CFG.Target.GetOrbit();
			if(orb != null && orb.referenceBody != VSL.Body)
			{
				Status("yellow", "This autopilot requires a target to be\n" +
				       "in the sphere of influence of the same planetary body.");
				return false;
			}
			return true;
		}

		protected virtual void setup_target()
		{
			SetTarget(Target2WP());
			if(CFG.Target != null)
				CFG.Target.UpdateCoordinates(Body);
		}

		protected bool setup()
		{
			if(VSL.Engines.NoActiveEngines)
			{
				Status("yellow", "No engines are active, unable to calculate trajectory.\n" +
					"Please, activate ship's engines and try again.");
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
			IsActive &= CFG.Target != null && VSL.orbit != null && VSL.orbit.referenceBody != null;
		}
	}
}

