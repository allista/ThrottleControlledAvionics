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
		}
		protected static Config TRJ { get { return TCAScenario.Globals.TRJ; } }

		protected TrajectoryCalculator(ModuleTCA tca) : base(tca) {}

		protected Orbit VesselOrbit { get { return VSL.vessel.orbitDriver.orbit; } }
		protected CelestialBody Body { get { return VSL.vessel.orbitDriver.orbit.referenceBody; } }

		public static Vector3d hV(Orbit o, double UT) 
		{ 
			return Vector3d.Exclude(o.getRelativePositionAtUT(UT), 
			                        o.getOrbitalVelocityAtUT(UT));
		}

		protected Vector3d hV(double UT) { return hV(VesselOrbit, UT); }
		protected double MinPeR { get { return Body.atmosphere? Body.Radius+Body.atmosphereDepth+1000 : Body.Radius+TRJ.MinPeA; } }
		protected bool ApAhead { get { return VesselOrbit.timeToAp < VesselOrbit.timeToPe; } }

		public static Orbit NewOrbit(Orbit old, Vector3d dV, double UT)
		{
			var obt = new Orbit();
			var pos = old.getRelativePositionAtUT(UT);
			var vel = old.getOrbitalVelocityAtUT(UT)+dV;
			obt.UpdateFromStateVectors(pos, vel, old.referenceBody, UT);
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

		protected Orbit CircularOrbit(double UT)
		{ return NewOrbit(VesselOrbit, dV4C(VesselOrbit, hV(UT), UT), UT); }

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
				Utils.Log("R: {0}, nR {1}, dV [{2}-{3}]", R, nR, min_dV, max_dV);//debug
			}
			return (max_dV+min_dV)/2*dVdir+add_dV;
		}

		public static Vector3d dV4Ap(Orbit old, double R, double UT, Vector3d add_dV = default(Vector3d))
		{
			var up     = old.ApR < R;
			var vel    = hV(old, UT);
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
				Utils.Log("R: {0}, nR {1}, dV [{2}-{3}]", R, nR, min_dV, max_dV);//debug
			}
			return (max_dV+min_dV)/2*dVdir+add_dV;
		}

		public static Vector3d dV4R(Orbit old, double R, double UT, double TargetUT, Vector3d add_dV = default(Vector3d))
		{
			var oldR   = old.getRelativePositionAtUT(TargetUT);
			var up     = oldR.magnitude < R;
			var dVdir  = hV(old, UT).normalized * (up? 1 : -1);
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
				Utils.LogF("dUT: {}, R: {}, dR {}, dV [{}-{}]", TargetUT-UT, R, nR-R, min_dV, max_dV);//debug
			}
			return (max_dV+min_dV)/2*dVdir+add_dV;
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

		public static double TimeToResonance(Orbit a, Orbit b, double UT, out double resonance, out double alpha)
		{
			var velA = a.getOrbitalVelocityAtUT(UT);
			var posA = a.getRelativePositionAtUT(UT);
			var posB = b.getRelativePositionAtUT(UT);
			alpha = Utils.ProjectionAngle(posA, posB, velA)/360;
			resonance = ResonanceA(a, b);
			var TTR = alpha*resonance;
			return TTR > 0? TTR : TTR+Math.Abs(resonance);
		}

		public static Vector3d dV4T(Orbit old, double T, double UT)
		{
			var body = old.referenceBody;
			var vel = old.getOrbitalVelocityAtUT(UT);
			var pos = old.getRelativePositionAtUT(UT);
			var R   = pos.magnitude;
			var sma = Math.Pow(body.gravParameter*T*T/Utils.TwoPI/Utils.TwoPI, 1/3.0);
			Utils.Log("\n{0}",
			    Utils.Format("vel {}\nR {}\nsma {}\nnew vel {}",
			                 vel, R, sma, Math.Sqrt((2/R - 1/sma)*body.gravParameter)));
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
			Utils.LogF("\nTTR {}, alpha {}, resonance {}", TTR, alpha, resonance);//debug
			if(TTR > max_TTR) dV = dV4Resonance(old, target, max_TTR*0.9, alpha, UT);
			else if(TTR > 1/2) return Vector3d.zero;
			else dV = dV4Resonance(old, target, 3/4, alpha, UT);
			min_dV = dV.magnitude;
			dVdir  = dV/min_dV;
			Utils.LogF("\ndV {}\n dV*velN {}", dV, Vector3d.Dot(dV, old.getOrbitalVelocityAtUT(UT).normalized));//debug
			if(min_dV > max_dV) return dVdir*max_dV;
			if(NewOrbit(old, dV, UT).PeR > min_PeR) return dV;
			max_dV = min_dV;
			min_dV = 0;
			//tune orbit for maximum dV but PeR above the min_PeR
			while(max_dV-min_dV > TRJ.dVtol)
			{
				var dVm  = (max_dV+min_dV)/2;
				var orb  = NewOrbit(old, dVdir*dVm, UT);
				var nTTR = TimeToResonance(orb, target, UT, out resonance, out alpha);//debug
				Utils.LogF("\nPeR {}, minPeR {}\ndVm {} [{} : {}], TTR: {}", 
				               orb.PeR, min_PeR, dVm, min_dV, max_dV, nTTR);//debug
				if(orb.PeR > min_PeR) min_dV = dVm;
				else max_dV = dVm;
			}
			return dVdir*(max_dV+min_dV)/2;
		}

		public static double SqrDistAtUT(Orbit a, Orbit b, double UT)
		{ return (a.getRelativePositionAtUT(UT)-b.getRelativePositionAtUT(UT)).sqrMagnitude; }

		public static double ClosestApproach(Orbit a, Orbit t, double StartUT, out double ApproachUT, double offset = 0, int max_iterations = 20)
		{
			int iters  = max_iterations;
			double dT  = a.period/3;
			double StopUT = StartUT+a.period;
			double MidUT  = StartUT+a.period/2;
			double UT1 = StartUT;
			double UT2 = StopUT;
			double d1 = double.MaxValue;
			double d2 = double.MaxValue;
			while(iters-- > 0 && dT > 0.01)
			{
				var d1p = UT1+dT > MidUT?   double.MaxValue : SqrDistAtUT(a, t, UT1+dT);
				var d1m = UT1-dT < StartUT? double.MaxValue : SqrDistAtUT(a, t, UT1-dT);
				var d2p = UT2+dT > StopUT?  double.MaxValue : SqrDistAtUT(a, t, UT2+dT);
				var d2m = UT2-dT < MidUT?   double.MaxValue : SqrDistAtUT(a, t, UT2-dT);
				if(d1p < d1m) { if(d1p < d1) { d1 = d1p; UT1 += dT; } }
				else { if(d1m < d1) { d1 = d1m; UT1 -= dT; } }
				if(d2p < d2m) { if(d2p < d2) { d2 = d2p; UT2 += dT; } }
				else { if(d2m < d2) { d2 = d2m; UT2 -= dT; } }
				dT /= 2;
			}
			if(d1 < d2 && UT1-StartUT > offset) { ApproachUT = UT1; return Math.Sqrt(d1); }
			ApproachUT = UT2; return Math.Sqrt(d2);
		}

		public static double NearestApproach(Orbit a, Orbit t, double StartUT, out double ApproachUT, int max_iterations = 20)
		{
			int iters  = max_iterations;
			double dT  = a.period/4;
			double StopUT = StartUT+a.period;
			double UT1 = StartUT;
			double d1 = -1;
			while(iters-- > 0 && dT > 0.01)
			{
				var d1p = UT1+dT > StopUT?  double.MaxValue : SqrDistAtUT(a, t, UT1+dT);
				var d1m = UT1-dT < StartUT? double.MaxValue : SqrDistAtUT(a, t, UT1-dT);
				if(d1p < d1m) { d1 = d1p; UT1 += dT; }
				else { d1 = d1m; UT1 -= dT; }
				dT /= 1.3333333333333333;
			}
			ApproachUT = UT1; return Math.Sqrt(d1);
		}

		protected Vector3d RadiusCorrection(RendezvousTrajectory old)
		{
			return new Vector3d(0,0, old.DeltaR * Body.gMagnitudeAtCenter/old.AtTargetPos.sqrMagnitude * TRJ.dV4dRf *
			                    Math.Sin(old.TimeToTarget / old.NewOrbit.period * Math.PI));
		}

		protected static Vector3d AoPCorrection(TargetedTrajectoryBase old, double amount)
		{ return new Vector3d(amount*Math.Sign(old.NewOrbit.period/2-old.TimeToTarget), 0, 0); }

		protected Vector3d PlaneCorrection(TargetedTrajectoryBase old, double angle)
		{
			angle *= Math.Abs(Math.Sin(old.TimeToTarget/old.NewOrbit.period*2*Math.PI))*
				Math.Sign(old.NewOrbit.period/2-old.TimeToTarget);
			var rot = QuaternionD.AngleAxis(-angle, old.AtTargetPos);
			return Orbit2NodeDeltaV(old.StartUT, (rot*old.StartVel)-old.StartVel);
		}

		protected Vector3d PlaneCorrection(TargetedTrajectoryBase old)
		{ return PlaneCorrection(old, old.DeltaFi); }

		protected Vector3d Orbit2NodeDeltaV(double StartUT, Vector3d OrbitDeltaV)
		{
			var norm = VesselOrbit.GetOrbitNormal().normalized;
			var prograde = hV(StartUT).normalized;
			var radial = Vector3d.Cross(prograde, norm).normalized;
			return new Vector3d(Vector3d.Dot(OrbitDeltaV, radial),
			                    Vector3d.Dot(OrbitDeltaV, norm),
			                    Vector3d.Dot(OrbitDeltaV, prograde));
		}

		protected Vector3d Node2OrbitDeltaV(double StartUT, Vector3d NodeDeltaV)
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
				TTB = ManeuverAutopilot.TTB(VSL, (float)dV.magnitude, 1);
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
	}

	public abstract class TrajectoryCalculator<T> : TrajectoryCalculator where T : BaseTrajectory
	{
		protected TrajectoryCalculator(ModuleTCA tca) : base(tca) {}

		public delegate T NextTrajectory(T old);
		public delegate bool TrajectoryPredicate(T trj);

		protected void add_node(Vector3d dV, double UT) 
		{ ManeuverAutopilot.AddNode(VSL, dV, UT); }

		protected void add_trajectory_node()
		{ ManeuverAutopilot.AddNode(VSL, trajectory.ManeuverDeltaV, trajectory.StartUT); }

		protected virtual void reset()
		{
			trajectory = null;
			trajectory_calculator = null;
		}

		IEnumerator<T> compute_trajectory()
		{
			Log("========= Computing trajectory =========");//debug
			T current = null;
			T best = null;
			var maxI = TRJ.MaxIterations;
			var frameI = TRJ.PerFrameIterations;
			Status("");//debug
			do {
//				//debug
//				if(best != null && 
//				   !string.IsNullOrEmpty(ThrottleControlledAvionics.StatusMessage)) 
//				{
//					yield return null;
//					continue;
//				}
//				else 
				clear_nodes();
//				//debug
				current = next_trajectory(current);
				if(best == null || current.IsBetter(best)) 
					best = current;
				frameI--; maxI--;
//				//debug
				add_node(current.ManeuverDeltaV, current.StartUT);
//				Status("Push to continue");
//				//debug
				if(frameI < 0)
				{
					yield return null;
					frameI = TRJ.PerFrameIterations;
				}
			} while(predicate(best) && maxI > 0);
			Log("Best trajectory:\n{0}", best);//debug
			clear_nodes();//debug
			yield return best;
		}

		protected T trajectory;
		protected NextTrajectory next_trajectory;
		protected TrajectoryPredicate predicate;
		IEnumerator<T> trajectory_calculator;
		protected bool computing { get { return trajectory_calculator != null; } }
		protected bool trajectory_computed()
		{
			if(trajectory != null) return true;
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
	}

	public abstract class TargetedTrajectoryCalculator<T, TT> : TrajectoryCalculator<T> 
		where T : TargetedTrajectory<TT> where TT : class, ITargetable
	{
		protected TargetedTrajectoryCalculator(ModuleTCA tca) : base(tca) {}

		protected ManeuverAutopilot MAN;

		protected double Dtol;
		protected TT Target;
		protected Timer CorrectionTimer = new Timer();

		protected bool target_is_far(T trj)
		{ return trj.DistanceToTarget > Dtol; }

		protected void add_target_node()
		{
			var dV = trajectory.Target.GetOrbit().getOrbitalVelocityAtUT(trajectory.AtTargetUT) -
				trajectory.NewOrbit.getOrbitalVelocityAtUT(trajectory.AtTargetUT);
			var dVm = (float)dV.magnitude;
			var max = (float)Dtol/10;
			var offset = dVm > max? ManeuverAutopilot.TTB(VSL, dVm-max, 1)-ManeuverAutopilot.TTB(VSL, dVm, 1)/2 : 0;
			LogF("dVm {}, max {}, offset {}", dVm, max, offset);
			ManeuverAutopilot.AddNode(VSL, dV, trajectory.AtTargetUT-offset);
		}

		protected void setup_calculation(NextTrajectory next)
		{
			next_trajectory = next;
			predicate = target_is_far;
		}

		protected override void reset()
		{
			base.reset();
			Target = null;
		}

		protected virtual bool check_target()
		{
			if(!VSL.HasTarget) return false;
			if(VSL.Target is WayPoint) return true;
			var orb = VSL.Target.GetOrbit();
			if(orb == null) return false;
			if(orb.referenceBody != VSL.mainBody)
			{
				Status("yellow", "This autopilot requires a target to be\n" +
				       "in the sphere of influence of the same planetary body.");
				return false;
			}
			return true;
		}

		protected abstract void setup_target();

		protected bool setup()
		{
			if(check_target())
			{
				clear_nodes();
				setup_target();
				return true;
			}
			return false;
		}
	}
}

