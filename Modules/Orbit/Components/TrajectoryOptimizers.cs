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

namespace ThrottleControlledAvionics
{
	public class TrajectoryOptimizer<T> : OrbitalComponent where T : BaseTrajectory
	{
		public delegate T NextTrajectory(T old, T best);
		public delegate bool TrajectoryPredicate(T first, T second);

		public TrajectoryOptimizer(ModuleTCA tca) : base(tca) {}

		public T Trajectory { get; protected set; }
		public bool Computing { get { return optimizer != null; } }

		protected IEnumerator<T> optimizer;

		public void AddTrajectoryNode()
		{ ManeuverAutopilot.AddNode(VSL, Trajectory.ManeuverDeltaV, Trajectory.StartUT); }

		public virtual void Setup(NextTrajectory next, TrajectoryPredicate is_better, TrajectoryPredicate not_found)
		{ 
			Trajectory = null;
			optimizer = optimize_trajectory(next, is_better, not_found); 
		}

		protected IEnumerator<T> optimize_trajectory(NextTrajectory next, TrajectoryPredicate is_better, TrajectoryPredicate not_found)
		{
			T current = null;
			T best = null;
			var maxI = GLB.TRJ.MaxIterations;
			var frameI = GLB.TRJ.PerFrameIterations;
			do {
//				if(best != null && !string.IsNullOrEmpty(TCAGui.StatusMessage)) //debug
//				{ yield return null; continue; }
				current = next(current, best);
				if(best == null || is_better(current, best)) 
					best = current;
				frameI--; maxI--;
				if(frameI <= 0)
				{
//					clear_nodes(); //debug
//					add_node(current.ManeuverDeltaV, current.StartUT);//debug
//					var lnd = current as LandingTrajectory;//debug
//					if(lnd != null) add_node(lnd.BrakeDeltaV, lnd.BrakeStartUT);//debug
//					Status("Push to continue");//debug
					yield return null;
					frameI = GLB.TRJ.PerFrameIterations;
				}
			} while(not_found(current, best) && maxI > 0);
//			Log("Best trajectory:\n{0}", best);//debug
//			clear_nodes();//debug
			yield return best;
		}

		public bool TrajectoryComputed()
		{
			if(Trajectory != null) return true;
			if(optimizer == null) return false;
			if(optimizer.MoveNext())
				Trajectory = optimizer.Current;
			if(Trajectory != null)
			{ 
				optimizer = null;
				return true;
			}
			return false;
		}
	}


	public class TargetedTrajectoryOptimizer<T> : TrajectoryOptimizer<T> where T : TargetedTrajectory
	{
		public double Dtol;

		public TargetedTrajectoryOptimizer(ModuleTCA tca, double Dtol) : base(tca) { this.Dtol = Dtol; }

		protected bool target_is_far(TargetedTrajectory cur, TargetedTrajectory best)
		{ return best.DistanceToTarget > Dtol; }

		protected virtual bool trajectory_is_better(T first, T second)
		{
			return second.DistanceToTarget < 0 || first.DistanceToTarget >= 0 && 
				(first.DistanceToTarget+first.ManeuverDeltaV.magnitude+first.BrakeDeltaV.magnitude < 
				 second.DistanceToTarget+second.ManeuverDeltaV.magnitude+second.BrakeDeltaV.magnitude);
		}

		public void Setup(NextTrajectory next)
		{ optimizer = optimize_trajectory(next, trajectory_is_better, target_is_far); }

		public void AddBrakeNode()
		{
			var dV = Trajectory.BrakeDeltaV;
			ManeuverAutopilot.AddNode(VSL, dV, 
			                          Trajectory.AtTargetUT
			                          -MatchVelocityAutopilot.BrakingNodeCorrection((float)dV.magnitude, VSL));
		}
	}
}

