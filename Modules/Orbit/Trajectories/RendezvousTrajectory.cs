//  Author:
//       allis <>
//
//  Copyright (c) 2016 allis
//
// This work is licensed under the Creative Commons Attribution-ShareAlike 4.0 International License. 
// To view a copy of this license, visit http://creativecommons.org/licenses/by-sa/4.0/ 
// or send a letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.
//
using System;

namespace ThrottleControlledAvionics
{
	public class RendezvousTrajectory : TargetedTrajectory<Vessel>
	{
		public double TargetOffset { get; private set; }
		public double SearchStart { get; private set; }
		public Vector3d TargetPos { get; private set; }
		public double DeltaTA { get; private set; }
		public double DeltaR { get; private set; }

		public RendezvousTrajectory(VesselWrapper vsl, Vector3d dV, double startUT, Vessel target, 
		                            double offset, double search_start_ut = -1, bool nearest_approach = false) 
			: base(vsl, dV, startUT, target) 
		{ 
			TargetOffset = offset;
			SearchStart = search_start_ut > 0? search_start_ut : StartUT; 
			update(nearest_approach); 
		}

		public override void UpdateOrbit(Orbit current)
		{
			base.UpdateOrbit(current);
			update(false);
		}

		void update(bool nearest_approach)
		{
			var dist = nearest_approach?
				TrajectoryCalculator.NearestApproach(NewOrbit, Target.orbit, SearchStart, out AtTargetUT) :
				TrajectoryCalculator.ClosestApproach(NewOrbit, Target.orbit, SearchStart, out AtTargetUT, TargetOffset);
			DistanceToTarget = Utils.ClampL(dist-VSL.Geometry.R, 0);
			TimeToTarget = AtTargetUT-StartUT;
			AtTargetPos = NewOrbit.getRelativePositionAtUT(AtTargetUT);
			AtTargetVel = NewOrbit.getOrbitalVelocityAtUT(AtTargetUT);
			TargetPos = Target.orbit.getRelativePositionAtUT(AtTargetUT);
			DeltaTA = Utils.ProjectionAngle(AtTargetPos, TargetPos, 
			                                Vector3d.Exclude(AtTargetPos, AtTargetVel))*
				Math.Sign(Target.orbit.period-OrigOrbit.period);
			DeltaFi = 90-Vector3d.Angle(NewOrbit.GetOrbitNormal(), TargetPos);
			DeltaR = Vector3d.Dot(TargetPos-AtTargetPos, AtTargetPos.normalized);

			Utils.Log("{0}", this);//debug
		}

		public override string ToString()
		{
			return base.ToString() +
				Utils.Format("\n\nTargetOrbit:\n{}\n" +
				             "DeltaTA: {} deg\n" +
				             "TimeToTarget: {} s\n" +
				             "DeltaR: {} m\n",
				             Target.orbit,
				             DeltaTA, 
				             TimeToTarget, 
				             DeltaR);
		}
	}
}

