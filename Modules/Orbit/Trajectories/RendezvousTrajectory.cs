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
using AT_Utils;

namespace ThrottleControlledAvionics
{
	public class RendezvousTrajectory : TargetedTrajectory
	{
		public double SearchStart { get; private set; }
		public Vector3d TargetPos { get; private set; }
		public double DeltaTA { get; private set; }
		public double DeltaR { get; private set; }
		public double MinPeR { get; private set; }
		public bool KillerOrbit { get; private set; }
		public Orbit TargetOrbit { get; private set; }
		Vessel TargetVessel;

		public RendezvousTrajectory(VesselWrapper vsl, Vector3d dV, double startUT, WayPoint target, double min_PeR, double transfer_time = -1) 
			: base(vsl, dV, startUT, target) 
		{ 
			MinPeR = min_PeR;
			TransferTime = transfer_time;
			TargetOrbit = Target.GetOrbit();
			TargetVessel = Target.GetVessel();
			update(); 
		}

		public override void UpdateOrbit(Orbit current)
		{
			base.UpdateOrbit(current);
			TransferTime = -1;
			update();
		}

		void update()
		{
			if(TransferTime < 0)
			{
				TrajectoryCalculator.ClosestApproach(Orbit, TargetOrbit, StartUT, out AtTargetUT);
				TransferTime = AtTargetUT-StartUT;
			}
			else AtTargetUT = StartUT+TransferTime;
			AtTargetPos = Orbit.getRelativePositionAtUT(AtTargetUT);
			AtTargetVel = Orbit.getOrbitalVelocityAtUT(AtTargetUT);
			TargetPos = TargetOrbit.getRelativePositionAtUT(AtTargetUT);
			DistanceToTarget = Utils.ClampL((AtTargetPos-TargetPos).magnitude-VSL.Geometry.R-TargetVessel.Radius(), 0);
			DeltaTA = Utils.ProjectionAngle(AtTargetPos, TargetPos, 
			                                Vector3d.Cross(Orbit.GetOrbitNormal(), AtTargetPos))*
				Math.Sign(TargetOrbit.period-OrigOrbit.period);
			DeltaFi = TrajectoryCalculator.RelativeInclination(Orbit, TargetPos);
			DeltaR = Vector3d.Dot(TargetPos-AtTargetPos, AtTargetPos.normalized);
			KillerOrbit = Orbit.PeR < MinPeR && Orbit.timeToPe < TransferTime;
//			DebugUtils.Log("{}", this);//debug
		}

		public override string ToString()
		{
			return base.ToString() +
				Utils.Format("\nTargetOrbit:\n{}\n" +
				             "DeltaTA: {} deg\n" +
				             "DeltaR: {} m\n" +
				             "MinPeR: {} m\n" +
				             "Killer: {}\n",
				             TargetOrbit,
				             DeltaTA, 
				             DeltaR, MinPeR, KillerOrbit);
		}
	}
}

