﻿//  Author:
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

		public RendezvousTrajectory(VesselWrapper vsl, Vector3d dV, double startUT, WayPoint target, double min_PeR, double transfer_time = -1) 
			: base(vsl, dV, startUT, target) 
		{ 
			MinPeR = min_PeR;
			TransferTime = transfer_time;
            TargetOrbit = TrajectoryCalculator.NextOrbit(Target.GetOrbit(), StartUT);
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
                TrajectoryCalculator.ClosestApproach(Orbit, TargetOrbit, StartUT, VSL.Geometry.MinDistance, out AtTargetUT);
				TransferTime = AtTargetUT-StartUT;
			}
			else AtTargetUT = StartUT+TransferTime;
            var obt = TrajectoryCalculator.NextOrbit(Orbit, AtTargetUT);
            AtTargetPos = obt.getRelativePositionAtUT(AtTargetUT);
			AtTargetVel = obt.getOrbitalVelocityAtUT(AtTargetUT);
            TargetPos = TrajectoryCalculator.NextOrbit(TargetOrbit, AtTargetUT).getRelativePositionAtUT(AtTargetUT);
            DistanceToTarget = Utils.ClampL((AtTargetPos-TargetPos).magnitude-VSL.Geometry.MinDistance, 0);
			DeltaTA = Utils.ProjectionAngle(AtTargetPos, TargetPos, 
			                                Vector3d.Cross(Orbit.GetOrbitNormal(), AtTargetPos))*
				Math.Sign(TargetOrbit.period-OrigOrbit.period);
			DeltaFi = TrajectoryCalculator.RelativeInclination(Orbit, TargetPos);
			DeltaR = Vector3d.Dot(TargetPos-AtTargetPos, AtTargetPos.normalized);
			var t_orbit = Target.GetOrbit();
			var t_vel = t_orbit != null? t_orbit.getOrbitalVelocityAtUT(AtTargetUT) : Vector3d.zero;
			BrakeDeltaV = t_vel-obt.getOrbitalVelocityAtUT(AtTargetUT);
			BrakeDuration = VSL.Engines.TTB((float)BrakeDeltaV.magnitude);
            KillerOrbit = Orbit.PeR < MinPeR && Orbit.timeToPe < TransferTime;
//            Utils.Log("{}", this);//debug
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

