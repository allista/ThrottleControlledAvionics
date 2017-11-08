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
		public bool KillerOrbit { get; private set; }
        public Vector3d AtTargetRelPos { get; private set; }
        public bool DirectHit { get; private set; }
		public Orbit TargetOrbit { get; private set; }

		public RendezvousTrajectory(VesselWrapper vsl, Vector3d dV, double startUT, WayPoint target, double transfer_time = -1) 
			: base(vsl, dV, startUT, target) 
		{ 
			TransferTime = transfer_time;
            TargetOrbit = Target.GetOrbit();
			update(); 
		}

		public override void UpdateOrbit(Orbit current)
		{
			base.UpdateOrbit(current);
			TransferTime = -1;
			update();
		}

        void update_killer(Orbit obt, double endUT)
        {
            while(!KillerOrbit && obt != null && obt.referenceBody != null)
            {
                var PeR_UT = obt.StartUT+obt.timeToPe;
                var MinPeR = obt.MinPeR();
                KillerOrbit |= obt.PeR < MinPeR && PeR_UT < obt.EndUT && PeR_UT < endUT;
                if(obt.patchEndTransition == Orbit.PatchTransitionType.FINAL) break;
                obt = obt.nextPatch;
            }
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
            var t_orbit = TrajectoryCalculator.NextOrbit(TargetOrbit, AtTargetUT);
            AtTargetPos = obt.getRelativePositionAtUT(AtTargetUT);
			AtTargetVel = obt.getOrbitalVelocityAtUT(AtTargetUT);
            TargetPos = TrajectoryCalculator.RelativePosAtUT(obt.referenceBody, t_orbit, AtTargetUT);
            AtTargetRelPos = AtTargetPos-TargetPos;
            DistanceToTarget = AtTargetRelPos.magnitude-VSL.Geometry.MinDistance;
            DirectHit = DistanceToTarget < 1;
            DistanceToTarget = Utils.ClampL(DistanceToTarget, 0);
            BrakeDeltaV = t_orbit.GetFrameVelAtUT(AtTargetUT)-obt.GetFrameVelAtUT(AtTargetUT);
            var brake_dV = (float)BrakeDeltaV.magnitude;
            BrakeDuration = VSL.Engines.TTB_Precise(brake_dV);
            BrakeFuel = VSL.Engines.FuelNeeded(brake_dV);
            FullBrake = GetTotalFuel() < VSL.Engines.AvailableFuelMass;
            //check if this trajectory is too close to any of celestial bodies it passes by
            KillerOrbit = TransferTime < BrakeDuration+ManeuverDuration;
            update_killer(OrigOrbit, StartUT);
            update_killer(Orbit, AtTargetUT);
//            Utils.Log("{}", this);//debug
		}

		public override string ToString()
		{
			return base.ToString() +
				Utils.Format("\n{}\n" +
				             "Killer: {}\n",
                             Utils.formatPatches(TargetOrbit, "TargetOrbit"),
                             KillerOrbit);
		}
	}
}

