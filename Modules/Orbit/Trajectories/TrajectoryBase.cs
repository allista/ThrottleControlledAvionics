//  Author:
//       Allis Tauri <allista@gmail.com>
//
//  Copyright (c) 2016 Allis Tauri
//
// This work is licensed under the Creative Commons Attribution-ShareAlike 4.0 International License. 
// To view a copy of this license, visit http://creativecommons.org/licenses/by-sa/4.0/ 
// or send a letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.

using System;
using AT_Utils;

namespace ThrottleControlledAvionics
{
	public abstract class BaseTrajectory
	{
		internal static Globals GLB { get { return Globals.Instance; } }

		public readonly VesselWrapper VSL;
		public readonly Orbit OrigOrbit;
        public Orbit StartOrbit { get { return TrajectoryCalculator.NextOrbit(OrigOrbit, StartUT); } }

		public Orbit  Orbit { get; protected set; }
        public CelestialBody Body { get; protected set; }
		public double StartUT { get; protected set; }
		public double TimeToStart { get { return StartUT-VSL.Physics.UT; } }
        public Vector3d ManeuverDeltaV { get; protected set; }
        public Vector3d NodeDeltaV { get; protected set; }
		public double ManeuverDuration { get; protected set; }
		public float  ManeuverFuel { get; protected set; }
		public bool   FullManeuver { get; protected set; }

		public Vector3d StartPos { get; protected set; }
		public Vector3d StartVel { get; protected set; }
        public Vector3d AfterStartVel { get; protected set; }

        public virtual double GetTotalDeltaV() { return ManeuverDeltaV.magnitude; }
        public virtual float GetTotalFuel() { return ManeuverFuel; }

		protected BaseTrajectory(VesselWrapper vsl, Vector3d dV, double startUT)
		{
			VSL = vsl;
			var dVm = (float)dV.magnitude;
			if(dVm > 0) 
			{
				ManeuverFuel = VSL.Engines.FuelNeeded(dVm);
                FullManeuver = ManeuverFuel < VSL.Engines.AvailableFuelMass;
				ManeuverDuration = VSL.Engines.TTB_Precise(dVm);
			}
			else 
			{
				ManeuverFuel = 0;
				ManeuverDuration = 0;
                FullManeuver = true;
			}
			StartUT = startUT;
            OrigOrbit = VSL.vessel.orbitDriver.orbit;
            var obt = StartOrbit;
            Body = obt.referenceBody;
            if(dVm > 0)
            {
                ManeuverDeltaV = dV;
                NodeDeltaV = ManeuverAutopilot.Orbital2NodeDeltaV(obt, ManeuverDeltaV, StartUT);
                try 
                { 
                    Orbit = TrajectoryCalculator.NewOrbit(obt, ManeuverDeltaV, StartUT);
                    var prev = Orbit;
                    for(int i = 0; i < 3; i++)
                    {
                        if(!PatchedConics.CalculatePatch(prev, prev.nextPatch ?? new Orbit(), prev.epoch, new PatchedConics.SolverParameters(), null)) break;
                        prev = prev.nextPatch;
                    }
                    Body = Orbit.referenceBody;
//                    if(Orbit.patchEndTransition != Orbit.PatchTransitionType.FINAL)//debug
//                    {
//                        Utils.Log("**************************************************************************************************");//debug
//                        RendezvousAutopilot.log_patches(Orbit, "Orbit");//deubg
//                    }
                }
                catch(ArithmeticException) 
                { 
                    Orbit = OrigOrbit;
                    StartUT = VSL.Physics.UT;
                    ManeuverFuel = 0;
                    ManeuverDuration = 0;
                    ManeuverDeltaV = Vector3d.zero;
                    FullManeuver = true;
                }
            }
            else Orbit = OrigOrbit;
			StartPos = obt.getRelativePositionAtUT(StartUT);
			StartVel = obt.getOrbitalVelocityAtUT(StartUT);
            AfterStartVel = Orbit.getOrbitalVelocityAtUT(StartUT);
		}

		public virtual void UpdateOrbit(Orbit current)
		{
            StartUT = VSL.Physics.UT;
            Orbit = current;
            Body = Orbit.referenceBody;
			ManeuverDeltaV = Vector3d.zero;
			ManeuverDuration = 0;
            FullManeuver = true;
			StartPos = Orbit.pos;
			StartVel = Orbit.vel;
            AfterStartVel = Orbit.vel;
		}

		public override string ToString()
		{
			return Utils.Format("[{}]\n" +
			                    "{}\n" +
			                    "{}\n" +
			                    "StartUT: {} s, TimeToStart: {} s, ManeuverDuration: {} s\n" +
			                    "ManeuverDeltaV: {}\n" +
			                    "ManeuverFuel: {}, Enough: {}", 
                                GetType().Name, 
                                Utils.formatPatches(OrigOrbit, "OrigOrbit"), 
                                Utils.formatPatches(Orbit, "NewOrbit"), 
			                    StartUT, TimeToStart, 
			                    ManeuverDuration, ManeuverDeltaV, 
			                    ManeuverFuel, FullManeuver);
		}
	}

	public abstract class TargetedTrajectoryBase : BaseTrajectory
	{
		public double AtTargetUT;
		public double TransferTime;
		public Vector3d AtTargetPos { get; protected set; }
		public Vector3d AtTargetVel { get; protected set; }
        public double DistanceToTarget { get; protected set; } = double.MaxValue;
        public double DeltaFi { get; protected set; } = double.MaxValue;

		public double TimeToTarget { get { return AtTargetUT-VSL.Physics.UT; } }
		public double RelDistanceToTarget { get { return DistanceToTarget/Orbit.semiMajorAxis; } }

        public Orbit EndOrbit { get { return TrajectoryCalculator.NextOrbit(Orbit, AtTargetUT); } }

		protected TargetedTrajectoryBase(VesselWrapper vsl, Vector3d dV, double startUT) 
			: base(vsl, dV, startUT) {}

		public Vector3d BrakeDeltaV { get; protected set; }
        public float BrakeFuel { get; protected set; }
		public float BrakeDuration;
        public bool FullBrake;

        public override double GetTotalDeltaV() { return base.GetTotalDeltaV()+BrakeDeltaV.magnitude; }
        public override float GetTotalFuel() { return base.GetTotalFuel()+BrakeFuel; }

        /// <summary>
        /// The numeric "quality" of the trajectory. The less, the better, i.e. closer to the target for less dV.
        /// </summary>
        public double Quality 
        {
            get
            {
                return DistanceToTarget + ManeuverDeltaV.magnitude+BrakeDeltaV.magnitude;
//                var mV = ManeuverDeltaV.sqrMagnitude;
//                var bV = BrakeDeltaV.sqrMagnitude;
//                return 
//                    DistanceToTarget*DistanceToTarget +
//                    mV+bV+2*Math.Sqrt(mV*bV);
            }
        }

		public override string ToString()
		{ 
			return base.ToString()+
				Utils.Format("\nDistance To Target: {} m\n" +
				             "Transfer Time:  {} s\n" +
				             "Time To Target: {} s\n" +
                             "AtTargetUT {}\n\n" +

				             "Brake DeltaV:   {} m/s\n" +
                             "Brake Fuel: {}, Full Brake: {}\n" +
				             "Brake Duration: {} s\n" +
				             "DeltaFi: {} deg\n",
                             DistanceToTarget, TransferTime, TimeToTarget, AtTargetUT,
                             BrakeDeltaV, BrakeFuel, FullBrake, BrakeDuration,
				             DeltaFi);
		}
	}

	public abstract class TargetedTrajectory : TargetedTrajectoryBase
	{
		public WayPoint Target;

		protected TargetedTrajectory(VesselWrapper vsl, Vector3d dV, double startUT, WayPoint target) 
			: base(vsl, dV, startUT) { Target = target; }

		public override string ToString()
		{ 
			return base.ToString()+
                string.Format("\nTarget: {0}", Target); 
		}
	}
}

