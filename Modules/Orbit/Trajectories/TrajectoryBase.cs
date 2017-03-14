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
		public readonly CelestialBody Body;
		public readonly Orbit OrigOrbit;

		public Orbit  Orbit { get; protected set; }
		public double StartUT { get; protected set; }
		public double TimeToStart { get { return StartUT-VSL.Physics.UT; } }
		public Vector3d ManeuverDeltaV { get; protected set; }
		public double ManeuverDuration { get; protected set; }
		public float  ManeuverFuel { get; protected set; }
		public bool   NotEnoughFuel { get; protected set; }

		public Vector3d StartPos { get; protected set; }
		public Vector3d StartVel { get; protected set; }

		protected BaseTrajectory(VesselWrapper vsl, Vector3d dV, double startUT)
		{
			VSL = vsl;
			var dVm = (float)dV.magnitude;
			ManeuverDeltaV = dV;
			if(dVm > 0) 
			{
				ManeuverFuel = VSL.Engines.FuelNeeded(dVm);
				NotEnoughFuel = ManeuverFuel > VSL.Engines.GetAvailableFuelMass();
				ManeuverDuration = VSL.Engines.TTB(dVm);
			}
			else 
			{
				ManeuverFuel = 0;
				ManeuverDuration = 0;
			}
			StartUT = startUT;
			Body = VSL.vessel.orbitDriver.orbit.referenceBody;
            OrigOrbit = TrajectoryCalculator.NextOrbit(VSL.vessel.orbitDriver.orbit, startUT);
            try { Orbit = TrajectoryCalculator.NewOrbit(OrigOrbit, ManeuverDeltaV, StartUT); }
            catch(ArithmeticException) 
            { 
                Orbit = OrigOrbit;
                ManeuverFuel = 0;
                ManeuverDuration = 0;
                ManeuverDeltaV = Vector3d.zero;
                NotEnoughFuel = false;
            }
			StartPos = Orbit.getRelativePositionAtUT(StartUT);
			StartVel = Orbit.getOrbitalVelocityAtUT(StartUT);
		}

		public virtual void UpdateOrbit(Orbit current)
		{
            StartUT = VSL.Physics.UT;
            Orbit = TrajectoryCalculator.NextOrbit(current, StartUT);
			ManeuverDeltaV = Vector3d.zero;
			ManeuverDuration = 0;
			NotEnoughFuel = false;
			StartPos = Orbit.pos;
			StartVel = Orbit.vel;
		}

		public override string ToString()
		{
			return Utils.Format("[{}]\n" +
			                    "OrigOrbit:\n{}\n" +
			                    "NewOrbit:\n{}\n" +
			                    "StartUT: {} s, TimeToStart: {} s, ManeuverDuration: {} s\n" +
			                    "ManeuverDeltaV: {}\n" +
			                    "ManeuverFuel: {}, NotEnough: {}", 
			                    GetType().Name, OrigOrbit, Orbit, 
			                    StartUT, TimeToStart, 
			                    ManeuverDuration, ManeuverDeltaV, 
			                    ManeuverFuel, NotEnoughFuel);
		}
	}

	public abstract class TargetedTrajectoryBase : BaseTrajectory
	{
		public double AtTargetUT;
		public double TransferTime;
		public Vector3d AtTargetPos { get; protected set; }
		public Vector3d AtTargetVel { get; protected set; }
		public double DistanceToTarget { get; protected set; } = -1;
		public double DeltaFi { get; protected set; }

		public double TimeToTarget { get { return AtTargetUT-VSL.Physics.UT; } }
		public double RelDistanceToTarget { get { return DistanceToTarget/Orbit.semiMajorAxis; } }

		protected TargetedTrajectoryBase(VesselWrapper vsl, Vector3d dV, double startUT) 
			: base(vsl, dV, startUT) {}

		public Vector3d BrakeDeltaV { get; protected set; }
		public float BrakeDuration;

		public override string ToString()
		{ 
			return base.ToString()+
				Utils.Format("\nDistance To Target: {} m\n" +
				             "Transfer Time:  {} s\n" +
				             "Time To Target: {} s\n" +
				             "Brake DeltaV:   {} m/s\n" +
				             "Brake Duration: {} s\n" +
				             "DeltaFi: {} deg\n",
				             DistanceToTarget, TransferTime, TimeToTarget, 
				             BrakeDeltaV, BrakeDuration,
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
				string.Format("\nTarget: {0}\n", Target); 
		}
	}
}

