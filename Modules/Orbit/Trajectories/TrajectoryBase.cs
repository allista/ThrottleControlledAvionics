//  Author:
//       Allis Tauri <allista@gmail.com>
//
//  Copyright (c) 2016 Allis Tauri
//
// This work is licensed under the Creative Commons Attribution-ShareAlike 4.0 International License. 
// To view a copy of this license, visit http://creativecommons.org/licenses/by-sa/4.0/ 
// or send a letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.
//
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
		public double TimeToStart { get; protected set; }
		public Vector3d ManeuverDeltaV { get; protected set; }
		public double ManeuverDuration { get; protected set; }

		public Vector3d StartPos { get; protected set; }
		public Vector3d StartVel { get; protected set; }

		protected BaseTrajectory(VesselWrapper vsl, Vector3d dV, double startUT)
		{
			VSL = vsl;
			ManeuverDeltaV = dV;
			ManeuverDuration = VSL.Engines.TTB((float)ManeuverDeltaV.magnitude);
			StartUT = startUT;
			TimeToStart = startUT-VSL.Physics.UT;
			Body = VSL.vessel.orbitDriver.orbit.referenceBody;
			OrigOrbit = VSL.vessel.orbitDriver.orbit;
			Orbit = TrajectoryCalculator.NewOrbit(OrigOrbit, ManeuverDeltaV, StartUT);
			StartPos = Orbit.getRelativePositionAtUT(StartUT);
			StartVel = Orbit.getOrbitalVelocityAtUT(StartUT);
		}

		public virtual void UpdateOrbit(Orbit current)
		{
			Orbit = current;
			StartUT = VSL.Physics.UT;
			TimeToStart = 0;
			ManeuverDeltaV = Vector3d.zero;
			ManeuverDuration = 0;
			StartPos = Orbit.pos;
			StartVel = Orbit.vel;
		}

		public override string ToString()
		{
			return Utils.Format("[{}]\n" +
			                     "OrigOrbit:\n{}\n" +
			                     "NewOrbit:\n{}\n" +
			                     "StartUT: {} s, TimeToStart: {} s, ManeuverDuration: {} s\n" +
			                     "ManeuverDeltaV: {}", 
			                     GetType().Name, OrigOrbit, Orbit, 
			                     StartUT, TimeToStart, ManeuverDuration, ManeuverDeltaV);
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

		protected TargetedTrajectoryBase(VesselWrapper vsl, Vector3d dV, double startUT) 
			: base(vsl, dV, startUT) {}

		public abstract Vector3d BrakeDeltaV { get; }

		public override string ToString()
		{ 
			return base.ToString()+
				Utils.Format("\nDistanceToTarget: {} m\n" +
				             "TransferTime: {} s\n" +
				             "DeltaFi: {} deg\n",
				             DistanceToTarget, TransferTime, DeltaFi);
		}
	}

	public abstract class TargetedTrajectory : TargetedTrajectoryBase
	{
		public readonly WayPoint Target;

		protected TargetedTrajectory(VesselWrapper vsl, Vector3d dV, double startUT, WayPoint target) 
			: base(vsl, dV, startUT) { Target = target; }

		public override Vector3d BrakeDeltaV
		{ 
			get 
			{ 
				var t_orbit = Target.GetOrbit();
				var t_vel = t_orbit != null? t_orbit.getOrbitalVelocityAtUT(AtTargetUT) : Vector3d.zero;
				return t_vel-Orbit.getOrbitalVelocityAtUT(AtTargetUT); 
			} 
		}

		public override string ToString()
		{ 
			return base.ToString()+
				string.Format("\nTarget: {0}\n", Target); 
		}
	}
}

