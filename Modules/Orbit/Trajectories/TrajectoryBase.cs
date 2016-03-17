//  Author:
//       Allis Tauri <allista@gmail.com>
//
//  Copyright (c) 2016 Allis Tauri
//
// This work is licensed under the Creative Commons Attribution-ShareAlike 4.0 International License. 
// To view a copy of this license, visit http://creativecommons.org/licenses/by-sa/4.0/ 
// or send a letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.
//

namespace ThrottleControlledAvionics
{
	public abstract class BaseTrajectory
	{
		public readonly VesselWrapper VSL;
		public readonly CelestialBody Body;
		public readonly Orbit OrigOrbit;

		public Orbit NewOrbit { get; protected set; }
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
			ManeuverDuration = ManeuverAutopilot.TTB(VSL, (float)ManeuverDeltaV.magnitude, 1);
			StartUT = startUT;
			TimeToStart = startUT-VSL.Physics.UT;
			Body = VSL.vessel.orbitDriver.orbit.referenceBody;
			OrigOrbit = VSL.vessel.orbitDriver.orbit;
			NewOrbit = TrajectoryCalculator.NewOrbit(OrigOrbit, ManeuverDeltaV, StartUT);
			StartPos = NewOrbit.getRelativePositionAtUT(StartUT);
			StartVel = NewOrbit.getOrbitalVelocityAtUT(StartUT);
		}

		public virtual void UpdateOrbit(Orbit current)
		{
			NewOrbit = current;
			StartUT = VSL.Physics.UT;
			TimeToStart = 0;
			ManeuverDeltaV = Vector3d.zero;
			ManeuverDuration = 0;
			StartPos = NewOrbit.pos;
			StartVel = NewOrbit.vel;
		}

		public abstract bool IsBetter(BaseTrajectory other);

		public override string ToString()
		{
			return string.Format("[{0}]\n" +
			                     "OrigOrbit:\n{1}\n" +
			                     "NewOrbit:\n{2}\n" +
			                     "StartUT: {3} s, TimeToStart: {4} s, ManeuverDuration: {5} s\n" +
			                     "ManeuverDeltaV: {6}", 
			                     GetType().Name, Utils.formatOrbit(OrigOrbit), Utils.formatOrbit(NewOrbit), 
			                     StartUT, TimeToStart, ManeuverDuration, ManeuverDeltaV);
		}
	}

	public abstract class TargetedTrajectoryBase : BaseTrajectory
	{
		public double AtTargetUT;
		public Vector3d AtTargetPos { get; protected set; }
		public Vector3d AtTargetVel { get; protected set; }
		public double TimeToTarget { get; protected set; }
		public double DistanceToTarget { get; protected set; } = -1;
		public double DeltaFi { get; protected set; }

		protected TargetedTrajectoryBase(VesselWrapper vsl, Vector3d dV, double startUT) 
			: base(vsl, dV, startUT) {}

		public override bool IsBetter(BaseTrajectory other)
		{
			var _other = other as TargetedTrajectoryBase;
			return _other == null || _other.DistanceToTarget < 0 || 
				DistanceToTarget > 0 && DistanceToTarget < _other.DistanceToTarget;
		}

		public override string ToString()
		{ 
			return base.ToString()+
				string.Format("\nDistanceToTarget: {0} m\n" +
				              "DeltaFi: {1} deg\n",
				              DistanceToTarget, DeltaFi);
		}
	}

	public abstract class TargetedTrajectory<T> : TargetedTrajectoryBase where T : ITargetable
	{
		public readonly T Target;

		protected TargetedTrajectory(VesselWrapper vsl, Vector3d dV, double startUT, T target) 
			: base(vsl, dV, startUT) { Target = target; }

		public override string ToString()
		{ 
			return base.ToString()+
				string.Format("\nTarget: {0}", Target); 
		}
	}
}

