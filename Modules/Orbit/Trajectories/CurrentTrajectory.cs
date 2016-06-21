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
	public abstract class CurrentTrajectory<T> : OrbitalComponent where T : BaseTrajectory
	{
		public T Trajectory;
		protected CurrentTrajectory(ModuleTCA tca) : base(tca) { Trajectory = Default; }

		protected abstract T Default { get; }

		public virtual void Update()
		{
			if(Trajectory == null) Trajectory = Default;
			else Trajectory.UpdateOrbit(VesselOrbit);
		}

		public static implicit operator T(CurrentTrajectory<T> t) { return t.Trajectory; }
	}


	public class CurrentLandingTrajectory : CurrentTrajectory<LandingTrajectory>
	{
		public CurrentLandingTrajectory(ModuleTCA tca) : base(tca) {}

		protected override LandingTrajectory Default
		{ get { return new LandingTrajectory(VSL, Vector3d.zero, VSL.Physics.UT, CFG.Target, CFG.Target.SurfaceAlt(Body), false); } }

		public void UpdateWithBrake() 
		{ 
			if(Trajectory == null) Trajectory = Default;
			Trajectory.UpdateOrbit(VesselOrbit, true); 
		}
	}


	public class CurrentRendezvousTrajectory : CurrentTrajectory<RendezvousTrajectory>
	{
		public CurrentRendezvousTrajectory(ModuleTCA tca) : base(tca) {}

		protected override RendezvousTrajectory Default 
		{ get { return new RendezvousTrajectory(VSL, Vector3d.zero, VSL.Physics.UT, CFG.Target, MinPeR); } }
	}
}

