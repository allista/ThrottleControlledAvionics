//   Autopilot.cs
//
//  Author:
//       Allis Tauri <allista@gmail.com>
//
//  Copyright (c) 2015 Allis Tauri
//
// This work is licensed under the Creative Commons Attribution 4.0 International License. 
// To view a copy of this license, visit http://creativecommons.org/licenses/by/4.0/ 
// or send a letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.

namespace ThrottleControlledAvionics
{
	public interface ITCAModule
	{
		VesselConfig CFG { get; }
		TCAState State { get; }
		void SetState(TCAState state);
		bool IsStateSet(TCAState state);
	}

	public class TCAModule : ITCAModule
	{
		public class ModuleConfig : ConfigNodeObject
		{
			public virtual void Init() {}
		}

		protected VesselWrapper vessel;

		public VesselConfig CFG { get { return vessel.CFG; } }
		public TCAState State { get { return vessel.State; } }
		public bool IsActive { get; protected set; }
		public void SetState(TCAState state) { vessel.State |= state; }
		public bool IsStateSet(TCAState state) { return vessel.IsStateSet(state); }

		public virtual void Load(ConfigNode node) {}
		public virtual void Init() {}
		public virtual void UpdateState() {}

	}
}

