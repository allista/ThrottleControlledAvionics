//  Author:
//       Allis Tauri <allista@gmail.com>
//
//  Copyright (c) 2016 Allis Tauri
//
// This work is licensed under the Creative Commons Attribution-ShareAlike 4.0 International License. 
// To view a copy of this license, visit http://creativecommons.org/licenses/by-sa/4.0/ 
// or send a letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.
//
using UnityEngine;

namespace ThrottleControlledAvionics
{
	public abstract class VesselProps
	{
		protected VesselProps(VesselWrapper vsl) { VSL = vsl; }

		protected readonly VesselWrapper VSL;
		protected VesselConfig CFG { get { return VSL.CFG; } }
		public TCAGlobals GLB { get { return TCAScenario.Globals; } }
		protected Vessel vessel { get { return VSL.vessel; } }
		protected Transform refT { get { return VSL.refT; } set { VSL.refT = value; } }

		public virtual void ClearFrameState() {}
		public abstract void Update();
	}
}

