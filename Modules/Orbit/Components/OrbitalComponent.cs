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
	public abstract class OrbitalComponent : TCAComponent
	{
		protected OrbitalComponent(ModuleTCA tca) : base(tca) {}

		protected Orbit VesselOrbit { get { return VSL.vessel.orbitDriver.orbit; } }
		protected CelestialBody Body { get { return VesselOrbit.referenceBody; } }
		protected Vector3d hV(double UT) { return VesselOrbit.hV(UT); }
	}
}

