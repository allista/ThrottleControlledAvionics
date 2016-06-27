//  Author:
//       Allis Tauri <allista@gmail.com>
//
//  Copyright (c) 2015 Allis Tauri
//
// This work is licensed under the Creative Commons Attribution-ShareAlike 4.0 International License. 
// To view a copy of this license, visit http://creativecommons.org/licenses/by-sa/4.0/ 
// or send a letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.

namespace ThrottleControlledAvionics
{
	public class AltitudeCondition : FloatCondition
	{
		public AltitudeCondition() { Suffix = "m"; }
		protected override float VesselValue(VesselWrapper VSL)
		{ return VSL.Altitude; }
	}

	public class PressureCondition : FloatCondition
	{
		public PressureCondition() { Suffix = "kPa"; }
		protected override float VesselValue(VesselWrapper VSL)
		{ return (float)VSL.vessel.staticPressurekPa; }
	}

	public class VelocityCondition : FloatCondition
	{
		public VelocityCondition() { Suffix = "m/s"; }
		protected override float VesselValue(VesselWrapper VSL)
		{ return (float)VSL.vessel.srfSpeed; }
	}

	public class HorizontalVelocityCondition : FloatCondition
	{
		public HorizontalVelocityCondition() { Suffix = "m/s"; }
		protected override float VesselValue(VesselWrapper VSL)
		{ return VSL.HorizontalSpeed; }
	}

	public class VerticalVelocityCondition : FloatCondition
	{
		public VerticalVelocityCondition() { Suffix = "m/s"; }
		protected override float VesselValue(VesselWrapper VSL)
		{ return VSL.VerticalSpeed.Absolute; }
	}

	public class OnPlanetCondition : Condition
	{
		protected override bool Evaluate(VesselWrapper VSL)
		{ return VSL.OnPlanet; }
	}

	public class InAtmosphereCondition : Condition
	{
		protected override bool Evaluate(VesselWrapper VSL)
		{ return VSL.OnPlanet && VSL.Body.atmosphere; }
	}

	public class LandedOrSplashedCondition : Condition
	{
		protected override bool Evaluate(VesselWrapper VSL)
		{ return VSL.LandedOrSplashed; }
	}
}