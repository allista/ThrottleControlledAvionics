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
    [ComponentInfo(Description = "Compare current altitude to specified value. If Follow Terrain mode is active, height above the ground is used.")]
    public class AltitudeCondition : FloatCondition
    {
        public AltitudeCondition() { Suffix = "m"; }
        protected override float VesselValue(VesselWrapper VSL)
        { return VSL.Altitude; }
    }

    [ComponentInfo(Description = "Compare static atmospheric pressre to specified value")]
    public class PressureCondition : FloatCondition
    {
        public PressureCondition() { Suffix = "kPa"; }
        protected override float VesselValue(VesselWrapper VSL)
        { return (float)VSL.vessel.staticPressurekPa; }
    }

    [ComponentInfo(Description = "Compare dynamic pressre experienced by the ship to specified value")]
    public class DynamicPressureCondition : FloatCondition
    {
        public DynamicPressureCondition() { Suffix = "kPa"; }
        protected override float VesselValue(VesselWrapper VSL)
        { return (float)VSL.vessel.dynamicPressurekPa; }
    }

    [ComponentInfo(Description = "Compare surface speed of the vessel to specified value")]
    public class VelocityCondition : FloatCondition
    {
        public VelocityCondition() { Suffix = "m/s"; }
        protected override float VesselValue(VesselWrapper VSL)
        { return (float)VSL.vessel.srfSpeed; }
    }

    [ComponentInfo(Description = "Compare horizontal surface speed of the vessel to specified value")]
    public class HorizontalVelocityCondition : FloatCondition
    {
        public HorizontalVelocityCondition() { Suffix = "m/s"; }
        protected override float VesselValue(VesselWrapper VSL)
        { return VSL.HorizontalSpeed; }
    }

    [ComponentInfo(Description = "Compare vertical surface speed of the vessel to specified value")]
    public class VerticalVelocityCondition : FloatCondition
    {
        public VerticalVelocityCondition() { Suffix = "m/s"; }
        protected override float VesselValue(VesselWrapper VSL)
        { return VSL.VerticalSpeed.Absolute; }
    }

    [ComponentInfo(Description = "True if the ship is NOT in orbit")]
    public class OnPlanetCondition : Condition
    {
        protected override bool Evaluate(VesselWrapper VSL)
        { return VSL.OnPlanet; }
    }

    [ComponentInfo(Description = "True if the ship is NOT in orbit and the planet has atmosphere")]
    public class InAtmosphereCondition : Condition
    {
        protected override bool Evaluate(VesselWrapper VSL)
        { return VSL.OnPlanet && VSL.Body.atmosphere; }
    }

    [ComponentInfo(Description = "True if the ship is landed or floating on the water")]
    public class LandedOrSplashedCondition : Condition
    {
        protected override bool Evaluate(VesselWrapper VSL)
        { return VSL.LandedOrSplashed; }
    }
}