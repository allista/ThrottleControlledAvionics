//  Author:
//       Allis Tauri <allista@gmail.com>
//
//  Copyright (c) 2016 Allis Tauri
//
// This work is licensed under the Creative Commons Attribution-ShareAlike 4.0 International License. 
// To view a copy of this license, visit http://creativecommons.org/licenses/by-sa/4.0/ 
// or send a letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.
//
using System;

namespace ThrottleControlledAvionics
{
    public class VerticalSpeedProps : VesselProps
    {
        public VerticalSpeedProps(VesselWrapper vsl) : base(vsl) {}

        public float Absolute { get; private set; }
        public float Relative { get; private set; }
        public float Display { get; private set; }
        public float Derivative { get; private set; }

        public override void Update()
        {
            //unlike the vessel.verticalSpeed, this method is unaffected by ship's rotation (from MechJeb)
            var current = (float)Vector3d.Dot(vessel.srf_velocity, VSL.Physics.Up);
            Derivative = (current-Absolute)/TimeWarp.fixedDeltaTime;
            Absolute = current;
            Display = Absolute;
            //use relative vertical speed instead of absolute if following terrain
            if(CFG.AltitudeAboveTerrain)
            {
                Relative  = (VSL.Altitude.Relative - VSL.Altitude.PrevRelative)/TimeWarp.fixedDeltaTime;
                Display = Relative;
            }
        }
    }
}

