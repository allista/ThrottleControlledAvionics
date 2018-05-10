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
    public class AltitudeProps : VesselProps
    {
        public AltitudeProps(VesselWrapper vsl) : base(vsl) {}

        public float Current { get; private set; }
        public float Absolute { get; private set; }
        public float Relative { get; private set; }
        public float PrevRelative { get; private set; }
        public float TerrainAltitude { get; private set; }
        public bool  AboveGround { get; private set; }
        public float Ahead = float.MinValue;
        public float LowerThreshold = float.MinValue;
        public bool  CorrectionAllowed = true;

        public static implicit operator float(AltitudeProps alt) { return alt.Current; }

        public override void ClearFrameState()
        {
            LowerThreshold = float.MinValue;
            CorrectionAllowed = true;
        }

        public void DontCorrectIfSlow()
        { CorrectionAllowed = VSL.HorizontalSpeed.MoovingFast; }

        public override void Update()
        {
            PrevRelative = Relative;
            Absolute = (float)vessel.altitude;
            TerrainAltitude = (float)((vessel.mainBody.ocean && vessel.terrainAltitude < 0)? 0 : vessel.terrainAltitude);
            Relative = Utils.ClampL((float)(vessel.altitude) - TerrainAltitude, 0);
            Current = CFG.AltitudeAboveTerrain? Relative : Absolute;
            AboveGround = 
                CFG.AltitudeAboveTerrain && CFG.DesiredAltitude >= VSL.Geometry.H ||
                !CFG.AltitudeAboveTerrain && CFG.DesiredAltitude >= TerrainAltitude+VSL.Geometry.H; 
        }
    }
}

