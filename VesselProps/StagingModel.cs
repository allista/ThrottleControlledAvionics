//   StagingModel.cs
//
//  Author:
//       Allis Tauri <allista@gmail.com>
//
//  Copyright (c) 2017 Allis Tauri
using System;
using System.Linq;
using System.Collections.Generic;
using UnityEngine;

namespace ThrottleControlledAvionics
{
    public class StagingModel : VesselProps
    {
        public class Stage : VesselProps, IShipconstruct
        {
            public int InverseIndex;

            public List<Part> Parts { get; private set; }
            public EnginesDB Engines { get; private set; }

            public Stage(VesselWrapper vsl) : base(vsl) {}

            public double M { get; private set; }

            public double DeltaV(double Ve, double fuel_mass) { return Ve*Math.Log(M/(M-fuel_mass)); }

            public EnginesStats GetEnginesStats()
            {
                var stats = new EnginesStats(Engines, VSL);
                stats.MaxDeltaV = DeltaV(stats.MaxThrust.magnitude/stats.MaxMassFlow, stats.MaxFuelMass);
                return stats;
            }
        }

        public StagingModel(VesselWrapper vsl) : base(vsl) {}

        /// <summary>
        /// The list of stages, from current to the last available, i.e. from greater inverse index to 0.
        /// </summary>
        public List<Stage> Stages = new List<Stage>();

        public List<EnginesStats> GetEnginesStats()
        { return Stages.Select(s => s.GetEnginesStats()).ToList(); }

        public void UpdateModel()
        {
            
        }
    }
}

