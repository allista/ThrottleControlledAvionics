//   TCA_Test.cs
//
//  Author:
//       Allis Tauri <allista@gmail.com>
//
//  Copyright (c) 2017 Allis Tauri
using System;
using AT_Utils;

namespace ThrottleControlledAvionics
{
    public abstract class TCA_Test : ITestScenario
    {
        static Globals GLB { get { return Globals.Instance; } }

        protected ModuleTCA TCA;
        protected VesselWrapper VSL { get { return TCA.VSL; } }
        protected VesselConfig CFG { get { return TCA.CFG; } }

        protected bool GetTCA()
        {
            if(FlightGlobals.ActiveVessel != null)
                TCA = ModuleTCA.EnabledTCA(FlightGlobals.ActiveVessel);
            return TCA != null;
        }

        #region ITestScenario implementation
        public abstract string Setup();
        public abstract bool Update(Random RND);
        public abstract void Cleanup();

        public virtual string Status { get; protected set; } = "";
        public abstract bool NeedsFixedUpdate { get; }
        public abstract bool NeedsUpdate { get; }
        #endregion
    }
}

