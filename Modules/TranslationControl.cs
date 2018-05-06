//  Author:
//       Allis Tauri <allista@gmail.com>
//
//  Copyright (c) 2015 Allis Tauri
//
// This work is licensed under the Creative Commons Attribution-ShareAlike 4.0 International License. 
// To view a copy of this license, visit http://creativecommons.org/licenses/by-sa/4.0/ 
// or send a letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.

using System;
using UnityEngine;
using AT_Utils;

namespace ThrottleControlledAvionics
{
    [CareerPart(typeof(ThrottleControl))]
    public class TranslationControl : AutopilotModule
    {
        public class Config : ComponentConfig<Config>
        {
            [Persistent] public float MinDeltaV         = 0.01f; //m/s
            [Persistent] public PIDf_Controller TransPID = new PIDf_Controller(0.5f, 0.01f, 0.5f, 0, 1);
        }
        public static Config C => Config.INST;

        public TranslationControl(ModuleTCA tca) : base(tca) {}

        readonly PIDf_Controller pid = new PIDf_Controller();

        Vector3 Translation;
        public void AddTranslation(Vector3 trans) { Translation += trans; }

        Vector3 DeltaV;
        public void AddDeltaV(Vector3 dV) { DeltaV += dV; }

        public override void Disable()
        { 
            DeltaV = Vector3.zero; 
            Translation = Vector3.zero; 
            pid.Reset(); 
        }


        public override void Init()
        {
            base.Init();
            pid.setPID(C.TransPID);
        }

        protected override void UpdateState()
        { 
            base.UpdateState();
            IsActive &= VSL.Controls.TranslationAvailable;
        }

        protected override void OnAutopilotUpdate()
        {
            var dVm = DeltaV.magnitude;
            if(dVm >= C.MinDeltaV)
            {
                pid.Update(dVm);
                Translation = pid.Action*DeltaV.CubeNorm();
            }
            else pid.Reset();
            if(!Translation.IsZero())
            { CS.X = Translation.x; CS.Z = Translation.y; CS.Y = Translation.z; }
            Translation = Vector3.zero;
            DeltaV = Vector3.zero;
        }
    }
}

