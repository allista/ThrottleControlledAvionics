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
using UnityEngine;
using AT_Utils;

namespace ThrottleControlledAvionics
{
    public class ManeuverExecutor : TCAComponent
    {
        #pragma warning disable 169
        ThrottleControl THR;
        TranslationControl TRA;
        AttitudeControl ATC;
        #pragma warning restore 169

        public delegate bool ManeuverCondition(float dV);

        readonly FuzzyThreshold<double> dVrem = new FuzzyThreshold<double>(1, 0.5);
        readonly StallDetectorMultiD stalled = new StallDetectorMultiD(5, 0.1, 1);
        double dVmin = -1;
        bool working = false;
        Vector3d course_correction = Vector3d.zero;

        public bool StopAtMinimum = false;
        public bool ThrustWhenAligned = false;

        public bool WithinThreshold { get { return dVrem; } }

        public double RemainingDeltaV { get { return dVrem; } }

        public ManeuverExecutor(ModuleTCA tca)
            : base(tca)
        { 
            InitModuleFields(); 
        }

        public void Reset()
        { 
            dVmin = -1; 
            working = false; 
            dVrem.Reset();
            stalled.Reset();
        }

        public void AddCourseCorrection(Vector3d dV)
        {
            course_correction += dV;
        }

        public bool Execute(Vector3d dV, float MinDeltaV = 0.1f, ManeuverCondition condition = null)
        {
            THR.Throttle = 0;
            var has_correction = !course_correction.IsZero();
            if(has_correction)
            {
                dV += course_correction;
                course_correction = Vector3d.zero;
            }
            dVrem.Lower = MinDeltaV * 5;
            dVrem.Upper = dVrem.Lower + 1;
            dVrem.Value = dV.magnitude;
            //prepare for the burn
            VSL.Engines.ActivateEngines();
            //orient along the burning vector
            if(dVrem && VSL.Controls.RCSAvailableInDirection(-dV, (float)dVrem))
                CFG.AT.OnIfNot(Attitude.KillRotation);
            else
            {
                CFG.AT.OnIfNot(Attitude.Custom);
                ATC.SetThrustDirW(-dV);
            }
            //check if need to be working
            if(!working)
            {
                VSL.Engines.RequestClusterActivationForManeuver(dV);
                working |= condition == null || condition((float)dVrem);
            }
            //end conditions for working execution
            if(working && !has_correction) 
            {
                //end if below the minimum dV
                if(dVrem < MinDeltaV) return false;
                //end if stalled
                stalled.Update(dVrem, VSL.Controls.AttitudeError);
                if(stalled) return false;
                //prevent infinite dV tuning with inaccurate thrust-attitude systems
                if(StopAtMinimum)
                {
                    if(dVrem)
                        VSL.Controls.GimbalLimit = 0;
                    if(dVmin < 0)
                        dVmin = dVrem;
                    else if(dVrem || !ThrustWhenAligned || VSL.Controls.Aligned)
                    {
                        if(dVrem < dVmin)
                            dVmin = dVrem;
                        else if(dVrem - dVmin > MinDeltaV)
                            return false;
                    }
                }
            }
            //if not working and no correction, nothing left to do
            if(!working && !has_correction)
                return true;
            //use translation controls
            if(VSL.Controls.TranslationAvailable &&
               (dVrem || VSL.Controls.AttitudeError > AttitudeControlBase.C.AttitudeErrorThreshold))
                TRA.AddDeltaV(-VSL.LocalDir(dV));
            //use main throttle
            if(ThrustWhenAligned)
                THR.MaxThrottle = VSL.Controls.Aligned ? VSL.Engines.MaxThrottleForDeltaV(dV) : 0;
            THR.CorrectThrottle = ThrustWhenAligned;
            THR.DeltaV = (float)dVrem;
            return true;
        }
    }
}

