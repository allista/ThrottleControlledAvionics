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
        readonly LowPassFilterD ddV = new LowPassFilterD();
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
            ddV.Tau = 1;
        }

        public void Reset()
        { 
            dVrem.Value = dVrem.Upper + 1; 
            dVmin = -1; 
            working = false; 
            ddV.Reset();
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
            //end if below the minimum dV
            if(dVrem < MinDeltaV)
                return false;
            VSL.Engines.ActivateEngines();
            //orient along the burning vector
            if(dVrem && VSL.Controls.RCSAvailableInDirection(-dV))
                CFG.AT.OnIfNot(Attitude.KillRotation);
            else
            {
                CFG.AT.OnIfNot(Attitude.Custom);
                ATC.SetThrustDirW(-dV);
            }
            if(!working)
            {
                VSL.Engines.RequestClusterActivationForManeuver(dV);
                if(!has_correction && condition != null && !condition((float)dVrem))
                    return true;
                working = true;
            }
            //prevent infinite dV tuning with inaccurate thrust-attitude systems
            if(StopAtMinimum)
            {
                if(dVrem)
                    VSL.Controls.GimbalLimit = 0;
                if(dVmin < 0)
                {
                    dVmin = dVrem;
                    ddV.Set(dVmin);
                }
                else if(!ThrustWhenAligned || VSL.Controls.Aligned)
                {
                    ddV.Update(Math.Max(dVmin - dVrem, 0) / TimeWarp.fixedDeltaTime);
//                    Log("dVrem {}, dVmin {}, ddV {}, aliF {}, aliE {}", 
//                        dVrem.Value, dVmin, 
//                        ddV, VSL.Controls.AlignmentFactor, VSL.Controls.AttitudeError);//debug
                    if(ddV < MinDeltaV)
                        return false;
                    if(dVrem < dVmin)
                        dVmin = dVrem;
                    else if(dVrem - dVmin > MinDeltaV)
                        return false;
                }
            }
            //use translation controls
            if(VSL.Controls.TranslationAvailable)
            {
                if(dVrem || VSL.Controls.AttitudeError > GLB.ATCB.AttitudeErrorThreshold)
                    TRA.AddDeltaV(-VSL.LocalDir(dV));
                if(dVrem && CFG.AT[Attitude.KillRotation])
                {
                    var errorF = Utils.ClampL(Vector3.Dot(VSL.Engines.Thrust.normalized, -dV.normalized), 0);
                    THR.DeltaV = (float)dVrem * errorF * errorF;
                    return true;
                }
            }
            //use main throttle
            if(ThrustWhenAligned)
                THR.DeltaV = VSL.Controls.Aligned ? (float)dVrem * VSL.Controls.AlignmentFactor : 0;
            else
                THR.DeltaV = (float)dVrem;
            return true;
        }
    }
}

