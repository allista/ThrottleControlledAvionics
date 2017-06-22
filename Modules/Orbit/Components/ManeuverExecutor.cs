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
		ThrottleControl THR;
		TranslationControl TRA;
		AttitudeControl ATC;

		public delegate bool ManeuverCondition(float dV);
		readonly FuzzyThreshold<double> dVrem = new FuzzyThreshold<double>(1, 0.5f);
        Vector3d course_correction = Vector3d.zero;

		public bool WithinThreshold { get { return dVrem; } }
		public double RemainingDeltaV { get { return dVrem; } }

		public ManeuverExecutor(ModuleTCA tca) : base(tca) { InitModuleFields(); }

		public void Reset() { dVrem.Value = dVrem.Upper+1; }

        public void AddCourseCorrection(Vector3d dV) { course_correction += dV; }

		public bool Execute(Vector3d dV, float MinDeltaV = 0.1f, ManeuverCondition condition = null)
		{
			THR.Throttle = 0;
            var has_correction = !course_correction.IsZero();
            if(has_correction)
            {
                dV += course_correction;
                course_correction = Vector3d.zero;
            }
			dVrem.Value = dV.magnitude;
			//end if below the minimum dV
			if(dVrem < MinDeltaV) return false;
			VSL.Engines.ActivateEngines();
			//orient along the burning vector
			if(dVrem && VSL.Controls.RCSAvailableInDirection(-dV)) 
				CFG.AT.OnIfNot(Attitude.KillRotation);
			else 
			{
				CFG.AT.OnIfNot(Attitude.Custom);
				ATC.SetThrustDirW(-dV);
				VSL.Engines.RequestClusterActivationForManeuver(-dV);
			}
			//check the condition
            if(!has_correction && condition != null && !condition((float)dVrem)) return true;
			if(VSL.Controls.TranslationAvailable)
			{
				if(dVrem || VSL.Controls.AttitudeError > GLB.ATCB.AttitudeErrorThreshold)
					TRA.AddDeltaV(-VSL.LocalDir(dV));
				if(dVrem && CFG.AT[Attitude.KillRotation]) 
				{
					var errorF = Utils.ClampL(Vector3.Dot(VSL.Engines.Thrust.normalized, -dV.normalized), 0);
					THR.DeltaV = (float)dVrem * errorF*errorF;
				}
				else THR.DeltaV = (float)dVrem;
			}
			else THR.DeltaV = (float)dVrem;
			return true;
		}
	}
}

