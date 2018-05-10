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
    public class ControlProps : VesselProps
    {
        public ControlProps(VesselWrapper vsl) : base(vsl) {}

        public Vector3 Steering { get; private set; }
        public Vector3 Translation { get; private set; }
        public Vector3 AutopilotSteering;
        public bool    TranslationAvailable;
        public Vector3 ManualTranslation;
        public Switch  ManualTranslationSwitch = new Switch();
        public float   GimbalLimit = 100;
        public bool    HaveControlAuthority = true;
        public bool    NoDewarpOffset;
        public bool    PauseWhenStopped;

        bool dewarp;
        double warp_to_time = -1;
        public double WarpToTime 
        {
            get { return dewarp? 0: warp_to_time; }
            set 
            {
                //if(!warp_to_time.Equals(value))//debug
                    //Log("WarpToTime.set: {} -> {}, dewarped {}", warp_to_time, value, dewarp);//debug
                warp_to_time = value;
            }
        }

        public bool  Aligned { get; private set; } = true;
        public bool  CanWarp { get; private set; } = true;
        public float AttitudeError { get; private set; }
        public float MinAlignmentTime { get; private set; }
        public float AlignmentFactor { get; private set; }
        public float InvAlignmentFactor { get; private set; }

        public override void ClearFrameState()
        {
            AutopilotSteering = Vector3.zero;
            GimbalLimit = 100;
            dewarp = false;
        }

        public float OffsetAlignmentFactor(float offset = 1)
        {
            var err = Utils.ClampL(AttitudeError-offset, 0);
            var max = Utils.ClampL(AttitudeControlBase.C.MaxAttitudeError-offset, 1e-5f);
            return Utils.ClampL(1-err/max, 0);
        }

        public void SetAttitudeError(float error)
        {
            AttitudeError = error;
            Aligned &= AttitudeError < AttitudeControlBase.C.MaxAttitudeError;
            Aligned |= AttitudeError < AttitudeControlBase.C.AttitudeErrorThreshold;
            CanWarp = CFG.WarpToNode && TimeWarp.WarpMode == TimeWarp.Modes.HIGH &&
                         (TimeWarp.CurrentRate > 1 || 
                          Aligned && (VSL.Physics.NoRotation || VSL.Physics.ConstantRotation));
            MinAlignmentTime = VSL.Torque.MaxCurrent.RotationTime2Phase(AttitudeError);
            AlignmentFactor = Utils.ClampL(1-AttitudeError/AttitudeControlBase.C.MaxAttitudeError, 0);
            InvAlignmentFactor = Utils.ClampH(AttitudeError/AttitudeControlBase.C.MaxAttitudeError, 1);
        }

        public void StopWarp() => dewarp = true;

        public void AbortWarp(bool instant = false)
        {
            if(TimeWarp.CurrentRateIndex > 0)
                TimeWarp.SetRate(0, instant);
            warp_to_time = -1;
            CFG.WarpToNode = false;
        }

        public void Update(FlightCtrlState s)
        {
            Steering = new Vector3(s.pitch, s.roll, s.yaw);
            Translation = new Vector3(s.X, s.Z, s.Y);
            if(!Steering.IsZero()) Steering = Steering/Steering.CubeNorm().magnitude;
            if(!Translation.IsZero()) Translation = Translation/Translation.CubeNorm().magnitude;
//            if(VSL.IsActiveVessel)
//                TCAGui.AddDebugMessage("Steering {}\nTranslation {}", 
//                                       Utils.formatComponents(Steering), 
//                                       Utils.formatComponents(Translation));//debug
        }

        public bool RCSAvailableInDirection(Vector3 wDir, float MinDeltaV = -1)
        {
            if(VSL.Engines.NumActiveRCS.Equals(0)) return false;
            var lDir = VSL.LocalDir(wDir).normalized;
            var thrust = VSL.Engines.MaxThrustRCS.Project(lDir);
//            Utils.Log("\nMaxThrustRCS:\n{}\nRCS dir: {}\nRCS thrust: {}\nRCS accel: {}\nActive RCS: {}\n",
//                       VSL.Engines.MaxThrustRCS, lDir, thrust, thrust.magnitude/VSL.Physics.M, VSL.Engines.NumActiveRCS);//debug
            if(MinDeltaV < 0) MinDeltaV = TranslationControl.C.MinDeltaV;
            return thrust.magnitude/VSL.Physics.M > MinDeltaV/2;
        }

        public void SetSteering(Vector3 steering)
        { AutopilotSteering = steering.ClampComponents(-1, 1); }

        public void AddSteering(Vector3 steering)
        { AutopilotSteering = (VSL.Controls.AutopilotSteering+steering).ClampComponents(-1, 1); }

        public void SetSteeringIfGrater(Vector3 steering)
        {
            if(Math.Abs(steering.x) > Math.Abs(AutopilotSteering.x)) 
                AutopilotSteering.x = Utils.Clamp(steering.x, -1, 1);
            if(Math.Abs(steering.y) > Math.Abs(AutopilotSteering.y)) 
                AutopilotSteering.y = Utils.Clamp(steering.y, -1, 1);
            if(Math.Abs(steering.z) > Math.Abs(AutopilotSteering.z)) 
                AutopilotSteering.z = Utils.Clamp(steering.z, -1, 1);
        }
    }
}

