//  Author:
//       Allis Tauri <allista@gmail.com>
//
//  Copyright (c) 2016 Allis Tauri
//
// This work is licensed under the Creative Commons Attribution-ShareAlike 4.0 International License. 
// To view a copy of this license, visit http://creativecommons.org/licenses/by-sa/4.0/ 
// or send a letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.
//
using UnityEngine;
using AT_Utils;

namespace ThrottleControlledAvionics
{
    public class TargetedToOrbitExecutor : ToOrbitExecutor
    {
        public new class Config : ComponentConfig<Config>
        {
            [Persistent] public PIDf_Controller3 TargetPitchPID = new PIDf_Controller3();
            [Persistent] public PIDf_Controller3 ThrottleCorrectionPID = new PIDf_Controller3();
        }
        public static new Config C => Config.INST;

        [Persistent] public bool InPlane;
        [Persistent] Ratchet correction_started = new Ratchet();
        protected PIDf_Controller3 throttle_correction = new PIDf_Controller3();
        protected LowPassFilterF throttle_fileter = new LowPassFilterF();
        protected LowPassFilterD angle2hor_filter = new LowPassFilterD();

        public TargetedToOrbitExecutor()
        {
            pitch.setPID(C.TargetPitchPID);
            pitch.Min = -AttitudeControlBase.C.MaxAttitudeError;
            pitch.Max = 0;
            throttle_correction.setPID(C.ThrottleCorrectionPID);
            norm_correction.setClamp(AttitudeControlBase.C.MaxAttitudeError);
            throttle_fileter.Tau = 1;
            angle2hor_filter.Tau = 1;
        }

        public override void Reset()
        {
            base.Reset();
            throttle_correction.Reset();
            throttle_fileter.Reset();
            angle2hor_filter.Reset();
            correction_started.Reset();
        }

        public override void StartGravityTurn()
        {
            base.StartGravityTurn();
            throttle_fileter.Set(1);
        }

        public bool TargetedGravityTurn(float Dtol)
        {
            UpdateTargetPosition();
            VSL.Engines.ActivateNextStageOnFlameout();
            update_state(Dtol);
            var pg_vel = get_pg_vel();
            if(!ErrorThreshold)
            {
                CircularizationOffset = -1;
                tune_THR();
                var startF = getStartF();
                var vel = TrajectoryCalculator.dV4ApV(VesselOrbit, target, VSL.Physics.UT);
                var AoA = Utils.Angle2(pg_vel, VesselOrbit.pos);
                var neededAoA = Utils.ClampH(Utils.Angle2(target - VesselOrbit.pos, VesselOrbit.pos) / 2, 45);
                var angle2Hor = angle2hor_filter.Update(90 - Utils.Angle2(vel, VesselOrbit.pos));
                pitch.Max = AoA < neededAoA ? 0 : (float)AoA;
                pitch.Update((float)angle2Hor);
                correction_started.Update(angle2Hor >= 0);
                if(AoA < neededAoA && !correction_started)
                    pitch.Action = (float)Utils.Clamp(AoA - neededAoA + angle2Hor, 
                                                      -AttitudeControlBase.C.MaxAttitudeError, 
                                                      AoA);
                vel = QuaternionD.AngleAxis(pitch.Action * startF,
                                            Vector3d.Cross(target, VesselOrbit.pos))
                                 * pg_vel;
                if(Vector3d.Dot(vel, VesselOrbit.pos) < 0)
                    vel = Vector3d.Exclude(VesselOrbit.pos, vel);
                vel = tune_needed_vel(vel, pg_vel, startF);
                throttle_correction.setClamp(Utils.ClampL(TimeToApA-10, 1));
                throttle_correction.Update((float)angle2Hor);
                throttle.Update(TimeToApA + throttle_correction - (float)TimeToClosestApA);
                throttle_fileter.Update(Utils.Clamp(0.5f + throttle, 0, max_G_throttle()));
                THR.Throttle = throttle_fileter * (float)Utils.ClampH(ErrorThreshold.Value / Dtol / 10, 1);
                //Log("alt {}, ApA {}, dApA {}, dArc {}, Err {}, angle2Hor {}, AoA {} < {}, startF {}, THR {}\n" +
                    //"thr.correction {}\nthrottle {}\npitch {}",
                    //VSL.Altitude.Absolute, VesselOrbit.ApA, dApA, dArc, ErrorThreshold,
                    //angle2Hor, AoA, neededAoA, startF, THR.Throttle, 
                    //throttle_correction, throttle, pitch);//debug
                CFG.AT.OnIfNot(Attitude.Custom);
                ATC.SetThrustDirW(-vel.xzy);
                if(CFG.AT.Not(Attitude.KillRotation))
                {
                    if(AoA < 85)
                    {
                        CFG.BR.OnIfNot(BearingMode.Auto);
                        BRC.ForwardDirection = htdir.xzy;
                    }
                    else
                        CFG.BR.OffIfOn(BearingMode.Auto);
                }
                Status("Gravity turn...");
                return true;
            }
            return coast(pg_vel);
        }
    }
}

