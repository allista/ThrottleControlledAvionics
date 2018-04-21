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
        protected PIDf_Controller3 throttle_correction = new PIDf_Controller3();
        protected LowPassFilterF throttle_fileter = new LowPassFilterF();

        public TargetedToOrbitExecutor(ModuleTCA tca) : base(tca)
        {
            pitch.setPID(ToOrbitAutopilot.C.TargetPitchPID);
            pitch.Min = -AttitudeControlBase.C.MaxAttitudeError;
            pitch.Max = 0;
            throttle_correction.setPID(ToOrbitAutopilot.C.ThrottleCorrectionPID);
            throttle_fileter.Tau = 1;
        }

        public override void StartGravityTurn()
        {
            base.StartGravityTurn();
            throttle_fileter.Set(1);
        }

        public bool TargetedGravityTurn(float ApA_offset, float min_throttle, float maxG, float Dtol)
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
                var angle2Hor = 90 - Utils.Angle2(vel, VesselOrbit.pos);
                pitch.Max = AoA < neededAoA ? 0 : (float)AoA;
                pitch.Update((float)angle2Hor);
                if(AoA < neededAoA && pitch.Action.Equals(pitch.Max))
                    pitch.Action = (float)Utils.ClampL(AoA - neededAoA, -AttitudeControlBase.C.MaxAttitudeError);
                vel = QuaternionD.AngleAxis(pitch.Action * startF,
                                            Vector3d.Cross(target, VesselOrbit.pos))
                                 * pg_vel;
                if(Vector3d.Dot(vel, VesselOrbit.pos) < 0)
                    vel = Vector3d.Exclude(VesselOrbit.pos, vel);
                vel = tune_needed_vel(vel, pg_vel, startF);
                throttle_correction.Update((float)angle2Hor);
                throttle.Update(ApA_offset + throttle_correction - (float)VesselOrbit.timeToAp);
                throttle_fileter.Update(Utils.Clamp(0.5f + throttle, min_throttle, max_G_throttle(maxG)));
                THR.Throttle = throttle_fileter * (float)Utils.ClampH(ErrorThreshold.Value / Dtol / 10, 1);
                Log("angle2Hor {}, AoA {} < {}, THR {}\nthr.correction {}\nthrottle {}\npitch {}",
                    angle2Hor, AoA, neededAoA, THR.Throttle, throttle_correction, throttle, pitch);//debug
                CFG.AT.OnIfNot(Attitude.Custom);
                ATC.SetThrustDirW(-vel.xzy);
                if(CFG.AT.Not(Attitude.KillRotation))
                {
                    if(AoA < 85)
                    {
                        CFG.BR.OnIfNot(BearingMode.Auto);
                        BRC.ForwardDirection = hv.xzy;
                    }
                    else
                        CFG.BR.OffIfOn(BearingMode.Auto);
                }
                Status("Gravity turn...");
                return true;
            }
            return coast(ApA_offset, pg_vel);
        }

        public bool TargetedGravityTurn3(float ApA_offset, float min_throttle, float maxG, float Dtol)
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
                //var dTime2ApA = ApA_offset - (float)VesselOrbit.timeToAp;
                //throttle.Update(dTime2ApA);
                THR.Throttle = //Utils.Clamp(0.5f + throttle.Action, min_throttle, max_G_throttle(maxG)) *
                    Mathf.Max(min_throttle, max_G_throttle(maxG)) *
                    (float)Utils.ClampH(ErrorThreshold.Value / Dtol / 10, 1);
                //float mflow;
                //float thrust = VSL.Engines.ThrustAtAlt((float)VSL.vessel.srfSpeed, VSL.Altitude.Absolute, out mflow);
                //var dVm = VSL.Engines.DeltaV(thrust/mflow, mflow*THR.Throttle);
                var dir = TrajectoryCalculator.dV4ApV(VesselOrbit, target, VSL.Physics.UT).normalized;
                var vel = TrajectoryCalculator.dV4ApV(VesselOrbit, target, VSL.Physics.UT);
                //var dVorb = VesselOrbit.getOrbitalVelocityAtUT(VSL.Physics.UT+1)-VesselOrbit.vel;
                //var norm = dVorb-Vector3d.Project(dVorb, dir);
                //var norm2 = norm.sqrMagnitude;
                //var orb2 = dVorb.sqrMagnitude;
                //var dV2 = dVm*dVm;
                //var vel = dV2 >= norm2? dir*Math.Sqrt(dV2-norm2)-norm : -norm;
                //var vel = dir*dVm-dVorb;
                vel = tune_needed_vel(vel, pg_vel, startF);
                vel = Utils.ClampDirection(vel,
                                           Vector3d.Lerp(pg_vel.normalized,
                                                         -VSL.Engines.DefManualThrust.xzy(),
                                                         ApV.magnitude / TargetR),
                                           (double)AttitudeControlBase.C.MaxAttitudeError);
                //Log("vel {}, dVm {}, dVorb {}, pgV {}, pg_velF {}, THR {}, throttle {}",
                //vel.magnitude, dVm, dVorb.magnitude, pg_vel.magnitude,
                //VSL.Physics.G / VSL.Physics.StG,
                //THR.Throttle, throttle);//debug
                CFG.AT.OnIfNot(Attitude.Custom);
                ATC.SetThrustDirW(-vel.xzy);
                if(CFG.AT.Not(Attitude.KillRotation))
                {
                    CFG.BR.OnIfNot(BearingMode.Auto);
                    BRC.ForwardDirection = hv.xzy;
                }
                Status("Gravity turn...");
                return true;
            }
            return coast(ApA_offset, pg_vel);
        }

        public bool TargetedGravityTurn2(float ApA_offset, float min_throttle, float maxG, float Dtol)
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
                var dV = TrajectoryCalculator.dV4ApV(VesselOrbit, target, VSL.Physics.UT);
                var tOrb = TrajectoryCalculator.NewOrbit(VesselOrbit, dV, VSL.Physics.UT);
                var tOrbR_at_ApR = tOrb.getRelativePositionFromTrueAnomaly(tOrb.GetTrueAnomalyOfZupVector(ApV)).magnitude;
                var ApAerr = (TargetR + tOrbR_at_ApR - VesselOrbit.ApR * 2) / (TargetR - tOrbR_at_ApR);
                var vel = pg_vel;
                if(dV.magnitude > pg_vel.magnitude * 0.1)
                {
                    var AoA = Utils.Angle2(pg_vel, VesselOrbit.pos);
                    if(AoA < 45)
                    {
                        pitch.Update((float)AoA - 45);
                        vel = QuaternionD.AngleAxis(pitch * startF, Vector3d.Cross(target, VesselOrbit.pos)) * pg_vel;
                    }
                    vel = tune_needed_vel(vel, pg_vel, startF);
                    vel = Utils.ClampDirection(vel, pg_vel, (double)AttitudeControlBase.C.MaxAttitudeError);
                    throttle.Update(ApA_offset - (float)VesselOrbit.timeToAp + ApA_offset / 2 * (float)ApAerr);
                    THR.Throttle = Utils.Clamp(0.5f + throttle.Action, min_throttle, max_G_throttle(maxG));
                    Log("ApAerr {}, dV {}, pgV {}, pg_velF {}, THR {}, pitch {}, throttle {}",
                        ApAerr, dV.magnitude, pg_vel.magnitude,
                        VSL.Physics.G / VSL.Physics.StG,
                        THR.Throttle, pitch, throttle);//debug
                    CFG.AT.OnIfNot(Attitude.Custom);
                    ATC.SetThrustDirW(-vel.xzy);
                }
                else
                {
                    vel = dV;
                    //Executor.Execute(dV.xzy, Utils.Clamp(1-VSL.Torque.MaxCurrent.AA_rad, 0.1f, 1));
                }
                if(CFG.AT.Not(Attitude.KillRotation))
                {
                    CFG.BR.OnIfNot(BearingMode.Auto);
                    BRC.ForwardDirection = hv.xzy;
                }
                Status("Gravity turn...");
                return true;
            }
            return coast(ApA_offset, pg_vel);
        }
    }
}

