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
    public class ToOrbitExecutor : OrbitalComponent
    {
        const double MinClimbTime = 5;

        ThrottleControl THR;
        AttitudeControl ATC;
        BearingControl BRC;

        readonly SingleAction GearAction = new SingleAction();
        readonly FuzzyThreshold<double> ErrorThreshold = new FuzzyThreshold<double>();
        readonly ManeuverExecutor Executor;

        Vector3d target;
        public Vector3d Target
        {
            get { return target; }
            set { target = value; TargetR = target.magnitude; }
        }
        public double TargetR { get; private set; }
        public double LaunchUT = -1;
        public double ApAUT = -1;
        public double GravityTurnStart { get; private set; }
        public bool CorrectOnlyAltitude;

        Vector3d hv;
        public State<double> dApA = new State<double>(-1);
        public State<double> dArc = new State<double>(-1);
        LowPassFilterD time2target_v = new LowPassFilterD();
        LowPassFilterD time2target_h = new LowPassFilterD();
        LowPassFilterD time2target_diff = new LowPassFilterD();
        LowPassFilterD pitch = new LowPassFilterD();
        PIDf_Controller3 norm_correction = new PIDf_Controller3();
        PIDf_Controller3 throttle = new PIDf_Controller3();
        double CircularizationOffset = -1;
        bool ApoapsisReached;

        public ToOrbitExecutor(ModuleTCA tca) : base(tca)
        {
            InitModuleFields();
            Executor = new ManeuverExecutor(tca);
            GearAction.action = () => VSL.GearOn(false);
            ErrorThreshold.Lower = GLB.ORB.Dtol / 10;
            ErrorThreshold.Upper = GLB.ORB.Dtol;
            throttle.setPID(GLB.ORB.ThrottlePID);
            norm_correction.setPID(GLB.ORB.NormCorrectionPID);
            norm_correction.Min = -GLB.ATCB.MaxAttitudeError;
            norm_correction.Max = GLB.ATCB.MaxAttitudeError;
            GravityTurnStart = 0;
            time2target_v.Tau = 0.5f;
            time2target_h.Tau = 0.5f;
            time2target_diff.Tau = 0.5f;
            pitch.Tau = 0.5f;
        }

        public void UpdateTargetPosition()
        {
            Target = QuaternionD.AngleAxis(Body.angularV * TimeWarp.fixedDeltaTime * Mathf.Rad2Deg,
                                           Body.zUpAngularVelocity.normalized) * Target;
        }

        public bool Liftoff(float min_throttle, float maxG)
        {
            UpdateTargetPosition();
            VSL.Engines.ActivateEngines();
            VSL.OnPlanetParams.ActivateLaunchClamps();
            if(VSL.VerticalSpeed.Absolute / VSL.Physics.G < MinClimbTime)
            {
                Status("Liftoff...");
                CFG.DisableVSC();
                CFG.HF.OnIfNot(HFlight.Stop);
                CFG.VTOLAssistON = true;
                THR.Throttle = max_G_throttle(maxG);
                if(VSL.vessel.srfSpeed > 1)
                {
                    var upErr = Utils.Angle2(-(Vector3)VSL.Physics.Up, VSL.Engines.CurrentDefThrustDir);
                    THR.Throttle = Utils.Clamp(THR.Throttle * (1 - VSL.HorizontalSpeed.Absolute - upErr),
                                               VSL.OnPlanetParams.GeeVSF*1.1f, 1);
                }
                THR.Throttle = Utils.ClampL(THR.Throttle, min_throttle);
                return true;
            }
            StartGravityTurn();
            return false;
        }

        public void StartGravityTurn()
        {
            GravityTurnStart = VSL.Altitude.Absolute;
            ApoapsisReached = false;
            GearAction.Run();
            CFG.VTOLAssistON = false;
            CFG.StabilizeFlight = false;
            CFG.HF.Off();
            update_state();
        }

        double time2dist(double v, double a, double d)
        {
            if(a.Equals(0)) return d / v;
            var D = v * v + 2 * d * a;
            if(D < 0) return double.NaN;
            return (Math.Sqrt(D) - v) / a;
        }

        void update_state()
        {
            var cApV = VesselOrbit.getRelativePositionAtUT(VSL.Physics.UT + VesselOrbit.timeToAp);
            hv = Vector3d.Exclude(VesselOrbit.pos, target - VesselOrbit.pos).normalized;
            dApA.current = TargetR - VesselOrbit.ApR;
            dArc.current = Utils.ProjectionAngle(cApV, target, hv) * Mathf.Deg2Rad * cApV.magnitude;
            ErrorThreshold.Value = CorrectOnlyAltitude ? dApA : dApA + dArc;
        }

        /// <summary>
        /// The arc distance in radians between current vessel position and the Target.
        /// </summary>
        public double ArcDistance
        { get { return Utils.ProjectionAngle(VesselOrbit.pos, target, target - VesselOrbit.pos) * Mathf.Deg2Rad; } }

        void tune_THR()
        {
            THR.CorrectThrottle = ApoapsisReached;
            if(VSL.vessel.dynamicPressurekPa > GLB.ORB.MaxDynPressure)
                THR.MaxThrottle = Mathf.Max(1 - ((float)VSL.vessel.dynamicPressurekPa - GLB.ORB.MaxDynPressure) / 5, 0);
        }

        double getStartF()
        {
            if(Body.atmosphere && VSL.vessel.atmDensity > GLB.ORB.AtmDensityOffset)
                GravityTurnStart = VSL.Altitude.Absolute;
            return Utils.Clamp((VSL.Altitude.Absolute - GravityTurnStart) / GLB.ORB.GTurnOffset, 0, 1);
        }

        Vector3d tune_needed_vel(Vector3d needed_vel, Vector3d pg_vel, double startF)
        {
            if(Vector3d.Dot(hv, Vector3d.Exclude(VesselOrbit.pos, VesselOrbit.vel).normalized) > 0.1)
            {
                norm_correction.Update((float)(90 - Utils.Angle2(VesselOrbit.GetOrbitNormal(), target)));
                needed_vel = QuaternionD.AngleAxis(norm_correction.Action, VesselOrbit.pos) * needed_vel;
            }
            else
                norm_correction.Update(0);
            return needed_vel.magnitude * Vector3d.Lerp(pg_vel.normalized, needed_vel.normalized, startF);
        }

        bool coast(float ApA_offset, Vector3d pg_vel)
        {
            Status("Coasting...");
            CFG.BR.OffIfOn(BearingMode.Auto);
            ATC.SetThrustDirW(-pg_vel.xzy);
            THR.Throttle = 0;
            if(CircularizationOffset < 0)
            {
                ApAUT = VSL.Physics.UT + VesselOrbit.timeToAp;
                CircularizationOffset = VSL.Engines.TTB_Precise((float)TrajectoryCalculator.dV4C(VesselOrbit, hV(ApAUT), ApAUT).magnitude) / 2;
            }
            return VesselOrbit.timeToAp > ApA_offset + CircularizationOffset &&
                              Body.atmosphere && VesselOrbit.radius < Body.Radius + Body.atmosphereDepth;
        }

        float max_G_throttle(float maxG) => 
        maxG / Utils.ClampL((VSL.Engines.MaxThrustM / VSL.Physics.M - VSL.Physics.G) / VSL.Physics.StG, maxG);

        public bool GravityTurn(float ApA_offset, float min_throttle, float maxG, float Dtol)
        {
            UpdateTargetPosition();
            VSL.Engines.ActivateNextStageOnFlameout();
            update_state();
            ApoapsisReached |= dApA < Dtol;
            var pg_vel = Vector3d.Lerp(VesselOrbit.vel, VSL.vessel.srf_velocity.xzy, VSL.Physics.G / VSL.Physics.StG);
            CFG.AT.OnIfNot(Attitude.Custom);
            if(!ErrorThreshold)
            {
                CircularizationOffset = -1;
                tune_THR();
                var startF = getStartF();
                var vel = pg_vel;
                var AoA = Utils.Angle2(pg_vel, VesselOrbit.pos);
                if(AoA < 45)
                {
                    pitch.Update(Utils.Clamp(AoA - 45, -GLB.ATCB.MaxAttitudeError, GLB.ATCB.MaxAttitudeError));
                    vel = QuaternionD.AngleAxis(pitch * startF, Vector3d.Cross(target, VesselOrbit.pos)) * pg_vel;
                }
                vel = tune_needed_vel(vel, pg_vel, startF);
                vel = Utils.ClampDirection(vel, pg_vel, (double)GLB.ATCB.MaxAttitudeError);
                throttle.Update(ApA_offset - (float)VesselOrbit.timeToAp);
                THR.Throttle = Utils.Clamp(0.5f + throttle.Action, min_throttle, max_G_throttle(maxG)) *
                    (float)Utils.ClampH(dApA / Dtol / 10, 1);
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

        public bool TargetedGravityTurn(float ApA_offset, float gturn_curve, float dist2vel, float Dtol)
        {
            UpdateTargetPosition();
            VSL.Engines.ActivateNextStageOnFlameout();
            update_state();
            ApoapsisReached |= dApA < Dtol;
            var pg_vel = Vector3d.Lerp(VesselOrbit.vel, VSL.vessel.srf_velocity.xzy, VSL.Physics.G / VSL.Physics.StG);
            CFG.AT.OnIfNot(Attitude.Custom);
            if(!ErrorThreshold)
            {
                CircularizationOffset = -1;
                tune_THR();
                var startF = getStartF();

                time2target_v.Update(ApoapsisReached ? 0 : Utils.ClampL(dApA.current * TimeWarp.fixedDeltaTime / (dApA.old - dApA.current), 0));
                time2target_h.Update(dArc < Dtol ? 0 : Utils.ClampL(dArc.current * TimeWarp.fixedDeltaTime / (dArc.old - dArc.current), 0));
                time2target_diff.Update(time2target_v - time2target_h);
                //var up = VesselOrbit.pos.normalized;
                var dist_v = TargetR - VesselOrbit.radius;
                var max_accel_v = Vector3d.Dot(VSL.Engines.MaxDefThrust, -VSL.Physics.Up) / VSL.Physics.M;
                var min_accel_v = Utils.ClampL(VSL.Physics.G - VSL.VerticalSpeed.Absolute * VSL.VerticalSpeed.Absolute / 2 / dist_v, 0);
                var min_throttle = min_accel_v < max_accel_v ? Utils.ClampH(min_accel_v / max_accel_v + 0.05f, 1) : 1;
                //var t2t_v = ApoapsisReached? 0 : 
                //    time2dist(Vector3d.Dot(VesselOrbit.vel, up), 
                //              Vector3d.Dot(VSL.Engines.Thrust, -VSL.Physics.Up)/VSL.Physics.M-VSL.Physics.G,
                //              dist_v);
                //var t2t_h = dArc < Dtol? 0 : 
                //time2dist(Vector3d.Exclude(up, VesselOrbit.vel).magnitude,
                //Vector3d.Exclude(VSL.Physics.Up, VSL.Engines.Thrust).magnitude/VSL.Physics.M,
                //arc);
                //startF *= Utils.Clamp(1-VSL.vessel.atmDensity/GLB.ORB.AtmDensityOffset, 0, 1);
                //if(ApA_offset > VesselOrbit.timeToAp+1)
                //{
                //    var AoA = Utils.Angle2(pg_vel, VesselOrbit.pos);
                //    pitch.Update(Utils.Clamp(AoA-45, -GLB.ATCB.MaxAttitudeError, GLB.ATCB.MaxAttitudeError));
                //}
                //else //
                //if(double.IsNaN(t2t_v))
                //{
                //    pitch.Update(GLB.ATCB.MaxAttitudeError);
                //    time2target_v.Update(ApA_offset);
                //}
                //else if(double.IsNaN(t2t_h))
                //{
                //    pitch.Update(-GLB.ATCB.MaxAttitudeError);
                //    time2target_v.Update(0);
                //}
                //else
                //{
                //time2target_v.Update(t2t_v-t2t_h);
                //pitch.Update(Utils.Clamp(time2target_v/Math.Max(t2t_v, t2t_h)*gturn_curve,
                ////*Utils.ClampH(1-(ApA_offset-VesselOrbit.timeToAp)/ApA_offset, 1), 
                //-GLB.ATCB.MaxAttitudeError, GLB.ATCB.MaxAttitudeError));
                //}
                //var AoA = 1-Utils.Angle2(pg_vel, VesselOrbit.pos)/45;
                var vel = TrajectoryCalculator.dV4ApV(VesselOrbit, target, VSL.Physics.UT);
                startF *= Utils.ClampH(pg_vel.magnitude * 1.1 / (vel + VesselOrbit.vel).magnitude, 1);
                Log("dV2ApV {}, needed.V {}, pg.V {}, startF {}, t2tv {}, t2th {}, dt2t {}",
                    vel.magnitude, (vel + VesselOrbit.vel).magnitude, pg_vel.magnitude, startF,
                    time2target_v.Value, time2target_h.Value, time2target_diff.Value);
                vel = tune_needed_vel(vel, pg_vel, startF);
                vel = Utils.ClampDirection(vel, -VSL.Engines.CurrentDefThrustDir.xzy(), 
                                           (double)GLB.ATCB.MaxAttitudeError);
                //pitch.Update(Utils.Clamp(time2target_diff/Math.Max(time2target_v, time2target_h)*GLB.ATCB.MaxAttitudeError //,
                //                         *Math.Pow(Utils.ClampH(VesselOrbit.ApA/((TargetR-Body.Radius)*0.9), 1), gturn_curve),
                //                         //*Utils.ClampH(1-(ApA_offset-VesselOrbit.timeToAp)/ApA_offset, 1), 
                //                         -GLB.ATCB.MaxAttitudeError, GLB.ATCB.MaxAttitudeError));
                //;
                //var vel = QuaternionD.AngleAxis(pitch*startF, Vector3d.Cross(target, VesselOrbit.pos)) * pg_vel;
                //if(VesselOrbit.timeToAp < ApA_offset)
                //    throttle.Update((float)(ApA_offset-VesselOrbit.timeToAp));
                //else
                //if(startF > 0.7)
                //    throttle.Update((float)time2target_diff);
                //else
                //throttle.Update(0);
                THR.Throttle = //Utils.Clamp(1+throttle.Action, (float)min_throttle, 1)*
                    (float)Utils.ClampH(ErrorThreshold.Value / Dtol / 10, 1);
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
    }
}

