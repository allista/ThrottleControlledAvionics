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
        public bool   CorrectOnlyAltitude;

        Vector3d hv;
        double arc;
        public State<double> dApA = new State<double>(-1);
        public State<double> dArc = new State<double>(-1);
        LowPassFilterD time2target_v = new LowPassFilterD();
        LowPassFilterD time2target_h = new LowPassFilterD();
        LowPassFilterD time2target_diff = new LowPassFilterD();
        LowPassFilterD pitch = new LowPassFilterD();
        PIDf_Controller3 throttle = new PIDf_Controller3();
        double CircularizationOffset = -1;
        bool ApoapsisReached;

        public ToOrbitExecutor(ModuleTCA tca) : base(tca) 
        { 
            InitModuleFields();
            Executor = new ManeuverExecutor(tca);
            GearAction.action = () => VSL.GearOn(false);
            ErrorThreshold.Lower = 2*GLB.ORB.Dtol;
            ErrorThreshold.Upper = 4*GLB.ORB.Dtol;
            throttle.setPID(GLB.ORB.ThrottlePID);
            GravityTurnStart = 0;
            time2target_v.Tau = 0.5f;
            time2target_h.Tau = 0.5f;
            time2target_diff.Tau = 0.5f;
            pitch.Tau = 0.5f;
        }

        public void UpdateTargetPosition()
        {
            Target = QuaternionD.AngleAxis(Body.angularV*TimeWarp.fixedDeltaTime*Mathf.Rad2Deg, 
                                           Body.zUpAngularVelocity.normalized)*Target;
        }

        public bool Liftoff()
        {
            UpdateTargetPosition();
            VSL.Engines.ActivateEngines();
            VSL.OnPlanetParams.ActivateLaunchClamps();
            Log("vV {}/G {} < {}", VSL.VerticalSpeed.Absolute, VSL.Physics.G, MinClimbTime);//debug
            if(VSL.VerticalSpeed.Absolute/VSL.Physics.G < MinClimbTime)
            { 
                Status("Liftoff...");
                CFG.DisableVSC();
                CFG.AT.OnIfNot(Attitude.Custom);
                ATC.SetThrustDirW(-VSL.Physics.Up);
                CFG.VTOLAssistON = true;
                THR.Throttle = 1;
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
            var cApV = VesselOrbit.getRelativePositionAtUT(VSL.Physics.UT+VesselOrbit.timeToAp);
            hv = Vector3d.Exclude(VesselOrbit.pos, target-VesselOrbit.pos).normalized;
            dApA.current = TargetR-VesselOrbit.ApR;
            dArc.current = Utils.ProjectionAngle(cApV, target, hv)*Mathf.Deg2Rad*cApV.magnitude;
            arc = Utils.ProjectionAngle(VesselOrbit.pos, target, hv)*Mathf.Deg2Rad*VesselOrbit.radius;
            ErrorThreshold.Value = CorrectOnlyAltitude? dApA : dApA+dArc;
        }

        /// <summary>
        /// The arc distance in radians between current vessel position and the Target.
        /// </summary>
        public double ArcDistance
        { get { return Utils.ProjectionAngle(VesselOrbit.pos, target, target-VesselOrbit.pos) * Mathf.Deg2Rad; } }

        public bool GravityTurn(double ApA_offset, double gturn_curve, double dist2vel, double Dtol)
        {
            UpdateTargetPosition();
            VSL.Engines.ActivateNextStageOnFlameout();
            update_state();
            ApoapsisReached |= dApA < Dtol;
            var pg_vel = Vector3d.Lerp(VesselOrbit.vel, VSL.vessel.srf_velocity.xzy, VSL.Physics.G/VSL.Physics.StG);
            CFG.AT.OnIfNot(Attitude.Custom);
            if(!ErrorThreshold)
            {
                CircularizationOffset = -1;
                THR.CorrectThrottle = ApoapsisReached;
                if(VSL.vessel.dynamicPressurekPa > GLB.ORB.MaxDynPressure)
                    THR.MaxThrottle = Mathf.Max(1-((float)VSL.vessel.dynamicPressurekPa-GLB.ORB.MaxDynPressure)/5, 0);
                time2target_v.Update(ApoapsisReached? 0 : Utils.ClampL(dApA.current*TimeWarp.fixedDeltaTime/(dApA.old-dApA.current), 0));
                time2target_h.Update(dArc < Dtol? 0 : Utils.ClampL(dArc.current*TimeWarp.fixedDeltaTime/(dArc.old-dArc.current), 0));
                time2target_diff.Update(time2target_v-time2target_h);
                //var up = VesselOrbit.pos.normalized;
                var dist_v = TargetR-VesselOrbit.radius;
                var max_accel_v = Vector3d.Dot(VSL.Engines.MaxDefThrust, -VSL.Physics.Up)/VSL.Physics.M;
                var min_accel_v = Utils.ClampL(VSL.Physics.G-VSL.VerticalSpeed.Absolute*VSL.VerticalSpeed.Absolute/2/dist_v, 0);
                var min_throttle = min_accel_v < max_accel_v? Utils.ClampH(min_accel_v/max_accel_v+0.05f, 1) : 1;
                //var t2t_v = ApoapsisReached? 0 : 
                //    time2dist(Vector3d.Dot(VesselOrbit.vel, up), 
                //              Vector3d.Dot(VSL.Engines.Thrust, -VSL.Physics.Up)/VSL.Physics.M-VSL.Physics.G,
                //              dist_v);
                //var t2t_h = dArc < Dtol? 0 : 
                    //time2dist(Vector3d.Exclude(up, VesselOrbit.vel).magnitude,
                              //Vector3d.Exclude(VSL.Physics.Up, VSL.Engines.Thrust).magnitude/VSL.Physics.M,
                              //arc);
                if(Body.atmosphere && VSL.vessel.atmDensity > GLB.ORB.AtmDensityOffset)
                    GravityTurnStart = VSL.Altitude.Absolute;
                var startF = Utils.Clamp((VSL.Altitude.Absolute-GravityTurnStart)/GLB.ORB.GTurnOffset, 0, 1);
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
                startF *= Math.Pow(Utils.ClampH(pg_vel.magnitude/(vel+VesselOrbit.vel).magnitude, 1), gturn_curve);
                Log("dV2ApV {}, needed.V {}, pg.V {}, startF {}, t2tv {}, t2th {}, dt2t {}", 
                    vel.magnitude, (vel+VesselOrbit.vel).magnitude, pg_vel.magnitude, startF,
                    time2target_v.Value, time2target_h.Value, time2target_diff.Value);
                if(vel.sqrMagnitude < 0.1) 
                    vel = pg_vel;
                var norm = VesselOrbit.GetOrbitNormal();
                var dFi = (90-Utils.Angle2(norm, target))*Mathf.Deg2Rad;
                vel += norm*Math.Sin(dFi)*vel.magnitude*startF
                                *Utils.Clamp(VSL.VerticalSpeed.Absolute/VSL.Physics.G-MinClimbTime, 0, 100)
                                *Utils.ClampL(Vector3d.Dot(hv, VesselOrbit.vel.normalized), 0);
                vel = vel.magnitude * Vector3d.Lerp(pg_vel.normalized, vel.normalized, startF);
                var thrust_dir = -VSL.Engines.CurrentDefThrustDir.xzy();
                var angle = Utils.Angle2((Vector3)vel, thrust_dir);
                if(angle > GLB.ATCB.MaxAttitudeError)
                    vel = QuaternionD.AngleAxis(GLB.ATCB.MaxAttitudeError,
                        //45*(1.1-VSL.vessel.atmDensity/GLB.ORB.AtmDensityOffset), 
                                                Vector3d.Cross(thrust_dir, vel)) 
                                     * thrust_dir * vel.magnitude;
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
                    (float)Utils.ClampH(ErrorThreshold.Value/Dtol/10, 1);
                ATC.SetThrustDirW(-vel.xzy);
                if(CFG.AT.Not(Attitude.KillRotation)) 
                {
                    CFG.BR.OnIfNot(BearingMode.Auto);
                    BRC.ForwardDirection = hv.xzy;
                }
                Status("Gravity turn...");
                return true;
            }
            Status("Coasting...");
            CFG.BR.OffIfOn(BearingMode.Auto);
            ATC.SetThrustDirW(-pg_vel.xzy);
            THR.Throttle = 0;
            if(CircularizationOffset < 0)
            {
                ApAUT = VSL.Physics.UT+VesselOrbit.timeToAp;
                CircularizationOffset = VSL.Engines.TTB_Precise((float)TrajectoryCalculator.dV4C(VesselOrbit, hV(ApAUT), ApAUT).magnitude)/2;
            }
            return VesselOrbit.timeToAp > ApA_offset+CircularizationOffset &&
                Body.atmosphere && VesselOrbit.radius < Body.Radius+Body.atmosphereDepth;
        }
    }
}

