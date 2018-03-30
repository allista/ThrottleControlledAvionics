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
        public State<double> dApA = new State<double>(-1);
        public State<double> dArc = new State<double>(-1);
        LowPassFilterD ddApA_dt = new LowPassFilterD();
        LowPassFilterD ddArc_dt = new LowPassFilterD();
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
            ddApA_dt.Tau = 0.5f;
            ddArc_dt.Tau = 0.5f;
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

        //double time2dist(double v, double a, double d) => a.Equals(0)? d/v :(Math.Sqrt(v*v+2*d*a)-v)/a;

        void update_state()
        {
            var cApV = VesselOrbit.getRelativePositionAtUT(VSL.Physics.UT+VesselOrbit.timeToAp);
            hv = Vector3d.Exclude(VesselOrbit.pos, target-VesselOrbit.pos).normalized;
            dApA.current = TargetR-VesselOrbit.ApR;
            dArc.current = Utils.ProjectionAngle(cApV, target, hv)*Mathf.Deg2Rad*cApV.magnitude;
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
            var pg_vel = Vector3d.Lerp(VesselOrbit.vel.normalized, VSL.vessel.srf_vel_direction.xzy, VSL.Physics.G/VSL.Physics.StG);
            CFG.AT.OnIfNot(Attitude.Custom);
            if(!ErrorThreshold)
            {
                CircularizationOffset = -1;
                THR.CorrectThrottle = ApoapsisReached;
                if(VSL.vessel.dynamicPressurekPa > GLB.ORB.MaxDynPressure)
                    THR.MaxThrottle = Mathf.Max(1-((float)VSL.vessel.dynamicPressurekPa-GLB.ORB.MaxDynPressure)/5, 0);
                ddApA_dt.Update(ApoapsisReached? 0 : dApA.current*TimeWarp.fixedDeltaTime/(dApA.old-dApA.current));
                ddArc_dt.Update(dArc < Dtol? 0 : dArc.current*TimeWarp.fixedDeltaTime/(dArc.old-dArc.current));
                var startF = Utils.Clamp((VSL.Altitude.Absolute-GravityTurnStart)/GLB.ORB.GTurnOffset, 0, 1);
                pitch.Update(Utils.Clamp((ddApA_dt-ddArc_dt)/Math.Max(ddApA_dt, ddArc_dt)*gturn_curve, 
                                         -GLB.ATCB.MaxAttitudeError, GLB.ATCB.MaxAttitudeError));
                var vel = QuaternionD.AngleAxis(pitch*startF, Vector3d.Cross(target, VesselOrbit.pos)) * pg_vel;
                var norm = VesselOrbit.GetOrbitNormal();
                var dFi = (90-Utils.Angle2(norm, target))*Mathf.Deg2Rad;
                vel += norm*Math.Sin(dFi)*vel.magnitude*startF
                    *Utils.Clamp(VSL.VerticalSpeed.Absolute/VSL.Physics.G-MinClimbTime, 0, 100)
                    *Utils.ClampL(Vector3d.Dot(hv, VesselOrbit.vel.normalized), 0);
                throttle.Update((float)(ApA_offset-VesselOrbit.timeToAp));
                THR.Throttle = Utils.ClampH(0.5f+throttle.Action, 1)
                    *(float)Utils.ClampH(ErrorThreshold.Value/Dtol/3, 1);
                ATC.SetThrustDirW(-vel.xzy);
                Log("dApA {}, dArc {}, dApA/(ddApA/dt) {}, dArc/(ddArc/dt) {}, pitch {}, toApA {}, ApA_offset {}\n" +
                    "pid {}, startF {}, errF {}, throttle {}", 
                    dApA, dArc, ddApA_dt, ddArc_dt, pitch, VesselOrbit.timeToAp, ApA_offset,
                    throttle, startF, Utils.ClampH(ErrorThreshold.Value/Dtol/3, 1), THR.Throttle);//debug
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

