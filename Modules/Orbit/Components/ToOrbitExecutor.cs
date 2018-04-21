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
        public class Config : ComponentConfig<Config>
        {
            [Persistent] public float MinClimbTime = 5;
        }

        protected ThrottleControl THR;
        protected AttitudeControl ATC;
        protected BearingControl BRC;

        protected readonly SingleAction GearAction = new SingleAction();
        protected readonly FuzzyThreshold<double> ErrorThreshold = new FuzzyThreshold<double>();

        protected Vector3d target;
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

        /// <summary>
        /// The arc distance in radians between current vessel position and the Target.
        /// </summary>
        public double ArcDistance
        { get { return Utils.ProjectionAngle(VesselOrbit.pos, target, target - VesselOrbit.pos) * Mathf.Deg2Rad; } }

        protected Vector3d hv, ApV;
        public State<double> dApA = new State<double>(-1);
        public State<double> dArc = new State<double>(-1);
        protected PIDf_Controller3 pitch = new PIDf_Controller3();
        protected PIDf_Controller3 norm_correction = new PIDf_Controller3();
        protected PIDf_Controller3 throttle = new PIDf_Controller3();
        protected double CircularizationOffset = -1;
        protected bool ApoapsisReached;

        public ToOrbitExecutor(ModuleTCA tca) : base(tca)
        {
            InitModuleFields();
            GearAction.action = () => VSL.GearOn(false);
            ErrorThreshold.Lower = ToOrbitAutopilot.C.Dtol / 10;
            ErrorThreshold.Upper = ToOrbitAutopilot.C.Dtol;
            pitch.setPID(ToOrbitAutopilot.C.PitchPID);
            pitch.setClamp(AttitudeControlBase.C.MaxAttitudeError);
            throttle.setPID(ToOrbitAutopilot.C.ThrottlePID);
            throttle.setClamp(0.5f);
            norm_correction.setPID(ToOrbitAutopilot.C.NormCorrectionPID);
            norm_correction.setClamp(AttitudeControlBase.C.MaxAttitudeError);
            GravityTurnStart = 0;
        }

        protected double time2dist(double v, double a, double d)
        {
            if(a.Equals(0)) return d / v;
            var D = v * v + 2 * d * a;
            if(D < 0) return double.NaN;
            return (Math.Sqrt(D) - v) / a;
        }

        protected void update_state(float Dtol)
        {
            hv = Vector3d.Exclude(VesselOrbit.pos, target - VesselOrbit.pos).normalized;
            ApV = VesselOrbit.getRelativePositionAtUT(VSL.Physics.UT + VesselOrbit.timeToAp);
            dApA.current = TargetR - VesselOrbit.ApR;
            dArc.current = Utils.ProjectionAngle(ApV, target, hv) * Mathf.Deg2Rad * ApV.magnitude;
            ErrorThreshold.Value = CorrectOnlyAltitude ? dApA : dApA + dArc;
            ApoapsisReached |= dApA < Dtol;
        }


        protected void tune_THR()
        {
            THR.CorrectThrottle = ApoapsisReached;
            if(VSL.vessel.dynamicPressurekPa > ToOrbitAutopilot.C.MaxDynPressure)
                THR.MaxThrottle = Mathf.Max(1 - ((float)VSL.vessel.dynamicPressurekPa - ToOrbitAutopilot.C.MaxDynPressure) / 5, 0);
        }

        protected double getStartF()
        {
            if(Body.atmosphere && VSL.vessel.atmDensity > ToOrbitAutopilot.C.AtmDensityOffset)
                GravityTurnStart = VSL.Altitude.Absolute;
            return Utils.Clamp((VSL.Altitude.Absolute - GravityTurnStart) / ToOrbitAutopilot.C.GTurnOffset, 0, 1);
        }

        protected Vector3d tune_needed_vel(Vector3d needed_vel, Vector3d pg_vel, double startF)
        {
            if(Vector3d.Dot(hv, Vector3d.Exclude(VesselOrbit.pos, VesselOrbit.vel).normalized) > 0.1)
            {
                norm_correction.Update((float)(90 - Utils.Angle2(VesselOrbit.GetOrbitNormal(), target)));
                needed_vel = QuaternionD.AngleAxis(norm_correction.Action, VesselOrbit.pos) * needed_vel;
            }
            else
                norm_correction.Update(0);
            return startF < 1 ?
                needed_vel.magnitude * Vector3d.Lerp(pg_vel.normalized,
                                                     needed_vel.normalized,
                                                     startF)
                              : needed_vel;
        }

        protected bool coast(float ApA_offset, Vector3d pg_vel)
        {
            Status("Coasting...");
            CFG.BR.OffIfOn(BearingMode.Auto);
            CFG.AT.OnIfNot(Attitude.Custom);
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

        protected float max_G_throttle(float maxG) =>
        maxG / Utils.ClampL((VSL.Engines.MaxThrustM / VSL.Physics.M - VSL.Physics.G) / VSL.Physics.StG, maxG);

        protected Vector3d get_pg_vel()
        => Vector3d.Lerp(VesselOrbit.vel, VSL.vessel.srf_velocity.xzy, VSL.Physics.G / VSL.Physics.StG);

        public void UpdateTargetPosition()
        {
            Target = QuaternionD.AngleAxis(Body.angularV * TimeWarp.fixedDeltaTime * Mathf.Rad2Deg,
                                           Body.zUpAngularVelocity.normalized) * Target;
        }

        public virtual bool Liftoff(float maxG)
        {
            UpdateTargetPosition();
            VSL.Engines.ActivateEngines();
            VSL.OnPlanetParams.ActivateLaunchClamps();
            if(VSL.VerticalSpeed.Absolute / VSL.Physics.G < Config.INST.MinClimbTime)
            {
                Status("Liftoff...");
                CFG.DisableVSC();
                CFG.HF.OnIfNot(HFlight.Stop);
                CFG.VTOLAssistON = true;
                THR.Throttle = max_G_throttle(maxG);
                if(VSL.vessel.srfSpeed > 1)
                {
                    var upErr = Utils.Angle2(-(Vector3)VSL.Physics.Up, VSL.Engines.CurrentDefThrustDir);
                    var velErr = Utils.Angle2(VSL.Physics.Up, VSL.vessel.srf_velocity) / VSL.vessel.srfSpeed;
                    //Log("velErr {}, upErr {}, F {}", velErr, upErr, 1 - velErr - upErr);//debug
                    THR.Throttle = Utils.Clamp(THR.Throttle * (1 - (float)velErr - upErr),
                                               VSL.OnPlanetParams.GeeVSF * 1.1f, 1);
                }
                return true;
            }
            StartGravityTurn();
            return false;
        }

        public virtual void StartGravityTurn()
        {
            GravityTurnStart = VSL.Altitude.Absolute;
            ApoapsisReached = false;
            GearAction.Run();
            CFG.VTOLAssistON = false;
            CFG.StabilizeFlight = false;
            CFG.HF.Off();
            update_state(0);
        }

        public bool GravityTurn(float ApA_offset, float min_throttle, float maxG, float Dtol)
        {
            UpdateTargetPosition();
            VSL.Engines.ActivateNextStageOnFlameout();
            update_state(Dtol);
            var pg_vel = get_pg_vel();
            if(!ErrorThreshold)
            {
                CFG.AT.OnIfNot(Attitude.Custom);
                CircularizationOffset = -1;
                tune_THR();
                var startF = getStartF();
                var vel = pg_vel;
                var AoA = Utils.Angle2(pg_vel, VesselOrbit.pos);
                if(AoA < 45)
                {
                    pitch.Update((float)AoA - 45);
                    vel = QuaternionD.AngleAxis(pitch * startF, Vector3d.Cross(target, VesselOrbit.pos)) * pg_vel;
                }
                vel = tune_needed_vel(vel, pg_vel, startF);
                vel = Utils.ClampDirection(vel, pg_vel, (double)AttitudeControlBase.C.MaxAttitudeError);
                throttle.Update(ApA_offset - (float)VesselOrbit.timeToAp);
                THR.Throttle = Utils.Clamp(0.5f + throttle, min_throttle, max_G_throttle(maxG)) *
                    (float)Utils.ClampH(dApA / Dtol / 10, 1);
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

        public void Draw(double target_inclination = -1)
        {
            GUILayout.BeginHorizontal();
            {
                GUILayout.BeginVertical();
                {
                    GUILayout.Label("Inclination:");
                    GUILayout.Label("Apoapsis:");
                    GUILayout.Label("Time to Apoapsis:");
                }
                GUILayout.EndVertical();
                GUILayout.BeginVertical();
                {
                    if(target_inclination >= 0)
                        GUILayout.Label(string.Format("{0:F3}° ► {1:F3}° Err: {2:F3}°",
                                                      VesselOrbit.inclination,
                                                      target_inclination,
                                                      norm_correction.LastError));
                    else
                        GUILayout.Label(string.Format("{0:F3}° Err: {1:F3}°",
                                                      VesselOrbit.inclination,
                                                      norm_correction.LastError));
                    GUILayout.Label(string.Format("{0} ► {1}",
                                                  Utils.formatBigValue((float)VesselOrbit.ApA, "m", "F3"),
                                                  Utils.formatBigValue((float)(TargetR - Body.Radius), "m", "F3")));
                    GUILayout.Label(KSPUtil.PrintDateDeltaCompact(VesselOrbit.timeToAp, true, true));
                }
                GUILayout.EndVertical();
            }
            GUILayout.EndHorizontal();
        }
    }
}

