//  Author:
//       Allis Tauri <allista@gmail.com>
//
//  Copyright (c) 2016 Allis Tauri
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
            [Persistent] public float MaxG = 3;
            [Persistent] public float MinThrottle = 10;
            [Persistent] public float MinClimbTime = 5;
            [Persistent] public float MaxDynPressure = 10f;
            [Persistent] public float AtmDensityOffset = 10f;
            [Persistent] public float AscentEccentricity = 0.3f;
            [Persistent] public float GTurnOffset = 0.1f;

            [Persistent] public PIDf_Controller3 PitchPID = new PIDf_Controller3();
            [Persistent] public PIDf_Controller3 ThrottlePID = new PIDf_Controller3();
            [Persistent] public PIDf_Controller3 NormCorrectionPID = new PIDf_Controller3();
        }
        public static Config C => Config.INST;

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
        [Persistent] public double LaunchUT = -1;
        [Persistent] public double ApAUT = -1;
        [Persistent] public double GravityTurnStart;
        [Persistent] public bool CorrectOnlyAltitude;

        [Persistent] public bool AutoTimeToApA = true;
        [Persistent] public FloatField TimeToApA = new FloatField(format: "F1", min: 5, max: 300);
        [Persistent] public FloatField MaxG = new FloatField(format: "F1", min: 0.1f, max: 100);
        [Persistent] public FloatField MinThrottle = new FloatField(format: "F1", min: 1, max: 100);

        /// <summary>
        /// The arc distance in radians between current vessel position and the Target.
        /// </summary>
        public double ArcDistance => 
        Utils.ProjectionAngle(VesselOrbit.pos, target, target - VesselOrbit.pos) * Mathf.Deg2Rad;

        public double MinApR => 
        VesselOrbit.MinPeR()+1000;

        public double MaxApR => 
        VesselOrbit.MinPeR()+ToOrbitAutopilot.C.RadiusOffset;

        public double TimeToClosestApA => 
        VesselOrbit.ApAhead()? VesselOrbit.timeToAp : VesselOrbit.timeToAp-VesselOrbit.period;

        public double dApA { get; protected set; } = -1;
        public double dArc { get; protected set; } = -1;
        protected Vector3d htdir, hvdir, ApV;
        protected PIDf_Controller3 pitch = new PIDf_Controller3();
        protected PIDf_Controller3 norm_correction = new PIDf_Controller3();
        protected PIDf_Controller3 throttle = new PIDf_Controller3();
        protected double CircularizationOffset = -1;
        protected bool ApoapsisReached;
        protected bool CourseOnTarget;

        public override void Save(ConfigNode node)
        {
            base.Save(node);
            node.AddValue("Target", ConfigNode.WriteVector(target));
        }

        public override void Load(ConfigNode node)
        {
            base.Load(node);
            var tgt = node.GetValue("Target");
            if(!string.IsNullOrEmpty(tgt))
                Target = ConfigNode.ParseVector3D(tgt);
        }

        public ToOrbitExecutor() : base(null)
        {
            ErrorThreshold.Lower = ToOrbitAutopilot.C.Dtol / 10;
            ErrorThreshold.Upper = ToOrbitAutopilot.C.Dtol;
            pitch.setPID(C.PitchPID);
            pitch.setClamp(AttitudeControlBase.C.MaxAttitudeError);
            throttle.setPID(C.ThrottlePID);
            throttle.setClamp(0.5f);
            norm_correction.setPID(C.NormCorrectionPID);
            norm_correction.setClamp(AttitudeControlBase.C.MaxAttitudeError);
            TimeToApA.Value = TrajectoryCalculator.C.ManeuverOffset;
            MinThrottle.Value = C.MinThrottle;
            MaxG.Value = C.MaxG;
        }

        public virtual void AttachTCA(ModuleTCA tca)
        {
            if(TCA == null)
            {
                TCA = tca;
                InitModuleFields();
                GearAction.action = () => VSL.GearOn(false);
                TimeToApA.Value = Mathf.Max(TimeToApA, VSL.Torque.MaxCurrent.TurnTime);
            }
        }

        public virtual void Reset()
        {
            dApA = dArc = -1;
            LaunchUT = -1;
            ApAUT = -1;
            GravityTurnStart = 0;
            CircularizationOffset = -1;
            ApoapsisReached = false;
            Target = Vector3d.zero;
            ErrorThreshold.Reset();
            pitch.Reset();
            throttle.Reset();
            norm_correction.Reset();
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
            hvdir = Vector3d.Exclude(VesselOrbit.pos, VesselOrbit.vel).normalized;
            htdir = Vector3d.Exclude(VesselOrbit.pos, target - VesselOrbit.pos).normalized;
            ApV = VesselOrbit.getRelativePositionAtUT(VSL.Physics.UT + VesselOrbit.timeToAp);
            dApA = Utils.ClampL(TargetR - VesselOrbit.ApR, 0);
            dArc = Utils.ClampL(Utils.ProjectionAngle(ApV, target, htdir) * ToOrbitAutopilot.C.Dtol, 0);
            ErrorThreshold.Value = CorrectOnlyAltitude ? dApA : dApA + dArc;
            ApoapsisReached |= dApA < Dtol;
            CourseOnTarget = Vector3d.Dot(htdir, hvdir) > 0.1;
        }


        protected void tune_THR()
        {
            THR.CorrectThrottle = ApoapsisReached;
            if(VSL.vessel.dynamicPressurekPa > C.MaxDynPressure)
                THR.MaxThrottle = Mathf.Max(1 - ((float)VSL.vessel.dynamicPressurekPa - C.MaxDynPressure) / 5, 0);
        }

        protected double getStartF()
        {
            if(Body.atmosphere && VSL.vessel.atmDensity > C.AtmDensityOffset)
                GravityTurnStart = VSL.Altitude.Absolute;
            return Utils.Clamp((VSL.Altitude.Absolute - GravityTurnStart) / (TargetR-Body.Radius)/C.GTurnOffset, 0, 1);
        }

        protected Vector3d tune_needed_vel(Vector3d needed_vel, Vector3d pg_vel, double startF)
        {
            if(CourseOnTarget)
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

        protected bool coast(Vector3d pg_vel)
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
            return VesselOrbit.timeToAp > TimeToApA + CircularizationOffset &&
                              Body.atmosphere && VesselOrbit.radius < Body.Radius + Body.atmosphereDepth;
        }

        protected float max_G_throttle()
        {
            var MaxThrust = Vector3.Dot(VSL.Physics.Up, -VSL.Engines.MaxThrust);
            if(MaxThrust <= 0) return 1;
            return MaxG / Utils.ClampL((MaxThrust / VSL.Physics.M - VSL.Physics.G) / VSL.Physics.StG, MaxG);
        }

        protected Vector3d get_pg_vel()
        => Vector3d.Lerp(VesselOrbit.vel, VSL.vessel.srf_velocity.xzy, VSL.Physics.G / VSL.Physics.StG);

        protected void auto_ApA_offset()
        {
            if(AutoTimeToApA)
            {
                var dEcc = C.AscentEccentricity - (float)VesselOrbit.eccentricity;
                var rel_dApA = (float)((TargetR - VesselOrbit.ApR) / (TargetR - Body.Radius) - 0.1);
                TimeToApA.Value = Mathf.Max(TrajectoryCalculator.C.ManeuverOffset * (1 + dEcc + rel_dApA), 
                                            VSL.Torque.MaxCurrent.TurnTime);
                TimeToApA.ClampValue();
            }
        }

        public void UpdateTargetPosition()
        {
            Target = QuaternionD.AngleAxis(Body.angularV * TimeWarp.fixedDeltaTime * Mathf.Rad2Deg,
                                           Body.zUpAngularVelocity.normalized) * Target;
        }

        public virtual bool Liftoff()
        {
            UpdateTargetPosition();
            VSL.Engines.ActivateEngines();
            VSL.OnPlanetParams.ActivateLaunchClamps();
            if(VSL.VerticalSpeed.Absolute / VSL.Physics.G < Config.INST.MinClimbTime)
            {
                Status("Liftoff...");
                CFG.DisableVSC();
                CFG.VTOLAssistON = true;
                THR.Throttle = max_G_throttle();
                CFG.AT.OnIfNot(Attitude.Custom);
                var vel = VSL.Physics.Up;
                vel = tune_needed_vel(vel, vel, 1);
                ATC.SetThrustDirW(vel);
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

        public bool GravityTurn(float Dtol)
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
                auto_ApA_offset();
                var startF = getStartF();
                var vel = pg_vel;
                var AoA = Utils.Angle2(pg_vel, VesselOrbit.pos);
                if(AoA < 45)
                {
                    pitch.Update((float)AoA - Mathf.Lerp(45, 15, (float)(VSL.vessel.atmDensity-0.1)));
                    vel = QuaternionD.AngleAxis(pitch * startF, Vector3d.Cross(target, VesselOrbit.pos)) * pg_vel;
                }
                vel = tune_needed_vel(vel, pg_vel, startF);
                vel = Utils.ClampDirection(vel, pg_vel, (double)AttitudeControlBase.C.MaxAttitudeError);
                throttle.Update(TimeToApA - (float)VesselOrbit.timeToAp);
                THR.Throttle = Utils.Clamp(0.5f + throttle, MinThrottle/100, max_G_throttle()) *
                    (float)Utils.ClampH(dApA / Dtol / VSL.Engines.TMR, 1);
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

        public void DrawOptions()
        {
            GUILayout.BeginHorizontal();
            {
                GUILayout.BeginVertical();
                {
                    GUILayout.Label(new GUIContent("Time to Apoapsis:",
                                                   "More time to apoapsis means steeper trajectory " +
                                                   "and greater acceleration. Low values can " +
                                                   "save a lot of fuel."),
                                    GUILayout.ExpandWidth(true));
                    GUILayout.Label(new GUIContent("Min. Throttle:",
                                                   "Minimum throttle value. " +
                                                   "Increasing it will shorten the last stage of the ascent."),
                                    GUILayout.ExpandWidth(true));
                    GUILayout.Label(new GUIContent("Max. Acceleration:",
                                                   "Maximum allowed acceleration (in gees of the current planet). " +
                                                   "Smooths gravity turn on low-gravity worlds. Saves fuel."),
                                    GUILayout.ExpandWidth(true));
                }
                GUILayout.EndVertical();
                GUILayout.BeginVertical();
                {
                    GUILayout.BeginHorizontal();
                    {
                        GUILayout.FlexibleSpace();
                        Utils.ButtonSwitch("Auto", ref AutoTimeToApA,
                                           "Tune time to apoapsis automatically",
                                           GUILayout.ExpandWidth(false));
                    }
                    GUILayout.EndHorizontal();
                    GUILayout.FlexibleSpace();
                    GUILayout.FlexibleSpace();
                }
                GUILayout.EndVertical();
                GUILayout.BeginVertical();
                {
                    TimeToApA.Draw("s", 5, "F1", suffix_width: 25);
                    MinThrottle.Draw("%", 5, "F1", suffix_width: 25);
                    MaxG.Draw("g", 0.5f, "F1", suffix_width: 25);
                }
                GUILayout.EndVertical();
            }
            GUILayout.EndHorizontal();
        }

        public void DrawInfo(double target_inclination = -1)
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
                    GUILayout.Label(string.Format("{0} ► {1}",
                                                  KSPUtil.PrintDateDeltaCompact(TimeToClosestApA, true, true),
                                                  KSPUtil.PrintDateDeltaCompact(TimeToApA, true, true)));
                }
                GUILayout.EndVertical();
            }
            GUILayout.EndHorizontal();
        }
    }
}

