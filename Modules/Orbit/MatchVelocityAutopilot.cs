//  Author:
//       Allis Tauri <allista@gmail.com>
//
//  Copyright (c) 2015 Allis Tauri
//
// This work is licensed under the Creative Commons Attribution-ShareAlike 4.0 International License. 
// To view a copy of this license, visit http://creativecommons.org/licenses/by-sa/4.0/ 
// or send a letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.

using System;
using UnityEngine;
using AT_Utils;

namespace ThrottleControlledAvionics
{
    [CareerPart]
    [RequireModules(typeof(ManeuverAutopilot),
                 typeof(AttitudeControl),
                 typeof(ThrottleControl),
                 typeof(TranslationControl),
                 typeof(TimeWarpControl))]
    public class MatchVelocityAutopilot : TCAModule
    {
        public class Config : ModuleConfig
        {
            [Persistent] public float TranslationThreshold = 5f; //m/s
            [Persistent] public float MaxApproachDistance = 10000f; //m
        }

        static Config MVA { get { return Globals.Instance.MVA; } }

        public MatchVelocityAutopilot(ModuleTCA tca) : base(tca)
        {
        }

        #pragma warning disable 169
        ThrottleControl THR;
        #pragma warning restore 169

        public float MinDeltaV = 1;
        ManeuverExecutor Executor;
        float TTA = -1;

        public enum Stage
        {
            Start,
            Brake,
            Wait
        }

        [Persistent] public Stage stage;

        public override void Init()
        {
            base.Init();
            CFG.AP1.AddCallback(MatchVelCallback, Autopilot1.MatchVel, Autopilot1.MatchVelNear);
            Executor = new ManeuverExecutor(TCA);
            Executor.ThrustWhenAligned = true;
            Executor.StopAtMinimum = true;
            MinDeltaV = GLB.THR.MinDeltaV;
        }

        public override void Disable()
        {
            CFG.AP1.OffIfOn(Autopilot1.MatchVel, Autopilot1.MatchVelNear);
        }

        protected override void UpdateState()
        { 
            base.UpdateState();
            IsActive &= (VSL.Engines.HaveThrusters || VSL.Engines.HaveNextStageEngines) &&
                !VSL.LandedOrSplashed && VSL.orbit != null &&
                CFG.Target && CFG.Target.GetOrbit() != null &&
                CFG.AP1.Any(Autopilot1.MatchVel, Autopilot1.MatchVelNear);
            var tVSL = VSL.TargetVessel;
            ControlsActive &= IsActive || VSL.InOrbit && VSL.Engines.HaveThrusters && tVSL != null && !tVSL.LandedOrSplashed && tVSL.mainBody == VSL.Body;
        }

        public void MatchVelCallback(Multiplexer.Command cmd)
        {
            switch(cmd)
            {
            case Multiplexer.Command.Resume:
            case Multiplexer.Command.On:
                Working = false;
                THR.Throttle = 0;
                stage = Stage.Start;
                SetTarget(VSL.TargetAsWP);
                CFG.AT.On(Attitude.KillRotation);
                break;

            case Multiplexer.Command.Off:
                CFG.AT.On(Attitude.KillRotation);
                StopUsingTarget();
                Reset();
                break;
            }
        }

        protected override void Reset()
        {
            base.Reset();
            if(Working)
            {
                THR.Throttle = 0;
                CFG.AT.On(Attitude.KillRotation);
            }
            CFG.AP1.OffIfOn(Autopilot1.MatchVel);
            CFG.AP1.OffIfOn(Autopilot1.MatchVelNear);
            MinDeltaV = GLB.THR.MinDeltaV;
            stage = Stage.Start;
            Executor.Reset();
            Working = false;
        }

        public static float BrakeDistance(float V0, VesselWrapper VSL, out float ttb)
        { 
            ttb = VSL.Engines.TTB(V0);
            var throttle = ThrottleControl.NextThrottle((float)V0, 1, VSL);
            if(CheatOptions.InfinitePropellant)
                return (V0 - VSL.Engines.MaxThrustM * throttle * ttb / 2 / VSL.Physics.M) * ttb;
            return V0 * ttb +
            VSL.Engines.MaxThrustM / VSL.Engines.MaxMassFlow *
            ((ttb - VSL.Physics.M / VSL.Engines.MaxMassFlow / throttle) *
            Mathf.Log((VSL.Physics.M - VSL.Engines.MaxMassFlow * throttle * ttb) / VSL.Physics.M) - ttb);
        }

        public static float BrakeDistance(float V0, float ttb, float thrust, float mass, float mflow, float throttle)
        {
            if(CheatOptions.InfinitePropellant)
                return (V0 - thrust * throttle * ttb / 2 / mass) * ttb;
            var _mflow = mflow * throttle;
            return V0 * ttb + thrust / mflow * ((ttb - mass / _mflow) * Mathf.Log((mass - _mflow * ttb) / mass) - ttb);
        }

        public static float BrakeDistancePrecise(float V0, VesselWrapper VSL, out float ttb)
        {
            ttb = 0f;
            var dist = 0f;
            var mass = VSL.Physics.M;
            var throttle = ThrottleControl.NextThrottle(V0, 1, mass, VSL.Engines.MaxThrustM, VSL.Engines.DecelerationTime10);
            float next_mass;
            float ttb_cur;
            while(V0 > 0.1)
            {
                throttle = ThrottleControl.NextThrottle(V0, throttle, mass, VSL.Engines.MaxThrustM, VSL.Engines.DecelerationTime10);
                ttb_cur = EnginesProps.TTB(V0 / 2, VSL.Engines.MaxThrustM, mass, VSL.Engines.MaxMassFlow, throttle, out next_mass);
                dist += BrakeDistance(V0, ttb_cur, VSL.Engines.MaxThrustM, mass, VSL.Engines.MaxMassFlow, throttle);
                ttb += ttb_cur;
                mass = next_mass;
                V0 /= 2;
            }
            throttle = ThrottleControl.NextThrottle(V0, throttle, mass, VSL.Engines.MaxThrustM, VSL.Engines.DecelerationTime10);
            ttb_cur = EnginesProps.TTB(V0, VSL.Engines.MaxThrustM, mass, VSL.Engines.MaxMassFlow, throttle, out next_mass);
            dist += BrakeDistance(V0, ttb_cur, VSL.Engines.MaxThrustM, mass, VSL.Engines.MaxMassFlow, throttle);
            ttb += ttb_cur;
            return dist;
        }

        public static float BrakingOffset(float V0, VesselWrapper VSL, out float ttb)
        {
            return BrakeDistancePrecise(V0, VSL, out ttb) / V0;
        }

        public static float BrakingOffset(float V0, VesselWrapper VSL)
        {
            float ttb;
            return BrakeDistancePrecise(V0, VSL, out ttb) / V0;
        }

        public static float BrakingNodeCorrection(float V0, VesselWrapper VSL)
        { 
            float ttb;
            var offset = BrakingOffset(V0, VSL, out ttb);
            return offset - ttb / 2;
        }

        bool StartCondition(float dV)
        {
            if(Working) return true;
            if(TTA > 0)
            {
                VSL.Info.Countdown = TTA - BrakingOffset(dV, VSL);
                if(VSL.Info.Countdown > 0)
                {
                    if(VSL.Controls.CanWarp)
                        VSL.Controls.WarpToTime = VSL.Physics.UT + VSL.Info.Countdown - VSL.Controls.MinAlignmentTime;
                    return false;
                }
            }
            VSL.Info.Countdown = 0;
            Working = true;
            return true;
        }

        protected override void Update()
        {
            Vector3 dV;
            if(CFG.AP1[Autopilot1.MatchVel])
            {
                Working = true;
                dV = CFG.Target.GetObtVelocity() - VSL.vessel.obt_velocity;
                if(!Executor.Execute(dV, GLB.THR.MinDeltaV))
                    Executor.Reset();
            }
            else
            {
                double ApprUT;
                var tOrb = CFG.Target.GetOrbit();
                var dist = TrajectoryCalculator.NearestApproach(VSL.orbit, tOrb, VSL.Physics.UT, VSL.Geometry.MinDistance+10, out ApprUT);
                TTA = (float)(ApprUT - VSL.Physics.UT);
                switch(stage)
                {
                case Stage.Start:
                    if(dist > MVA.MaxApproachDistance)
                    {
                        Status(string.Format("<color=yellow>WARNING:</color> Nearest approach distance is <color=magenta><b>{0}</b></color>\n" +
                        "<color=red><b>Push to proceed. At your own risk.</b></color>", 
                           Utils.formatBigValue((float)dist, "m")));
                        stage = Stage.Wait;
                        goto case Stage.Wait;
                    }
                    stage = Stage.Brake;
                    goto case Stage.Brake;
                case Stage.Wait:
                    if(!string.IsNullOrEmpty(TCAGui.StatusMessage)) break;
                    stage = Stage.Brake;
                    goto case Stage.Brake;
                case Stage.Brake:
                    dV = (TrajectoryCalculator.NextOrbit(tOrb, ApprUT).GetFrameVelAtUT(ApprUT) - VSL.orbit.GetFrameVelAtUT(ApprUT)).xzy;
                    if(!Executor.Execute(dV, GLB.THR.MinDeltaV, StartCondition)) Reset();
                    break;
                }
            }
        }

        public override void Draw()
        {
            if(ControlsActive)
            {
                if(Utils.ButtonSwitch("Match Velocity", CFG.AP1[Autopilot1.MatchVel], 
                          "Continuously match orbital velocity with the target", GUILayout.ExpandWidth(true)))
                    CFG.AP1.XToggle(Autopilot1.MatchVel);
                if(Utils.ButtonSwitch("Brake Near Target", CFG.AP1[Autopilot1.MatchVelNear], 
                          "Match orbital velocity with the target at closest approach", GUILayout.ExpandWidth(true)))
                    CFG.AP1.XToggle(Autopilot1.MatchVelNear);
            }
            else
            {
                GUILayout.Label(new GUIContent("Match Velocity", "Continuously match orbital velocity with the target"), 
                    Styles.inactive_button, GUILayout.ExpandWidth(true));
                GUILayout.Label(new GUIContent("Brake Near Target", "Match orbital velocity with the target at closest approach"), 
                    Styles.inactive_button, GUILayout.ExpandWidth(true));
            }
        }
    }
}

