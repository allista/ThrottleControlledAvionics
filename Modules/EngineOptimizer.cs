//  Author:
//       Allis Tauri <allista@gmail.com>
//
//  Copyright (c) 2015 Allis Tauri
//
// This work is licensed under the Creative Commons Attribution-ShareAlike 4.0 International License. 
// To view a copy of this license, visit http://creativecommons.org/licenses/by-sa/4.0/ 
// or send a letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.

using System.Collections.Generic;
using System.Diagnostics.CodeAnalysis;
using System.Linq;
using AT_Utils;
using AT_Utils.UI;
using JetBrains.Annotations;
using UnityEngine;

namespace ThrottleControlledAvionics
{
    public abstract class TorqueOptimizer : TCAModule
    {
        [SuppressMessage("ReSharper", "FieldCanBeMadeReadOnly.Global"),
         SuppressMessage("ReSharper", "ConvertToConstant.Global")]
        public abstract class Config<T> : ComponentConfig<T> where T : ComponentConfig, new()
        {
            /// <summary>
            /// maximum number of optimizations per fixed frame
            /// </summary>
            [Persistent] public int MaxIterations = 50;
            /// <summary>
            /// optimize engines limits until torque error or delta torque error is less than this
            /// </summary>
            [Persistent] public float OptimizationPrecision = 0.01f;
            /// <summary>
            /// maximum angle in degrees between torque and torque demand
            /// that is considered optimized
            /// </summary>
            [Persistent] public float OptimizationAngleCutoff = 45f;
            /// <summary>
            /// linear weight of angle error in the total error = torque_error+angle_error
            /// </summary>
            [Persistent] public float AngleErrorWeight = 1f;
            /// <summary>
            /// maximum delta between torque and torque demand
            /// that is considered optimized
            /// </summary>
            [Persistent] public float OptimizationTorqueCutoff = 1f;
            /// <summary>
            /// torque-ratio curve
            /// </summary>
            [Persistent] public float TorqueRatioFactor = 0.1f;
            /// <summary>
            /// torque ratio threshold for switching an engine to unbalanced mode
            /// </summary>
            [Persistent] public float UnBalancedThreshold = 0.0001f; //<1 deg

            public float TorqueCutoff;

            public override void Load(ConfigNode node)
            {
                base.Load(node);
                // working in radians
                OptimizationAngleCutoff *= Mathf.Deg2Rad * AngleErrorWeight;
                // working in square magnitude
                OptimizationTorqueCutoff *= OptimizationTorqueCutoff;
                OptimizationPrecision *= OptimizationPrecision;
                // precalculate frequently used values
                TorqueCutoff = OptimizationTorqueCutoff * OptimizationPrecision;
            }
        }

        public float TorqueError { get; protected set; }
        public float TorqueAngle { get; protected set; }

        protected TorqueOptimizer(ModuleTCA tca) : base(tca) { }
    }

    [UsedImplicitly]
    public class EngineOptimizer : TorqueOptimizer
    {
        [SuppressMessage("ReSharper", "FieldCanBeMadeReadOnly.Global"),
         SuppressMessage("ReSharper", "ConvertToConstant.Global")]
        public class Config : Config<Config>
        {
            /// <summary>
            /// default value of P slider
            /// </summary>
            [Persistent] public float MaxP = 1f;
            /// <summary>
            /// default value of I slider
            /// </summary>
            [Persistent] public float MaxI = 1f;
            /// <summary>
            /// thrustPercentage master PI controller defaults
            /// </summary>
            [Persistent] public PI_Controller EnginesPI = new PI_Controller(0.4f, 0.2f);
            /// <summary>
            /// float curve for P value of Engines PI controller = F(torque/MoI)
            /// </summary>
            [Persistent] public FloatCurve EnginesCurve = new FloatCurve();
            /// <summary>
            /// float curve for Pitch,Yaw,Roll steering modifiers = F(torque/MoI)
            /// </summary>
            [Persistent] public FloatCurve SteeringCurve = new FloatCurve();
        }
        public static Config C => Config.INST;

        public EngineOptimizer(ModuleTCA tca) : base(tca) { }

        public override void Disable() { }

        private Vector3 Steering;

        private static bool optimization_for_torque_pass(
            IList<EngineWrapper> engines,
            int num_engines,
            Vector3 target,
            float target_m,
            float eps,
            bool useDefTorque,
            bool torqueOnly
        )
        {
            var compensation = Vector3.zero;
            var maneuver = Vector3.zero;
            var target_n = target/target_m;
            for(var i = 0; i < num_engines; i++)
            {
                var e = engines[i];
                e.limit_tmp = -Vector3.Dot(e.getCurrentTorque(useDefTorque), target_n) / e.getCurrentTorqueM(useDefTorque) * e.getTorqueRatio(useDefTorque || e.Role == TCARole.MANEUVER);
                if(e.limit_tmp > 0)
                    compensation += e.getSpecificTorque(useDefTorque) * e.nominalCurrentThrust(e.throttle * e.limit);
                else if(e.Role == TCARole.MANEUVER)
                {
                    if(e.limit <= 0)
                        e.limit = torqueOnly ? -e.limit_tmp : eps;
                    maneuver += e.getSpecificTorque(useDefTorque) * e.nominalCurrentThrust(e.throttle * e.limit);
                }
                else
                    e.limit_tmp = 0f;
            }
            var compensation_m = compensation.magnitude;
            var maneuver_m = maneuver.magnitude;
            if(compensation_m < eps && maneuver_m.Equals(0)) return false;
            var limits_norm = Mathf.Clamp01(target_m / compensation_m);
            var maneuver_norm = Mathf.Clamp01(target_m / maneuver_m);
            for(var i = 0; i < num_engines; i++)
            {
                var e = engines[i];
                e.limit = e.limit_tmp > 0 ?
                    Mathf.Clamp01(e.limit * (1 - e.limit_tmp * limits_norm)) :
                    Mathf.Clamp01(e.limit * (1 - e.limit_tmp * maneuver_norm));
            }
            return true;
        }

        public static bool OptimizeLimitsForTorque(IList<EngineWrapper> engines, Vector3 needed_torque, Vector3 start_imbalance, Vector3 MoI, bool useDefTorque,
            out float max_limit, out float torque_error, out float angle_error)
        {
            torque_error = -1f;
            angle_error = -1f;
            max_limit = 0;
            var num_engines = engines.Count;
            if(num_engines == 0)
                return true;
            var zero_torque = needed_torque.IsZero();
            var preset_limits = engines.Any(e => e.preset_limit >= 0);
            var torqueOnly = !preset_limits && engines.All(e => e.Role == TCARole.MANEUVER);
            var last_error = -1f;
            for(var i = 0; i < C.MaxIterations; i++)
            {
                // calculate current target
                var cur_imbalance = start_imbalance;
                for(var j = 0; j < num_engines; j++)
                { var e = engines[j]; cur_imbalance += e.Torque(e.throttle * e.limit, useDefTorque); }
                var target = needed_torque - cur_imbalance;
                if(target.IsZero())
                    break;
                // calculate torque and angle errors
                var error = TorqueProps.AngularAcceleration(target, MoI).sqrMagnitude;
                var angle = zero_torque ? 0f : Utils.Angle2Rad(cur_imbalance, needed_torque) * C.AngleErrorWeight;
                //remember the best state
                if(zero_torque && error < torque_error
                   || angle + error < angle_error + torque_error
                   || angle_error < 0)
                {
                    for(var j = 0; j < num_engines; j++)
                    { var e = engines[j]; e.best_limit = e.limit; }
                    torque_error = error;
                    if(!zero_torque && !cur_imbalance.IsZero())
                        angle_error = angle;
                }
                // check convergence conditions
                if(error < C.TorqueCutoff ||
                   last_error > 0 && Mathf.Abs(error - last_error) < C.OptimizationPrecision * last_error)
                    break;
                last_error = error;
                //normalize limits of main and balanced engines before optimization
                if(!preset_limits)
                {
                    var limit_norm = 0f;
                    for(var j = 0; j < num_engines; j++)
                    {
                        var e = engines[j];
                        if(e.Role == TCARole.MANEUVER) continue;
                        if(limit_norm < e.limit) limit_norm = e.limit;
                    }
                    if(limit_norm > 0)
                    {
                        for(var j = 0; j < num_engines; j++)
                        {
                            var e = engines[j];
                            if(e.Role == TCARole.MANEUVER) continue;
                            e.limit = Mathf.Clamp01(e.limit / limit_norm);
                        }
                    }
                }
                //optimize limits
                if(!optimization_for_torque_pass(engines, num_engines, target, target.magnitude, C.OptimizationPrecision, useDefTorque, torqueOnly))
                    break;
            }
            var optimized = torque_error < C.OptimizationTorqueCutoff
                            || (angle_error >= 0 && angle_error < C.OptimizationAngleCutoff);
            //treat single-engine crafts specially
            if(num_engines == 1)
            {
                engines[0].limit = optimized ? engines[0].best_limit : 0f;
                max_limit = engines[0].limit;
            }
            else //restore the best state
            {
                max_limit = 0;
                for(var j = 0; j < num_engines; j++)
                {
                    var e = engines[j];
                    e.limit = e.best_limit;
                    if(e.limit > max_limit)
                        max_limit = e.limit;
                }
            }
            return optimized;
        }

        public bool OptimizeLimitsForTorque(IList<EngineWrapper> engines, Vector3 needed_torque, bool useDefTorque) =>
            OptimizeLimitsForTorque(engines, needed_torque, useDefTorque, out _);

        public bool OptimizeLimitsForTorque(IList<EngineWrapper> engines, Vector3 needed_torque, bool useDefTorque, out float max_limit)
        {
            var ret = OptimizeLimitsForTorque(engines, needed_torque, VSL.Torque.Imbalance.Torque, VSL.Physics.MoI, useDefTorque,
                out max_limit, out var torque_error, out var angle_error);
            TorqueError = torque_error;
            TorqueAngle = angle_error;
            return ret;
        }

        public static void PresetLimitsForTranslation(IList<EngineWrapper> engines, Vector3 translation)
        {
            if(translation.IsZero())
                return;
            var num_engines = engines.Count;
            for(var i = 0; i < num_engines; i++)
            {
                var e = engines[i];
                e.limit_tmp = Vector3.Dot(e.thrustDirection, translation);
                e.limit = e.preset_limit = e.limit_tmp > 0 ? e.limit_tmp : 0;
            }
        }

        public static void LimitInDirection(IList<EngineWrapper> engines, Vector3 dir)
        {
            var num_engines = engines.Count;
            for(var i = 0; i < num_engines; i++)
            {
                var e = engines[i];
                e.limit_tmp = Vector3.Dot(e.thrustDirection, dir);
                e.limit *= 1 - Mathf.Abs(e.limit_tmp);
            }
        }

        public void Steer()
        {
            var engines = VSL.Engines.Active.Steering;
            var num_engines = engines.Count;
            if(num_engines == 0) return;
            //calculate steering
            if(CFG.AutoTune) tune_steering_params();
            var needed_torque = Vector3.zero;
            //tune steering if MaxAA has changed drastically
            Steering = VSL.Controls.Steering * Mathf.Lerp(Utils.ClampH(VSL.Torque.MaxAAMod, 1), 1, VSL.Controls.InvAlignmentFactor);
            if(Steering.sqrMagnitude >= Globals.Instance.InputDeadZone)
            {
                //correct steering
                if(!CFG.AutoTune) Steering *= CFG.SteeringGain;
                Steering.Scale(CFG.SteeringModifier);
                //calculate needed torque
                for(var i = 0; i < num_engines; i++)
                {
                    var e = engines[i];
                    if(Vector3.Dot(e.currentTorque, Steering) > 0)
                        needed_torque += e.currentTorque;
                }
                needed_torque = Vector3.Project(needed_torque, Steering) * Steering.magnitude;
                needed_torque = VSL.Torque.EnginesLimits.Clamp(needed_torque);
            }
            //optimize engines; if failed, set the flag and kill torque if requested
            if(!OptimizeLimitsForTorque(engines, needed_torque, false, out var max_limit) && !needed_torque.IsZero())
            {
                for(var j = 0; j < num_engines; j++) engines[j].InitLimits();
                OptimizeLimitsForTorque(engines, Vector3.zero, false, out max_limit);
                SetState(TCAState.Unoptimized);
            }
            if(VSL.Engines.HaveMainEngines &&
               max_limit < VSL.PostUpdateControls.mainThrottle * 0.05f &&
               !engines.Any(e => e.preset_limit >= 0))
                Status(0.1, Colors.Danger, "Thrust is disabled because engines cannot be balanced.");
        }

        private void tune_steering_params()
        {
            //tune steering modifiers
            if(CFG.AT) CFG.SteeringModifier = Vector3.one;
            else
            {
                CFG.SteeringModifier.x = Mathf.Clamp01(C.SteeringCurve.Evaluate(VSL.Torque.MaxCurrent.AA.x) / 100f);
                CFG.SteeringModifier.y = Mathf.Clamp01(C.SteeringCurve.Evaluate(VSL.Torque.MaxCurrent.AA.y) / 100f);
                CFG.SteeringModifier.z = Mathf.Clamp01(C.SteeringCurve.Evaluate(VSL.Torque.MaxCurrent.AA.z) / 100f);
            }
            //tune PI coefficients
            CFG.Engines.P = C.EnginesCurve.Evaluate(VSL.Torque.Engines.AA_rad);
            CFG.Engines.I = CFG.Engines.P / 2f;
        }
    }
}
