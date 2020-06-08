//  Author:
//       Allis Tauri <allista@gmail.com>
//
//  Copyright (c) 2015 Allis Tauri
//
// This work is licensed under the Creative Commons Attribution-ShareAlike 4.0 International License. 
// To view a copy of this license, visit http://creativecommons.org/licenses/by-sa/4.0/ 
// or send a letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.

using System.Collections.Generic;
using System.Linq;
using AT_Utils;
using JetBrains.Annotations;
using UnityEngine;

namespace ThrottleControlledAvionics
{
    [UsedImplicitly]
    public class RCSOptimizer : TorqueOptimizer
    {
        public class Config : Config<Config> { }
        public static Config C => Config.INST;

        public RCSOptimizer(ModuleTCA tca) : base(tca) { }

        public override void Disable() { }

        private static bool optimization_pass(IList<RCSWrapper> engines, int num_engines, Vector3 target, float target_m, float eps)
        {
            var compensation = Vector3.zero;
            for(var i = 0; i < num_engines; i++)
            {
                var e = engines[i];
                e.limit_tmp = -Vector3.Dot(e.currentTorque, target) / target_m / e.currentTorque_m * e.torqueRatio;
                if(e.limit_tmp > 0) compensation += e.Torque(e.limit);
            }
            var compensation_m = compensation.magnitude;
            if(compensation_m < eps) return false;
            var limits_norm = Mathf.Clamp01(target_m / compensation_m);
            for(var i = 0; i < num_engines; i++)
            {
                var e = engines[i];
                if(e.limit_tmp > 0)
                    e.limit = Mathf.Clamp01(e.limit * (1 - e.limit_tmp * limits_norm));
            }
            return true;
        }

        public bool Optimize(IList<RCSWrapper> engines, Vector3 needed_torque)
        {
            var num_engines = engines.Count;
            if(num_engines == 0)
                return true;
            var zero_torque = needed_torque.IsZero();
            var preset_limits = engines.Any(e => e.preset_limit >= 0);
            var last_error = -1f;
            TorqueAngle = TorqueError = -1f;
            for(var i = 0; i < C.MaxIterations; i++)
            {
                // calculate current target
                var cur_imbalance = Vector3.zero;
                for(var j = 0; j < num_engines; j++)
                { var e = engines[j]; cur_imbalance += e.Torque(e.limit); }
                var target = needed_torque - cur_imbalance;
                if(target.IsZero())
                    break;
                // calculate torque and angle errors
                var error = VSL.Torque.AngularAcceleration(target).sqrMagnitude;
                var angle = zero_torque ? 0f : Utils.Angle2Rad(cur_imbalance, needed_torque) * C.AngleErrorWeight;
                //remember the best state
                if(zero_torque && error < TorqueError || angle + error < TorqueAngle + TorqueError || TorqueAngle < 0)
                {
                    for(var j = 0; j < num_engines; j++)
                    { var e = engines[j]; e.best_limit = e.limit; }
                    TorqueError = error;
                    if(!zero_torque && !cur_imbalance.IsZero())
                        TorqueAngle = angle;
                }
                //check convergence conditions
                if(error < C.TorqueCutoff ||
                   last_error > 0 && Mathf.Abs(error - last_error) < C.OptimizationPrecision * last_error)
                    break;
                last_error = error;
                //normalize limits before optimization
                if(!preset_limits)
                {
                    var limit_norm = 0f;
                    for(var j = 0; j < num_engines; j++)
                    {
                        var e = engines[j];
                        if(limit_norm < e.limit) limit_norm = e.limit;
                    }
                    if(limit_norm > 0)
                    {
                        for(var j = 0; j < num_engines; j++)
                        { var e = engines[j]; e.limit = Mathf.Clamp01(e.limit / limit_norm); }
                    }
                }
                if(!optimization_pass(engines, num_engines, target, target.magnitude, C.OptimizationPrecision))
                    break;
            }
            var optimized = TorqueError < C.OptimizationTorqueCutoff
                            || (TorqueAngle >= 0 && TorqueAngle < C.OptimizationAngleCutoff);
            //treat single-engine crafts specially
            if(num_engines == 1)
                engines[0].limit = optimized ? 1f : 0f;
            else //restore the best state
                for(var j = 0; j < num_engines; j++)
                { var e = engines[j]; e.limit = e.best_limit; }
            return optimized;
        }

        public void Steer()
        {
            if(VSL.Engines.NoActiveRCS) return;
            //calculate needed torque
            var needed_torque = Vector3.zero;
            if(CFG.RotateWithRCS && VSL.Controls.HasSteering)
            {
                for(var i = 0; i < VSL.Engines.NumActiveRCS; i++)
                    needed_torque += VSL.Engines.ActiveRCS[i].currentTorque;
                needed_torque = Vector3.Project(needed_torque, VSL.Controls.Steering);
                needed_torque = VSL.Torque.RCSLimits.Clamp(needed_torque);
            }
            //optimize engines; if failed, set the flag and kill torque if requested
            if(Optimize(VSL.Engines.ActiveRCS, needed_torque) || needed_torque.IsZero())
                return;
            for(var j = 0; j < VSL.Engines.NumActiveRCS; j++)
                VSL.Engines.ActiveRCS[j].InitLimits();
            Optimize(VSL.Engines.ActiveRCS, Vector3.zero);
        }
    }
}

