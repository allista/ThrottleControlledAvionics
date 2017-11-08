//  Author:
//       Allis Tauri <allista@gmail.com>
//
//  Copyright (c) 2015 Allis Tauri
//
// This work is licensed under the Creative Commons Attribution-ShareAlike 4.0 International License. 
// To view a copy of this license, visit http://creativecommons.org/licenses/by-sa/4.0/ 
// or send a letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.

using System;
using System.Collections.Generic;
using UnityEngine;
using AT_Utils;

namespace ThrottleControlledAvionics
{
	public class RCSOptimizer : TorqueOptimizer
	{
		static RCSOptimizer.Config RCS { get { return Globals.Instance.RCS; } }
		public RCSOptimizer(ModuleTCA tca) : base(tca) {}

        public override void Disable() {}

		static bool optimization_pass(IList<RCSWrapper> engines, int num_engines, Vector3 target, float target_m, float eps)
		{
			var compensation = Vector3.zero;
			for(int i = 0; i < num_engines; i++)
			{
				var e = engines[i];
				e.limit_tmp = -Vector3.Dot(e.currentTorque, target)/target_m/e.currentTorque_m*e.torqueRatio;
				if(e.limit_tmp > 0)	compensation += e.Torque(e.limit);
			}
			var compensation_m = compensation.magnitude;
			if(compensation_m < eps) return false;
			var limits_norm = Mathf.Clamp01(target_m/compensation_m);
			for(int i = 0; i < num_engines; i++)
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
			var zero_torque = needed_torque.IsZero();
			TorqueAngle = TorqueError = -1f;
			float error, angle;
			var last_error = -1f;
			Vector3 cur_imbalance, target;
			for(int i = 0; i < RCS.MaxIterations; i++)
			{
				//calculate current errors and target
				cur_imbalance = Vector3.zero;
				for(int j = 0; j < num_engines; j++) 
				{ var e = engines[j]; cur_imbalance += e.Torque(e.limit); }
				angle  = zero_torque? 0f : Utils.Angle2(cur_imbalance, needed_torque);
				target = needed_torque-cur_imbalance;
				error  = VSL.Torque.AngularAcceleration(target).magnitude;
				//remember the best state
				if(angle <= 0f && error < TorqueError || angle < TorqueAngle || TorqueAngle < 0) 
				{ 
					for(int j = 0; j < num_engines; j++) 
					{ var e = engines[j]; e.best_limit = e.limit; }
					TorqueAngle = angle;
					TorqueError = error;
				}
				//check convergence conditions
				if(error < RCS.OptimizationTorqueCutoff*RCS.OptimizationPrecision || 
				   last_error > 0 && Mathf.Abs(error-last_error) < RCS.OptimizationPrecision*last_error)
					break;
				last_error = error;
				//normalize limits before optimization
				var limit_norm = 0f;
				for(int j = 0; j < num_engines; j++) 
				{ 
					var e = engines[j];
					if(limit_norm < e.limit) limit_norm = e.limit; 
				}
				if(limit_norm > 0)
				{
					for(int j = 0; j < num_engines; j++) 
					{ var e = engines[j]; e.limit = Mathf.Clamp01(e.limit / limit_norm); }
				}
				if(!optimization_pass(engines, num_engines, target, error, RCS.OptimizationPrecision)) 
					break;
			}
			var optimized = TorqueError < RCS.OptimizationTorqueCutoff || 
				(!zero_torque && TorqueAngle < RCS.OptimizationAngleCutoff);
			//treat single-engine crafts specially
			if(num_engines == 1) 
				engines[0].limit = optimized? 1f : 0f;
			else //restore the best state
				for(int j = 0; j < num_engines; j++) 
				{ var e = engines[j]; e.limit = e.best_limit; }
			return optimized;
		}

		public void Steer()
		{
			if(VSL.Engines.NoActiveRCS) return;
			//calculate needed torque
			var needed_torque = Vector3.zero;
			if(VSL.Controls.Steering.sqrMagnitude >= Globals.Instance.InputDeadZone)
			{
				for(int i = 0; i < VSL.Engines.NumActiveRCS; i++)
				{ needed_torque += VSL.Engines.ActiveRCS[i].currentTorque; }
				needed_torque = Vector3.Project(needed_torque, VSL.Controls.Steering);
                needed_torque = VSL.Torque.RCSLimits.Clamp(needed_torque);
			}
			//optimize engines; if failed, set the flag and kill torque if requested
			if(!Optimize(VSL.Engines.ActiveRCS, needed_torque) && !needed_torque.IsZero())
			{
				for(int j = 0; j < VSL.Engines.NumActiveRCS; j++) VSL.Engines.ActiveRCS[j].InitLimits();
				Optimize(VSL.Engines.ActiveRCS, Vector3.zero);
			}
		}
	}
}

