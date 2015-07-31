//   RCSOptimizer.cs
//
//  Author:
//       Allis Tauri <allista@gmail.com>
//
//  Copyright (c) 2015 Allis Tauri
//
// This work is licensed under the Creative Commons Attribution 4.0 International License. 
// To view a copy of this license, visit http://creativecommons.org/licenses/by/4.0/ 
// or send a letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.

using System;
using System.Collections.Generic;
using UnityEngine;

namespace ThrottleControlledAvionics
{
	public class RCSOptimizer : TCAModule
	{
		public class Config : ModuleConfig
		{
			new public const string NODE_NAME = "RCS";

			[Persistent] public int   MaxIterations            = 50;    //maximum number of optimizations per fixed frame
			[Persistent] public float OptimizationPrecision    = 0.01f; //optimize engines limits until torque error or delta torque error is less than this
			[Persistent] public float OptimizationAngleCutoff  = 5f;    //maximum angle between torque imbalance and torque demand that is considered optimized
			[Persistent] public float OptimizationTorqueCutoff = 0.1f;  //maximum torque delta between imbalance and demand that is considered optimized
			[Persistent] public float TorqueRatioFactor        = 0.1f;  //torque-ratio curve
		}
		static RCSOptimizer.Config RCS { get { return TCAConfiguration.Globals.RCS; } }

		public RCSOptimizer(VesselWrapper vsl) { vessel = vsl; }
		public override void Init() {  }

		public float TorqueError { get; private set; }
		public float TorqueAngle { get; private set; }

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
//			var error_threshold = RCS.OptimizationTorqueCutoff*vessel.StockMoI.magnitude;
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
				angle  = zero_torque? 0f : Vector3.Angle(cur_imbalance, needed_torque);
				target = needed_torque-cur_imbalance;
				error  = target.magnitude;
				//remember the best state
				if(angle <= 0f && error < TorqueError || angle < TorqueAngle || TorqueAngle < 0) 
				{ 
					for(int j = 0; j < num_engines; j++) 
					{ var e = engines[j]; e.best_limit = e.limit; }
					TorqueAngle = angle;
					TorqueError = error;
				}
				//check convergence conditions
				if(error < RCS.OptimizationPrecision || 
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
				//optimize limits
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
			if(vessel.NoActiveRCS) return;
			//calculate needed torque
			var needed_torque = Vector3.zero;
			if(vessel.Steering.sqrMagnitude >= TCAConfiguration.Globals.InputDeadZone)
			{
				for(int i = 0; i < vessel.NumActiveRCS; i++)
				{ needed_torque += vessel.ActiveRCS[i].currentTorque; }
				needed_torque = Vector3.Project(needed_torque, vessel.Steering);
			}
			//optimize engines; if failed, set the flag and kill torque if requested
			if(!Optimize(vessel.ActiveRCS, needed_torque) && !needed_torque.IsZero())
			{
				for(int j = 0; j < vessel.NumActiveRCS; j++) vessel.ActiveRCS[j].InitLimits();
				Optimize(vessel.ActiveRCS, Vector3.zero);
			}
		}
	}
}

