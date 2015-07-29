//   TorqueBalancer.cs
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
using System.Linq;
using System.Collections.Generic;
using UnityEngine;

namespace ThrottleControlledAvionics
{
	public class TorqueOptimizer : TCAModule
	{
		public class Config : ModuleConfig
		{
			new public const string NODE_NAME = "TRQ";

			//engine balancing parameters
			[Persistent] public int   MaxIterations            = 50;    //maximum number of optimizations per fixed frame
			[Persistent] public float OptimizationPrecision    = 0.1f;  //optimize engines limits until torque error or delta torque error is less than this
			[Persistent] public float OptimizationAngleCutoff  = 45f;   //maximum angle between torque imbalance and torque demand that is considered optimized
			[Persistent] public float OptimizationTorqueCutoff = 1f;    //maximum torque delta between imbalance and demand that is considered optimized
			[Persistent] public float TorqueRatioFactor        = 0.1f;  //torque-ratio curve
			//default values for PI controllers
			[Persistent] public float MaxP = 1f; //value of P slider
			[Persistent] public float MaxI = 1f; //value of I slider
			[Persistent] public PI_Controller EnginesPI = new PI_Controller(0.4f, 0.2f); //thrustPercentage master PI controller defaults
			[Persistent] public FloatCurve EnginesCurve = new FloatCurve();  //float curve for P value of Engines PI controller = F(torque/MoI)
			[Persistent] public FloatCurve SteeringCurve = new FloatCurve(); // float curve for Pitch,Yaw,Roll steering modifiers = F(torque/MoI)

			public override void Load(ConfigNode node) 
			{ 
				var n = node.GetNode(Utils.PropertyName(new {SteeringCurve}));
				if(n != null) SteeringCurve.Load(n);
				n = node.GetNode(Utils.PropertyName(new {EnginesCurve}));
				if(n != null) EnginesCurve.Load(n);
			}
		}
		static Config TRQ { get { return TCAConfiguration.Globals.TRQ; } }

		public TorqueOptimizer(VesselWrapper vsl) { vessel = vsl; }
		public override void Init() {  }

		public Vector3 Steering { get; private set; }
		public float   TorqueError { get; private set; }
		public float   TorqueAngle { get; private set; }

		static bool optimization_pass(IList<EngineWrapper> engines, int num_engines, Vector3 target, float target_m, float eps)
		{
			var compensation = Vector3.zero;
			var maneuver = Vector3.zero;
			for(int i = 0; i < num_engines; i++)
			{
				var e = engines[i];
				e.limit_tmp = -Vector3.Dot(e.currentTorque, target)/target_m/e.currentTorque_m*e.torqueRatio;
				if(e.limit_tmp > 0)
					compensation += e.specificTorque * e.nominalCurrentThrust(e.throttle * e.limit);
				else if(e.Role == TCARole.MANEUVER)
				{
					if(e.limit.Equals(0)) e.limit = eps;
					maneuver +=  e.specificTorque * e.nominalCurrentThrust(e.throttle * e.limit);
				} else e.limit_tmp = 0f;
			}
			var compensation_m = compensation.magnitude;
			var maneuver_m = maneuver.magnitude;
			if(compensation_m < eps && maneuver_m.Equals(0)) return false;
			var limits_norm = Mathf.Clamp01(target_m/compensation_m);
			var maneuver_norm = Mathf.Clamp01(target_m/maneuver_m);
			for(int i = 0; i < num_engines; i++)
			{
				var e = engines[i];
				e.limit = e.limit_tmp > 0 ? 
					Mathf.Clamp01(e.limit * (1 - e.limit_tmp * limits_norm)) : 
					Mathf.Clamp01(e.limit * (1 - e.limit_tmp * maneuver_norm));
			}
			return true;
		}

		public bool OptimizeEngines(IList<EngineWrapper> engines, Vector3 needed_torque)
		{
			var num_engines = engines.Count;
			var zero_torque = needed_torque.IsZero();
			TorqueAngle = -1f;
			TorqueError = -1f;
			float error, angle;
			var last_error = -1f;
			Vector3 cur_imbalance, target;
			for(int i = 0; i < TRQ.MaxIterations; i++)
			{
				//calculate current errors and target
				cur_imbalance = vessel.Torque;
				for(int j = 0; j < num_engines; j++) 
				{ var e = engines[j]; cur_imbalance += e.Torque(e.throttle * e.limit); }
				angle  = zero_torque? 0f : Vector3.Angle(cur_imbalance, needed_torque);
				target = needed_torque-cur_imbalance;
				error  = target.magnitude;
				//remember the best state
				if(angle <= 0f && error < TorqueError || angle+error < TorqueAngle+TorqueError || TorqueAngle < 0) 
				{ 
					for(int j = 0; j < num_engines; j++) 
					{ var e = engines[j]; e.best_limit = e.limit; }
					TorqueAngle = angle;
					TorqueError = error;
				}
				//check convergence conditions
				if(error < TRQ.OptimizationPrecision || 
				   last_error > 0 && Mathf.Abs(error-last_error) < TRQ.OptimizationPrecision/10)
					break;
				last_error = error;
				//normalize limits before optimization
				if(vessel.NormalizeLimits)
				{   //this is much faster than linq
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
				}
				//optimize limits
				if(!optimization_pass(engines, num_engines, target, error, TRQ.OptimizationPrecision)) 
					break;
			}
			var optimized = TorqueError < TRQ.OptimizationTorqueCutoff || 
				(!zero_torque && TorqueAngle < TRQ.OptimizationAngleCutoff);
			//treat single-engine crafts specially
			if(num_engines == 1) 
				engines[0].limit = optimized? 1f : 0f;
			else //restore the best state
				for(int j = 0; j < num_engines; j++) 
				{ var e = engines[j]; e.limit = e.best_limit; }
			return optimized;
		}

		public void SteerWithEngines()
		{
			if(vessel.SteeringEngines.Count == 0) return;
			//calculate steering
			if(CFG.AutoTune) tune_steering_params();
			Steering = new Vector3(vessel.ctrlState.pitch, vessel.ctrlState.roll, vessel.ctrlState.yaw);
			var num_engines = vessel.SteeringEngines.Count;
			if(!Steering.IsZero())
			{
				Steering = Steering/Steering.CubeNorm().magnitude;
				if(!CFG.AutoTune) Steering *= CFG.SteeringGain;
				Steering.Scale(CFG.SteeringModifier);
			}
			//calculate needed torque
			var needed_torque = Vector3.zero;
			for(int i = 0; i < num_engines; i++)
			{
				var e = vessel.SteeringEngines[i];
				if(Vector3.Dot(e.currentTorque, Steering) > 0)
					needed_torque += e.currentTorque;
			}
			needed_torque = vessel.E_TorqueLimits.Clamp(Vector3.Project(needed_torque, Steering) * Steering.magnitude);
			//optimize engines; if failed, set the flag and kill torque if requested
			if(!OptimizeEngines(vessel.SteeringEngines, needed_torque) && !needed_torque.IsZero())
			{
				//				DebugEngines(engines, needed_torque);//debug
				for(int j = 0; j < num_engines; j++) vessel.SteeringEngines[j].InitLimits();
				OptimizeEngines(vessel.SteeringEngines, Vector3.zero);
				SetState(TCAState.Unoptimized);
			}
			//			DebugEngines(engines, needed_torque);//debug
		}

		void tune_steering_params()
		{
			//tune steering modifiers
			CFG.SteeringModifier.x = Mathf.Clamp(TRQ.SteeringCurve.Evaluate(vessel.MaxAngularA.x)/100f, 0, 1);
			CFG.SteeringModifier.y = Mathf.Clamp(TRQ.SteeringCurve.Evaluate(vessel.MaxAngularA.y)/100f, 0, 1);
			CFG.SteeringModifier.z = Mathf.Clamp(TRQ.SteeringCurve.Evaluate(vessel.MaxAngularA.z)/100f, 0, 1);
			//tune PI coefficients
			CFG.Engines.P = TRQ.EnginesCurve.Evaluate(vessel.MaxAngularA.magnitude);
			CFG.Engines.I = CFG.Engines.P/2f;
			#if DEBUG
			//			Utils.Log("max_torque: {0}\n" + //debug
			//                    "MoI {1}\n" +
			//                    "angularA {2}", 
			//                     max_torque, MoI, angularA);
			#endif
		}

		#if DEBUG
		void DebugEngines(IList<EngineWrapper> engines, Vector3 needed_torque)
		{
			Utils.Log("Engines:\n"+
			          engines.Aggregate("", (s, e) => s 
			                            +string.Format("engine(vec{0}, vec{1}, vec{2}, {3}, {4}),\n",
			                                           vessel.refT.InverseTransformDirection(e.thrustInfo.pos-vessel.wCoM),
			                                           e.thrustDirection,e.specificTorque, e.minThrust, e.maxThrust)));
			Utils.Log("Engines Torque:\n"+engines.Aggregate("", (s, e) => s + "vec"+e.Torque(e.throttle*e.limit)+",\n"));
			Utils.Log(
				"Steering: {0}\n" +
				"Needed Torque: {1}\n" +
				"Torque Imbalance: {2}\n" +
				"Torque Error: {3}kNm, {4}deg\n" +
				"Torque Clamp:\n   +{5}\n   -{6}\n" +
				"Limits: [{7}]", 
				Steering,
				needed_torque,
				engines.Aggregate(Vector3.zero, (v,e) => v+e.Torque(e.throttle*e.limit)),
				TorqueError, TorqueAngle,
				vessel.E_TorqueLimits.positive, 
				vessel.E_TorqueLimits.negative,
				engines.Aggregate("", (s, e) => s+e.limit+" ").Trim()
			);
		}
		#endif
	}
}

