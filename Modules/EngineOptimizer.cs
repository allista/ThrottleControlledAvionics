//  Author:
//       Allis Tauri <allista@gmail.com>
//
//  Copyright (c) 2015 Allis Tauri
//
// This work is licensed under the Creative Commons Attribution-ShareAlike 4.0 International License. 
// To view a copy of this license, visit http://creativecommons.org/licenses/by-sa/4.0/ 
// or send a letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.

using System;
using System.Linq;
using System.Collections.Generic;
using UnityEngine;
using AT_Utils;

namespace ThrottleControlledAvionics
{
	public abstract class TorqueOptimizer : TCAModule
	{
		public class Config : ModuleConfig
		{
			[Persistent] public int   MaxIterations            = 50;    //maximum number of optimizations per fixed frame
			[Persistent] public float OptimizationPrecision    = 0.01f;  //optimize engines limits until torque error or delta torque error is less than this
			[Persistent] public float OptimizationAngleCutoff  = 45f;   //maximum angle between torque imbalance and torque demand that is considered optimized
			[Persistent] public float OptimizationTorqueCutoff = 1f;    //maximum torque delta between imbalance and demand that is considered optimized
			[Persistent] public float TorqueRatioFactor        = 0.1f;  //torque-ratio curve
			[Persistent] public float UnBalancedThreshold      = 0.0001f; //<1 deg
		}

		public float TorqueError { get; protected set; }
		public float TorqueAngle { get; protected set; }

		protected TorqueOptimizer(ModuleTCA tca) : base(tca) {}
	}

	public class EngineOptimizer : TorqueOptimizer
	{
		public new class Config : TorqueOptimizer.Config
		{
			[Persistent] public float ThrustOptimizationPrecision = 0.001f;
			[Persistent] public float ThrustOptimizationCutoff    = 1f;

			//default values for PI controllers
			[Persistent] public float MaxP = 1f; //value of P slider
			[Persistent] public float MaxI = 1f; //value of I slider
			[Persistent] public PI_Controller EnginesPI  = new PI_Controller(0.4f, 0.2f); //thrustPercentage master PI controller defaults
			[Persistent] public FloatCurve EnginesCurve  = new FloatCurve();  //float curve for P value of Engines PI controller = F(torque/MoI)
			[Persistent] public FloatCurve SteeringCurve = new FloatCurve(); // float curve for Pitch,Yaw,Roll steering modifiers = F(torque/MoI)
		}
		static Config ENG { get { return Globals.Instance.ENG; } }

		public EngineOptimizer(ModuleTCA tca) : base(tca) {}

        public override void Disable() {}

		Vector3 Steering;

		static bool optimization_for_torque_pass(IList<EngineWrapper> engines, int num_engines, Vector3 target, float target_m, float eps, bool useDefTorque)
		{
			var compensation = Vector3.zero;
			var maneuver = Vector3.zero;
			for(int i = 0; i < num_engines; i++)
			{
				var e = engines[i];
                e.limit_tmp = -Vector3.Dot(e.getCurrentTorque(useDefTorque), target)/target_m/e.getCurrentTorqueM(useDefTorque)*e.getTorqueRatio(useDefTorque);
				if(e.limit_tmp > 0)
                    compensation += e.getSpecificTorque(useDefTorque) * e.nominalCurrentThrust(e.throttle * e.limit);
				else if(e.Role == TCARole.MANEUVER)
				{
					if(e.limit.Equals(0)) e.limit = eps;
                    maneuver +=  e.getSpecificTorque(useDefTorque) * e.nominalCurrentThrust(e.throttle * e.limit);
				} 
                else 
                    e.limit_tmp = 0f;
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

        public static bool OptimizeLimitsForTorque(IList<EngineWrapper> engines, Vector3 needed_torque, Vector3 start_imbalance, Vector3 MoI, bool useDefTorque, 
		                                           out float max_limit, out float torque_error, out float angle_error)
		{
			var num_engines = engines.Count;
			var zero_torque = needed_torque.IsZero();
			var preset_limits = engines.Any(e => e.preset_limit >= 0);
			float error, angle;
			var last_error = -1f;
			Vector3 cur_imbalance = start_imbalance;
			Vector3 target;
			torque_error = -1f;
			angle_error = -1f;
//			Utils.Log("=============================== Optimization ===============================\n" +
//			          "needed torque {}\n" +
//			          "start imbalance {}\n" +
//			          "MoI {}\n" +
//			          "engines:\n{}", 
//			          needed_torque, start_imbalance, MoI, engines);//debug
			for(int i = 0; i < ENG.MaxIterations; i++)
			{
				//calculate current errors and target
				cur_imbalance = start_imbalance;
				for(int j = 0; j < num_engines; j++) 
                { var e = engines[j]; cur_imbalance += e.Torque(e.throttle * e.limit, useDefTorque); }
				angle  = zero_torque? 0f : Utils.Angle2(cur_imbalance, needed_torque);
				target = needed_torque-cur_imbalance;
				error  = TorqueProps.AngularAcceleration(target, MoI).magnitude;
//				Utils.Log("current imbalance: {}\nerror: {} < {}", cur_imbalance, error, ENG.OptimizationTorqueCutoff*ENG.OptimizationPrecision);//debug
				//remember the best state
				if(angle <= 0 && error < torque_error || angle+error < angle_error+torque_error || angle_error < 0) 
				{ 
					for(int j = 0; j < num_engines; j++) 
					{ var e = engines[j]; e.best_limit = e.limit; }
					angle_error = angle;
					torque_error = error;
				}
				//check convergence conditions
				if(error < ENG.OptimizationTorqueCutoff*ENG.OptimizationPrecision || 
				   last_error > 0 && Mathf.Abs(error-last_error) < ENG.OptimizationPrecision*last_error)
					break;
				last_error = error;
				//normalize limits of main and balanced engines before optimization
				if(!preset_limits)
				{
					var limit_norm = 0f;
					for(int j = 0; j < num_engines; j++) 
					{ 
						var e = engines[j];
						if(e.Role == TCARole.MANEUVER) continue;
						if(limit_norm < e.limit) limit_norm = e.limit; 
					}
					if(limit_norm > 0)
					{
						for(int j = 0; j < num_engines; j++) 
						{ 
							var e = engines[j]; 
							if(e.Role == TCARole.MANEUVER) continue;
							e.limit = Mathf.Clamp01(e.limit / limit_norm); 
						}
					}
				}
				//optimize limits
                if(!optimization_for_torque_pass(engines, num_engines, target, target.magnitude, ENG.OptimizationPrecision, useDefTorque)) 
					break;
			}
			var optimized = torque_error < ENG.OptimizationTorqueCutoff || 
				(!zero_torque && angle_error < ENG.OptimizationAngleCutoff);
//			Utils.Log("num engines {}, optimized {}, TorqueError {}, TorqueAngle {}\nneeded torque {}\ncurrent turque {}\nlimits:\n{}\n" +
//				"-------------------------------------------------------------------------------------------------", 
//			    num_engines, optimized, torque_error, angle_error, needed_torque, cur_imbalance,
//			    engines.Aggregate("", (s, e) => s+string.Format("{0}: VSF {1:P1}, throttle {2:P1}, best limit {3:P1}\n", 
//			                                                    e.name, e.VSF, e.throttle, e.best_limit)));//debug
			//treat single-engine crafts specially
			if(num_engines == 1) 
			{
				engines[0].limit = optimized? engines[0].best_limit : 0f;
				max_limit = engines[0].limit;
			}
			else //restore the best state
			{
				max_limit = 0;
				for(int j = 0; j < num_engines; j++) 
				{ 
					var e = engines[j]; 
					e.limit = e.best_limit; 
					if(e.limit > max_limit)
						max_limit = e.limit;
				}
			}
			return optimized;
		}

		public bool OptimizeLimitsForTorque(IList<EngineWrapper> engines, Vector3 needed_torque, bool useDefTorque)
        { float max_limit; return OptimizeLimitsForTorque(engines, needed_torque, useDefTorque, out max_limit); }

        public bool OptimizeLimitsForTorque(IList<EngineWrapper> engines, Vector3 needed_torque, bool useDefTorque, out float max_limit)
		{
			float torque_error, angle_error;
            var ret = OptimizeLimitsForTorque(engines, needed_torque, VSL.Torque.Imbalance.Torque, VSL.Physics.MoI, useDefTorque, 
			                                  out max_limit, out torque_error, out angle_error);
			TorqueError = torque_error;
			TorqueAngle = angle_error;
			return ret;
		}

		public void PresetLimitsForTranslation(IList<EngineWrapper> engines, Vector3 translation)
		{
			var num_engines = engines.Count;
			for(int i = 0; i < num_engines; i++)
			{
				var e = engines[i];
				e.limit_tmp = Vector3.Dot(e.thrustDirection, translation);
				e.limit = e.limit_tmp > 0? e.limit_tmp : 0;
			}
		}

		public void LimitInDirection(IList<EngineWrapper> engines, Vector3 dir)
		{
			var num_engines = engines.Count;
			for(int i = 0; i < num_engines; i++)
			{
				var e = engines[i];
				e.limit_tmp = Vector3.Dot(e.thrustDirection, dir);
				e.limit *= 1-Mathf.Abs(e.limit_tmp);
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
			Steering = VSL.Controls.Steering*Mathf.Lerp(Utils.ClampH(VSL.Torque.MaxAAMod, 1), 1, VSL.Controls.InvAlignmentFactor);
			if(Steering.sqrMagnitude >= Globals.Instance.InputDeadZone)
			{
				//correct steering
				if(!CFG.AutoTune) Steering *= CFG.SteeringGain;
				Steering.Scale(CFG.SteeringModifier);
				//calculate needed torque
				for(int i = 0; i < num_engines; i++)
				{
					var e = engines[i];
					if(Vector3.Dot(e.currentTorque, Steering) > 0)
						needed_torque += e.currentTorque;
				}
				needed_torque = Vector3.Project(needed_torque, Steering) * Steering.magnitude;
				needed_torque = VSL.Torque.EnginesLimits.Clamp(needed_torque);
			}
			//optimize engines; if failed, set the flag and kill torque if requested
			float max_limit;
			if(!OptimizeLimitsForTorque(engines, needed_torque, false, out max_limit) && !needed_torque.IsZero())
			{
				for(int j = 0; j < num_engines; j++) engines[j].InitLimits();
				OptimizeLimitsForTorque(engines, Vector3.zero, false, out max_limit);
				SetState(TCAState.Unoptimized);
			}
			if(VSL.Engines.HaveMainEngines && 
               max_limit < VSL.PostUpdateControls.mainThrottle*0.05f && 
			   !engines.Any(e => e.preset_limit >= 0)) 
				Status(0.1, "red", "Thrust is disabled because engines cannot be balanced.");
		}

		void tune_steering_params()
		{
			//tune steering modifiers
			if(CFG.AT) CFG.SteeringModifier = Vector3.one;
			else
			{
                CFG.SteeringModifier.x = Mathf.Clamp01(ENG.SteeringCurve.Evaluate(VSL.Torque.MaxCurrent.AA.x)/100f);
                CFG.SteeringModifier.y = Mathf.Clamp01(ENG.SteeringCurve.Evaluate(VSL.Torque.MaxCurrent.AA.y)/100f);
                CFG.SteeringModifier.z = Mathf.Clamp01(ENG.SteeringCurve.Evaluate(VSL.Torque.MaxCurrent.AA.z)/100f);
			}
			//tune PI coefficients
            CFG.Engines.P = ENG.EnginesCurve.Evaluate(VSL.Torque.Engines.AA_rad);
    		CFG.Engines.I = CFG.Engines.P/2f;
		}

		#if DEBUG
//		public override void Init() 
//		{ 
//			base.Init(); 
//			RenderingManager.AddToPostDrawQueue(1, RadarBeam);
//		}
//
//		public void RadarBeam()
//		{
//			if(VSL == null || VSL.vessel == null) return;
//			for(int i = 0; i < VSL.Engines.NumActive; i++)
//			{
//				var e = VSL.Engines.Active[i];
//				if(e.thrustInfo == null) continue;
//				Utils.GLVec(e.wThrustPos, e.wThrustDir * 0.5f, Color.yellow);
//			}
//		}
//
//		public override void Reset()
//		{
//			base.Reset();
//			RenderingManager.RemoveFromPostDrawQueue(1, RadarBeam);
//		}

		void DebugEngines(IList<EngineWrapper> engines, Vector3 needed_torque)
		{
			Utils.Log("Engines:\n{}",
			          engines.Aggregate("", (s, e) => s 
		                +string.Format("engine(vec{0}, vec{1}, vec{2}, {3}, {4}),\n",
						VSL.LocalDir(e.wThrustPos-VSL.Physics.wCoM),
                                       e.thrustDirection,e.specificTorque, e.nominalCurrentThrust(0), e.nominalFullThrust)));
			Utils.Log("Engines Torque:\n{}", engines.Aggregate("", (s, e) => s + "vec"+e.Torque(e.throttle*e.limit)+",\n"));
			Utils.Log(
				"Steering: {}\n" +
				"Needed Torque: {}\n" +
				"Torque Imbalance: {}\n" +
				"Torque Error: {}kNm, {}°\n" +
				"Torque Clamp:\n   +{}\n   -{}\n" +
				"Limits: [{}]", 
				Steering,
				needed_torque,
				engines.Aggregate(Vector3.zero, (v,e) => v+e.Torque(e.throttle*e.limit)),
				TorqueError, TorqueAngle,
				VSL.Torque.EnginesLimits.positive, 
				VSL.Torque.EnginesLimits.negative,
				engines.Aggregate("", (s, e) => s+e.limit+" ").Trim()
			);
		}
		#endif
	}
}

