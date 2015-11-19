//   TorqueOptimizer.cs
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

			[Persistent] public int   MaxIterations            = 50;    //maximum number of optimizations per fixed frame
			[Persistent] public float OptimizationPrecision    = 0.01f;  //optimize engines limits until torque error or delta torque error is less than this
			[Persistent] public float OptimizationAngleCutoff  = 45f;   //maximum angle between torque imbalance and torque demand that is considered optimized
			[Persistent] public float OptimizationTorqueCutoff = 1f;    //maximum torque delta between imbalance and demand that is considered optimized
			[Persistent] public float TorqueRatioFactor        = 0.1f;  //torque-ratio curve
		}

		public float TorqueError { get; protected set; }
		public float TorqueAngle { get; protected set; }
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
		static Config ENG { get { return TCAScenario.Globals.ENG; } }

		public EngineOptimizer(VesselWrapper vsl) { VSL = vsl; }

		Vector3 Steering;

		static bool optimization_for_torque_pass(IList<EngineWrapper> engines, int num_engines, Vector3 target, float target_m, float eps)
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

		public bool OptimizeLimitsForTorque(IList<EngineWrapper> engines, Vector3 needed_torque)
		{
			var num_engines = engines.Count;
			var zero_torque = needed_torque.IsZero();
			TorqueAngle = TorqueError = -1f;
			float error, angle;
			var last_error = -1f;
			Vector3 cur_imbalance = VSL.Torque, target;
			for(int i = 0; i < ENG.MaxIterations; i++)
			{
				//calculate current errors and target
				cur_imbalance = VSL.Torque;
				for(int j = 0; j < num_engines; j++) 
				{ var e = engines[j]; cur_imbalance += e.Torque(e.throttle * e.limit); }
				angle  = zero_torque? 0f : Vector3.Angle(cur_imbalance, needed_torque);
				target = needed_torque-cur_imbalance;
				error  = VSL.AngularAcceleration(target).magnitude;
				//remember the best state
				if(angle <= 0f && error < TorqueError || angle+error < TorqueAngle+TorqueError || TorqueAngle < 0) 
				{ 
					for(int j = 0; j < num_engines; j++) 
					{ var e = engines[j]; e.best_limit = e.limit; }
					TorqueAngle = angle;
					TorqueError = error;
				}
				//check convergence conditions
				if(error < ENG.OptimizationTorqueCutoff*ENG.OptimizationPrecision || 
				   last_error > 0 && Mathf.Abs(error-last_error) < ENG.OptimizationPrecision*last_error)
					break;
				last_error = error;
				//optimize limits
				if(!optimization_for_torque_pass(engines, num_engines, target, error, ENG.OptimizationPrecision)) 
					break;
			}
			var optimized = TorqueError < ENG.OptimizationTorqueCutoff || 
				(!zero_torque && TorqueAngle < ENG.OptimizationAngleCutoff);
//			Log("optimized {0}, TorqueError {1}, TorqueAngle {2}\nneeded torque {3}\ncurrent turque {4}", 
//			    optimized, TorqueError, TorqueAngle, needed_torque, cur_imbalance);//debug
			//treat single-engine crafts specially
			if(num_engines == 1) 
				engines[0].limit = optimized? 1f : 0f;
			else //restore the best state
				for(int j = 0; j < num_engines; j++) 
				{ var e = engines[j]; e.limit = e.best_limit; }
			return optimized;
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
				e.limit *= e.limit_tmp > 0? 1-e.limit_tmp : 1+e.limit_tmp;
			}
		}

		public void Steer()
		{
			var num_engines = VSL.SteeringEngines.Count;
			if(num_engines == 0) return;
			if(num_engines == 1 && 
			   (VSL.Torque.IsZero() || 
			    VSL.AngularAcceleration(VSL.Torque).magnitude < ENG.OptimizationTorqueCutoff))
			{ VSL.SteeringEngines[0].limit = 1; return; }
			//calculate steering
			if(CFG.AutoTune) tune_steering_params();
			var needed_torque = Vector3.zero;
			Steering = VSL.Steering;
			if(Steering.sqrMagnitude >= TCAScenario.Globals.InputDeadZone)
			{
				//correct steering
				if(!CFG.AutoTune) Steering *= CFG.SteeringGain;
				Steering.Scale(CFG.SteeringModifier);
				//calculate needed torque
				for(int i = 0; i < num_engines; i++)
				{
					var e = VSL.SteeringEngines[i];
					if(Vector3.Dot(e.currentTorque, Steering) > 0)
						needed_torque += e.currentTorque;
				}
				needed_torque = Vector3.Project(needed_torque, Steering) * Steering.magnitude;
				needed_torque = VSL.E_TorqueLimits.Clamp(needed_torque);
			}
			//optimize engines; if failed, set the flag and kill torque if requested
			if(!OptimizeLimitsForTorque(VSL.SteeringEngines, needed_torque) && !needed_torque.IsZero())
			{
				for(int j = 0; j < num_engines; j++) VSL.SteeringEngines[j].InitLimits();
				OptimizeLimitsForTorque(VSL.SteeringEngines, Vector3.zero);
				SetState(TCAState.Unoptimized);
			}
		}

		void tune_steering_params()
		{
			//tune steering modifiers
			if(CFG.AT) CFG.SteeringModifier = Vector3.one;
			else
			{
				CFG.SteeringModifier.x = Mathf.Clamp01(ENG.SteeringCurve.Evaluate(VSL.MaxAngularA.x)/100f);
				CFG.SteeringModifier.y = Mathf.Clamp01(ENG.SteeringCurve.Evaluate(VSL.MaxAngularA.y)/100f);
				CFG.SteeringModifier.z = Mathf.Clamp01(ENG.SteeringCurve.Evaluate(VSL.MaxAngularA.z)/100f);
			}
			//tune PI coefficients
			CFG.Engines.P = ENG.EnginesCurve.Evaluate(VSL.MaxAngularA_m);
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
//			for(int i = 0, VSLEnginesCount = VSL.Engines.Count; i < VSLEnginesCount; i++)
//			{
//				var e = VSL.Engines[i];
//				if(e.thrustInfo == null) continue;
//				GLUtils.GLVec(e.wThrustPos, e.wThrustDir * 0.5f, Color.yellow);
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
			Utils.Log("Engines:\n"+
			          engines.Aggregate("", (s, e) => s 
		                +string.Format("engine(vec{0}, vec{1}, vec{2}, {3}, {4}),\n",
						VSL.refT.InverseTransformDirection(e.wThrustPos-VSL.wCoM),
                       	e.thrustDirection,e.specificTorque, e.nominalCurrentThrust(0), e.nominalCurrentThrust(1))));
			Utils.Log("Engines Torque:\n"+engines.Aggregate("", (s, e) => s + "vec"+e.Torque(e.throttle*e.limit)+",\n"));
			Utils.Log(
				"Steering: {0}\n" +
				"Needed Torque: {1}\n" +
				"Torque Imbalance: {2}\n" +
				"Torque Error: {3}kNm, {4}°\n" +
				"Torque Clamp:\n   +{5}\n   -{6}\n" +
				"Limits: [{7}]", 
				Steering,
				needed_torque,
				engines.Aggregate(Vector3.zero, (v,e) => v+e.Torque(e.throttle*e.limit)),
				TorqueError, TorqueAngle,
				VSL.E_TorqueLimits.positive, 
				VSL.E_TorqueLimits.negative,
				engines.Aggregate("", (s, e) => s+e.limit+" ").Trim()
			);
		}
		#endif
	}
}

