//  Author:
//       Allis Tauri <allista@gmail.com>
//
//  Copyright (c) 2015 Allis Tauri
//
// This work is licensed under the Creative Commons Attribution-ShareAlike 4.0 International License. 
// To view a copy of this license, visit http://creativecommons.org/licenses/by-sa/4.0/ 
// or send a letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.

// Original idea of EngineWrapper came from HoneyFox's EngineIgnitor mod: https://github.com/HoneyFox/EngineIgnitor

using System;
using UnityEngine;
using AT_Utils;

namespace ThrottleControlledAvionics
{
	public abstract class ThrusterWrapper
	{
		public Vector3 specificTorque   = Vector3.zero;
		public Vector3 currentTorque    = Vector3.zero;
		public Vector3 thrustDirection  = Vector3.zero;
		public Vector3 wThrustLever     = Vector3.zero;
		public float   currentTorque_m;
		public float   torqueRatio;
		public float   limit, best_limit, limit_tmp;
		public float   thrustMod;
		protected float zeroIsp;

		public abstract Vessel vessel { get; }
		public abstract Part part { get; }

		public bool Valid(VesselWrapper VSL) 
		{ return part != null && vessel != null && vessel.id == VSL.vessel.id; }

		public abstract bool isOperational { get; }

		public abstract float finalThrust { get; }
		public abstract float thrustLimit { get; set; }

		public abstract Vector3 wThrustPos { get; }
		public abstract Vector3 wThrustDir { get; }

		public abstract Vector3 Thrust(float throttle);
		public abstract Vector3 Torque(float throttle);

		public abstract void InitLimits();
		public abstract void InitState();

		public virtual  void InitTorque(VesselWrapper VSL, float ratio_factor)
		{
			wThrustLever = wThrustPos-VSL.Physics.wCoM;
			thrustDirection = VSL.refT.InverseTransformDirection(wThrustDir);
			specificTorque = VSL.refT.InverseTransformDirection(Vector3.Cross(wThrustLever, wThrustDir));
			torqueRatio = Mathf.Pow(Mathf.Clamp01(1-Mathf.Abs(Vector3.Dot(wThrustLever.normalized, wThrustDir))), ratio_factor);
		}
	}

	public class RCSWrapper : ThrusterWrapper
	{
		public readonly ModuleRCS rcs;

		Vector3 avg_thrust_dir;
		Vector3 avg_thrust_pos;
		float current_thrust;
		float current_max_thrust;

		public RCSWrapper(ModuleRCS rcs)
		{
			zeroIsp = rcs.atmosphereCurve.Evaluate(0f);
			this.rcs = rcs;
		}

		public override void InitLimits()
		{ limit = best_limit = limit_tmp = 1f; }

		public override void InitState()
		{
			thrustMod = rcs.atmosphereCurve.Evaluate((float)(rcs.part.staticPressureAtm))/zeroIsp;
			var total_sthrust = 0f;
			avg_thrust_dir = Vector3.zero;
			avg_thrust_pos = Vector3.zero;
			for(int i = 0, count = rcs.thrusterTransforms.Count; i < count; i++)
			{
				var sthrust = rcs.thrustForces[i];
				var T = rcs.thrusterTransforms[i];
				if(T == null) continue;
				avg_thrust_dir += (rcs.useZaxis? T.forward : T.up)*sthrust;
				avg_thrust_pos += T.position*sthrust;
				total_sthrust += sthrust;
			}
			var current_sthrust = avg_thrust_dir.magnitude;
			if(total_sthrust > 0) avg_thrust_pos /= total_sthrust;
			else avg_thrust_pos = rcs.transform.position;
			avg_thrust_dir.Normalize();
			current_max_thrust = current_sthrust*rcs.thrusterPower*thrustMod;
			current_thrust = current_sthrust*rcs.thrustPercentage*0.01f*rcs.thrusterPower*thrustMod;
			InitLimits();
		}

		public override Vessel vessel { get { return rcs.vessel; } }
		public override Part part { get { return rcs.part; } }

		public override Vector3 wThrustDir { get { return avg_thrust_dir; } }
		public override Vector3 wThrustPos { get { return avg_thrust_pos; } }

		public float currentMaxThrust { get { return current_max_thrust; } }
		public override float finalThrust { get { return current_thrust; } }
		public float maxThrust { get { return rcs.thrusterPower*thrustMod; } }

		public override Vector3 Thrust (float throttle)
		{ return thrustDirection*current_max_thrust*throttle; }

		public override Vector3 Torque (float throttle)
		{ return specificTorque*current_max_thrust*throttle; }

		public override float thrustLimit
		{
			get { return rcs.thrustPercentage*0.01f; }
			set { rcs.thrustPercentage = Mathf.Clamp(Utils.EWA(rcs.thrustPercentage, value*100), 0, 100); }
		}

		public override bool isOperational 
		{ get { return rcs.rcsEnabled && rcs.thrusterTransforms.Count > 0 && rcs.thrusterTransforms.Count == rcs.thrustForces.Length; } }
	}

	public class EngineID
	{
		uint id;

		//FIXME: generates Number overflow on flight scene load
		public EngineID(EngineWrapper e) 
		{
			if(e.part == null || e.engine == null) return;
			var rT  = e.part.localRoot == null? e.part.transform : e.part.localRoot.transform;
			var to  = rT.InverseTransformPoint(e.part.partTransform.position);
			var td  = rT.InverseTransformDirection(e.part.partTransform.forward);
			var ids = string.Format("{0} {1} {2:F1} {3:F1} {4:F1} {5:F1} {6:F1} {7:F1}",
			                        e.part.partInfo.name, e.engine.engineID, to.x, to.y, to.z, td.x, td.y, td.z);
			id = (uint)ids.GetHashCode();
//			Utils.Log("{}: {}", ids, id);//debug
		}

		public static implicit operator uint(EngineID eid) { return eid.id; }
	}

	public class EngineWrapper : ThrusterWrapper
	{
		public static readonly PI_Controller ThrustPI = new PI_Controller();
		protected PIf_Controller thrustController = new PIf_Controller();

		public readonly ModuleEngines engine;
		public readonly ModuleGimbal  gimbal;
		public readonly TCAEngineInfo info;
		public string name { get; private set; }

		public uint ID { get; private set; }
		public uint flightID { get { return part.flightID; } }

		public float   throttle;
		public float   realIsp;
		public float   flowMod;
		public float   VSF;   //vertical speed factor
		public bool    isVSC; //vertical speed controller
		public bool    isSteering;
		public TCARole Role { get { return info.Role; } }
		public int     Group { get { return info.Group; } }
		public CenterOfThrustQuery thrustInfo;

		public EngineWrapper(ModuleEngines engine) 
		{
			//init
			thrustController.setMaster(ThrustPI);
			zeroIsp = engine.atmosphereCurve.Evaluate(0f);
			name = Utils.ParseCamelCase(engine.part.Title());
			if(engine.engineID.Length > 0 && engine.engineID != "Engine") 
				name += " (" + engine.engineID + ")";
			//generate engine ID
			this.engine = engine;
			ID = new EngineID(this);
			//get info
			info = engine.part.Modules.GetModule<TCAEngineInfo>();
			//find gimbal
			gimbal = engine.part.Modules.GetModule<ModuleGimbal>();
		}

		#region methods
		public void SetRole(TCARole role) { info.SetRole(role); }

		public override void InitLimits()
		{
			isVSC = isSteering = false;
			switch(Role)
			{
			case TCARole.MAIN:
			case TCARole.BALANCE:
			case TCARole.UNBALANCE:
				limit = best_limit = 1f;
				isSteering = Role == TCARole.MAIN;
				isVSC = true;
				break;
			case TCARole.MANEUVER:
				limit = best_limit = 0f;
				isSteering = true;
				break;
			case TCARole.MANUAL:
				limit = best_limit = thrustLimit;
				break;
			}
		}

		public override void InitState()
		{
			if(engine == null || part == null || vessel == null) return;
			//update thrust info
			thrustInfo = new CenterOfThrustQuery();
			engine.OnCenterOfThrustQuery(thrustInfo);
			thrustInfo.dir.Normalize();
			realIsp = GetIsp((float)(part.staticPressureAtm), (float)(part.atmDensity/1.225), (float)part.machNumber);
			flowMod = GetFlowMod((float)(part.atmDensity/1.225), (float)part.machNumber);
			thrustMod = realIsp*flowMod/zeroIsp;
			//update Role
			if(engine.throttleLocked && info.Role != TCARole.MANUAL) 
				info.SetRole(TCARole.MANUAL);
			InitLimits();
//			Utils.Log("Engine.InitState: {}\n" +
//			          "wThrustDir {}\n" +
//			          "wThrustPos {}\n" +
//			          "zeroIsp {}, realIsp {}, flowMod {}, thrustMod {}\n" +
//			          "P {}, Rho {}, Mach {}, multIsp {}, multFlow {}\n" +
//			          "###############################################################",
//			          name, wThrustDir, wThrustPos, 
//			          zeroIsp, realIsp, flowMod, thrustMod,
//			          part.staticPressureAtm, part.atmDensity, part.machNumber, 
//			          engine.multIsp, engine.multFlow);//debug
		}

		public override Vector3 Thrust(float throttle)
		{
			return thrustDirection * 
				(Role != TCARole.MANUAL? 
				 nominalCurrentThrust(throttle) :
				 engine.finalThrust);
		}

		public override Vector3 Torque(float throttle)
		{
			return specificTorque * 
				(Role != TCARole.MANUAL? 
				 nominalCurrentThrust(throttle) :
				 engine.finalThrust);
		}


		float GetIsp(float pressureAtm, float rel_density, float vel_mach) 
		{
			var Isp = engine.atmosphereCurve.Evaluate(pressureAtm) * engine.multIsp;
			if(engine.useAtmCurveIsp)
				Isp *= engine.atmCurveIsp.Evaluate(rel_density);
			if(engine.useVelCurveIsp)
				Isp *= engine.velCurveIsp.Evaluate(vel_mach);	
			return Isp;
		}

		float GetFlowMod(float rel_density, float vel_mach)
		{
			var fmod = engine.multFlow;
			if(engine.atmChangeFlow)
			{
				fmod = rel_density;
				if(engine.useAtmCurve)
					fmod = engine.atmCurve.Evaluate(fmod);
			}
			if(engine.useVelCurve)
				fmod *= engine.velCurve.Evaluate(vel_mach);
			if(fmod > engine.flowMultCap)
			{
				float to_cap = fmod - engine.flowMultCap;
				fmod = engine.flowMultCap + to_cap / (engine.flowMultCapSharpness + to_cap / engine.flowMultCap);
			}
			//the "< 1" check is needed for TCA to work with SolverEngines, 
			//as it sets the CLAMP to float.MaxValue:
			//SolverEngines/SolverEngines/EngineModule.cs:
			//https://github.com/KSP-RO/SolverEngines/blob/eba89da74767fb66ea96661a5950fa52233e8822/SolverEngines/EngineModule.cs#L572
			if(fmod < engine.CLAMP && engine.CLAMP < 1) fmod = engine.CLAMP;
			return fmod;
		}

		public float ThrustAtAlt(float vel, float alt, out float mFlow) 
		{
			mFlow = 0;
			if(!vessel.mainBody.atmosphere) return zeroIsp;
			var atm = vessel.mainBody.AtmoParamsAtAltitude(alt);
			var rel_density = (float)(atm.Rho/1.225);
			var vel_mach = (float)(vel/atm.Mach1);
			var Ve = GetIsp((float)(atm.P/1013.25), rel_density, vel_mach) * Utils.G0;
			mFlow = engine.maxFuelFlow * GetFlowMod(rel_density, vel_mach);
			return mFlow * Ve;
		}

		public float MaxFuelFlow { get { return engine.maxFuelFlow*flowMod; } }
		public float RealFuelFlow { get { return engine.requestedMassFlow*engine.propellantReqMet/100; } }
		#endregion

		#region Accessors
		public override Vessel vessel { get { return engine.vessel; } }
		public override Part part { get { return engine.part; } }

		public override Vector3 wThrustDir { get { return thrustInfo.dir; } }
		public override Vector3 wThrustPos { get { return thrustInfo.pos; } }

		public bool  useEngineResponseTime { get { return engine.useEngineResponseTime; } }
		public float engineAccelerationSpeed { get { return engine.engineAccelerationSpeed; } }
		public float engineDecelerationSpeed { get { return engine.engineDecelerationSpeed; } }

		public override float finalThrust { get { return engine.finalThrust; } }

		public float nominalCurrentThrust(float throttle)
		{ 
			return thrustMod * (engine.throttleLocked ? 
				engine.maxThrust : Mathf.Lerp(engine.minThrust, engine.maxThrust, throttle)); 
		}

		public override float thrustLimit
		{
			get { return engine.thrustPercentage/100f; }
			set
			{
				if(engine.throttleLocked) return;
				thrustController.Update(value*100-engine.thrustPercentage);
				engine.thrustPercentage = Mathf.Clamp(engine.thrustPercentage+thrustController.Action, 0, 100);
			}
		}
		public void forceThrustPercentage(float value) 
		{ if(!engine.throttleLocked) engine.thrustPercentage = Mathf.Clamp(value, 0, 100); }

		public override bool isOperational { get { return engine.isOperational; } }
		#endregion

		public override string ToString()
		{
			return Utils.Format("[EngineWrapper: name={}, ID={}, flightID={}, Stage={}, Role={}, Group={},\n" + 
			                    "useEngineResponseTime={}, engineAccelerationSpeed={}, engineDecelerationSpeed={},\n" + 
			                    "finalThrust={}, thrustLimit={}, isOperational={}]", 
			                    name, ID, flightID, part.inverseStage, Role, Group, 
			                    useEngineResponseTime, engineAccelerationSpeed, engineDecelerationSpeed, 
			                    finalThrust, thrustLimit, isOperational);
		}
	}
}
