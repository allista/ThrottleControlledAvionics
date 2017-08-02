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
using System.Collections.Generic;
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
		public float   preset_limit = -1;
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

        public abstract float ThrustM(float throttle);
		
        public Vector3 Thrust(float throttle)
        { return thrustDirection*ThrustM(throttle); }

		public Vector3 Torque(float throttle)
        { return specificTorque*ThrustM(throttle); }

		public abstract void InitLimits();
		public abstract void InitState();
		public abstract void UpdateCurrentTorque(float throttle);

		//needed for autopilots that are executed before FixedUpdate
		public void ApplyPreset() { if(preset_limit >= 0) limit = best_limit = limit_tmp = preset_limit; }

		public void InitTorque(VesselWrapper VSL, float ratio_factor)
		{ InitTorque(VSL.refT, VSL.Physics.wCoM, ratio_factor); }

		public virtual void InitTorque(Transform vesselTransform, Vector3 CoM, float ratio_factor)
		{
			wThrustLever = wThrustPos-CoM;
			thrustDirection = vesselTransform.InverseTransformDirection(wThrustDir);
			specificTorque = vesselTransform.InverseTransformDirection(Vector3.Cross(wThrustLever, wThrustDir));
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

		public override void UpdateCurrentTorque(float throttle)
		{
			currentTorque = Torque(throttle);
			currentTorque_m = currentTorque.magnitude;
		}

		public override Vessel vessel { get { return rcs.vessel; } }
		public override Part part { get { return rcs.part; } }

		public override Vector3 wThrustDir { get { return avg_thrust_dir; } }
		public override Vector3 wThrustPos { get { return avg_thrust_pos; } }

		public float currentMaxThrust { get { return current_max_thrust; } }
		public override float finalThrust { get { return current_thrust; } }
		public float maxThrust { get { return rcs.thrusterPower*thrustMod; } }

        public override float ThrustM(float throttle)
        { return current_max_thrust*throttle; }

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

		Vector3 act_thrust_dir;
		Vector3 act_thrust_pos;
        public override Vector3 wThrustDir { get { return act_thrust_dir; } }
		public override Vector3 wThrustPos { get { return act_thrust_pos; } }

        public Vector3 defThrustDir { get; private set; }
        public Vector3 defThrustDirL { get; private set; }
        public Vector3 defSpecificTorque { get; private set; }
        public Vector3 defCurrentTorque { get; private set; }
        public float defCurrentTorque_m { get; private set; }
        public float defTorqueRatio { get; private set; }

        public float nominalFullThrust { get; private set; }

		class GimbalInfo
		{
			public Transform transform;
			public Quaternion iniRot;
			public Vector3 localThrustDir;

			public GimbalInfo(Transform thrustTransform, Transform gimbalTransform, Quaternion initialRotation)
			{
				transform = gimbalTransform;
				iniRot = initialRotation;
				localThrustDir = thrustTransform == gimbalTransform? Vector3.forward :
					gimbalTransform.InverseTransformDirection(thrustTransform.forward);
			}

			public Vector3 defaultThrustDir()
			{ return transform.TransformDirection(transform.localRotation.Inverse() * iniRot * localThrustDir); }
		}
		List<GimbalInfo> gimbals;

		public EngineWrapper(ModuleEngines engine) 
		{
			//generate engine ID
			this.engine = engine;
			name = Utils.ParseCamelCase(engine.part.Title());
			if(engine.engineID.Length > 0 && engine.engineID != "Engine") 
				name += " (" + engine.engineID + ")";
			ID = new EngineID(this);
			//init
			thrustController.setMaster(ThrustPI);
			zeroIsp = GetIsp(0,0,0);
			//get info
			info = engine.part.Modules.GetModule<TCAEngineInfo>();
			//find gimbal
			gimbal = engine.part.Modules.GetModule<ModuleGimbal>();
			gimbals = new List<GimbalInfo>(engine.thrustTransforms.Count);
			if(gimbal != null)
			{
				for(int i = 0, eCount = engine.thrustTransforms.Count; i < eCount; i++)
				{
					var eT = engine.thrustTransforms[i];
					for(int j = 0, gCount = gimbal.gimbalTransforms.Count; j < gCount; j++)
					{
						var gT = gimbal.gimbalTransforms[j];
						if(Part.FindTransformInChildrenExplicit(gT, eT))
						{
							gimbals.Add(new GimbalInfo(eT, gT, gimbal.initRots[j]));
							break;
						}
					}
					if(gimbals.Count == i) gimbals.Add(null);
				}
			}
		}

		#region methods
		public void SetRole(TCARole role) { info.SetRole(role); }
		public void SetGroup(int group) { info.group = group; }

        public override void InitTorque(Transform vesselTransform, Vector3 CoM, float ratio_factor)
        {
            base.InitTorque(vesselTransform, CoM, ratio_factor);
            if(gimbal != null)
            {
                defThrustDirL = vesselTransform.InverseTransformDirection(defThrustDir);
                defSpecificTorque = vesselTransform.InverseTransformDirection(Vector3.Cross(wThrustLever, defThrustDir));
                defTorqueRatio = Mathf.Pow(Mathf.Clamp01(1-Mathf.Abs(Vector3.Dot(wThrustLever.normalized, defThrustDir))), ratio_factor);
            }
            else
            {
                defThrustDirL = thrustDirection;
                defSpecificTorque = specificTorque;
                defTorqueRatio = torqueRatio;
            }
        }

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

		public void UpdateThrustInfo()
		{
			defThrustDir = Vector3.zero;
			act_thrust_dir = Vector3.zero;
			act_thrust_pos = Vector3.zero;
			int count = engine.thrustTransforms.Count;
			for(int i = 0; i < count; i++)
			{
				var eT = engine.thrustTransforms[i];
				var mult = engine.thrustTransformMultipliers[i];
				var act_dir = eT.forward * mult;
				if(gimbal != null)
				{
					var gi = gimbals[i];
					if(gi != null) defThrustDir += gi.defaultThrustDir() * mult;
					else defThrustDir += act_dir;
				}
				act_thrust_dir += act_dir;
				act_thrust_pos += eT.position * mult;
			}
			if(gimbal == null) defThrustDir = act_thrust_dir;
		}

		public override void InitState()
		{
			if(engine == null || part == null || vessel == null) return;
			//update thrust info
			UpdateThrustInfo();
			realIsp = GetIsp((float)(part.staticPressureAtm), (float)(part.atmDensity/1.225), (float)part.machNumber);
			flowMod = GetFlowMod((float)(part.atmDensity/1.225), (float)part.machNumber);
			thrustMod = realIsp*flowMod/zeroIsp;
			//update Role
			if(engine.throttleLocked && info.Role != TCARole.MANUAL) 
				info.SetRole(TCARole.MANUAL);
			InitLimits();
            nominalFullThrust = nominalCurrentThrust(1);
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

		public override void UpdateCurrentTorque(float throttle)
		{
			this.throttle = throttle;
            var thrust = ThrustM(throttle);
            currentTorque = specificTorque*thrust;
			currentTorque_m = currentTorque.magnitude;
            if(gimbal != null)
            {
                defCurrentTorque = defSpecificTorque*thrust;
                defCurrentTorque_m = defCurrentTorque.magnitude;
            }
            else
            {
                defCurrentTorque = currentTorque;
                defCurrentTorque_m = currentTorque_m;
            }
		}

        public override float ThrustM(float throttle)
        {
            return (Role != TCARole.MANUAL? 
                    nominalCurrentThrust(throttle) :
                    engine.finalThrust);
        }

        public Vector3 Thrust(float throttle, bool useDefDirection)
        { 
            return useDefDirection?
                defThrustDirL * ThrustM(throttle) :
                Thrust(throttle);
        }

        public Vector3 Torque(float throttle, bool useDefDirection)
        { 
            return useDefDirection?
                defSpecificTorque * ThrustM(throttle) :
                Torque(throttle);
        }

        public Vector3 getSpecificTorque(bool useDefDirection)
        { return useDefDirection? defSpecificTorque : specificTorque; }

        public Vector3 getCurrentTorque(bool useDefDirection)
        { return useDefDirection? defCurrentTorque : currentTorque; }

        public float getCurrentTorqueM(bool useDefDirection)
        { return useDefDirection? defCurrentTorque_m : currentTorque_m; }

        public float getTorqueRatio(bool useDefDirection)
        { return useDefDirection? defTorqueRatio : torqueRatio; }

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
			mFlow = engine.maxFuelFlow;
			var atm = vessel.mainBody.AtmoParamsAtAltitude(alt);
			var rel_density = (float)(atm.Rho/1.225);
			var vel_mach = atm.Mach1 > 0? (float)(vel/atm.Mach1) : 0;
			var Ve = GetIsp((float)(atm.P/1013.25), rel_density, vel_mach) * Utils.G0;
			mFlow *= GetFlowMod(rel_density, vel_mach);
			return mFlow * Ve;
		}

		public float MaxFuelFlow { get { return engine.maxFuelFlow*flowMod; } }
		public float RealFuelFlow { get { return engine.requestedMassFlow*engine.propellantReqMet/100; } }
		#endregion

		#region Accessors
		public override Vessel vessel { get { return engine.vessel; } }
		public override Part part { get { return engine.part; } }

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
		{ 
            if(!engine.throttleLocked) 
                engine.thrustPercentage = Mathf.Clamp(value, 0, 100); 
//            if(Role == TCARole.MANUAL && value.Equals(0))//debug
//                Utils.Log("Manual engine was dethrottled.");
        }

		public override bool isOperational { get { return engine.isOperational; } }
		#endregion

		public override string ToString()
		{
			return Utils.Format("[{}, ID {}, flightID {}, Stage {}, Role {}, isVSC {}, Group {}, flameout {}]\n" + 
			                    "useEngineResponseTime: {}, engineAccelerationSpeed={}, engineDecelerationSpeed={}\n" + 
			                    "finalThrust: {}, thrustLimit: {}, isOperational: {}\n" +
			                    "limit: {}, best_limit: {}, limit_tmp: {}, preset: {}\n" +
			                    "thrust: {}\ndir {}\ndef dir: {}\npos {}\nlever: {}\ntorque: {}\ntorqueRatio: {}\n", 
			                    name, ID, flightID, part.inverseStage, Role, isVSC, Group, engine.flameout,
			                    useEngineResponseTime, engineAccelerationSpeed, engineDecelerationSpeed, 
			                    finalThrust, thrustLimit, isOperational, 
			                    limit, best_limit, limit_tmp, preset_limit,
                                nominalFullThrust, 
			                    wThrustDir, defThrustDir, wThrustPos, wThrustLever, 
			                    currentTorque, torqueRatio
			                   );
		}
	}
}
