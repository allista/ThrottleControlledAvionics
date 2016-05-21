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
		protected float zeroISP;

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
			zeroISP = rcs.atmosphereCurve.Evaluate(0f);
			this.rcs = rcs;
		}

		public override void InitLimits()
		{ limit = best_limit = limit_tmp = 1f; }

		public override void InitState()
		{
			thrustMod = rcs.atmosphereCurve.Evaluate((float)(rcs.part.staticPressureAtm))/zeroISP;
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
		{ get { return rcs.rcsEnabled && rcs.thrusterTransforms.Count > 0; } }
	}

	public class EngineID
	{
		uint id;

		public EngineID(EngineWrapper e) //FIXME: generates Number overflow on flight scene load
		{
			if(e.part == null || e.engine == null) return;
			var rT  = e.part.localRoot == null? e.part.transform : e.part.localRoot.transform;
			var to  = rT.InverseTransformPoint(e.part.partTransform.position);
			var td  = rT.InverseTransformDirection(e.part.partTransform.forward);
			var ids = string.Format("{0} {1} {2:F1} {3:F1} {4:F1} {5:F1} {6:F1} {7:F1}",
			                        e.part.partInfo.name, e.engine.engineID, to.x, to.y, to.z, td.x, td.y, td.z);
			id = (uint)ids.GetHashCode();
//			Utils.Log("{0}: {1}", ids, id);//debug
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
		public float   VSF;   //vertical speed factor
		public bool    isVSC; //vertical speed controller
		public TCARole Role { get { return info.Role; } }
		public int     Group { get { return info.Group; } }
		public CenterOfThrustQuery thrustInfo;

		public EngineWrapper(ModuleEngines engine) 
		{
			//init
			thrustController.setMaster(ThrustPI);
			zeroISP = engine.atmosphereCurve.Evaluate(0f);
			name = Utils.ParseCamelCase(engine.part.Title());
			if(engine.engineID.Length > 0 && engine.engineID != "Engine") 
				name += " (" + engine.engineID + ")";
			//generate engine ID
			this.engine = engine;
			ID = new EngineID(this);
			//get info
			info = engine.part.GetModule<TCAEngineInfo>();
			//find gimbal
			gimbal = engine.part.GetModule<ModuleGimbal>();
		}

		#region methods
		public void SetRole(TCARole role) { info.SetRole(role); }

		public override void InitLimits()
		{
			isVSC = false;
			switch(Role)
			{
			case TCARole.MAIN:
			case TCARole.BALANCE:
			case TCARole.UNBALANCE:
				limit = best_limit = 1f;
				isVSC = true;
				break;
			case TCARole.MANEUVER:
				limit = best_limit = 0f;
				break;
			case TCARole.MANUAL:
				limit = best_limit = thrustLimit;
				break;
			}
		}

		public override void InitState()
		{
			//update thrust info
			thrustInfo = new CenterOfThrustQuery();
			engine.OnCenterOfThrustQuery(thrustInfo);
			thrustInfo.dir.Normalize();
			//compute velocity and atmosphere thrust modifier
			thrustMod = engine.atmosphereCurve.Evaluate((float)(engine.part.staticPressureAtm))/zeroISP;
			if(engine.atmChangeFlow)
			{
				thrustMod = (float)(engine.part.atmDensity / 1.225);
				if(engine.useAtmCurve)
					thrustMod = engine.atmCurve.Evaluate(thrustMod);
			}
			if(engine.useVelCurve)
				thrustMod *= engine.velCurve.Evaluate((float)engine.part.machNumber);
			//update Role
			if(engine.throttleLocked && info.Role != TCARole.MANUAL) 
				info.SetRole(TCARole.MANUAL);
			InitLimits();
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
