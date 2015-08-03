//------------------------------------------------------------------------------
// This EngineWrapper class came from HoneyFox's EngineIgnitor mod:
// Edited by Willem van Vliet 2014, by Allis Tauri 2015
// More info: https://github.com/HoneyFox/EngineIgnitor
//------------------------------------------------------------------------------

using System;
using UnityEngine;

namespace ThrottleControlledAvionics
{
	public abstract class ThrusterWrapper
	{
		public Vector3 specificTorque   = Vector3.zero;
		public Vector3 currentTorque    = Vector3.zero;
		public Vector3 thrustDirection  = Vector3.zero;
		public float   currentTorque_m;
		public float   torqueRatio;
		public float   limit, best_limit, limit_tmp;
		public float   thrustMod;
		protected float zeroISP;

		public abstract Vessel vessel { get; }
		public abstract Part part { get; }

		public bool Valid { get { return part != null && vessel != null; } }
		public abstract bool isOperational { get; }

		public abstract float finalThrust { get; }
		public abstract float thrustLimit { get; set; }

		public abstract Vector3 wThrustPos { get; }
		public abstract Vector3 wThrustDir { get; }

		public abstract Vector3 Torque(float throttle);

		public abstract void InitLimits();
		public abstract void InitState();
	}

	public class RCSWrapper : ThrusterWrapper
	{
		public readonly ModuleRCS rcs;
		public readonly float nominalThrusterPower;

		Vector3 avg_thrust_dir;
		Vector3 avg_thrust_pos;
		float current_thrust;
		float current_max_thrust;

		public RCSWrapper(ModuleRCS rcs)
		{
			zeroISP = rcs.atmosphereCurve.Evaluate(0f);
			nominalThrusterPower = rcs.thrusterPower;
			this.rcs = rcs;
		}

		public override void InitLimits()
		{ limit = best_limit = limit_tmp = 1f; }

		public override void InitState()
		{
			thrustMod = rcs.atmosphereCurve.Evaluate((float)(rcs.vessel.staticPressurekPa * PhysicsGlobals.KpaToAtmospheres))/zeroISP;
			var total_sthrust = 0f;
			avg_thrust_dir = Vector3.zero;
			avg_thrust_pos = Vector3.zero;
			for(int i = 0; i < rcs.thrusterTransforms.Count; i++)
			{
				var sthrust = rcs.thrustForces[i];
				var T = rcs.thrusterTransforms[i];
				avg_thrust_dir += T.up*sthrust;
				avg_thrust_pos += T.position*sthrust;
				total_sthrust += sthrust;
			}
			var current_sthrust = avg_thrust_dir.magnitude;
			avg_thrust_pos /= total_sthrust;
			avg_thrust_dir.Normalize();
			current_max_thrust = current_sthrust*nominalThrusterPower*thrustMod;
			current_thrust = current_sthrust*rcs.thrusterPower*thrustMod;
			InitLimits();
		}

		public override Vessel vessel { get { return rcs.vessel; } }
		public override Part part { get { return rcs.part; } }

		public override Vector3 wThrustDir { get { return avg_thrust_dir; } }
		public override Vector3 wThrustPos { get { return avg_thrust_pos; } }

		public float maxThrust { get { return current_max_thrust; } }
		public override float finalThrust { get { return current_thrust; } }

		public override Vector3 Torque (float throttle)
		{ return specificTorque*current_max_thrust*throttle; }

		public override float thrustLimit
		{
			get { return rcs.thrusterPower/nominalThrusterPower; }
			set { rcs.thrusterPower = Mathf.Clamp(Utils.WAverage(rcs.thrusterPower, value*nominalThrusterPower), 0, nominalThrusterPower); }
		}

		public override bool isOperational 
		{ get { return rcs.rcsEnabled && rcs.thrusterTransforms.Count > 0 && rcs.thrusterTransforms.Count == rcs.thrustForces.Count; } }
	}

	public class EngineWrapper : ThrusterWrapper
	{
		public static readonly PI_Controller ThrustPI = new PI_Controller();
		protected PIf_Controller thrustController = new PIf_Controller();

		public readonly ModuleEngines engine;
		readonly TCAEngineInfo einfo;
		public string name { get; private set; }

		public float   throttle;
		public float   VSF;   //vertical speed factor
		public bool    isVSC; //vertical speed controller
		public TCARole Role;
		public int     Group;
		public CenterOfThrustQuery thrustInfo;

		public EngineWrapper(ModuleEngines engine) 
		{
			thrustController.setMaster(ThrustPI);
			einfo = engine.part.GetModule<TCAEngineInfo>();
			zeroISP = engine.atmosphereCurve.Evaluate(0f);
			name = Utils.ParseCamelCase(engine.part.name);
			if(engine.engineID.Length > 0 && engine.engineID != "Engine") 
				name += " (" + engine.engineID + ")";
			this.engine = engine;
		}

		#region methods
		public override void InitLimits()
		{
			isVSC = false;
			switch(Role)
			{
			case TCARole.MAIN:
			case TCARole.BALANCE:
				limit = best_limit = 1f;
				isVSC = true;
				break;
			case TCARole.MANEUVER:
				limit = best_limit = 0f;
				break;
			case TCARole.MANUAL:
				limit = best_limit = thrustLimit/100;
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
			thrustMod = engine.atmosphereCurve.Evaluate((float)(engine.vessel.staticPressurekPa * PhysicsGlobals.KpaToAtmospheres))/zeroISP;
			if(engine.atmChangeFlow)
			{
				thrustMod = (float)(engine.part.atmDensity / 1.225);
				if(engine.useAtmCurve)
					thrustMod = engine.atmCurve.Evaluate(thrustMod);
			}
			if(engine.useVelCurve)
				thrustMod *= engine.velCurve.Evaluate((float)engine.part.machNumber);
			//update Role
			if(einfo != null)
			{
				if(engine.throttleLocked && einfo.Role != TCARole.MANUAL) 
					einfo.SetRole(TCARole.MANUAL);
				Role = einfo.Role;
				Group = einfo.Group;
			} else Role = TCARole.MAIN;
			InitLimits();
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
				thrustController.Update(value*100-engine.thrustPercentage);
				Utils.Log("Old thrust limit {0}; new value {1}; action {2}, P {3}, I {4}", 
				          engine.thrustPercentage, value, thrustController.Action, thrustController.P, thrustController.I);//debug
				engine.thrustPercentage = Mathf.Clamp(engine.thrustPercentage+thrustController.Action, 0, 100);
				Utils.Log("New thrust limit: {0}", engine.thrustPercentage);//debug
			}
		}
		public void forceThrustPercentage(float value) { engine.thrustPercentage = Mathf.Clamp(value, 0, 100); }

		public override bool isOperational { get { return engine.isOperational; } }
		#endregion
	}
}
