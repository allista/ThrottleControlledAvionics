//------------------------------------------------------------------------------
// This EngineWrapper class came from HoneyFox's EngineIgnitor mod:
// Edited by Willem van Vliet 2014, by Allis Tauri 2015
// More info: https://github.com/HoneyFox/EngineIgnitor
//------------------------------------------------------------------------------

using System;
using System.Collections.Generic;
using UnityEngine;

namespace ThrottleControlledAvionics
{
	public class EngineWrapper
	{
		readonly ModuleEngines engine;
		readonly ModuleEnginesFX engineFX;
		readonly TCAEngineInfo einfo;

		public static readonly PI_Dummy ThrustPI = new PI_Dummy();
		PIf_Controller thrustController = new PIf_Controller();
		public Vector3 specificTorque   = Vector3.zero;
		public Vector3 currentTorque    = Vector3.zero;
		public Vector3 thrustDirection  = Vector3.zero;
		public float   limit, best_limit, limit_tmp;
		public float   currentTorque_m;
		public bool    isModuleEngineFX;
		public bool    throttleLocked;
		public float   thrustMod;
		public TCARole Role;
		public CenterOfThrustQuery thrustInfo;

		protected EngineWrapper(PartModule module)
		{ 
			thrustController.setMaster(ThrustPI);
			einfo = module.part.GetModule<TCAEngineInfo>();
		}

		public EngineWrapper(ModuleEngines engine) 
			: this((PartModule)engine)
		{
			isModuleEngineFX = false;
			this.engine = engine;
		}

		public EngineWrapper(ModuleEnginesFX engineFX) 
			: this((PartModule)engineFX)
		{
			isModuleEngineFX = true;
			this.engineFX = engineFX;
		}

		#region methods
		public void InitState()
		{
			//update thrust info
			thrustInfo = new CenterOfThrustQuery();
			if (isModuleEngineFX) engineFX.OnCenterOfThrustQuery(thrustInfo);
			else engine.OnCenterOfThrustQuery(thrustInfo);
			thrustInfo.dir.Normalize();
			//compute velocity thrust modifier
			if(isModuleEngineFX? engineFX.useVelocityCurve : engine.useVelocityCurve)
			{
				var vc = isModuleEngineFX? engineFX.velocityCurve : engine.velocityCurve;
				thrustMod = vc.Evaluate((float)FlightGlobals.ActiveVessel.srf_velocity.magnitude);
			} else thrustMod = 1f;
			//update Role
			throttleLocked = isModuleEngineFX ? engineFX.throttleLocked : engine.throttleLocked;
			if(einfo != null)
			{
				if(throttleLocked && einfo.Role != TCARole.MANUAL) 
					einfo.SetRole(TCARole.MANUAL);
				Role = einfo.Role;
			} else Role = TCARole.MAIN;
			switch(Role)
			{
			case TCARole.MAIN:
				limit = best_limit = 1f;
				break;
			case TCARole.MANEUVER:
				limit = best_limit = 0f;
				break;
			case TCARole.MANUAL:
				limit = best_limit = thrustPercentage/100;
				break;
			}
		}

		public Vector3 Torque(float throttle)
		{
			return specificTorque * 
				(Role != TCARole.MANUAL? 
				 nominalCurrentThrust(throttle) :
				 finalThrust);
		}

		/// <summary>
		/// If the wrapper is still points to the valid ModuleEngines(FX)
		/// </summary>
		public bool Valid 
		{ 
			get 
			{ 
				return isModuleEngineFX? 
					engineFX.part != null && engineFX.vessel != null :
					engine.part != null && engine.vessel != null;
			}
		}
		#endregion

		#region Accessors
		public Vessel vessel
		{ get { return isModuleEngineFX ? engineFX.vessel : engine.vessel; } }

		public Part part
		{ get { return isModuleEngineFX ? engineFX.part : engine.part; } }

		public float nominalCurrentThrust(float throttle)
		{ return thrustMod * (throttleLocked ? maxThrust : Mathf.Lerp(minThrust, maxThrust, throttle)); }

		public float requestedThrust
		{ get { return isModuleEngineFX ? engineFX.requestedThrust : engine.requestedThrust; } }

		public float finalThrust
		{ get { return isModuleEngineFX ? engineFX.finalThrust : engine.finalThrust; } }

		public List<Propellant> propellants
		{ get { return isModuleEngineFX ? engineFX.propellants : engine.propellants; } }

		public BaseEventList Events
		{ get { return isModuleEngineFX ? engineFX.Events : engine.Events; } }

		public void BurstFlameoutGroups()
		{
			if(!isModuleEngineFX) engine.BurstFlameoutGroups();
			else engineFX.part.Effects.Event(engineFX.flameoutEffectName);
		}

		public bool allowShutdown
		{ get { return isModuleEngineFX ? engineFX.allowShutdown : engine.allowShutdown; } }

		public bool useEngineResponseTime
		{ get { return isModuleEngineFX ? engineFX.useEngineResponseTime : engine.useEngineResponseTime; } }

		public float engineAccelerationSpeed
		{ get { return isModuleEngineFX ? engineFX.engineAccelerationSpeed : engine.engineAccelerationSpeed; } }

		public float engineDecelerationSpeed
		{ get { return isModuleEngineFX ? engineFX.engineDecelerationSpeed : engine.engineDecelerationSpeed; } }

		public float maxThrust
		{ get { return isModuleEngineFX ? engineFX.maxThrust : engine.maxThrust; } }

		public float minThrust
		{ get { return isModuleEngineFX ? engineFX.minThrust : engine.minThrust; } }

		public float thrustPercentage
		{
			get { return isModuleEngineFX ? engineFX.thrustPercentage : engine.thrustPercentage; }
			set
			{
				thrustController.Update(value-thrustPercentage);
				if(!isModuleEngineFX) engine.thrustPercentage = Mathf.Clamp(engine.thrustPercentage+thrustController, 0, 100);
				else engineFX.thrustPercentage = Mathf.Clamp(engineFX.thrustPercentage+thrustController, 0, 100);
			}
		}

		public void forceThrustPercentage(float value) 
		{
			if(!isModuleEngineFX) engine.thrustPercentage = value;
			else engineFX.thrustPercentage = value;
		}

		public bool isEnabled
		{ get { return isModuleEngineFX ? engineFX.EngineIgnited : engine.EngineIgnited; } }

		public String name
		{ get { return isModuleEngineFX ? engineFX.part.name : engine.part.name; } }

		public String getName()
		{
			String r = "";
			if(!isModuleEngineFX)
			{
				r += engine.part.name;
				if(engine.engineID.Length > 0)
					r += " (" + engine.engineID + ")";
			}
			else
			{
				r += engineFX.part.name;
				if(engineFX.engineID.Length > 0)
					r += " (" + engineFX.engineID + ")";
			}
			return r;
		}
		#endregion
	}
}
