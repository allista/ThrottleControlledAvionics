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
		public bool isModuleEngineFX;
		readonly ModuleEngines engine;
		readonly ModuleEnginesFX engineFX;

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

		public static readonly PI_Dummy ThrustPI = new PI_Dummy();
		PIf_Controller thrustController = new PIf_Controller();
		public Vector3 specificTorque   = Vector3.zero;
		public Vector3 currentTorque    = Vector3.zero;
		public Vector3 thrustDirection  = Vector3.zero;
		public float   limit, best_limit, limit_tmp;

		protected EngineWrapper()
		{ thrustController.setMaster(ThrustPI);	}

		public EngineWrapper(ModuleEngines engine) : this()
		{
			isModuleEngineFX = false;
			this.engine = engine;
		}

		public EngineWrapper(ModuleEnginesFX engineFX) : this()
		{
			isModuleEngineFX = true;
			this.engineFX = engineFX;
		}

		public Vessel vessel
		{ get { return isModuleEngineFX ? engineFX.vessel : engine.vessel; } }

		public void SetRunningGroupsActive(bool active)
		{
			if(!isModuleEngineFX)
				engine.SetRunningGroupsActive(active);
			// Do not need to worry about ModuleEnginesFX.
		}

		public CenterOfThrustQuery thrustInfo
		{ 
			get 
			{ 
				var query = new CenterOfThrustQuery();
				if (isModuleEngineFX) engineFX.OnCenterOfThrustQuery(query);
				else engine.OnCenterOfThrustQuery(query);
				return query;
			}
		}

		public float nominalCurrentThrust(float throttle)
		{ return Mathf.Lerp(minThrust, maxThrust, throttle); }

		public float requestedThrust
		{ get { return isModuleEngineFX ? engineFX.requestedThrust : engine.requestedThrust; } }

		public float finalThrust
		{ get { return isModuleEngineFX ? engineFX.finalThrust : engine.finalThrust; } }

		public List<Propellant> propellants
		{ get { return isModuleEngineFX ? engineFX.propellants : engine.propellants; } }

		public Part part
		{ get { return isModuleEngineFX ? engineFX.part : engine.part; } }

		public BaseEventList Events
		{ get { return isModuleEngineFX ? engineFX.Events : engine.Events; } }

		public void BurstFlameoutGroups()
		{
			if(!isModuleEngineFX) engine.BurstFlameoutGroups();
			else engineFX.part.Effects.Event(engineFX.flameoutEffectName);
		}

		public bool allowShutdown
		{ get { return isModuleEngineFX ? engineFX.allowShutdown : engine.allowShutdown; } }

		public bool throttleLocked
		{ get { return isModuleEngineFX ? engineFX.throttleLocked : engine.throttleLocked; } }

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
	}
}
