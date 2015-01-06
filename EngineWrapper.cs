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

		public Vector3 currentTorque = Vector3.zero;
		public Vector3 thrustDirection = Vector3.zero;
		public float efficiency;

		public EngineWrapper(ModuleEngines engine)
		{
			isModuleEngineFX = false;
			this.engine = engine;
		}

		public EngineWrapper(ModuleEnginesFX engineFX)
		{
			isModuleEngineFX = true;
			this.engineFX = engineFX;
		}

		public Vessel vessel
		{ get { return !isModuleEngineFX ? engine.vessel : engineFX.vessel; } }

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

		public float requestedThrust
		{ get { return !isModuleEngineFX ? engine.requestedThrust : engineFX.requestedThrust; } }

		public float finalThrust
		{ get { return !isModuleEngineFX ? engine.finalThrust : engineFX.finalThrust; } }

		public List<Propellant> propellants
		{ get { return !isModuleEngineFX ? engine.propellants : engineFX.propellants; } }

		public Part part
		{ get { return !isModuleEngineFX ? engine.part : engineFX.part; } }

		public BaseEventList Events
		{ get { return !isModuleEngineFX ? engine.Events : engineFX.Events; } }

		public void BurstFlameoutGroups()
		{
			if(!isModuleEngineFX) engine.BurstFlameoutGroups();
			else engineFX.part.Effects.Event(engineFX.flameoutEffectName);
		}

		public bool allowShutdown
		{ get { return !isModuleEngineFX ? engine.allowShutdown : engineFX.allowShutdown; } }

		public bool throttleLocked
		{ get { return !isModuleEngineFX ? engine.throttleLocked : engineFX.throttleLocked; } }

		public float maxThrust
		{ get { return !isModuleEngineFX ? engine.maxThrust : engineFX.maxThrust; } }

		public float thrustPercentage
		{
			get { return !isModuleEngineFX ? engine.thrustPercentage : engineFX.thrustPercentage; }
			set
			{
				if(!isModuleEngineFX) engine.thrustPercentage = value;
				else engineFX.thrustPercentage = value;
			}
		}

		public bool isEnabled
		{ get { return !isModuleEngineFX ? engine.EngineIgnited : engineFX.EngineIgnited; } }

		public String name
		{ get { return !isModuleEngineFX ? engine.part.name : engineFX.part.name; } }

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
