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
		/// The torque this engine can deliver represented as a rotation axis
		/// </summary>
		public Vector3 steeringVector = Vector3.zero;

		/// <summary>
		/// The direction this engine is fireing in, measured form the down vector, relative form the root part
		/// </summary>
		public Vector3 thrustVector = Vector3.zero;

		public float efficiency;
		public float steering;

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

		public float requestedThrust
		{ get { return !isModuleEngineFX ? engine.requestedThrust : engineFX.requestedThrust; } }

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
