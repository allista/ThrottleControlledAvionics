//------------------------------------------------------------------------------
// This EngineWrapper class came from HoneyFox's EngineIgnitor mod:
// Edited by Willem van Vliet 2014
// More info: https://github.com/HoneyFox/EngineIgnitor
//------------------------------------------------------------------------------
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using UnityEngine;

namespace ThrottleControlledAvionics
{
	public class EngineWrapper
	{
		public bool isModuleEngineFX = false;
		private ModuleEngines engine = null;
		private ModuleEnginesFX engineFX = null;
		private Vector3 mSteeringVector = Vector3.zero;
		private Vector3 mThrustVector = Vector3.zero;


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
		{
			get
			{
				if(isModuleEngineFX == false)
					return engine.vessel;
				else
					return engineFX.vessel;
			}
		}
		
		public void SetRunningGroupsActive(bool active)
		{
			if (isModuleEngineFX == false)
				engine.SetRunningGroupsActive(active);
			// Do not need to worry about ModuleEnginesFX.
		}
		
		public float requestedThrust
		{
			get
			{
				if (isModuleEngineFX == false)
					return engine.requestedThrust;
				else
					return engineFX.requestedThrust;
			}
		}
		
		public List<Propellant> propellants
		{
			get
			{
				if (isModuleEngineFX == false)
					return engine.propellants;
				else
					return engineFX.propellants;
			}
		}
		
		public Part part
		{
			get
			{
				if (isModuleEngineFX == false)
					return engine.part;
				else
					return engineFX.part;
			}
		}
		
		public BaseEventList Events
		{
			get
			{
				if (isModuleEngineFX == false)
					return engine.Events;
				else
					return engineFX.Events;
			}
		}
		
		public void BurstFlameoutGroups()
		{
			if (isModuleEngineFX == false)
				engine.BurstFlameoutGroups();
			else
				engineFX.part.Effects.Event(engineFX.flameoutEffectName);
		}
		
		public bool allowShutdown
		{
			get
			{
				if (isModuleEngineFX == false)
					return engine.allowShutdown;
				else
					return engineFX.allowShutdown;
			}
		}

		public bool throttleLocked
		{
			get 
			{
				if (isModuleEngineFX == false)
					return engine.throttleLocked;
				else
					return engineFX.throttleLocked;
			}
		}

		public float maxThrust
		{
			get 
			{
				if (isModuleEngineFX == false)
					return engine.maxThrust;
				else
					return engineFX.maxThrust;
			}
		}

		public float thrustPercentage
		{
			get 
			{
				if (isModuleEngineFX == false)
					return engine.thrustPercentage;
				else
					return engineFX.thrustPercentage;
			}
			set
			{
				if (isModuleEngineFX == false)
					engine.thrustPercentage = value;
				else
					engineFX.thrustPercentage = value;
			}
		}

		public bool isEnabled {
			get 
			{
				if (isModuleEngineFX == false)
					return engine.EngineIgnited;
				else
					return engineFX.EngineIgnited;
			}
		}

		public String name {
			get {
				if (isModuleEngineFX == false)
					return engine.part.name;
				else
					return engineFX.part.name;
			}
		}

		public String getName() {
			String r = "";
			if (isModuleEngineFX == false) {
				r += engine.part.name;
				if (engine.engineID.Length>0) {
					r += " (" + engine.engineID + ")";
				}
			} else {
				r += engineFX.part.name;
				if (engineFX.engineID.Length>0) {
					r += " (" + engineFX.engineID + ")";
				}
			}
			return r;
		}

		public Vector3 steeringVector {
			get {
				return mSteeringVector;
			}
			set {
				mSteeringVector = value;
			}
		}

		public Vector3 thrustVector {
			get {
				return mThrustVector;
			}
			set {
				mThrustVector = value;
			}
		}

	}
}

