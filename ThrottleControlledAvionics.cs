/* Name: Throttle Controlled Avionics, Fork by Allis Tauri
 *
 * Authors: Quinten Feys & Willem van Vliet & Allis Tauri
 * License: BY: Attribution-ShareAlike 3.0 Unported (CC BY-SA 3.0): 
 * http://creativecommons.org/licenses/by-sa/3.0/
 * 
 */

using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace ThrottleControlledAvionics
{
	[KSPAddon(KSPAddon.Startup.Flight, false)]
	public class ThrottleControlledAvionics : MonoBehaviour
	{
		TCAGui GUI;

		#region State
		Vessel vessel;
		public VesselConfig CFG { get; private set; }
		public readonly List<EngineWrapper> Engines = new List<EngineWrapper>();
		public bool haveEC { get; private set; } = true;
		public float TorqueError { get; private set; }
		Vector3 wCoM;    //center of mass in world space
		Transform refT;  //transform of the controller-part
		float last_upV;  //previous velue of vertical speed
		Vector3 steering; //previous steering vector
		#endregion

		#region Engine Logic
		public void Awake()
		{
			TCAConfiguration.Load();
			GUI = new TCAGui(this);
			GameEvents.onVesselChange.Add(onVesselChange);
			GameEvents.onVesselWasModified.Add(onVesselModify);
			GameEvents.onGameStateSave.Add(onSave);
		}

		internal void OnDestroy() 
		{ 
			TCAConfiguration.Save();
			if(GUI != null) GUI.OnDestroy();
			GameEvents.onVesselChange.Remove(onVesselChange);
			GameEvents.onVesselWasModified.Remove(onVesselModify);
			GameEvents.onGameStateSave.Remove(onSave);
		}

		void onVesselChange(Vessel vsl)
		{ 
			if(vsl == null || vsl.Parts == null) return;
			TCAConfiguration.Save();
			vessel = vsl;
			UpdateEnginesList();
		}

		void onVesselModify(Vessel vsl)
		{ if(vessel == vsl) UpdateEnginesList(); }

		void onSave(ConfigNode node) { TCAConfiguration.Save(); }

		public void ActivateTCA(bool state)
		{
			if(CFG == null || state == CFG.Enabled) return;
			CFG.Enabled = state;
			if(!CFG.Enabled) //reset engine limiters
				Engines.ForEach(e => e.forceThrustPercentage(100));
		}

		void UpdateEnginesList()
		{
			if(vessel == null) return;
			CFG = TCAConfiguration.GetConfig(vessel);
			EngineWrapper.ThrustPI.setMaster(CFG.Engines);
			Engines.Clear();
			foreach(Part p in vessel.Parts)
				foreach(var module in p.Modules)
				{	
					EngineWrapper engine = null;
					if(module is ModuleEngines)
						engine = new EngineWrapper(module as ModuleEngines);
					else if(module is ModuleEnginesFX)
						engine = new EngineWrapper(module as ModuleEnginesFX);
					if(engine != null && !engine.throttleLocked) Engines.Add(engine);
				}
		}

		public void OnGUI() 
		{ 
			GUI.DrawGUI(); 
			GUI.UpdateToolbarIcon();
		}

		public void Update()
		{ 
			if(Input.GetKeyDown(TCAConfiguration.Globals.TCA_Key)) 
				ActivateTCA(!CFG.Enabled); 
		}

		public void FixedUpdate()
		{
			if(!CFG.Enabled || vessel == null) return;
			//check for throttle and Electrich Charge
			if(vessel.ctrlState.mainThrottle <= 0) return;
			haveEC = vessel.ElectricChargeAvailible();
			if(!haveEC) return;
			//update engines if needed
			if(Engines.Any(e => !e.Valid)) UpdateEnginesList();
			var active_engines = Engines.Where(e => e.isEnabled).ToList();
			if(active_engines.Count == 0) return;
			//calculate steering
			wCoM         = vessel.findWorldCenterOfMass();
			refT         = vessel.GetReferenceTransformPart().transform; //should be in a callback?
			var new_steering = refT.TransformDirection(new Vector3(vessel.ctrlState.pitch, vessel.ctrlState.roll, vessel.ctrlState.yaw))/TCAGlobals.MAX_STEERING;
			CFG.Steering.Update(new_steering-steering);
			steering += CFG.Steering.Action;
			//tune engines limits
			optimizeLimitsIteratively(active_engines, 
			                          TCAConfiguration.Globals.OptimizationPrecision, 
			                          TCAConfiguration.Globals.MaxIterations);
			setThrustPercentage(active_engines, verticalSpeedLimit);
		}

		static bool optimizeLimits(List<EngineWrapper> engines, Vector3 target, float target_m, float eps)
		{
			if(target_m < eps) return false;
			var compensation = Vector3.zero;
			for(int i = 0; i < engines.Count; i++)
			{
				var e = engines[i];
				var d = Vector3.Dot(e.currentTorque, target);
				e.limit_tmp = d < 0? -d/target_m/e.currentTorque.magnitude : 0f;
				compensation += e.limit_tmp > 0? e.limit_tmp * e.currentTorque * e.limit : Vector3.zero;
			}
			var compensation_m = compensation.magnitude;
			if(compensation_m < eps) return false;
			var limits_norm = target_m/compensation_m;
			engines.ForEach(e => e.limit *= (1-Mathf.Clamp01(e.limit_tmp*limits_norm)));
			return true;
		}

		void optimizeLimitsIteratively(List<EngineWrapper> engines, float eps, int max_iter)
		{
			//calculate initial imbalance and needed torque
			var torque_imbalance = Vector3.zero;
			var needed_torque = Vector3.zero;
			for(int i = 0; i < engines.Count; i++)
			{
				var e = engines[i];
				var info = e.thrustInfo;
				e.limit = 1f;
				e.thrustDirection = info.dir;
				e.specificTorque  = Vector3.Cross(info.pos-wCoM, info.dir);
				e.currentThrust   = e.maxThrust * vessel.ctrlState.mainThrottle;
				e.currentTorque   = e.specificTorque * e.currentThrust;
				torque_imbalance += e.currentTorque;
				if(Vector3.Dot(e.currentTorque, steering) > 0)
					needed_torque += e.currentTorque;
			}
			needed_torque = Vector3.Project(needed_torque, steering) * steering.magnitude;
			//optimize engines' limits
			TorqueError = 0f;
			float last_error = -1f;
			for(int i = 0; i < max_iter; i++)
			{
				var target   = needed_torque - torque_imbalance;
				var target_m = target.magnitude;
				if(!optimizeLimits(engines, target, target_m, eps)) break;
				TorqueError = (engines.Aggregate(Vector3.zero, (v, e) => v + e.currentTorque * e.limit)-needed_torque).magnitude;
				if(TorqueError < eps || last_error > 0f && Mathf.Abs(last_error-TorqueError) < eps) break;
				torque_imbalance = engines.Aggregate(Vector3.zero, (v, e) => v + e.currentTorque * e.limit);
				last_error = TorqueError;
			}
		}

		static void setThrustPercentage(List<EngineWrapper> engines, float speedLimit)
		{
			for(int i = 0; i < engines.Count; i++)
			{
				var e = engines[i];
				if(e.specificTorque.magnitude * e.maxThrust < TCAConfiguration.Globals.TorqueThreshold)
					e.forceThrustPercentage(100);
				else e.thrustPercentage = Mathf.Clamp(100 * speedLimit * e.limit, 0f, 100f);
			}
		}

		float verticalSpeedLimit
		{
			get
			{
				if(vessel.situation == Vessel.Situations.DOCKED ||
				   vessel.situation == Vessel.Situations.ORBITING ||
				   vessel.situation == Vessel.Situations.ESCAPING ||
				   CFG.VerticalCutoff >= TCAConfiguration.Globals.MaxCutoff) return 1f;
				//unlike the vessel.verticalSpeed, this method is unaffected by ship's rotation 
				var up  = (wCoM - vessel.mainBody.position).normalized;
				var upV = (float)Vector3d.Dot(vessel.srf_velocity, up); //from MechJeb
				var upA = (upV-last_upV)/TimeWarp.fixedDeltaTime;
				var err = CFG.VerticalCutoff-upV;
				last_upV = upV;
				return upV < CFG.VerticalCutoff?
					Mathf.Clamp01(err/TCAConfiguration.Globals.K0/Mathf.Pow(Utils.ClampL(upA/TCAConfiguration.Globals.K1+1, TCAConfiguration.Globals.L1), 2f)) :
					Mathf.Clamp01(err*upA/Mathf.Pow(Utils.ClampL(-err*TCAConfiguration.Globals.K2, TCAConfiguration.Globals.L2), 2f));
			}
		}
		#endregion
	}
}
