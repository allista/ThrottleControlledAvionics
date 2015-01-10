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

		Vessel vessel;
		public readonly List<EngineWrapper> engines = new List<EngineWrapper>();
		public bool haveEC = true;

		#region Controls
		public bool  isActive; //TODO: remeber state per vessel

		//attitude control
		public readonly PIv_Controller torque   = new PIv_Controller();
		public readonly PIv_Controller steering = new PIv_Controller();
		static readonly float MAX_STEERING = Mathf.Sqrt(3);
		public float steeringThreshold   = 0.01f; //too small steering vector may cause oscilations
		public float stabilityCurve      = 0.3f;  /* coefficient of non-linearity of efficiency response to partial steering 
												    (2 means response is quadratic, 1 means response is linear, 0 means no response) */
		public float torqueThreshold     = 0.1f;  //engines which produce less specific torque are considered to be main thrusters and excluded from TCA control
		public float idleResponseSpeed   = 0.25f;

		//vertical speed limit
		public const float maxCutoff = 10f; //max. positive vertical speed m/s (configuration limit)
		public float verticalCutoff  = 1f;  //max. positive vertical speed m/s (configurable)
		float upV_old = 0f; //previous velue of vertical speed
		#endregion

		#region Engine Logic
		public void Awake()
		{
			GUI = new TCAGui(this);
			GameEvents.onVesselChange.Add(onVessel);
			GameEvents.onVesselWasModified.Add(onVessel);
		}

		internal void OnDestroy() 
		{ 
			if(GUI != null) GUI.OnDestroy();
			GameEvents.onVesselChange.Remove(onVessel);
			GameEvents.onVesselWasModified.Remove(onVessel);
		}

		void onVessel(Vessel vsl)
		{ if(vessel == null || vsl == vessel) UpdateEnginesList(); }

		public void ActivateTCA(bool state)
		{
			if(state == isActive) return;
			isActive = state;
			if(!isActive)
			{
				//reset engine limiters
				engines.ForEach(e => e.thrustPercentage = 100);
				//set throttle to zero?
				vessel.ctrlState.mainThrottle = 0f;
				if(vessel == FlightGlobals.ActiveVessel)
					FlightInputHandler.state.mainThrottle = 0f; //so that the on-screen throttle gauge reflects the autopilot throttle
				vessel.FeedInputFeed();
			}
		}

		void UpdateEnginesList()
		{
			vessel = FlightGlobals.ActiveVessel;
			if(vessel == null) return;
			engines.Clear();
			foreach(Part p in vessel.Parts)
				foreach(var module in p.Modules)
				{	
					EngineWrapper engine = null;
					if(module is ModuleEngines)
						engine = new EngineWrapper(module as ModuleEngines);
					else if(module is ModuleEnginesFX)
						engine = new EngineWrapper(module as ModuleEnginesFX);
					if(engine != null && !engine.throttleLocked) engines.Add(engine);
				}
		}

		public void OnGUI() 
		{ 
			GUI.DrawGUI(); 
			GUI.UpdateToolbarIcon();
		}

		public void Update()
		{ if(Input.GetKeyDown("y")) ActivateTCA(!isActive); }

		public void FixedUpdate()
		{
			if(!isActive || vessel == null) return;
			//check for throttle and Electrich Charge
			if(vessel.ctrlState.mainThrottle <= 0) return;
			haveEC = vessel.ElectricChargeAvailible();
			if(!haveEC) return;
			//update engines if needed
			if(engines.Any(e => !e.Valid)) UpdateEnginesList();
			var active_engines = engines.Where(e => e.isEnabled).ToList();
			if(active_engines.Count == 0) return;
			//calculate steering parameters
			var wCoM          = vessel.findWorldCenterOfMass();
			var refT          = vessel.GetReferenceTransformPart().transform; //should be in a callback
			steering.Update(refT.TransformDirection(new Vector3(vessel.ctrlState.pitch, vessel.ctrlState.roll, vessel.ctrlState.yaw)));
			var demand        = steering.Value;
			var demand_m      = demand.magnitude;
			var idle          = demand_m < steeringThreshold;
			var responseSpeed = demand_m/MAX_STEERING;
			//calculate current and maximum torque
			var totalTorque  = Vector3.zero;
			var maxTorque    = Vector3.zero;
			foreach(var e in active_engines)
			{
				var info = e.thrustInfo;
				//total thrust direction of the engine in the controller-part's coordinates
				e.thrustDirection = info.dir;
				//current thrust
				e.currentThrust   = e.finalThrust > 0? e.finalThrust: e.maxThrust*vessel.ctrlState.mainThrottle;
				//the torque this engine currentely delivers
				e.specificTorque  = Vector3.Cross(info.pos-wCoM, info.dir);
				e.currentTorque   = e.specificTorque * e.currentThrust;
				if(idle)
				{
					totalTorque += e.currentTorque;
					maxTorque   += e.specificTorque*e.maxThrust;
				}
			}
			//change steering parameters if attitude controls are idle
			if(idle)
			{
				torque.Update(-totalTorque);
				demand        = torque;
				demand_m      = demand.magnitude;
				responseSpeed = Mathf.Clamp01(demand_m/maxTorque.magnitude);
			}
			//calculate engines efficiency in current maneuver
			bool first = true;
			float min_eff = 0, max_eff = 0, max_abs_eff = 0;
			foreach(var e in active_engines)
			{
				var eff = Vector3.Dot(e.currentTorque, demand);
				var abs_eff = Mathf.Abs(eff);
				if(first || min_eff > eff) min_eff = eff;
				if(first || max_eff < eff) max_eff = eff;
				if(max_abs_eff < abs_eff) max_abs_eff = abs_eff;
				first = false;
				e.efficiency = eff;
			}
			var eff_span = max_eff - min_eff;
			//new thrust limits are computed only after we know that some thrusters should be firing
			if(eff_span <= 0) return;
			//calculate thrust coefficient for vertical speed limit
			var vK = 1f;
			if(vessel.situation != Vessel.Situations.DOCKED &&
			   vessel.situation != Vessel.Situations.ORBITING &&
			   vessel.situation != Vessel.Situations.ESCAPING &&
			   verticalCutoff < maxCutoff)
			{
				//unlike the vessel.verticalSpeed, this method is unaffected by ship's rotation 
				var up  = (wCoM - vessel.mainBody.position).normalized;
				var upV = (float)Vector3d.Dot(vessel.srf_velocity, up); //from MechJeb
				var upA = (upV-upV_old)/TimeWarp.fixedDeltaTime;
				var err = verticalCutoff-upV;
				vK = upV < verticalCutoff?
					Mathf.Clamp01(err/Mathf.Pow(Utils.ClampL(upA/10+1, 1f), 2f)) :
					Mathf.Clamp01(err*upA/Mathf.Pow(Utils.ClampL(-err*10f, 10f), 2f));
				upV_old = upV;
			}
			//disable unneeded engines and scale thrust limit to [-1; 0]
			//then set thrust limiters of the engines
			//empirically: non-linear speed change works better
			responseSpeed = Mathf.Pow(responseSpeed, stabilityCurve);
			foreach(var e in active_engines)
			{
				if(e.specificTorque.magnitude * e.maxThrust < torqueThreshold) continue;
				e.efficiency = (e.efficiency - max_eff)/eff_span*responseSpeed;
				var needed_percentage = Mathf.Clamp(100 * vK * (1 + e.efficiency), 0f, 100f);
				e.thrustPercentage += (needed_percentage-e.thrustPercentage) * (idle? idleResponseSpeed : responseSpeed);
			}
		}
		#endregion
	}
}
