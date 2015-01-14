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

//this compilerservices bit makes Xamarin compile this project. else I get "Missing compuler required member" errors. No idea how all of this works. But hey.
//more info: http://stackoverflow.com/questions/205644/error-when-using-extension-methods-in-c-sharp
namespace System.Runtime.CompilerServices {
	public class ExtensionAttribute : Attribute { }
}

namespace ThrottleControlledAvionics
{
	[KSPAddon(KSPAddon.Startup.Flight, false)]
	public class ThrottleControlledAvionics : MonoBehaviour
	{
		TCAGui GUI;

		public Vessel vessel;
		public VesselConfig CFG;
		public readonly List<EngineWrapper> engines = new List<EngineWrapper>();
		public bool haveEC = true;

		public readonly PIv_Controller torque   = new PIv_Controller();
		public readonly PIv_Controller steering = new PIv_Controller();
		float upV_old; //previous velue of vertical speed

		#region Engine Logic
		public void Awake()
		{
			TCAConfiguration.Load();
			GUI = (TCAGui)this.gameObject.AddComponent<TCAGui> ();
			GameEvents.onVesselChange.Add(onVessel);
			GameEvents.onVesselWasModified.Add(onVessel);
			GameEvents.onGameStateSave.Add(onSave);
		}

		internal void OnDestroy() 
		{ 
			TCAConfiguration.Save();
			if(GUI != null) GUI.OnDestroy();
			GameEvents.onVesselChange.Remove(onVessel);
			GameEvents.onVesselWasModified.Remove(onVessel);
			GameEvents.onGameStateSave.Remove(onSave);
		}

		void onVessel(Vessel vsl)
		{ 
			if(vessel == null || vsl == vessel)
			{
				UpdateEnginesList();
				GUI.onVessel(vessel);
			}
		}

		void onSave(ConfigNode node) { TCAConfiguration.Save(); }

		public void ActivateTCA(bool state)
		{
			if(state == CFG.Enabled) return;
			CFG.Enabled = state;
			if(!CFG.Enabled)
			{
				//reset engine limiters
				engines.ForEach(e => e.thrustPercentage = 100);
				//set throttle to zero?
			}
		}

		void UpdateEnginesList()
		{
			vessel = FlightGlobals.ActiveVessel;
			if(vessel == null) return;
			CFG = TCAConfiguration.GetConfig(vessel);
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
			if(engines.Any(e => !e.Valid)) UpdateEnginesList();
			var active_engines = engines.Where(e => e.isEnabled).ToList();
			if(active_engines.Count == 0) return;
			//calculate steering parameters
			var wCoM          = vessel.findWorldCenterOfMass();
			var refT          = vessel.GetReferenceTransformPart().transform; //should be in a callback
			steering.Update(refT.TransformDirection(new Vector3(vessel.ctrlState.pitch, vessel.ctrlState.roll, vessel.ctrlState.yaw)));
			var demand        = steering.Value;
			var demand_m      = demand.magnitude;
			var idle          = demand_m < TCAConfiguration.Globals.SteeringThreshold;
			var responseSpeed = demand_m/TCAGlobals.MAX_STEERING;
			//calculate current and maximum torque
			var totalTorque  = Vector3.zero;
			var maxTorque    = Vector3.zero;
			foreach(var e in active_engines)
			{
				var info = e.thrustInfo;
				//total thrust direction of the engine in the controller-part's coordinates
				e.thrustDirection = info.dir;
				//the torque this engine currentely delivers
				e.specificTorque  = Vector3.Cross(info.pos-wCoM, info.dir);
				e.currentTorque   = e.specificTorque * (e.finalThrust > 0? e.finalThrust: e.maxThrust*vessel.ctrlState.mainThrottle);
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
			   CFG.VerticalCutoff < TCAConfiguration.Globals.MaxCutoff)
			{
				//unlike the vessel.verticalSpeed, this method is unaffected by ship's rotation 
				var up  = (wCoM - vessel.mainBody.position).normalized;
				var upV = (float)Vector3d.Dot(vessel.srf_velocity, up); //from MechJeb
				var upA = (upV-upV_old)/TimeWarp.fixedDeltaTime;
				var err = CFG.VerticalCutoff-upV;
				vK = upV < CFG.VerticalCutoff?
					Mathf.Clamp01(err/Mathf.Pow(Utils.ClampL(upA/TCAConfiguration.Globals.K1+1, TCAConfiguration.Globals.L1), 2f)) :
					Mathf.Clamp01(err*upA/Mathf.Pow(Utils.ClampL(-err*TCAConfiguration.Globals.K2, TCAConfiguration.Globals.L2), 2f));
				upV_old = upV;
			}
			//disable unneeded engines and scale thrust limit to [-1; 0]
			//then set thrust limiters of the engines
			//empirically: non-linear speed change works better
			responseSpeed = Mathf.Pow(responseSpeed, TCAConfiguration.Globals.StabilityCurve);
			foreach(var e in active_engines)
			{
				if(e.specificTorque.magnitude * e.maxThrust < TCAConfiguration.Globals.TorqueThreshold) continue;
				e.efficiency = (e.efficiency - max_eff)/eff_span*responseSpeed;
				var needed_percentage = Mathf.Clamp(100 * vK * (1 + e.efficiency), 0f, 100f);
				e.thrustPercentage += (needed_percentage-e.thrustPercentage) * (idle? TCAConfiguration.Globals.IdleResponseSpeed : responseSpeed);
			}
		}
		#endregion
	}
}
