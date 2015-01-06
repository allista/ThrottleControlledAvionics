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
using System.IO;
using KSP.IO;

namespace ThrottleControlledAvionics
{
	[KSPAddon(KSPAddon.Startup.Flight, false)]
	public class ThrottleControlledAvionics : MonoBehaviour
	{
		#region Configuration
		protected static PluginConfiguration configfile = PluginConfiguration.CreateForType<ThrottleControlledAvionics>();
		public SaveFile save; //deprecated
		TCAGui GUI;

		Vessel vessel;
		public readonly List<EngineWrapper> engines = new List<EngineWrapper>();
		public bool haveEC = true;
		Vector3 wCoM;
		Transform refT;
		Vector3 steering = Vector3.zero;
		readonly float MAX_STEERING = Mathf.Sqrt(3);

		public float steeringThreshold = 0.01f; //too small steering vector may cause oscilations
		public float responseSpeed     = 0.25f; //speed at which the value of a thrust limiter is changed towards the needed value (part-of-difference/FixedTick)
		public float verticalCutoff    = 1f;    //max. positive vertical speed m/s (configurable)
		public const float maxCutoff   = 10f;   //max. positive vertical speed m/s (configuration limit)
		public float resposeCurve      = 0.3f;  //coefficient of non-linearity of efficiency response to partial steering (1 means response is linear, 0 means no response)

		public bool isActive;
		#endregion

		#region Engine Logic
		public void Awake()
		{
			// open save
			save = new SaveFile(); 
			save.Load();
			GUI = new TCAGui(this);
			GameEvents.onVesselChange.Add(onVessel);
			GameEvents.onVesselWasModified.Add(onVessel);
		}

		internal void OnDestroy() 
		{ 
			save.Save();
			if(GUI != null) GUI.OnDestroy();
			GameEvents.onVesselChange.Remove(onVessel);
			GameEvents.onVesselWasModified.Remove(onVessel);
		}

		void onVessel(Vessel vsl)
		{ if(vessel == null || vsl == vessel) UpdateEnginesList(); }

		public void ActivateTCA(bool state)
		{
			isActive = state;
			if(!isActive) 
				engines.ForEach(e => e.thrustPercentage = 100);
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
			//check for Electrich Charge
			haveEC = vessel.ElectricChargeAvailible();
			if(!haveEC) return;
			//prerequisites
			wCoM = vessel.findWorldCenterOfMass();
			refT = vessel.GetReferenceTransformPart().transform; //should be in a callback
			//update engines if needed
			if(engines.Any(e => !e.Valid)) UpdateEnginesList();
			var active_engines = engines.Where(e => e.isEnabled).ToList();
			if(active_engines.Count == 0) return;
			//calculate steering attributes
			steering        = new Vector3(vessel.ctrlState.pitch, vessel.ctrlState.roll, vessel.ctrlState.yaw);
			var demand      = steering;
			var demand_m    = demand.magnitude;
			var eK          = demand_m/MAX_STEERING;
			var is_steering = demand_m < steeringThreshold;
			//calculate current and maximum torque
			var totalTorque = Vector3.zero;
			var maxTorque   = Vector3.zero;
			foreach(var eng in active_engines)
			{
				var info = eng.thrustInfo;
				//total thrust vector of the engine in the controller-part's coordinates
				eng.thrustDirection = refT.InverseTransformDirection(info.dir);
				//rhe torque this engine currentely delivers
				var spec_torque = refT.InverseTransformDirection(Vector3.Cross(info.pos-wCoM, info.dir));
				eng.currentTorque = spec_torque * (eng.finalThrust > 0? eng.finalThrust: eng.requestedThrust);
				if(is_steering)
				{
					totalTorque += eng.currentTorque;
					maxTorque += spec_torque*eng.maxThrust;
				}
			}
			//calculate efficiency
			if(is_steering)
			{
				demand   = -totalTorque;
				demand_m =  totalTorque.magnitude;
				eK       =  demand_m/maxTorque.magnitude;
			}
			active_engines.ForEach(e => e.efficiency = Vector3.Dot(e.currentTorque, demand));
			//scale efficiency to [-1; 0]
			var max_eff = active_engines.Max(e => e.efficiency);
			active_engines.ForEach(e => e.efficiency -= max_eff);
			max_eff = active_engines.Max(e => Mathf.Abs(e.efficiency));
			if(max_eff <= 0) return;
			//non-linear coefficient works better
			eK = Mathf.Pow(eK, resposeCurve); 
			//thrust coefficient for vertical speed limit
			var upV = Vector3d.Dot(vessel.srf_velocity, (wCoM - vessel.mainBody.position).normalized); //from MechJeb
			var tK = verticalCutoff < maxCutoff? 1-Mathf.Clamp01((float)upV/verticalCutoff) : 1;
			//set thrust limiters
			foreach(var eng in active_engines)
			{
				eng.efficiency = eng.efficiency/max_eff * eK;
				var needed_percentage = Mathf.Clamp(100 * tK * (1 + eng.efficiency), 0f, 100f); 
				eng.thrustPercentage += (needed_percentage-eng.thrustPercentage)*save.GetActiveSensitivity();
			}
		}
		#endregion
	}

	public class SaveFile
	{
		int activeSave;
		readonly object[,] saves;

		public SaveFile()
		{
			saves = new object[3, 3];
			activeSave = 0;
		}

		public void Save()
		{
			String text = (String)saves[0, 0] + "\n" + (float)saves[0, 1] + "\n" + (float)saves[0, 2] + "\n\n" +
			              (String)saves[1, 0] + "\n" + (float)saves[1, 1] + "\n" + (float)saves[1, 2] + "\n\n" +
			              (String)saves[2, 0] + "\n" + (float)saves[2, 1] + "\n" + (float)saves[2, 2] + "\n\n";
			using(var writer = new StreamWriter("GameData/ThrottleControlledAvionics/TCASave.txt", false))
				writer.Write(text);
		}

		public void Load()
		{
			try
			{
				using(var reader = new StreamReader("GameData/ThrottleControlledAvionics/TCASave.txt"))
				{
					for(int i = 0; i < 3; i++)
					{
						saves[i, 0] = reader.ReadLine();
						saves[i, 1] = float.Parse(reader.ReadLine());
						saves[i, 2] = float.Parse(reader.ReadLine());
						reader.ReadLine();
					}
				}
			}
			catch(Exception e)
			{                
				if(e is FileNotFoundException || e is System.IO.IsolatedStorage.IsolatedStorageException)
					SetDefault();
				else
				{
					Utils.writeToFile("We found a serous problem during Load:" + e);
					throw;
				}
			}
		}

		void SetDefault()
		{
			for(int i = 0; i < 3; i++)
			{
				Utils.writeToFile("i = " + i);
				saves[i, 0] = "Setting " + i;
				saves[i, 1] = 1f;
				saves[i, 2] = 100f;
			}
		}

		#region setters
		public void SetActiveSave(int i) { activeSave = i; }

		public void SetActiveName(string name) { saves[activeSave, 0] = name; }

		public void SetActiveSensitivity(float tf) { saves[activeSave, 1] = tf; }

		public void SetActiveMeanThrust(float mt) { saves[activeSave, 2] = mt; }
		#endregion

		#region getters
		public string GetActiveName() { return (string)saves[activeSave, 0]; }

		public string GetName(int i) { return (string)saves[i, 0]; }

		public float GetActiveSensitivity() { return (float)saves[activeSave, 1]; }

		public float GetActiveMeanThrust() { return (float)saves[activeSave, 2]; }
		#endregion
	}
}
