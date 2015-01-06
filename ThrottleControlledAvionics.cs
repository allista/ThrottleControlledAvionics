/* Name: Throttle Controlled Avionics, Fork by Allis Tauri
 *
 * Authors: Quinten Feys & Willem van Vliet & Allis Tauri
 * License: BY: Attribution-ShareAlike 3.0 Unported (CC BY-SA 3.0): 
 * http://creativecommons.org/licenses/by-sa/3.0/
 * 
 */

//TODO: fix null pointer when engines are destroyed
//TODO: fix GUI for breaking system

using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using System.IO;

namespace ThrottleControlledAvionics
{
	[KSPAddon(KSPAddon.Startup.Flight, false)]
	public class ThrottleControlledAvionics : MonoBehaviour
	{
		#region variables
		Vessel vessel;
		protected Rect windowPos = new Rect(50, 50, 450, 200);
		protected Rect windowPosHelp = new Rect(500, 100, 400, 50);
		readonly List<EngineWrapper> engines = new List<EngineWrapper>();
		float steeringThreshold = 0.01f;
		float verticalCutoff = 10f;
		const float maxCutoff = 10f;
		float resposeCurve = 0.3f;
		// as a pointing vector
		readonly float MAX_DEMAND = Mathf.Sqrt(3);
		Vector3 steering = Vector3.zero;
		bool haveEC = true;
		Vector3 wCoM;
		Transform refT;

		bool isActive;
		bool showAny;
		bool showEngines;
		bool showHelp;
		bool showHUD = true;
		
		SaveFile save;
		GUIContent[] saveList;
		ComboBox saveListBox;
		Vector2 positionScrollViewEngines;

		const string ICON = "ThrottleControlledAvionics/Icons/icon_button_off";

		IButton TCAToolbarButton;
		ApplicationLauncherButton TCAButton;
		#endregion

		#region Unity interface
		/// <summary>
		/// Makes sure the toolbar icon resembles the present situation
		/// </summary>
		void UpdateToolbarIcon() 
		{
			if(TCAToolbarButton == null) return;
			if(isActive)
				TCAToolbarButton.TexturePath = haveEC ? 
					"ThrottleControlledAvionics/Icons/icon_button_on" : 
					"ThrottleControlledAvionics/Icons/icon_button_noCharge";
			else TCAToolbarButton.TexturePath = "ThrottleControlledAvionics/Icons/icon_button_off";
		}

		void OnGUIAppLauncherReady()
		{
			if (ApplicationLauncher.Ready)
			{
				TCAButton = ApplicationLauncher.Instance.AddModApplication(
					onAppLaunchToggleOn,
					onAppLaunchToggleOff,
					DummyVoid, DummyVoid, DummyVoid, DummyVoid,
					ApplicationLauncher.AppScenes.FLIGHT,
					GameDatabase.Instance.GetTexture(ICON, false));
			}
		}

		void onAppLaunchToggleOn() { showAny = true; }

		void onAppLaunchToggleOff() { showAny = false; }

		void DummyVoid() {}

		public void Awake()
		{
			// open save
			save = new SaveFile();
			save.Load();

			if(ToolbarManager.ToolbarAvailable)
			{
				TCAToolbarButton = ToolbarManager.Instance.add("ThrottleControlledAvionics", "ThrottleControlledAvionicsButton");
				TCAToolbarButton.TexturePath = ICON;
				TCAToolbarButton.ToolTip = "Throttle Controlled Avionics";
				TCAToolbarButton.Visibility = new GameScenesVisibility(GameScenes.FLIGHT);
				TCAToolbarButton.Visible = true;
				TCAToolbarButton.OnClick += e => showAny = !showAny;
			}
			else 
				GameEvents.onGUIApplicationLauncherReady.Add(OnGUIAppLauncherReady);
			
			saveList = new GUIContent[3];
			saveList[0] = new GUIContent(save.GetName(0));
			saveList[1] = new GUIContent(save.GetName(1));
			saveList[2] = new GUIContent(save.GetName(2));
			saveListBox = new ComboBox();

			GameEvents.onHideUI.Add(onHideUI);
			GameEvents.onShowUI.Add(onShowUI);
			GameEvents.onVesselChange.Add(onVessel);
			GameEvents.onVesselWasModified.Add(onVessel);
		}

		internal void OnDestroy() 
		{ 
			save.Save(); 
			GameEvents.onGUIApplicationLauncherReady.Remove(OnGUIAppLauncherReady);
			if(TCAButton != null)
				ApplicationLauncher.Instance.RemoveModApplication(TCAButton);
			if(TCAToolbarButton != null)
				TCAToolbarButton.Destroy();
			GameEvents.onHideUI.Remove(onHideUI);
			GameEvents.onShowUI.Remove(onShowUI);
			GameEvents.onVesselChange.Remove(onVessel);
			GameEvents.onVesselWasModified.Remove(onVessel);
		}

		void onShowUI() { showHUD = true; }
		void onHideUI() { showHUD = false; }

		void onVessel(Vessel vsl)
		{ 
			Utils.Log("OnVessel: {0}", vsl);//debug
			if(vessel == null || vsl == vessel) 
				UpdateEnginesList();
		}

		public void OnGUI()
		{
			drawGUI();
			UpdateToolbarIcon();
		}
		#endregion

		#region GUI
		void drawGUI()
		{
			if(!showAny || !showHUD) return;
			windowPos = GUILayout.Window(1, windowPos, WindowGUI, "Throttle Controlled Avionics");
			if(showHelp)
				windowPosHelp = GUILayout.Window(2, windowPosHelp, windowHelp, "Instructions");
		}

		void WindowGUI(int windowID)
		{
			if(GUI.Button(new Rect(windowPos.width - 23f, 2f, 20f, 18f), "?"))
				showHelp = !showHelp;

			GUILayout.BeginVertical();
			if(!haveEC)	GUILayout.Label("WARNING! Electric charge has run out!");

			GUILayout.BeginHorizontal();
			ActivateTCA(GUILayout.Toggle(isActive, "Toggle TCA"));
			GUILayout.Label("Settings: ", GUILayout.ExpandWidth(true));
			InsertDropdownboxSave();
			GUILayout.EndHorizontal();

			GUILayout.BeginHorizontal();
			GUILayout.Label("Response Speed: ", GUILayout.ExpandWidth(false));
			GUILayout.Label(save.GetActiveSensitivity().ToString("P2"), GUILayout.ExpandWidth(false));
			save.SetActiveSensitivity(GUILayout.HorizontalSlider(save.GetActiveSensitivity(), 0f, 1f));
			GUILayout.EndHorizontal();

			GUILayout.BeginHorizontal();
			GUILayout.Label("Response Curve: ", GUILayout.ExpandWidth(false));
			GUILayout.Label(resposeCurve.ToString("F2"), GUILayout.ExpandWidth(false));
			resposeCurve = GUILayout.HorizontalSlider(resposeCurve, 0f, 1f);
			GUILayout.EndHorizontal();

			GUILayout.BeginHorizontal();
			GUILayout.Label("Steering Threshold: ", GUILayout.ExpandWidth(false));
			GUILayout.Label(steeringThreshold.ToString("P1"), GUILayout.ExpandWidth(false));
			steeringThreshold = GUILayout.HorizontalSlider(steeringThreshold, 0f, 0.1f);
			GUILayout.EndHorizontal();

			GUILayout.BeginHorizontal();
			GUILayout.Label("Vertical Cutoff: ", GUILayout.ExpandWidth(false));
			GUILayout.Label(verticalCutoff >= maxCutoff? "inf." :
				verticalCutoff.ToString("F1"), GUILayout.ExpandWidth(false));
			verticalCutoff = GUILayout.HorizontalSlider(verticalCutoff, 0f, maxCutoff);
			GUILayout.EndHorizontal();

			showEngines = GUILayout.Toggle(showEngines, "Show/hide engine information");
			if(showEngines)
			{
				GUILayout.BeginHorizontal();
				GUILayout.Label("Steering: ", GUILayout.ExpandWidth(true));
				GUILayout.Label(steering.ToString(), GUILayout.ExpandWidth(false));
				GUILayout.EndHorizontal();
				positionScrollViewEngines = GUILayout.BeginScrollView(positionScrollViewEngines, GUILayout.Height(300));
				foreach(var eng in engines)
				{
					if(!eng.Valid) continue;
					GUILayout.Label(eng.getName() + "\n" +
					                string.Format(
						                "Torque: {0}\n" +
						                "Thrust Dir: {1}\n" +
						                "Efficiency: {2:P1}\n" +
						                "Thrust: {3:F1}%",
						                eng.currentTorque, eng.thrustDirection,
						                eng.efficiency, eng.thrustPercentage));
				}
				GUILayout.EndScrollView();
			}
			GUILayout.EndVertical();
			GUI.DragWindow();
		}

		/// <summary>
		/// Inserts a dropdownbox to select the deired saved settings from
		/// </summary>
		void InsertDropdownboxSave()
		{
			int i = saveListBox.GetSelectedItemIndex();
			i = saveListBox.List(saveList[i].text, saveList, "Box");
			save.SetActiveSave(i);
		}

		static void windowHelp(int windowID)
		{
			const string instructions = "Welcome to the instructions manual.\nFor simple use:\n\t 1)Put TCA on ('y'),\n\t 2)Put SAS on ('t'), \n\t 3) Launch \n\n" +
			                            "For more advanced use:\n\t -The sensitivity determines the amount of thrust differences TCA will utilise. A high value will give a very fast and abrupt respose, a low value will be a lot smoother" +
			                            ", but might not be as fast.\n\t -Mean thrust is the virtual average thrust. A value below 100 means that the engines will be started throttled down and " +
			                            "will correct by both throttling up and down. A value above 100 means that they will wait untill the deviation becomes rather big. This might be good if " +
			                            "you think that the standard avionics are strong enough to handle the load. \n\t -3 different settings can be saved at the same time. Saving happens automaticly." +
			                            " \n\t -Detect reaction control thrusters will cause engines that are not sufficiently aligned with the direction you want to go in to only be used as reaction control engines. " +
			                            "This direction can be chosen as up, where normal rockets or planes want to go to; mean, which takes the weighted avarage of all the engines and tries to go that way, " +
			                            "or you can chose the direction yourself with custom. The stearing threshold is the maximum angle between an engine and the desired direction so that that engine will fire " +
			                            "(near) full throttle. A higher angle will result in more engines firing, but with potential less efficiency" +
			                            "\n\nWarning: \n\t -TCA assumes that the engine bells are aligned allong the y-axis of the engine parts. This is the standard orientation for most of them. " +
			                            "If you possess engines that can change the orientation of their engine bells, like some form 'Ferram aerospace' please make sure that they are alligned allong" +
			                            " the right axis before enabeling TCA. \n\t -Note that while jet engines can be throttled, they have some latancy, what might controling VTOL's based on these" +
			                            " rather tricky. \n\t SRB's can't be controlled. That's because they're SRB's";
			GUILayout.Label(instructions, GUILayout.MaxWidth(400));
			GUI.DragWindow();
		}
		#endregion

		
		#region Engine Logic
		void ActivateTCA(bool state)
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
			{
				foreach(var module in p.Modules)
				{	
					EngineWrapper engine = null;
					if(module is ModuleEngines)
						engine = new EngineWrapper(module as ModuleEnginesFX);
					else if(module is ModuleEnginesFX)
						engine = new EngineWrapper(module as ModuleEnginesFX);
					if(engine != null && !engine.throttleLocked) engines.Add(engine);
				}
			}
		}

		public void Update()
		{ if(Input.GetKeyDown("y")) ActivateTCA(!isActive); }

		public void FixedUpdate()
		{
			if(!isActive || vessel == null) return;
			haveEC = vessel.ElectricChargeAvailible();
			if(haveEC)
			{
				wCoM = vessel.findWorldCenterOfMass();
				refT = vessel.GetReferenceTransformPart().transform; //should be in a callback
				steering = new Vector3(vessel.ctrlState.pitch, vessel.ctrlState.roll, vessel.ctrlState.yaw);
				SetThrottle();
			}
		}

		/// <summary>
		/// Here is the thrust modifier of every engine calculated. This is based on how much steering they provide and how efficient they can thrust along the main thrust axis.
		/// </summary>
		void SetThrottle()
		{
			var demand      = steering;
			var demand_m    = demand.magnitude;
			var eK           = demand_m/MAX_DEMAND;
			var is_steering = demand_m < steeringThreshold;
			//calculate thrusts
			var totalTorque = Vector3.zero;
			var maxTorque   = Vector3.zero;
			foreach(var eng in engines)
			{
				if(!eng.isEnabled) continue;
				var info = eng.thrustInfo;
				//Total thrust vector in the controller-part's coordinates
				eng.thrustDirection = refT.InverseTransformDirection(info.dir);
				//The maximum torque this engine can deliver with current throttle
				eng.currentTorque = refT.InverseTransformDirection(Vector3.Cross(info.pos-wCoM, info.dir) * eng.finalThrust);
				if(is_steering)
				{
					totalTorque += eng.currentTorque;
					maxTorque += eng.finalThrust > 0? eng.currentTorque/eng.finalThrust*eng.maxThrust : Vector3.zero;
				}
			}
			//calculate efficiency
			if(is_steering)
			{
				demand   = -totalTorque;
				demand_m = totalTorque.magnitude;
				eK        = demand_m/maxTorque.magnitude;
			}
			foreach(var eng in engines)
			{
				if(!eng.isEnabled) continue;
				eng.efficiency = Vector3.Dot(eng.currentTorque, demand);
			}
			//scale efficiency to [-1; 0]
			var max_eff = engines.Max(e => e.efficiency);
			engines.ForEach(e => e.efficiency -= max_eff);
			max_eff = engines.Max(e => Mathf.Abs(e.efficiency));
			if(max_eff <= 0) return;
			eK = Mathf.Pow(eK, resposeCurve);
			var tK = verticalCutoff < maxCutoff? 1-Mathf.Clamp01((float)vessel.verticalSpeed/verticalCutoff) : 1;
			foreach(var eng in engines)
			{
				if(!eng.isEnabled) continue;
				eng.efficiency = eng.efficiency/max_eff * eK;
				var needed_percentage = Mathf.Clamp(100 * tK * (1 + eng.efficiency), 0f, 100f); 
				eng.thrustPercentage += (needed_percentage-eng.thrustPercentage)*save.GetActiveSensitivity();
			}
		}
		#endregion
	}

	public static class Extensions
	{
		#region Resources
		const string ElectricChargeName = "ElectricCharge";
		static PartResourceDefinition _electric_charge;

		public static PartResourceDefinition ElectricCharge
		{ 
			get
			{ 
				if(_electric_charge == null)
					_electric_charge = PartResourceLibrary.Instance.GetDefinition(ElectricChargeName);
				return _electric_charge;
			} 
		}
		#endregion

		#region Logging
		public static string Title(this Part p) { return p.partInfo != null? p.partInfo.title : p.name; }

		public static void Log(this Part p, string msg, params object[] args)
		{
			var vname = p.vessel == null? "" : p.vessel.vesselName;
			var _msg = string.Format("{0}.{1} [{2}]: {3}", 
			                         vname, p.name, p.flightID, msg);
			Utils.Log(_msg, args);
		}
		#endregion

		public static bool ElectricChargeAvailible(this Vessel v)
		{
			var ec = v.GetActiveResource(ElectricCharge);
			return ec != null && ec.amount > 0;
		}
	}

	public static class Utils
	{
		public static void writeToFile(String text)
		{
			using(var writer = new StreamWriter("DumpFile.txt", true))
				writer.Write(text + " \n");
		}

		public static string formatVector(Vector3 v)
		{ return string.Format("({0}, {1}, {2}); |v| = {3}", v.x, v.y, v.z, v.magnitude); }

		public static string formatVector(Vector3d v)
		{ return string.Format("({0}, {1}, {2}); |v| = {3}", v.x, v.y, v.z, v.magnitude); }

		public static void Log(string msg, params object[] args)
		{ 
			for(int i = 0; i < args.Length; i++) 
			{
				if(args[i] is Vector3) args[i] = formatVector((Vector3)args[i]);
				else if(args[i] is Vector3d) args[i] = formatVector((Vector3d)args[i]);
				else if(args[i] == null) args[i] = "null";
			}
			Debug.Log(string.Format("[TCA] "+msg, args)); 
		}
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
