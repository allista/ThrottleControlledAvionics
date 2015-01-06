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
		SaveFile save; //deprecated

		Vessel vessel;
		readonly List<EngineWrapper> engines = new List<EngineWrapper>();
		bool haveEC = true;
		Vector3 wCoM;
		Transform refT;
		Vector3 steering = Vector3.zero;
		readonly float MAX_STEERING = Mathf.Sqrt(3);

		float steeringThreshold = 0.01f; //too small steering vector may cause oscilations
		float responseSpeed     = 0.25f; //speed at which the value of a thrust limiter is changed towards the needed value (part-of-difference/FixedTick)
		float verticalCutoff    = 1f;    //max. positive vertical speed m/s (configurable)
		const float maxCutoff   = 10f;   //max. positive vertical speed m/s (configuration limit)
		float resposeCurve      = 0.3f;  //coefficient of non-linearity of efficiency response to partial steering (1 means response is linear, 0 means no response)

		bool isActive;
		#endregion

		#region GUI
		bool showAny;
		bool showEngines;
		bool showHelp;
		bool showHUD = true;

		GUIContent[] saveList;
		ComboBox saveListBox;
		Vector2 positionScrollViewEngines;

		protected Rect windowPos = new Rect(50, 50, 450, 200);
		protected Rect windowPosHelp = new Rect(500, 100, 400, 50);
		const string ICON = "ThrottleControlledAvionics/Icons/icon_button_off";

		IButton TCAToolbarButton;
		ApplicationLauncherButton TCAButton;
		#endregion

		#region GUI methods
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

		public void OnGUI()
		{
			drawGUI();
			UpdateToolbarIcon();
		}

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
			GUILayout.Label("Response Curve: ", GUILayout.ExpandWidth(false)); //redundant?
			GUILayout.Label(resposeCurve.ToString("F2"), GUILayout.ExpandWidth(false));
			resposeCurve = GUILayout.HorizontalSlider(resposeCurve, 0f, 1f);
			GUILayout.EndHorizontal();

			GUILayout.BeginHorizontal();
			GUILayout.Label("Steering Threshold: ", GUILayout.ExpandWidth(false)); //redundant?
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

		void InsertDropdownboxSave()
		{
			int i = saveListBox.GetSelectedItemIndex();
			i = saveListBox.List(saveList[i].text, saveList, "Box");
			save.SetActiveSave(i);
		}

		//TODO: rewrite the Help text
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
		void onVessel(Vessel vsl)
		{ if(vessel == null || vsl == vessel) UpdateEnginesList(); }

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

		void SetThrottle()
		{
			var demand      = steering;
			var demand_m    = demand.magnitude;
			var eK          = demand_m/MAX_STEERING;
			var is_steering = demand_m < steeringThreshold;
			//calculate thrust
			var totalTorque = Vector3.zero;
			var maxTorque   = Vector3.zero;
			foreach(var eng in engines)
			{
				if(!eng.isEnabled) continue;
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
			//non-linear coefficient works better
			eK = Mathf.Pow(eK, resposeCurve); 
			//thrust coefficient for vertical speed limit
			var upV = Vector3d.Dot(vessel.srf_velocity, (wCoM - vessel.mainBody.position).normalized); //from MechJeb
			var tK = verticalCutoff < maxCutoff? 1-Mathf.Clamp01((float)upV/verticalCutoff) : 1;
			//set thrust limiters
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
