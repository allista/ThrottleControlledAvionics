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

		#region variables
		Vessel vessel;
		protected Rect windowPos;
		protected Rect windowPosHelp;
		bool enginesCounted;
		List<EngineWrapper> engineTable;
		Vector3 demand;
		float minEfficiency;
		Vector3 mainThrustAxis;
		// as a pointing vector
		enum thrustDirections { up, mean, custom };

		thrustDirections direction;
		Vector2 customDirectionVector;
		// As (p, y)
		
		bool isActive;
		bool showAny;
		bool showEngines;
		bool contUpdate;
		bool detectStearingThrusters;
		bool reverseThrust;
		bool showHelp;
		bool showDirection;
		bool showDirectionSelector = true;
		bool showHUD = true;
		
		SaveFile save;
		GUIContent[] saveList;
		ComboBox saveListBox;
		GUIContent[] directionList;
		ComboBox directionListBox;
		Rect windowDirectionPos;
		Vector2 positionScrollViewEngines;


		const string ICON = "ThrottleControlledAvionics/textures/icon_button_off";

		IButton TCAToolbarButton;
		ApplicationLauncherButton TCAButton;
		#endregion

		/// <summary>
		/// Makes sure the toolbar icon resembles the present situation
		/// </summary>
		void UpdateToolbarIcon() 
		{
			if(TCAToolbarButton == null) return;
			if(isActive)
			{
				if(ElectricChargeAvailible(vessel))
				{
					TCAToolbarButton.TexturePath = reverseThrust ? 
						"ThrottleControlledAvionics/textures/icon_button_R" : 
						"ThrottleControlledAvionics/textures/icon_button_on";
				}
				else TCAToolbarButton.TexturePath = "ThrottleControlledAvionics/textures/icon_button_noCharge";
			}
			else TCAToolbarButton.TexturePath = "ThrottleControlledAvionics/textures/icon_button_off";
		}

		void OnGUIAppLauncherReady()
		{
			if (ApplicationLauncher.Ready)
			{
				TCAButton = ApplicationLauncher.Instance.AddModApplication(
					onAppLaunchToggleOn,
					onAppLaunchToggleOff,
					DummyVoid, DummyVoid, DummyVoid, DummyVoid,
					ApplicationLauncher.AppScenes.SPH|ApplicationLauncher.AppScenes.VAB|ApplicationLauncher.AppScenes.FLIGHT,
					GameDatabase.Instance.GetTexture(ICON, false));
			}
		}

		void onAppLaunchToggleOn() { showAny = true; }

		void onAppLaunchToggleOff() { showAny = false; }

		void DummyVoid() {}

		#region Unity interface
		public void Awake()
		{
			// open save
			save = new SaveFile();
			save.Load();
			// Initialising all variables
			vessel = null;
			windowPos = new Rect(50, 50, 400, 200);
			windowPosHelp = new Rect(500, 100, 400, 50);
			enginesCounted = false;
			engineTable = new List<EngineWrapper>();
			demand = new Vector3(0f, 0f, 0f);
			minEfficiency = 0.66f;
			mainThrustAxis = Vector3.down;
			direction = thrustDirections.up;

			isActive = false;
			showAny = false;
			showEngines = false;
			contUpdate = true;
			showHelp = false;
			detectStearingThrusters = false;
			reverseThrust = false;
			showDirection = false;

			if(ToolbarManager.ToolbarAvailable)
			{
				TCAToolbarButton = ToolbarManager.Instance.add ("Hangar", "HangarButton");
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

			directionList = new GUIContent[3];
			directionList[0] = new GUIContent("up");
			directionList[1] = new GUIContent("mean");
			directionList[2] = new GUIContent("custom");
			directionListBox = new ComboBox();
		}

		/// <summary>
		/// Will be called every phisics tick. We check wether the vessel is acquired, list all the engines and calculate the engine limiters.
		/// </summary>
		public void FixedUpdate()
		{
			if(vessel == null)
			{
				vessel = FlightGlobals.ActiveVessel;
				if(vessel == null) return;
				Debug.Log("TCA acquired vessel: " + vessel);
			}
            
			if(contUpdate || !enginesCounted)
			{
				enginesCounted = true;
				List<EngineWrapper> engines = ListEngines();
				CalculateEngines(engines);

				mainThrustAxis = vessel.GetReferenceTransformPart().orgRot * DetermineMainThrustAxis(engines);
				engineTable = engines;
			}
            
			if(isActive && ElectricChargeAvailible(vessel))
			{
				AjustDirection();
				SetThrottle();
			}
		}

		public void Update()
		{
			if(Input.GetKeyDown("y"))
				ActivateTCA(!isActive);
			//if (Input.GetKeyDown("b"))
			//{
			//    reverseThrust = !reverseThrust;
			//}
			if(Input.GetKeyDown(KeyCode.F2))
				showHUD = !showHUD;
		}

		public void OnGUI()
		{
			if(showAny && showHUD) drawGUI();
			UpdateToolbarIcon();
		}
		#endregion

		#region GUI
		void drawGUI()
		{
			windowPos = GUILayout.Window(1, windowPos, WindowGUI, "Throttle Controlled Avionics");
			if(showHelp)
				windowPosHelp = GUILayout.Window(2, windowPosHelp, windowHelp, "Instructions");
			if(showDirection) DrawDirection();
			if(direction == thrustDirections.custom && showDirectionSelector)
			{
				if(windowDirectionPos.xMin <= 1)
					windowDirectionPos = new Rect(windowPos.xMax, windowPos.yMin + 50, 120, 50);
				windowDirectionPos = GUILayout.Window(3, windowDirectionPos, WindowDirectionSelector, "Direction Selector");
			}
		}

		void WindowGUI(int windowID)
		{
			if(GUI.Button(new Rect(windowPos.width - 23f, 2f, 20f, 18f), "?"))
				showHelp = !showHelp;

			GUILayout.BeginVertical();

			if(!ElectricChargeAvailible(vessel))
				GUILayout.Label("WARNING! Electric charge has run out!");

			isActive = GUILayout.Toggle(isActive, "Toggle TCA");

			GUILayout.BeginHorizontal();
			GUILayout.Label("Settings: ", GUILayout.ExpandWidth(true));
			InsertDropdownboxSave();
			GUILayout.EndHorizontal();

			GUILayout.BeginHorizontal();
			GUILayout.Label("Sensitivity: ", GUILayout.ExpandWidth(true));
			GUILayout.Label("" + save.GetActiveSensitivity(), GUILayout.ExpandWidth(false));
			GUILayout.EndHorizontal();
			save.SetActiveSensitivity(Mathf.Exp(GUILayout.HorizontalSlider(Mathf.Log(save.GetActiveSensitivity()), -2.0f, 2.0f)));

			GUILayout.BeginHorizontal();
			GUILayout.Label("Mean thrust: ", GUILayout.ExpandWidth(true));
			GUILayout.Label("" + save.GetActiveMeanThrust(), GUILayout.ExpandWidth(false));
			GUILayout.EndHorizontal();
			save.SetActiveMeanThrust(GUILayout.HorizontalSlider(save.GetActiveMeanThrust(), 80f, 120f));

			detectStearingThrusters = GUILayout.Toggle(detectStearingThrusters, "Detect reaction control thrusters");
			if(detectStearingThrusters)
			{
				GUILayout.Label("Stearing Threshold: " + (Math.Acos(minEfficiency) * 180 / Math.PI) + "°");
				minEfficiency = Mathf.Cos(GUILayout.HorizontalSlider(Mathf.Acos(minEfficiency), 0f, Mathf.PI / 2));
				GUILayout.BeginHorizontal();
				GUILayout.Label("Direction");
				InsertDropdownboxDirection();
				GUILayout.EndHorizontal();
				GUILayout.BeginHorizontal();
				GUILayout.Label("");
				if(direction == thrustDirections.custom)
				{
					if(GUILayout.Button("Open/Close Direction Selector"))
					{
						showDirectionSelector = !showDirectionSelector;
					}
				}
				GUILayout.EndHorizontal();
				//showDirection = GUILayout.Toggle(showDirection, "Show direction");        //Is kinda broken
			}
			//contUpdate = GUILayout.Toggle(contUpdate, "Continuous engine update");
			//if (!contUpdate)
			//{
			//    if (GUILayout.Button("recalculate engine torque"))
			//    {
			//        enginesCounted = false;
			//    }
			//}

			showEngines = GUILayout.Toggle(showEngines, "show/hide engine information");
			if(showEngines)
			{
				GUILayout.BeginHorizontal();
				GUILayout.Label("torque demand: ", GUILayout.ExpandWidth(true));
				GUILayout.Label("" + demand, GUILayout.ExpandWidth(false));
				GUILayout.EndHorizontal();
				GUILayout.BeginHorizontal();
				GUILayout.Label("Thrust Axis: ", GUILayout.ExpandWidth(true));
				GUILayout.Label("" + mainThrustAxis, GUILayout.ExpandWidth(false));
				GUILayout.EndHorizontal();
				//GUILayout.BeginHorizontal();
				//GUILayout.Label("ReferenceTransformPart: ", GUILayout.ExpandWidth(true));
				//GUILayout.Label("" + vessel.GetReferenceTransformPart(), GUILayout.ExpandWidth(false));
				//GUILayout.EndHorizontal();
				//GUILayout.BeginHorizontal();
				//GUILayout.Label("ReferenceTransformPartID: ", GUILayout.ExpandWidth(true));
				//GUILayout.Label("" + vessel.referenceTransformId, GUILayout.ExpandWidth(false));
				//GUILayout.EndHorizontal();
				//GUILayout.BeginHorizontal();
				//GUILayout.Label("ReferenceTransformRot: ", GUILayout.ExpandWidth(true));
				//GUILayout.Label("" + (vessel.GetReferenceTransformPart().orgRot * Vector3.down).ToString(), GUILayout.ExpandWidth(false));
				//GUILayout.EndHorizontal();
				positionScrollViewEngines = GUILayout.BeginScrollView(positionScrollViewEngines, GUILayout.Height(300));
				foreach(EngineWrapper eng in engineTable)
				{
					GUILayout.Label(eng.getName() + " \n" +
					"steering vector: " + eng.steeringVector + "\n" +
					"thrustvector: " + eng.thrustVector + "\n" +
					"Steering: " + eng.steering +
					" Efficiancy: " + eng.efficiency + "\n" +
					"Thrust: " + eng.thrustPercentage
					);
				}
				GUILayout.EndScrollView();
			}
			else
				GUILayout.Label(".");

			GUILayout.EndVertical();
			GUI.DragWindow();
		}

		void WindowDirectionSelector(int windowID)
		{
			GUILayout.BeginVertical();
			GUILayout.Label("pitch: " + Mathf.RoundToInt(customDirectionVector.x) + "°");
			GUILayout.BeginHorizontal();
			for(int a = -90; a < 200; a += 90)
			{ if(GUILayout.Button(a + "°")) customDirectionVector.x = a; }
			GUILayout.EndHorizontal();
			customDirectionVector.x = GUILayout.HorizontalSlider(customDirectionVector.x, -180f, 180f);
			GUILayout.Label("yaw: " + Mathf.RoundToInt(customDirectionVector.y) + "°");
			GUILayout.BeginHorizontal();
			for(int a = -90; a < 200; a += 90)
			{ if(GUILayout.Button(a + "°")) customDirectionVector.y = a; }
			GUILayout.EndHorizontal();
			customDirectionVector.y = GUILayout.HorizontalSlider(customDirectionVector.y, -180f, 180f);
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

		/// <summary>
		/// Inserts a dropdownbox to select the desired direction
		/// </summary>
		void InsertDropdownboxDirection()
		{
			int i = directionListBox.GetSelectedItemIndex();
			i = directionListBox.List(directionList[i].text, directionList, "Box");
			switch(i)
			{
			case 0:
				direction = thrustDirections.up;
				break;
			case 1:
				direction = thrustDirections.mean;
				break;
			case 2:
				direction = thrustDirections.custom;
                    
				break;
			default:
				Debug.LogError("Invalid direction given");
				break;
                
			}
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

		/// <summary>
		/// This funtion will draw the direction specifid by the chosen direction. Momentarly it will also draw th fwdVector of the craft.
		/// </summary>
		void DrawDirection()
		{
			//Debug.Log("trying to draw line from " + vessel.findWorldCenterOfMass() +", in direction: " + mainThrustAxis);
			var dir = mainThrustAxis.normalized * 5;
			var whiteDiffuseMat = new Material(Shader.Find("Unlit/Texture"));

			var DirectionLine = new GameObject("direction line");
			var directionLine = DirectionLine.AddComponent<LineRenderer>();
			directionLine.SetWidth(-0.1f, 0.1f);
			directionLine.SetPosition(0, vessel.findWorldCenterOfMass());
			directionLine.SetPosition(1, vessel.findWorldCenterOfMass() + dir + vessel.GetFwdVector());
			directionLine.SetColors(Color.red, Color.red);
			directionLine.useWorldSpace = true;
			directionLine.material = whiteDiffuseMat;
			Destroy(DirectionLine.renderer.material, 0.04f);
			Destroy(DirectionLine, 0.04f);


			var FwdLine = new GameObject("fwd line");
			var fwdLine = FwdLine.AddComponent<LineRenderer>();
			fwdLine.SetWidth(-0.1f, 0.1f);
			fwdLine.SetPosition(0, vessel.findWorldCenterOfMass());
			fwdLine.SetPosition(1, vessel.findWorldCenterOfMass() + vessel.GetFwdVector() * 1000);
			fwdLine.SetColors(Color.green, Color.green);
			fwdLine.useWorldSpace = true;
			fwdLine.material = whiteDiffuseMat;
			Destroy(FwdLine.renderer.material, 0.1f);
			Destroy(FwdLine, 0.11f);
		}
		#endregion

		
		#region Engine Logic
		/// <summary>
		/// This function activates or deactivates TCA. On deactivation, all engines are reset to max
		/// </summary>
		/// <param name="state">Whether TCA should be active or not</param>
		void ActivateTCA(bool state)
		{
			isActive = state;
			if(!isActive) SetAllEnginesMax();
		}

		/// <summary>
		/// Checks if the given vessel has any electric charge left
		/// </summary>
		/// <param name="v">The vessel</param>
		/// <returns>True if there is charge left</returns>
		static bool ElectricChargeAvailible(Vessel v)
		{
			var ec = v.GetActiveResource(ElectricCharge);
			return ec != null && ec.amount > 0;
		}

		/// <summary>
		/// Lists all the engines of the active vessel. This function does not calculate any other thing.
		/// </summary>
		/// <returns></returns>
		List<EngineWrapper> ListEngines()
		{
			vessel = FlightGlobals.ActiveVessel;
			var engines = new List<EngineWrapper>();
			
			foreach(Part p in vessel.Parts)
			{
				EngineWrapper engine = null;
				foreach(PartModule module in p.Modules)
				{	
					if(module.moduleName == "ModuleEngines")
					{
						engine = new EngineWrapper((ModuleEngines)module);
						if(engine.isEnabled && !engine.throttleLocked)
						{
							engines.Add(engine);
						}
					}
					else
					{
						if(module.moduleName == "ModuleEnginesFX")
						{
							engine = new EngineWrapper((ModuleEnginesFX)module);
							if(engine.isEnabled && !engine.throttleLocked)
							{
								engines.Add(engine);
							}
						}
					}
				}				
			}
			return engines;
		}

		/// <summary>
		/// This function collects and calculates all the possibilities of every engine. It adds this information to the engineWrapper that are already listed.
		/// </summary>
		/// <param name="engines">A list of all the engine wrappers</param>
		void CalculateEngines(List<EngineWrapper> engines)
		{
			//Debug.Log ("building engine table");
			foreach(EngineWrapper eng in engines)
			{
				float thrust = eng.maxThrust;
				Vector3 com = eng.vessel.findLocalCenterOfMass();				
				Vector3 r = eng.part.orgPos - com;
				Vector3 F = thrust * (eng.part.orgRot * Vector3.down) / vessel.GetTotalMass();
				Vector3 torque = Vector3.Cross(r, F);
				//Debug.Log (" F" + F.ToString() + "org" + eng.part.orgRot.ToString() + "down" + (eng.part.orgRot*Vector3.down).ToString() + "res:" + Vector3.Dot (eng.part.orgRot * Vector3.down, Vector3.down)); //we assume we want to go up. RCS support comes later);
				eng.steeringVector = torque;                            // The torque this engine can deliver represented as a rotation axis
				eng.thrustVector = (eng.part.orgRot * Vector3.down);    // The direction this engine is fireing in, measured form the down vector, relative form the root part
			}
		}

		/// <summary>
		/// This function determans the main thrust axis of the vessel. This can be preset or
		/// calculated based on the average direction of all the possible thrust.
		/// </summary>
		/// <param name="engines">The list of all the engineWrappers</param>
		/// <returns>The normalized avarage of all thrust, or preset directions</returns>
		Vector3 DetermineMainThrustAxis(List<EngineWrapper> engines)
		{
			switch(direction)
			{
			case thrustDirections.mean:
				return engines.Aggregate(Vector3.zero, (v, e) => v + e.maxThrust * e.thrustVector).normalized;
			case thrustDirections.up:
				return Vector3.down;
			case thrustDirections.custom:
				Vector3 ret;
                        //x = cos(yaw)*cos(pitch);
                        //y = sin(yaw)*cos(pitch);
                        //z = sin(pitch);
				float pitchAngle = customDirectionVector.y * Mathf.Deg2Rad;
				float yawAngle = customDirectionVector.x * Mathf.Deg2Rad + Mathf.PI;
				ret.y = Mathf.Cos(yawAngle) * Mathf.Cos(pitchAngle);
				ret.z = Mathf.Sin(yawAngle) * Mathf.Cos(pitchAngle);
				ret.x = Mathf.Sin(pitchAngle);
				return ret;
			default:
				return Vector3.down;
			}
		}

		/// <summary>
		/// Reads the SAS
		/// </summary>
		void AjustDirection()
		{
			FlightCtrlState ctrls = vessel.ctrlState;

			if(ctrls.pitch == 1 && demand.x >= 1)
				demand.x += Time.fixedDeltaTime;
			else if(ctrls.pitch == -1 && demand.x <= -1)
				demand.x -= Time.fixedDeltaTime;
			else
				demand.x = ctrls.pitch;

			if(ctrls.roll == 1 && demand.y >= 1)
				demand.y += Time.fixedDeltaTime;
			else if(ctrls.roll == -1 && demand.y <= -1)
				demand.y -= Time.fixedDeltaTime;
			else
				demand.y = ctrls.roll;

			if(ctrls.yaw == 1 && demand.z >= 1)
				demand.z += Time.fixedDeltaTime;
			else if(ctrls.yaw == -1 && demand.z <= -1)
				demand.z -= Time.fixedDeltaTime;
			else
				demand.z = ctrls.yaw;

			//demand = new Vector3(ctrls.pitch, ctrls.roll, ctrls.yaw);
		}

		/// <summary>
		/// Here is the thrust modifier of every engine calculated. This is based on how much steering they provide and how efficient they can thrust along the main thruxt axis.
		/// </summary>
		void SetThrottle()
		{
			foreach(EngineWrapper eng in engineTable)
			{
				float steering = Vector3.Dot(Quaternion.AngleAxis(eng.steeringVector.magnitude, eng.steeringVector).eulerAngles,
					                         -demand); //The amount this engine can fulfill the torque demand
				steering = steering/100; //Because the numbers were to high
				eng.steering = steering;

				float efficiency;
				efficiency = reverseThrust ? 
					Vector3.Dot(eng.thrustVector, -mainThrustAxis) : 
					Vector3.Dot(eng.thrustVector, mainThrustAxis);
				//The amount this engine is pointing along the main thrust axis
				eng.efficiency = efficiency; 

				float throttle; //The amount of throttle this engine has to deliver
				if(detectStearingThrusters)
					throttle = efficiency > minEfficiency ? save.GetActiveMeanThrust() : 0f;
				else
					throttle = save.GetActiveMeanThrust();

				throttle += steering * save.GetActiveSensitivity(); //Adding the steering
				throttle = Mathf.Clamp(throttle, 0f, 100f);
				eng.thrustPercentage = throttle;
			}
		}
		
		void SetAllEnginesMax() { engineTable.ForEach(e => e.thrustPercentage = 100); }

		#endregion
		internal void OnDestroy() 
		{ 
			save.Save(); 
			GameEvents.onGUIApplicationLauncherReady.Remove(OnGUIAppLauncherReady);
			if(TCAButton != null)
				ApplicationLauncher.Instance.RemoveModApplication(TCAButton);
			if(TCAToolbarButton != null)
				TCAToolbarButton.Destroy(); 
		}
		
		// if other mods want to use the mod
		#region interface
		public bool IsActive() { return isActive; }
		public void SetActive(bool setting) { isActive = setting; }
		#endregion
	}

	public static class Utils
	{
		public static void writeToFile(String text)
		{
			using(var writer = new StreamWriter("DumpFile.txt", true))
				writer.Write(text + " \n");
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
