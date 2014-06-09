/* Name:    Throttle Controlled Avionics
 * Version: 1.4.0  (KSP 0.23.5)
 * 
 * Author: Quinten Feys & Willem van Vliet
 * License: BY: Attribution-ShareAlike 3.0 Unported (CC BY-SA 3.0): http://creativecommons.org/licenses/by-sa/3.0/
 * 
 * 
 * Disclaimer: You use this at your own risk; this is an alpha plugin for an alpha game; if your computer disintegrates, it's not my fault. :P
 * Much thanks to everybodey who's code I could look into
 * 
 * 
 * Toolbar integration powered by blizzy78's Toolbar plugin; used with permission
 *	http://forum.kerbalspaceprogram.com/threads/60863
 */


//TODO: fix null pointer when engines are destroyed
//TODO: fix GUI for breaking system

using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using UnityEngine;
using KSP.IO;
using System.IO;
using Toolbar;

namespace ThrottleControlledAvionics
{
	[KSPAddon(KSPAddon.Startup.Flight, false)]
	public class ThrottleControlledAvionics : MonoBehaviour
	{
		#region variables
		
		private Vessel vessel;
		protected Rect windowPos;
		protected Rect windowPosHelp;
		private bool enginesCounted;
		private List<EngineWrapper> engineTable;
		private Vector3 presentTorque;
        private Vector3 demand;           // Has to become local
        private float minEfficiency;
        private Vector3 mainThrustAxis;
        private enum thrustDirections { auto, up, forward };
        private thrustDirections direction;
		
		private bool isActive;
		private bool showAny;
		private bool showEngines;
		private bool contUpdate;
		private bool detectStearingThrusters;
		private bool reverseThrust;
		private bool showHelp;
        private bool showDirection;
		
		private IButton TCAButton;
		private SaveFile save;
        GUIContent[] saveList;
        private ComboBox saveListBox;
        GUIContent[] directionList;
        private ComboBox directionListBox;	
		
		#endregion
		
		#region Unity interface
		
		public void Awake()  
		{
			
			// open save
			save = new SaveFile();
			save.Load();
			// Initialising all variables
			vessel = FlightGlobals.ActiveVessel;
			windowPos = new Rect(50, 50, 400, 200);
			windowPosHelp = new Rect(500, 100, 400, 50);
			enginesCounted = false;
			engineTable = new List<EngineWrapper>();
			presentTorque = new Vector3(0f, 0f, 0f);
			demand = new Vector3(0f, 0f, 0f);
            minEfficiency = 0.66f;
            mainThrustAxis = Vector3.down;
            direction = thrustDirections.auto;

			
			isActive = false;
			showAny = false;
			showEngines = false;
			contUpdate = true;
			showHelp = false;
			detectStearingThrusters = false;
			reverseThrust = false;
            showDirection = false;
			
			TCAButton = ToolbarManager.Instance.add("TCA", "TCAButton");
			TCAButton.TexturePath = "ThrottleControlledAvionics/textures/icon_button_off";
			TCAButton.ToolTip = "Throttle Controlled Avionics";
			TCAButton.Visibility = new GameScenesVisibility(GameScenes.FLIGHT);
			TCAButton.Visible = true;
			TCAButton.OnClick += (e) =>
			{
				showAny = !showAny;
			};
			
			saveList = new GUIContent[3];
			saveList[0] = new GUIContent(save.GetName(0));
			saveList[1] = new GUIContent(save.GetName(1));
			saveList[2] = new GUIContent(save.GetName(2));
			saveListBox = new ComboBox();

            directionList = new GUIContent[3];
            directionList[0] = new GUIContent("auto");
            directionList[1] = new GUIContent("up");
            directionList[2] = new GUIContent("forward");
            directionListBox = new ComboBox();
		}
		
		public void FixedUpdate()
		{
			//Will be called continuesly
			if (contUpdate || !enginesCounted)
			{
				enginesCounted = true;
				List<EngineWrapper> engines = CountEngines();
				BuildEngineTable(engines);
			}

            if (isActive && vessel.GetActiveResource(new PartResourceDefinition("ElectricCharge")).amount != 0)
			{
				AjustDirection();
				SetThrottle();
			}			
		}
		
		public void Update() {
			if (Input.GetKeyDown ("y")) {
				ActivateTCA(!isActive);
			}
			if (Input.GetKeyDown ("b")) {
				reverseThrust = !reverseThrust;
			}
            if (Input.GetKeyDown(KeyCode.F2))
            {
                showAny = !showAny;
            }
		}
		
		public void OnGUI()
		{
			if (showAny) drawGUI();
		}
		
		#endregion
		
		#region GUI
		
		private void drawGUI()
		{
			windowPos = GUILayout.Window(1, windowPos, WindowGUI, "Throttle Controlled Avionics");
			if (showHelp)
			{
				windowPosHelp = GUILayout.Window(2, windowPosHelp, windowHelp, "Instructions");
			}
            if (showDirection)
            {
                DrawDirection();
            }
            UpdateToolbarIcon();
		}
		
		private void WindowGUI(int windowID)
		{
            if (GUI.Button(new Rect(windowPos.width - 23f, 2f, 20f, 18f), "?"))
            {
                showHelp = !showHelp;
            }
			
			GUILayout.BeginVertical();

            isActive = GUILayout.Toggle(isActive, "Toggle TCA");
			
			GUILayout.BeginHorizontal();
			GUILayout.Label("Settings: ", GUILayout.ExpandWidth(true));
			InsertDropdownboxSave();
			GUILayout.EndHorizontal();

			GUILayout.BeginHorizontal();
			GUILayout.Label("Sensitivity: ", GUILayout.ExpandWidth(true));
			GUILayout.Label("" + save.GetActiveSensitivity(), GUILayout.ExpandWidth(false));
			GUILayout.EndHorizontal();
			save.SetActiveSensitivity((float)Math.Exp(GUILayout.HorizontalSlider((float)Math.Log(save.GetActiveSensitivity()), 0.0f, 7.0f)));

			GUILayout.BeginHorizontal();
			GUILayout.Label("Mean thrust: ", GUILayout.ExpandWidth(true));
			GUILayout.Label("" + save.GetActiveMeanThrust(), GUILayout.ExpandWidth(false));
			GUILayout.EndHorizontal();
			save.SetActiveMeanThrust(GUILayout.HorizontalSlider(save.GetActiveMeanThrust(), 80f, 120f));
			
			detectStearingThrusters = GUILayout.Toggle(detectStearingThrusters, "Detect reaction control thrusters");
            if (detectStearingThrusters)
            {
                GUILayout.Label("Stearing Treshold: " + (Math.Acos(minEfficiency)*180/Math.PI) +"°");
                minEfficiency = (float)Math.Cos(GUILayout.HorizontalSlider((float)Math.Acos(minEfficiency), 0f, (float)(Math.PI/2)));
                GUILayout.BeginHorizontal();
                GUILayout.Label("Direction");
                InsertDropdownboxDirection();
                GUILayout.EndHorizontal();
                showDirection = GUILayout.Toggle(showDirection, "Show direction");
            }
			contUpdate = GUILayout.Toggle(contUpdate, "Continuous engine update");
			if (!contUpdate)
			{
				if (GUILayout.Button("recalculate engine torque"))
				{
					enginesCounted = false;
				}
			}

            showEngines = GUILayout.Toggle(showEngines, "show/hide engine information");
			if (showEngines)
            {
                GUILayout.BeginHorizontal();
                GUILayout.Label("torque demand: ", GUILayout.ExpandWidth(true));
                GUILayout.Label("" + demand.ToString(), GUILayout.ExpandWidth(false));
                GUILayout.EndHorizontal();
				foreach (EngineWrapper eng in engineTable)
				{
					GUILayout.Label(eng.getName() + " \r\n" +
					                eng.steeringVector.ToString() + " " + eng.thrustVector.ToString() + "\r\n" + 
					                Vector3.Dot(eng.steeringVector, demand).ToString() + " " + Vector3.Dot(eng.thrustVector, Vector3.down).ToString() + " " + eng.thrustPercentage.ToString()
					                );
				}
			}
			
			GUILayout.EndVertical();
			GUI.DragWindow();
			
			
		}
		
        /// <summary>
        /// Makes sure the toolbar icon resembles the present situation
        /// </summary>
		private void UpdateToolbarIcon() {
            if (isActive)
            {
                if(reverseThrust)
                    TCAButton.TexturePath = "ThrottleControlledAvionics/textures/icon_button_R";
                else
                    TCAButton.TexturePath = "ThrottleControlledAvionics/textures/icon_button_on";
            }
            else
                TCAButton.TexturePath = "ThrottleControlledAvionics/textures/icon_button_off";
		}
		
		/// <summary>
		/// Inserts a dropdownbox to select the deired saved settings from
		/// </summary>
		private void InsertDropdownboxSave()
		{
			int i = saveListBox.GetSelectedItemIndex();
			i = saveListBox.List( saveList[i].text, saveList, "Box");
			save.SetActiveSave(i);
		}

        /// <summary>
        /// Inserts a dropdownbox to select the desired direction
        /// </summary>
        private void InsertDropdownboxDirection()
        {
            int i = directionListBox.GetSelectedItemIndex();
            i = directionListBox.List(directionList[i].text, directionList, "Box");
            switch(i)
            {
                case 0:
                    direction = thrustDirections.auto;
                    break;
                case 1:
                    direction = thrustDirections.up;
                    break;
                case 2:
                    direction = thrustDirections.forward;
                    break;
                default:
                    Debug.LogError("Invalid direction given");
                    break;
                
            }
        }
		
		
		private void windowHelp(int windowID)
		{
			String instructions = "Welcome to the instructions manual.\nFor simple use:\n\t 1)Put TCA on,\n\t 2)Put SAS on, \n\t 3) Launch \n\n"+
				"For more advanced use:\n\t -Thrust factor determens the agressivness of TCA. A high value will react verry fast and abrupt, a low value will be a lot smoother"+
					", but might not be as fast.\n\t -Mean thrust is the virtual average thrust. A value below 100 means that the engines will be stadard throttled down and"+
					"will correct by both throttling up and down. A value above 100 means that they will wait untill the deviation becomes rather big. This might be good if"+
					"you think that the standard avionics is strong enough to handel the load. \n\t -3 different configurations can be saved at the same time. Saving happens automaticly."+
					"\n\nWarning: \n\t -TCA will compute everything based on the orientation of the root part. This means that if your command module has a different orientation "+
					"than the root part, errors WILL occur. As long as TCA is active, make sure your oriantation keeps the same. If you know how to solve this problem, please contact"+
					" me in the forums. \n\t -TCA assumes that the engine bells are allignt allong the y-axis of the engine parts. This is the standard orientation for most of them. "+
					"If you possess engines that can change the orientation of their engine bells, like those form 'Ferram aerospace' please make sure that they are alligned allong"+
					" the right axis before enabeling TCA. \n\t -Note that while jet engines can be throttled, they have some latancy, what might controling VTOL's based on these"+
					" rather tricky. \n\t SRB's can't be controlled. That's because they're SRB's";
			GUILayout.Label(instructions, GUILayout.MaxWidth(400));
			GUI.DragWindow();
		}

        /// <summary>
        /// This funtion will draw the direction specifid by the chosen direction. Momentarly it will also draw th fwdVector of the craft.
        /// </summary>
        private void DrawDirection()
        {
            Debug.Log("trying to draw line from " + vessel.findWorldCenterOfMass() +", in direction: " + mainThrustAxis);
            Vector3 direction = mainThrustAxis.normalized * 5;
            Material whiteDiffuseMat = new Material(Shader.Find("Unlit/Texture"));

            GameObject DirectionLine = new GameObject("direction line");
            LineRenderer directionLine = DirectionLine.AddComponent<LineRenderer>();
            directionLine.SetWidth(-0.1f, 0.1f);
            directionLine.SetPosition(0, vessel.findWorldCenterOfMass());
            directionLine.SetPosition(1, vessel.findWorldCenterOfMass() + direction);
            directionLine.SetColors(Color.red, Color.red);
            directionLine.useWorldSpace = true;
            directionLine.material = whiteDiffuseMat;
            Destroy(DirectionLine.renderer.material,0.04f);
            Destroy(DirectionLine,0.04f);


            GameObject FwdLine = new GameObject("fwd line");
            LineRenderer fwdLine = FwdLine.AddComponent<LineRenderer>();
            fwdLine.SetWidth(-0.1f, 0.1f);
            fwdLine.SetPosition(0, vessel.findWorldCenterOfMass());
            fwdLine.SetPosition(1, vessel.findWorldCenterOfMass() + vessel.GetFwdVector());
            fwdLine.SetColors(Color.green, Color.green);
            fwdLine.useWorldSpace = true;
            fwdLine.material = whiteDiffuseMat;
            Destroy(FwdLine.renderer.material,0.1f);
            Destroy(FwdLine,0.11f);
            
        }
		#endregion
		
		
		#region Engine Logic

        /// <summary>
        /// This function activates or deactivates TCA. On deactivation, all engines are reset to max
        /// </summary>
        /// <param name="state">Whether TCA should be active or not</param>
        private void ActivateTCA(bool state)
        {
            isActive = state;
            if (!isActive)
                SetAllEnginesMax();
        }


		private List<EngineWrapper> CountEngines()
		{
			vessel = FlightGlobals.ActiveVessel;
			List<EngineWrapper> engines = new List<EngineWrapper>();
			
			foreach (Part p in vessel.Parts)
			{
				EngineWrapper engine = null;
				foreach (PartModule module in p.Modules) {	
					if(module.moduleName == "ModuleEngines"){
						engine = new EngineWrapper((ModuleEngines) module);
						if (engine.isEnabled && !engine.throttleLocked){
							engines.Add(engine);
						}
					} else {
						if (module.moduleName == "ModuleEnginesFX")
						{
							engine = new EngineWrapper((ModuleEnginesFX) module);
							if (engine.isEnabled && !engine.throttleLocked){
								engines.Add(engine);
							}
						}
					}
				}				
			}
			return engines;
		}
		
		/// <summary>
		/// This function collects and calculates all the possibilitys of every engine.
		/// </summary>
		/// <param name="engines">A list of all the engine wrappers</param>
		private void BuildEngineTable(List<EngineWrapper> engines)
		{
            //Debug.Log ("building engine table");
			engineTable = new List<EngineWrapper>();
			foreach (EngineWrapper eng in engines)
			{
				float thrust = eng.maxThrust;
				Vector3 com = eng.vessel.findLocalCenterOfMass();				
				Vector3 r = eng.part.orgPos - com;
				Vector3 F = thrust * (eng.part.orgRot * Vector3.down) / vessel.GetTotalMass();
				Vector3 torque = Vector3.Cross(r, F);
				//Debug.Log (" F" + F.ToString() + "org" + eng.part.orgRot.ToString() + "down" + (eng.part.orgRot*Vector3.down).ToString() + "res:" + Vector3.Dot (eng.part.orgRot * Vector3.down, Vector3.down)); //we assume we want to go up. RCS support comes later);
				eng.steeringVector = torque;                            // The torque this engine can deliver
				eng.thrustVector = (eng.part.orgRot * Vector3.down);    // The direction this engine is fireing in, measured form the down vector
				engineTable.Add(eng);
			}

            mainThrustAxis = DetermineMainThrustAxis(engineTable);
		}


        /// <summary>
        /// This function determans the main thrust axis of the vessel. This can be preset or
        /// calculated based on the average direction of all the possible thrust.
        /// </summary>
        /// <param name="engineTable">The list of all the engineWrappers</param>
        /// <returns>The normalized avarage of all thrust, or preset directions</returns>
        private Vector3 DetermineMainThrustAxis(List<EngineWrapper> engineTable)
        {
            switch (direction)
            {
                case thrustDirections.auto:
                    Vector3 sum = new Vector3(0, 0, 0);
                    foreach (EngineWrapper eng in engineTable)
                    {
                        sum += eng.maxThrust * (eng.part.orgRot * Vector3.down);
                    }
                    return sum.normalized;
                case thrustDirections.up:
                    return Vector3.down;
                case thrustDirections.forward:
                    return Vector3.back;
                default:
                    return Vector3.down;
            }
        }
		
		/// <summary>
		/// Reads the SAS
		/// </summary>
		private void AjustDirection()
		{
			VesselSAS sas = vessel.vesselSAS;
			FlightCtrlState ctrls = vessel.ctrlState;
			demand = new Vector3(ctrls.pitch, ctrls.roll, ctrls.yaw);
		}
		
		/// <summary>
		/// Here is the thrust modifier of every engine calculated. This is based on how much steering they provide and how efficient they can thrust along the main thruxt axis.
		/// </summary>
		private void SetThrottle()
		{
			foreach (EngineWrapper eng in engineTable)
			{
				
				float steering = Vector3.Dot(eng.steeringVector, demand);  // The amount this engine can fulfill the torque demand
				float efficiency;                                          // The amount this engine is pointing along the main thrust axis
				if(reverseThrust){
					efficiency = Vector3.Dot(eng.thrustVector, -mainThrustAxis); //inverse for reverse thrust!
				} else {
					efficiency = Vector3.Dot(eng.thrustVector, mainThrustAxis);
				}
				float throttle = 0f;                                          // The amount of throttle this engine has to deliver
				//a = a * save.GetActiveThrustfactor();
				//a += save.GetActiveMeanThrust() - 100;
				//if (a > 0) a = 0;
				//if (a < -100) a = -100;
				
				
				if(detectStearingThrusters) // This means that inefficient enigines will only be used for steering
                {
					if(efficiency > minEfficiency) { //only take engines that point the right way
						throttle = save.GetActiveMeanThrust();
					} else {
						throttle = 0f;
					}
					throttle += steering * save.GetActiveSensitivity();//we assume we want to go up. RCS support comes later
					throttle = Mathf.Clamp(throttle, 0f, 100f);
					eng.thrustPercentage = throttle;
				} 
                else 
                {
					throttle = 1.0f * save.GetActiveMeanThrust() + steering * save.GetActiveSensitivity();//we assume we want to go up. RCS support comes later
					throttle = Mathf.Clamp(throttle, 0f, 100f);
					eng.thrustPercentage = throttle; //100 means we assume they all help with ups thrust and thus should be fully on per default.
				}
				
				/*
				if(detectReverseThrusters) {
					float b = Vector3.Dot (i.Key.thrustVector, Vector3.down); //we assume we want to go up. RCS support comes later
					i.Key.thrustPercentage = b * save.GetActiveMeanThrust() + a * save.GetActiveThrustfactor;
				} else {
					a = a * save.GetActiveThrustfactor();
					a += save.GetActiveMeanThrust() - 100;
					i.Key.thrustPercentage = 100 + a; //100 means we assume they all help with ups thrust and thus should be fully on per default.
				}
				*/
			}
		}
		
		
		private void SetAllEnginesMax()
		{
			foreach (EngineWrapper eng in engineTable)
			{
				eng.thrustPercentage = 100;
			}
		}
		
		#endregion
		
		internal void OnDestroy()
		{
			save.Save();
			TCAButton.Destroy();    // cleaning up
		}
		
		
		// if other mods want to use the mod
		#region interface
		
		public bool IsActive()
		{
			return isActive;
		}
		
		public void SetActive(bool setting)
		{
			isActive = setting;
		}
		
		#endregion
		
		// used for debugging
		private void writeToFile(String text)
		{
			using (StreamWriter writer = new System.IO.StreamWriter("DumpFile.txt",true))
			{
				writer.Write(text+" \r\n");
			}
		}
		
		
		
	}
	
	public class SaveFile
	{
		int activeSave;
		object[,] saves;
		
		public SaveFile()
		{
			saves = new object[3,3];
			activeSave = 0;
		}
		
		public void Save()
		{
			String text = (String)saves[0, 0] + "\r\n" + (float)saves[0, 1] + "\r\n" + (float)saves[0, 2] + "\r\n\r\n" +
				(String)saves[1, 0] + "\r\n" + (float)saves[1, 1] + "\r\n" + (float)saves[1, 2] + "\r\n\r\n" +
					(String)saves[2, 0] + "\r\n" + (float)saves[2, 1] + "\r\n" + (float)saves[2, 2] + "\r\n\r\n";
			using (StreamWriter writer = new System.IO.StreamWriter("GameData/ThrottleControlledAvionics/TCASave.txt", false))
			{
				writer.Write(text);
			}
		}
		
		public void Load()
		{
			try
			{
				using (StreamReader reader = new System.IO.StreamReader("GameData/ThrottleControlledAvionics/TCASave.txt"))
				{
					for (int i = 0; i < 3; i++)
					{
						saves[i, 0] = reader.ReadLine();
						saves[i, 1] = float.Parse(reader.ReadLine());
						saves[i, 2] = float.Parse(reader.ReadLine());
						reader.ReadLine();
					}
				}
			}
			catch (Exception e)            
			{                
				if (e is System.IO.FileNotFoundException || e is System.IO.IsolatedStorage.IsolatedStorageException)
				{
					SetDefault();
				}
				else{
					writeToFile("We found a serous problem during Load:" + e);
					throw;
				}
			}
			
		}
		
		private void SetDefault()
		{
			for (int i = 0; i < 3; i++)
			{
				writeToFile("i = " + i);
				saves[i, 0] = "Setting " + i;
				saves[i, 1] = 40f;
				saves[i, 2] = 100f;
			}
		}
		
		#region setters
		public void SetActiveSave(int i)
		{
			activeSave = i;
		}
		
		public void SetActiveName(string name)
		{
			saves[activeSave, 0] = name;
		}
		
		public void SetActiveSensitivity(float tf)
		{
			saves[activeSave, 1] = tf;
		}
		
		public void SetActiveMeanThrust(float mt)
		{
			saves[activeSave, 2] = mt;
		}
		#endregion
		
		#region getters
		public string GetActiveName()
		{
			return (string)saves[activeSave, 0];
		}
		
		public string GetName(int i)
		{
			return (string)saves[i,0];
		}
		
		public float GetActiveSensitivity()
		{
			return (float)saves[activeSave, 1];
		}
		
		public float GetActiveMeanThrust()
		{
			return (float)saves[activeSave, 2];
		}
		#endregion
		
		// used for debugging
		private void writeToFile(String text)
		{
			using (StreamWriter writer = new System.IO.StreamWriter("DumpFile.txt", true))
			{
				writer.Write(text + " \r\n");
			}
		}
	}
}
