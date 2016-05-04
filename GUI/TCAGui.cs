/* The GUI for ThrottleControlledAvionics.
 * Authors: Quinten Feys, Willem van Vliet, Allis Tauri
 * License: BY: Attribution-ShareAlike 3.0 Unported (CC BY-SA 3.0): http://creativecommons.org/licenses/by-sa/3.0/
 */

using System;
using System.Linq;
using System.Collections.Generic;
using System.Reflection;
using UnityEngine;

namespace ThrottleControlledAvionics
{
	public partial class ThrottleControlledAvionics
	{
		#region GUI Parameters
		const string LockName = "TCAGui";
		public const int ControlsWidth = 650, ControlsHeight = 100, LineHeight = 35;

		static NamedConfig selected_config;
		static string config_name = string.Empty;
		static readonly DropDownList named_configs = new DropDownList();

		public static KeyCode TCA_Key = KeyCode.Y;
		static bool selecting_key;
		static bool adv_options;

		static List<TCAPart> parts;

		public static string StatusMessage;
		#endregion

		#if DEBUG
		public static string DebugMessage;
		#endif

		#region ControlPanels
		public static AttitudePanel AttitudeControls;
		public static InOrbitPanel InOrbitControls;
		public static OnPlanetPanel OnPlanetControls;
		public static NavigationPanel NavigationControls;
		public static MacrosPanel MacroControls;
		public static SquadPanel SquadControls;
		static List<ControlPanel> AllPanels = new List<ControlPanel>();
		static List<FieldInfo> AllPanelFields = typeof(ThrottleControlledAvionics)
			.GetFields(BindingFlags.Static|BindingFlags.Public)
			.Where(fi => fi.FieldType.IsSubclassOf(typeof(ControlPanel))).ToList();
		#endregion

		#region Configs Selector
		static void update_configs()
		{ 
			var configs = TCAScenario.NamedConfigs.Keys.ToList();
			var first = named_configs.Items.Count == 0;
			configs.Add(string.Empty); named_configs.Items = configs; 
			if(first) named_configs.SelectItem(configs.Count-1);
		}

		static void SelectConfig_start() 
		{ 
			named_configs.styleListBox  = Styles.list_box;
			named_configs.styleListItem = Styles.list_item;
			named_configs.windowRect    = MainWindow;
			if(TCAScenario.NamedConfigs.Count > 0)
				named_configs.DrawBlockingSelector(); 
		}

		static void SelectConfig()
		{
			if(TCAScenario.NamedConfigs.Count == 0)
				GUILayout.Label("[Nothing Saved]", GUILayout.ExpandWidth(true));
			else
			{
				named_configs.DrawButton();
				var new_config = TCAScenario.GetConfig(named_configs.SelectedIndex);
				if(new_config != selected_config)
				{
					selected_config = new_config;
					config_name = selected_config != null? selected_config.Name : string.Empty;
				}
			}
		}

		static void SelectConfig_end()
		{
			if(TCAScenario.NamedConfigs.Count == 0) return;
			named_configs.DrawDropDown();
			named_configs.CloseOnOutsideClick();
		}
		#endregion

		protected override void DrawMainWindow(int windowID)
		{
			//help button
			if(GUI.Button(new Rect(MainWindow.width - 23f, 2f, 20f, 18f), 
			                  new GUIContent("?", "Help"))) TCAManual.Toggle();
			if(TCA.Controllable)
			{
				//options button
				if(GUI.Button(new Rect(2f, 2f, 70f, 18f), 
				              new GUIContent("advanced", "Advanced configuration"), 
				              adv_options? Styles.enabled_button : Styles.normal_button)) 
					adv_options = !adv_options;
				GUILayout.BeginVertical();
				GUILayout.BeginHorizontal();
				//tca toggle
				if(GUILayout.Button("Enabled", 
				                    CFG.Enabled? Styles.enabled_button : Styles.inactive_button,
				                    GUILayout.Width(70)))
				{
					if(SQD == null) TCA.ToggleTCA();
					else SQD.Apply(tca => tca.ToggleTCA());
				}
				//squad mode switch
				SquadControls.Draw();
				GUILayout.FlexibleSpace();
				StatusString();
				GUILayout.EndHorizontal();
				SelectConfig_start();
				AdvancedOptions();
				AttitudeControls.Draw();
				InOrbitControls.Draw();
				OnPlanetControls.Draw();
				NavigationControls.Draw();
				MacroControls.Draw();
				NavigationControls.WaypointList();
				EnginesControl();
				#if DEBUG
				DebugInfo();
//				EnginesInfo();
				#endif
				if(!string.IsNullOrEmpty(StatusMessage) && 
				   GUILayout.Button(new GUIContent(StatusMessage, "Click to dismiss"), 
				                    Styles.boxed_label, GUILayout.ExpandWidth(true)))
					StatusMessage = "";
				SelectConfig_end();
				GUILayout.EndVertical();
			}
			else GUILayout.Label("Vessel is Uncontrollable", Styles.label, GUILayout.ExpandWidth(true), GUILayout.ExpandHeight(true));
			base.DrawMainWindow(windowID);
		}

		static void StatusString()
		{
			var state = "Disabled";
			var style = Styles.grey;
			if(TCA.IsStateSet(TCAState.Enabled))
			{
				if(TCA.IsStateSet(TCAState.ObstacleAhead))
				{ state = "Obstacle On Course"; style = Styles.red; }
				else if(TCA.IsStateSet(TCAState.GroundCollision))
				{ state = "Ground Collision Possible"; style = Styles.red; }
				else if(TCA.IsStateSet(TCAState.LoosingAltitude))
				{ state = "Loosing Altitude"; style = Styles.red; }
                else if(TCA.IsStateSet(TCAState.Unoptimized))
				{ state = "Engines Unoptimized"; style = Styles.yellow; }
				else if(TCA.IsStateSet(TCAState.Ascending))
				{ state = "Ascending"; style = Styles.yellow; }
				else if(TCA.IsStateSet(TCAState.VTOLAssist))
				{ state = "VTOL Assist On"; style = Styles.yellow; }
				else if(TCA.IsStateSet(TCAState.StabilizeFlight))
				{ state = "Stabilizing Flight"; style = Styles.yellow; }
				else if(TCA.IsStateSet(TCAState.Landing))
				{ state = "Landing..."; style = Styles.green; }
				else if(TCA.IsStateSet(TCAState.CheckingSite))
				{ state = "Checking Landing Site"; style = Styles.yellow; }
				else if(TCA.IsStateSet(TCAState.Searching))
				{ state = "Searching For Landing Site"; style = Styles.yellow; }
				else if(TCA.IsStateSet(TCAState.Scanning))
				{ state = string.Format("Scanning Surface {0:P0}", VSL.Info.ScanningProgress); style = Styles.yellow; }
				else if(TCA.IsStateSet(TCAState.AltitudeControl))
				{ state = "Altitude Control"; style = Styles.green; }
				else if(TCA.IsStateSet(TCAState.VerticalSpeedControl))
				{ state = "Vertical Speed Control"; style = Styles.green; }
				else if(TCA.State == TCAState.Nominal)
				{ state = "Systems Nominal"; style = Styles.green; }
				else if(TCA.State == TCAState.NoActiveEngines)
				{ state = "No Active Engines"; style = Styles.yellow; }
				else if(TCA.State == TCAState.NoEC)
				{ state = "No Electric Charge"; style = Styles.red; }
				else //this should never happen
				{ state = "Unknown State"; style = Styles.magenta; }
			}
			GUILayout.Label(state, style, GUILayout.ExpandWidth(false));
		}

		static void AdvancedOptions()
		{
			if(!adv_options) return;
			GUILayout.BeginVertical(Styles.white);
			GUILayout.Label(TCATitle, Styles.label, GUILayout.ExpandWidth(true));
			if(GUILayout.Button("Reload TCA Settings", Styles.active_button, GUILayout.ExpandWidth(true))) 
			{
				TCAScenario.LoadGlobals();
				Styles.ConfigureButtons();
				TCA.OnReloadGlobals();
			}
			PartsInfo();
			//change key binding
			GUILayout.BeginHorizontal();
			GUILayout.Label("Press to change TCA hotkey:", GUILayout.ExpandWidth(false));
			if(GUILayout.Button(selecting_key? new GUIContent("?", "Choose new TCA hotkey") : 
			                    new GUIContent(TCA_Key.ToString(), "Select TCA Hotkey"), 
			                    selecting_key? Styles.enabled_button : Styles.active_button, 
			                    GUILayout.Width(40)))
			{ selecting_key = true; ScreenMessages.PostScreenMessage("Enter new key to toggle TCA", 5, ScreenMessageStyle.UPPER_CENTER); }
			GUILayout.EndHorizontal();
			GUILayout.BeginHorizontal();
			if(VTOL != null) 
			{
				if(Utils.ButtonSwitch("VTOL Mode", CFG.CTRL[ControlMode.VTOL], 
			                          "Keyboard controls thrust direction instead of torque", GUILayout.ExpandWidth(true)))
					CFG.CTRL.XToggle(ControlMode.VTOL);
			}
			if(VLA != null) Utils.ButtonSwitch("VTOL Assist", ref CFG.VTOLAssistON, 
			                                   "Assist with vertical takeoff and landing", GUILayout.ExpandWidth(true));
			if(STB != null) Utils.ButtonSwitch("Flight Stabilizer", ref CFG.StabilizeFlight, 
			                                   "Try to stabilize flight if spinning uncontrollably", GUILayout.ExpandWidth(true));
			GUILayout.EndHorizontal();
			if(THR != null)
			{
				GUILayout.BeginHorizontal();
				CFG.VSControlSensitivity = Utils.FloatSlider("Sensitivity of throttle controls", CFG.VSControlSensitivity, 0.001f, 0.05f, "P2");
				GUILayout.EndHorizontal();
			}
			ControllerProperties();
			ConfigsGUI();
			GUILayout.EndVertical();
		}

		static void ConfigsGUI()
		{
			GUILayout.BeginVertical();
			GUILayout.Label("Manage named configurations", Styles.label, GUILayout.ExpandWidth(true));
			GUILayout.BeginHorizontal();
			GUILayout.Label("Name:", GUILayout.Width(50));
			config_name = GUILayout.TextField(config_name, GUILayout.ExpandWidth(true), GUILayout.MinWidth(50));
			if(TCAScenario.NamedConfigs.ContainsKey(config_name))
			{
				if(GUILayout.Button(new GUIContent("Overwrite", "Overwrite selected configuration with the current one"), 
				                    Styles.danger_button, GUILayout.Width(70)))
					TCAScenario.SaveNamedConfig(config_name, CFG, true);
			}
			else if(GUILayout.Button(new GUIContent("Add", "Save current configuration"), 
			                         Styles.add_button, GUILayout.Width(50)) && config_name != string.Empty) 
			{
				TCAScenario.SaveNamedConfig(config_name, CFG);
				update_configs();
				named_configs.SelectItem(TCAScenario.NamedConfigs.IndexOfKey(config_name));
			}
			SelectConfig();
			if(GUILayout.Button(new GUIContent("Load", "Load selected configuration"),
			                    Styles.active_button, GUILayout.Width(50)) && selected_config != null) 
				CFG.CopyFrom(selected_config);
			if(GUILayout.Button(new GUIContent("Delete", "Delete selected configuration"), 
			                    Styles.danger_button, GUILayout.Width(50)) && selected_config != null)
			{ 
				TCAScenario.NamedConfigs.Remove(selected_config.Name);
				named_configs.SelectItem(named_configs.SelectedIndex-1);
				update_configs();
				selected_config = null;
			}
			GUILayout.EndHorizontal();
			GUILayout.EndVertical();
		}

		static void ControllerProperties()
		{
			Utils.ButtonSwitch("Autotune engines' controller parameters", ref CFG.AutoTune, "", GUILayout.ExpandWidth(true));
			if(CFG.AutoTune) return;
			//steering modifiers
			GUILayout.BeginHorizontal();
			CFG.SteeringGain = Utils.FloatSlider("Steering Gain", CFG.SteeringGain, 0, 1, "P1");
			CFG.PitchYawLinked = GUILayout.Toggle(CFG.PitchYawLinked, "Link Pitch&Yaw", GUILayout.ExpandWidth(false));
			GUILayout.EndHorizontal();
			GUILayout.BeginHorizontal();
			if(CFG.PitchYawLinked && !CFG.AutoTune)
			{
				CFG.SteeringModifier.x = Utils.FloatSlider("Pitch&Yaw", CFG.SteeringModifier.x, 0, 1, "P1");
				CFG.SteeringModifier.z = CFG.SteeringModifier.x;
			}
			else
			{
				CFG.SteeringModifier.x = Utils.FloatSlider("Pitch", CFG.SteeringModifier.x, 0, 1, "P1");
				CFG.SteeringModifier.z = Utils.FloatSlider("Yaw", CFG.SteeringModifier.z, 0, 1, "P1");
			}
			CFG.SteeringModifier.y = Utils.FloatSlider("Roll", CFG.SteeringModifier.y, 0, 1, "P1");
			GUILayout.EndHorizontal();
			//engines
			CFG.Engines.DrawControls("Engines Controller");
		}

		static bool show_parts_info;
		static void PartsInfo()
		{
			if(parts == null || parts.Count == 0) return;
			Utils.ButtonSwitch("Show status of TCA Modules", ref show_parts_info, "", GUILayout.ExpandWidth(true));
			if(show_parts_info)
			{
				GUILayout.BeginVertical(Styles.white);
				for(int i = 0, partsCount = parts.Count; i < partsCount; i++)
				{
					var part = parts[i];
					GUILayout.BeginHorizontal();
					GUILayout.Label(part.Title);
					GUILayout.FlexibleSpace();
					GUILayout.Label(part.Active? "Active" : "Dependencies Unsatisfied",
					                part.Active? Styles.green : Styles.red);
					GUILayout.EndHorizontal();
				}
				GUILayout.EndVertical();
			}
		}

		static void EnginesControl()
		{
			GUILayout.BeginVertical();
			if(CFG.ActiveProfile.NumManual > 0)
			{
				if(GUILayout.Button(CFG.ShowManualLimits? "Hide Manual Engines" : "Show Manual Engines", 
				                    Styles.active_button,
				                    GUILayout.ExpandWidth(true)))
					CFG.ShowManualLimits = !CFG.ShowManualLimits;
				if(CFG.ShowManualLimits) CFG.EnginesProfiles.DrawManual(Utils.ClampH(LineHeight*CFG.ActiveProfile.NumManual, ControlsHeight));
			}
			if(GUILayout.Button(CFG.ShowEnginesProfiles? "Hide Engines Profiles" : "Show Engines Profiles", 
			                    Styles.active_button,
			                    GUILayout.ExpandWidth(true)))
				CFG.ShowEnginesProfiles = !CFG.ShowEnginesProfiles;
			if(CFG.ShowEnginesProfiles) CFG.EnginesProfiles.Draw(ControlsHeight*2);
			GUILayout.EndVertical();
		}

		#if DEBUG
		static Vector2 eInfoScroll;
		static void EnginesInfo()
		{
			GUILayout.BeginVertical();
			GUILayout.BeginHorizontal();
			GUILayout.Label(string.Format("Steering: {0}", VSL.Controls.Steering), GUILayout.ExpandWidth(false));
			GUILayout.Label(string.Format("Angular Accel Error: {0:F3}rad/s2", TCA.ENG.TorqueError), GUILayout.ExpandWidth(false));
			GUILayout.Label(string.Format("Vertical Speed Factor: {0:P1}", VSL.OnPlanetParams.VSF), GUILayout.ExpandWidth(false));
			GUILayout.EndHorizontal();
			eInfoScroll = GUILayout.BeginScrollView(eInfoScroll, GUILayout.Height(ControlsHeight*4));
			GUILayout.BeginVertical();
			foreach(var e in VSL.Engines.Active)
			{
				if(!e.Valid(VSL)) continue;
				GUILayout.BeginHorizontal();
				GUILayout.Label(e.name + "\n" +
				                string.Format(
					                "Torque: {0}\n" +
					                "Attitude Modifier: {1:P1}\n" +
					                "Thrust Limit:      {2:F1}%",
					                e.currentTorque,
					                e.limit, e.thrustLimit*100));
				GUILayout.EndHorizontal();
			}
			GUILayout.EndVertical();
			GUILayout.EndScrollView();
			GUILayout.EndVertical();
		}

		static void DebugInfo()
		{
			GUILayout.BeginHorizontal();
			GUILayout.Label(string.Format("vV: {0:0.0}m/s", VSL.VerticalSpeed.Absolute), GUILayout.Width(100));
			GUILayout.Label(string.Format("A: {0:0.0}m/s2", VSL.VerticalSpeed.Derivative), GUILayout.Width(80));
			GUILayout.Label(string.Format("ApA: {0:0.0}m", VSL.orbit.ApA), GUILayout.Width(120));
			GUILayout.Label(string.Format("hV: {0:0.0}m/s", VSL.HorizontalSpeed.Absolute), GUILayout.Width(100));
			GUILayout.EndHorizontal();
			GUILayout.BeginHorizontal();
			GUILayout.Label(string.Format("VSP: {0:0.0m/s}", CFG.VerticalCutoff), GUILayout.Width(100));
			GUILayout.Label(string.Format("TWR: {0:0.0}", VSL.OnPlanetParams.DTWR), GUILayout.Width(80));
			if(VSL.Altitude.Ahead.Equals(float.MinValue)) GUILayout.Label("Obst: N/A", GUILayout.Width(120));
			else GUILayout.Label(string.Format("Obst: {0:0.0}m", VSL.Altitude.Ahead), GUILayout.Width(120));
			GUILayout.Label(string.Format("Orb: {0:0.0}m/s", Math.Sqrt(VSL.Physics.StG*VSL.Physics.Radial.magnitude)), GUILayout.Width(100));
			GUILayout.EndHorizontal();
			if(!string.IsNullOrEmpty(DebugMessage))
				GUILayout.Label(DebugMessage, Styles.boxed_label, GUILayout.ExpandWidth(true));
		}
		#endif

		public void OnGUI()
		{
			if(TCA == null || !CFG.GUIVisible || !showHUD) 
			{
				Utils.LockIfMouseOver(LockName, MainWindow, false);
				return;
			}
			Styles.Init();
			Utils.LockIfMouseOver(LockName, MainWindow);
			MainWindow = 
				GUILayout.Window(TCA.GetInstanceID(), 
				                 MainWindow, 
								 DrawMainWindow, 
				                 VSL.vessel.vesselName,
				                 GUILayout.Width(ControlsWidth),
				                 GUILayout.Height(ControlsHeight));
			MainWindow.clampToScreen();
		}
	}
}
