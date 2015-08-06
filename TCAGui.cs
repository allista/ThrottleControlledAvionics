//   TCAGui.cs
//
//  Author:
//       Allis Tauri <allista@gmail.com>
//
//  Copyright (c) 2015 Allis Tauri
//
// This work is licensed under the Creative Commons Attribution 4.0 International License. 
// To view a copy of this license, visit http://creativecommons.org/licenses/by/4.0/ 
// or send a letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.

using System;
using System.Reflection;
using System.Linq;
using System.Collections.Generic;
using UnityEngine;

namespace ThrottleControlledAvionics
{

	public partial class ThrottleControlledAvionics
	{
		#region GUI Parameters
		static bool showHelp;
		static bool showHUD = true;
		//named configs
		static NamedConfig selected_config;
		static string config_name = string.Empty;
		static readonly DropDownList namedConfigsListBox = new DropDownList();
		//dimensions
		public const int controlsWidth = 500, controlsHeight = 100;
		public const int helpWidth = 500, helpHeight = 500;
		static Rect ControlsPos = new Rect(50, 100, controlsWidth, controlsHeight);
		static Rect HelpPos     = new Rect(Screen.width/2-helpWidth/2, 100, helpWidth, helpHeight);
		static Vector2 enginesScroll, helpScroll;
		//keybindings
		public static KeyCode TCA_Key = KeyCode.Y;
		static bool selecting_key;
		#endregion

		void onShowUI() { showHUD = true; }
		void onHideUI() { showHUD = false; }

		#region Configs Selector
		static void updateConfigs()
		{ 
			var configs = TCAConfiguration.NamedConfigs.Keys.ToList();
			var first = namedConfigsListBox.Items.Count == 0;
			configs.Add(string.Empty); namedConfigsListBox.Items = configs; 
			if(first) namedConfigsListBox.SelectItem(configs.Count-1);
		}

		static void SelectConfig_start() 
		{ 
			if(TCAConfiguration.NamedConfigs.Count < 2) return;
			namedConfigsListBox.styleListBox  = Styles.list_box;
			namedConfigsListBox.styleListItem = Styles.list_item;
			namedConfigsListBox.windowRect    = ControlsPos;
			namedConfigsListBox.DrawBlockingSelector(); 
		}

		static void SelectConfig()
		{
			if(TCAConfiguration.NamedConfigs.Count == 0)
				GUILayout.Label("[Nothing Saved]", GUILayout.ExpandWidth(true));
			else
			{
				namedConfigsListBox.DrawButton();
				var new_config = TCAConfiguration.GetConfig(namedConfigsListBox.SelectedIndex);
				if(new_config != selected_config)
				{
					selected_config = new_config;
					config_name = selected_config != null? selected_config.Name : string.Empty;
				}
			}
		}

		static void SelectConfig_end()
		{
			if(TCAConfiguration.NamedConfigs.Count < 2) return;
			namedConfigsListBox.DrawDropDown();
			namedConfigsListBox.CloseOnOutsideClick();
		}
		#endregion

		static void TCA_Window(int windowID)
		{
			//help button
			if(GUI.Button(new Rect(ControlsPos.width - 23f, 2f, 20f, 18f), "?")) showHelp = !showHelp;
			GUILayout.BeginVertical();
			GUILayout.BeginHorizontal();
			//tca toggle
			if(GUILayout.Button(CFG.Enabled? "Disable" : "Enable", 
			                    CFG.Enabled? Styles.red_button : Styles.green_button,
			                    GUILayout.Width(70)))
				TCA.ToggleTCA();
			//change key binding
			if(GUILayout.Button(selecting_key? "?" : TCA_Key.ToString(), 
			                    selecting_key? Styles.yellow_button : Styles.green_button, 
			                    GUILayout.Width(40)))
			{ selecting_key = true; ScreenMessages.PostScreenMessage("Enter new key to toggle TCA", 5, ScreenMessageStyle.UPPER_CENTER); }
			//autotune switch
			CFG.AutoTune = GUILayout.Toggle(CFG.AutoTune, "Autotune Parameters", GUILayout.ExpandWidth(true));
			#if DEBUG
			if(GUILayout.Button("Reload Globals", Styles.yellow_button, GUILayout.Width(120))) 
			{
				TCAConfiguration.LoadGlobals();
				TCA.OnReloadGlobals();
			}
			#endif
			StatusString();
			GUILayout.EndHorizontal();
			SelectConfig_start();
			ConfigsGUI();
			ControllerProperties();
			ManualEnginesControl();
			#if DEBUG
			EnginesInfo();
			#endif
			SelectConfig_end();
			GUILayout.EndVertical();
			GUI.DragWindow();
		}

		static void StatusString()
		{
			var state = "Disabled";
			var style = Styles.grey;
			if(TCA.IsStateSet(TCAState.Enabled))
			{
				if(TCA.IsStateSet(TCAState.Unoptimized))
				{ state = "Engines Unoptimized"; style = Styles.red; }
				else if(TCA.IsStateSet(TCAState.LoosingAltitude))
				{ state = "Loosing Altitude"; style = Styles.red; }
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
				{ state = "Unknown State"; style = Styles.magenta_button; }
			}
			GUILayout.Label(state, style, GUILayout.ExpandWidth(false));
		}

		static void ControllerProperties()
		{
			//steering modifiers
			if(!CFG.AutoTune)
			{
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
			if(VSL.OnPlanet)
			{
				//vertical speed or altitude limit
				GUILayout.BeginHorizontal();
				if(CFG.ControlAltitude)
				{
					GUILayout.Label("Altitude: " + 
					                (VSL.Altitude.ToString("F2")+"m"), 
					                GUILayout.Width(120));
					GUILayout.Label("Set Point: " + (CFG.DesiredAltitude.ToString("F1") + "m"), 
					                GUILayout.Width(125));
					if(GUILayout.Button("-10m", Styles.normal_button, GUILayout.Width(50))) CFG.DesiredAltitude -= 10;
					if(GUILayout.Button("+10m", Styles.normal_button, GUILayout.Width(50))) CFG.DesiredAltitude += 10;
					GUILayout.Label("Vertical Speed: " + 
					                (TCA.IsStateSet(TCAState.VerticalSpeedControl)? VSL.VerticalSpeed.ToString("F2")+"m/s" : "N/A"), 
					                GUILayout.Width(180));
					GUILayout.FlexibleSpace();
				}
				else
				{
					GUILayout.Label("Vertical Speed: " + 
					                (TCA.IsStateSet(TCAState.VerticalSpeedControl)? VSL.VerticalSpeed.ToString("F2")+"m/s" : "N/A"), 
					                GUILayout.Width(180));
					GUILayout.Label("Set Point: " + (CFG.VerticalCutoff < GLB.VSC.MaxSpeed? 
					                                 CFG.VerticalCutoff.ToString("F1") + "m/s" : "OFF"), 
					                GUILayout.ExpandWidth(false));
					CFG.VerticalCutoff = GUILayout.HorizontalSlider(CFG.VerticalCutoff, 
					                                                -GLB.VSC.MaxSpeed, 
					                                                GLB.VSC.MaxSpeed);
				}
				GUILayout.EndHorizontal();
				//autopilot toggles
				GUILayout.BeginHorizontal();
				TCA.BlockThrottle(GUILayout.Toggle(CFG.BlockThrottle, 
				                                   CFG.ControlAltitude?
				                                   "Change altitude with throttle controls." :
				                                   "Set vertical speed with throttle controls.", 
				                                   GUILayout.ExpandWidth(false)));
				CFG.VSControlSensitivity = Utils.FloatSlider("Sensitivity", CFG.VSControlSensitivity, 0.001f, 0.05f, "P2");
				GUILayout.EndHorizontal();
				GUILayout.BeginHorizontal();
				if(GUILayout.Button("Kill Horizontal Velocity", 
				                    CFG.KillHorVel? Styles.red_button : Styles.dark_yellow_button,
				                    GUILayout.Width(150)))
					TCA.ToggleHvAutopilot();
				if(GUILayout.Button("Cruise Control", 
				                    CFG.CruiseControl? Styles.red_button : Styles.dark_yellow_button,
				                    GUILayout.Width(100)))
					TCA.ToggleCruiseControl();
				if(GUILayout.Button("Maintain Altitude", 
				                    CFG.ControlAltitude? Styles.red_button : Styles.dark_yellow_button,
				                    GUILayout.Width(120)))
					TCA.ToggleAltitudeAutopilot();
				TCA.AltitudeAboveTerrain(GUILayout.Toggle(CFG.AltitudeAboveTerrain, 
				                                          "Above Terrain", 
				                                          GUILayout.ExpandWidth(false)));
				GUILayout.EndHorizontal();
			}
			else 
			{
				GUILayout.BeginHorizontal();
				GUILayout.Label("Autopilot Not Available", Styles.grey);
				GUILayout.FlexibleSpace();
				GUILayout.EndHorizontal();
			}
		}

		static void ConfigsGUI()
		{
			GUILayout.BeginHorizontal();
			GUILayout.Label("Name:", GUILayout.Width(50));
			config_name = GUILayout.TextField(config_name, GUILayout.ExpandWidth(true), GUILayout.MinWidth(50));
			if(TCAConfiguration.NamedConfigs.ContainsKey(config_name))
			{
				if(GUILayout.Button("Overwrite", Styles.red_button, GUILayout.Width(70)))
				{
					TCAConfiguration.SaveNamedConfig(config_name, CFG, true);
					TCAConfiguration.Save();
				}
			}
			else if(GUILayout.Button("Add", Styles.green_button, GUILayout.Width(50)) && 
			        config_name != string.Empty) 
			{
				TCAConfiguration.SaveNamedConfig(config_name, CFG);
				TCAConfiguration.Save();
				updateConfigs();
				namedConfigsListBox.SelectItem(TCAConfiguration.NamedConfigs.IndexOfKey(config_name));
			}
			SelectConfig();
			if(GUILayout.Button("Load", Styles.yellow_button, GUILayout.Width(50)) && selected_config != null) 
				CFG.CopyFrom(selected_config);
			if(GUILayout.Button("Delete", Styles.red_button, GUILayout.Width(50)) && selected_config != null)
			{ 
				TCAConfiguration.NamedConfigs.Remove(selected_config.Name);
				TCAConfiguration.Save();
				namedConfigsListBox.SelectItem(namedConfigsListBox.SelectedIndex-1);
				updateConfigs();
				selected_config = null;
			}
			GUILayout.EndHorizontal();
		}

		static void ManualEnginesControl()
		{
			if(VSL.ManualEngines.Count == 0) return;
			GUILayout.BeginVertical();
			if(GUILayout.Button(CFG.ShowManualLimits? "Hide Manual Limits" : "Show Manual Limits", 
			                    CFG.ShowManualLimits? Styles.yellow_button : Styles.dark_yellow_button,
			                    GUILayout.ExpandWidth(true)))
				CFG.ShowManualLimits = !CFG.ShowManualLimits;
			if(CFG.ShowManualLimits)
			{
				GUILayout.BeginVertical();
				enginesScroll = GUILayout.BeginScrollView(enginesScroll, GUILayout.Height(controlsHeight));
				GUILayout.BeginVertical();
				var added = new HashSet<int>();
				foreach(var e in VSL.ManualEngines.Where(ew => ew.Group > 0))
				{
					if(!e.Valid || added.Contains(e.Group)) continue;
					GUILayout.BeginHorizontal();
					GUILayout.Label(string.Format("Group {0} thrust:", e.Group), GUILayout.Width(180));
					CFG.ManualLimits.Groups[e.Group] = 
						Utils.FloatSlider("", CFG.ManualLimits.GetLimit(e), 0f, 1f, "P1");
					added.Add(e.Group);
					GUILayout.EndHorizontal();
				}
				foreach(var e in VSL.ManualEngines.Where(ew => ew.Group == 0))
				{
					if(!e.Valid) continue;
					GUILayout.BeginHorizontal();
					GUILayout.Label(string.Format("{0}:", e.name), GUILayout.Width(180));
					var lim = Utils.FloatSlider("", CFG.ManualLimits.GetLimit(e), 0f, 1f, "P1");
					CFG.ManualLimits.Single[e.part.flightID] = lim;
					e.forceThrustPercentage(lim*100);
					GUILayout.EndHorizontal();
				}
				GUILayout.EndVertical();
				GUILayout.EndScrollView();
				GUILayout.EndVertical();
			}
			GUILayout.EndVertical();
		}

		#if DEBUG
		static Vector2 eInfoScroll;
		static void EnginesInfo()
		{
			GUILayout.BeginVertical();
			GUILayout.BeginHorizontal();
			GUILayout.Label(string.Format("Torque Error: {0:F1}kNm", TCA.TorqueError), GUILayout.ExpandWidth(false));
			GUILayout.Label(string.Format("Vertical Speed Factor: {0:P1}", VSL.VSF), GUILayout.ExpandWidth(false));
			GUILayout.EndHorizontal();
			eInfoScroll = GUILayout.BeginScrollView(eInfoScroll, GUILayout.Height(controlsHeight*4));
			GUILayout.BeginVertical();
			foreach(var e in VSL.ActiveEngines)
			{
				if(!e.Valid) continue;
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
		#endif

		static void windowHelp(int windowID)
		{
			GUILayout.BeginVertical();
			helpScroll = GUILayout.BeginScrollView(helpScroll);
			GUILayout.Label(GLB.Instructions, GUILayout.MaxWidth(helpWidth));
			GUILayout.EndScrollView();
			if(GUILayout.Button("Close")) showHelp = !showHelp;
			GUILayout.EndVertical();
			GUI.DragWindow();
		}

		public void OnGUI()
		{
			if(TCA == null || !TCA.Controllable || !CFG.GUIVisible || !showHUD) return;
			Styles.Init();
			ControlsPos = 
				GUILayout.Window(TCA.GetInstanceID(), 
				                 ControlsPos, 
				                 TCA_Window, 
				                 "Throttle Controlled Avionics - " + 
				                 Assembly.GetCallingAssembly().GetName().Version,
				                 GUILayout.Width(controlsWidth),
				                 GUILayout.Height(controlsHeight));
			Utils.CheckRect(ref ControlsPos);
			if(showHelp) 
			{
				HelpPos = 
					GUILayout.Window(TCA.GetInstanceID()+1, 
					                 HelpPos, 
					                 windowHelp, 
					                 "Instructions",
					                 GUILayout.Width(helpWidth),
					                 GUILayout.Height(helpHeight));
				Utils.CheckRect(ref HelpPos);
			}
		}
	}
}