/* The GUI for ThrottleControlledAvionics.
 * Authors: Quinten Feys, Willem van Vliet, Allis Tauri
 * License: BY: Attribution-ShareAlike 3.0 Unported (CC BY-SA 3.0): http://creativecommons.org/licenses/by-sa/3.0/
 */

using System;
using System.Reflection;
using System.Linq;
using System.Collections.Generic;
using UnityEngine;
using KSP.IO;

namespace ThrottleControlledAvionics
{
	public class TCAGui
	{
		static PluginConfiguration GUI_CFG = PluginConfiguration.CreateForType<ThrottleControlledAvionics>();
		readonly ThrottleControlledAvionics TCA;
		VesselWrapper vessel { get { return TCA.vessel; } }
		static TCAGlobals GLB { get { return TCAConfiguration.Globals; } }
		VesselConfig CFG { get { return TCA.CFG; } }

		#region GUI Parameters
		bool showHelp;
		bool showHUD = true;
		//named configs
		NamedConfig selected_config;
		string config_name = string.Empty;
		readonly DropDownList namedConfigsListBox = new DropDownList();
		//dimensions
		Vector2 enginesScroll, helpScroll;
		public const int controlsWidth = 500, controlsHeight = 100;
		public const int helpWidth = 500, helpHeight = 500;
		static Rect ControlsPos = new Rect(50, 100, controlsWidth, controlsHeight);
		static Rect HelpPos     = new Rect(Screen.width/2-helpWidth/2, 100, helpWidth, helpHeight);
		//keybindings
		public static KeyCode TCA_Key = KeyCode.Y;
		bool selecting_key;
		#endregion

		//constructor
		public TCAGui(ThrottleControlledAvionics tca)
		{
			TCA = tca;
			TCAToolbarManager.AttachTCA(TCA);
			LoadConfig();
			GameEvents.onHideUI.Add(onHideUI);
			GameEvents.onShowUI.Add(onShowUI);
			updateConfigs();
		}

		void onShowUI() { showHUD = true; }
		void onHideUI() { showHUD = false; }

		public void LoadConfig()
		{
			GUI_CFG.load();
			ControlsPos = GUI_CFG.GetValue<Rect>(Utils.PropertyName(new {ControlsPos}), ControlsPos);
			HelpPos = GUI_CFG.GetValue<Rect>(Utils.PropertyName(new {HelpPos}), HelpPos);
			TCA_Key = GUI_CFG.GetValue<KeyCode>(Utils.PropertyName(new {TCA_Key}), TCA_Key);
		}

		public void SaveConfig()
		{
			GUI_CFG.SetValue(Utils.PropertyName(new {ControlsPos}), ControlsPos);
			GUI_CFG.SetValue(Utils.PropertyName(new {HelpPos}), HelpPos);
			GUI_CFG.SetValue(Utils.PropertyName(new {TCA_Key}), TCA_Key);
			GUI_CFG.save();
		}

		public void OnDestroy() 
		{ 
			TCAToolbarManager.AttachTCA(null);
			GameEvents.onHideUI.Remove(onHideUI);
			GameEvents.onShowUI.Remove(onShowUI);
			SaveConfig();
		}

		#region Configs Selector
		void updateConfigs()
		{ 
			var configs = TCAConfiguration.NamedConfigs.Keys.ToList();
			var first = namedConfigsListBox.Items.Count == 0;
			configs.Add(string.Empty); namedConfigsListBox.Items = configs; 
			if(first) namedConfigsListBox.SelectItem(configs.Count-1);
		}

		void SelectConfig_start() 
		{ 
			if(TCAConfiguration.NamedConfigs.Count < 2) return;
			namedConfigsListBox.styleListBox  = Styles.list_box;
			namedConfigsListBox.styleListItem = Styles.list_item;
			namedConfigsListBox.windowRect    = ControlsPos;
			namedConfigsListBox.DrawBlockingSelector(); 
		}

		void SelectConfig()
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

		void SelectConfig_end()
		{
			if(TCAConfiguration.NamedConfigs.Count < 2) return;
			namedConfigsListBox.DrawDropDown();
			namedConfigsListBox.CloseOnOutsideClick();
		}
		#endregion

		#region Main GUI
		void TCA_Window(int windowID)
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

		void StatusString()
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

		void ControllerProperties()
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
			//speed limit
			if(vessel.OnPlanet)
			{
				GUILayout.BeginHorizontal();
				if(CFG.ControlAltitude)
				{
					GUILayout.Label("Altitude: " + 
					                (vessel.Altitude.ToString("F2")+"m"), 
					                GUILayout.Width(150));
					GUILayout.Label("Set Point: " + (CFG.DesiredAltitude.ToString("F1") + "m"), 
					                GUILayout.Width(150));
					GUILayout.Label("Vertical Speed: " + 
					                (TCA.IsStateSet(TCAState.VerticalSpeedControl)? vessel.VerticalSpeed.ToString("F2")+"m/s" : "N/A"), 
					                GUILayout.Width(180));
					GUILayout.FlexibleSpace();
				}
				else
				{
					GUILayout.Label("Vertical Speed: " + 
					                (TCA.IsStateSet(TCAState.VerticalSpeedControl)? vessel.VerticalSpeed.ToString("F2")+"m/s" : "N/A"), 
					                GUILayout.Width(180));
					GUILayout.Label("Set Point: " + (CFG.VerticalCutoff < GLB.VSC.MaxSpeed? 
					                                 CFG.VerticalCutoff.ToString("F1") + "m/s" : "OFF"), 
					                GUILayout.ExpandWidth(false));
					CFG.VerticalCutoff = GUILayout.HorizontalSlider(CFG.VerticalCutoff, 
					                                                    -GLB.VSC.MaxSpeed, 
					                                                    GLB.VSC.MaxSpeed);
				}
				GUILayout.EndHorizontal();
				GUILayout.BeginHorizontal();
				TCA.BlockThrottle(GUILayout.Toggle(CFG.BlockThrottle, 
				                                   CFG.ControlAltitude?
				                                   "Change altitude with throttle controls." :
				                                   "Set vertical speed with throttle controls.", 
				                                   GUILayout.ExpandWidth(false)));
				CFG.VSControlSensitivity = Utils.FloatSlider("Sensitivity", CFG.VSControlSensitivity, 0.001f, 0.05f, "P2");
				GUILayout.EndHorizontal();
			}
			GUILayout.BeginHorizontal();
			if(vessel.OnPlanet)
			{
				if(GUILayout.Button("Kill Horizontal Velocity", 
				                    CFG.KillHorVel? Styles.red_button : Styles.dark_yellow_button,
				                    GUILayout.Width(150)))
					TCA.ToggleHvAutopilot();
				if(GUILayout.Button("Maintain Altitude", 
				                    CFG.ControlAltitude? Styles.red_button : Styles.dark_yellow_button,
				                    GUILayout.Width(150)))
					TCA.ToggleAltitudeAutopilot();
				TCA.AltitudeAboveTerrain(GUILayout.Toggle(CFG.AltitudeAboveTerrain, 
				                        	              "Above Terrain", 
				                            	          GUILayout.ExpandWidth(false)));
			}
			else GUILayout.Label("Autopilot Not Available", Styles.grey);
			GUILayout.FlexibleSpace();
			GUILayout.EndHorizontal();
		}

		void ConfigsGUI()
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

		void ManualEnginesControl()
		{
			if(vessel.ManualEngines.Count == 0) return;
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
				foreach(var e in vessel.ManualEngines.Where(ew => ew.Group > 0))
				{
					if(!e.Valid || added.Contains(e.Group)) continue;
					GUILayout.BeginHorizontal();
					GUILayout.Label(string.Format("Group {0} thrust:", e.Group), GUILayout.Width(180));
					CFG.ManualLimits.Groups[e.Group] = 
						Utils.FloatSlider("", CFG.ManualLimits.GetLimit(e), 0f, 1f, "P1");
					added.Add(e.Group);
					GUILayout.EndHorizontal();
				}
				foreach(var e in vessel.ManualEngines.Where(ew => ew.Group == 0))
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
		Vector2 eInfoScroll;
		void EnginesInfo()
		{
			GUILayout.BeginVertical();
			GUILayout.BeginHorizontal();
			GUILayout.Label(string.Format("Torque Error: {0:F1}kNm", TCA.TorqueError), GUILayout.ExpandWidth(false));
			GUILayout.Label(string.Format("Vertical Speed Factor: {0:P1}", vessel.VSF), GUILayout.ExpandWidth(false));
			GUILayout.EndHorizontal();
			eInfoScroll = GUILayout.BeginScrollView(eInfoScroll, GUILayout.Height(controlsHeight*4));
			GUILayout.BeginVertical();
			foreach(var e in vessel.ActiveEngines)
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

		void windowHelp(int windowID)
		{
			GUILayout.BeginVertical();
			helpScroll = GUILayout.BeginScrollView(helpScroll);
			GUILayout.Label(GLB.Instructions, GUILayout.MaxWidth(helpWidth));
			GUILayout.EndScrollView();
			if(GUILayout.Button("Close")) showHelp = !showHelp;
			GUILayout.EndVertical();
			GUI.DragWindow();
		}

		public void DrawGUI()
		{
			if(!CFG.GUIVisible || !showHUD) return;
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

		public void OnUpdate()
		{
			if(selecting_key)
			{ 
				var e = Event.current;
				if(e.isKey)
				{
					if(e.keyCode != KeyCode.Escape)
					{
						//try to get the keycode if the Unity provided us only with the character
						if(e.keyCode == KeyCode.None && e.character >= 'a' && e.character <= 'z')
						{
							var ec = new string(e.character, 1).ToUpper();
							try { e.keyCode = (KeyCode)Enum.Parse(typeof(KeyCode), ec); }
							catch {}
						}
						if(e.keyCode == KeyCode.None) 
							ScreenMessages
								.PostScreenMessage(string.Format("Unable to convert '{0}' to keycode.\nPlease, try an alphabet character.", e.character), 
							    	                             5, ScreenMessageStyle.UPPER_CENTER);
						else TCA_Key = e.keyCode;
						Utils.Log("TCA: new key slected: {0}", TCA_Key);
					}
					selecting_key = false;
				}
			}
			else if(Input.GetKeyDown(TCA_Key)) TCA.ToggleTCA();
		}
		#endregion
	}

	public static class Styles 
	{
		//This code is based on Styles class from Extraplanetary Launchpad plugin.
		public static GUISkin skin;

		public static GUIStyle normal_button;
		public static GUIStyle red_button;
		public static GUIStyle dark_red_button;
		public static GUIStyle green_button;
		public static GUIStyle dark_green_button;
		public static GUIStyle yellow_button;
		public static GUIStyle dark_yellow_button;
		public static GUIStyle cyan_button;
		public static GUIStyle magenta_button;
		public static GUIStyle white;
		public static GUIStyle grey;
		public static GUIStyle red;
		public static GUIStyle yellow;
		public static GUIStyle green;
		public static GUIStyle blue;
		public static GUIStyle label;
		public static GUIStyle slider;
		public static GUIStyle slider_text;

		public static GUIStyle list_item;
		public static GUIStyle list_box;

		static bool initialized;

		public static void InitSkin()
		{
			if(skin != null) return;
			GUI.skin = null;
			skin = (GUISkin)UnityEngine.Object.Instantiate(GUI.skin);
		}

		public static void InitGUI()
		{
			if (initialized) return;
			initialized = true;

			normal_button = new GUIStyle(GUI.skin.button);
			normal_button.normal.textColor = normal_button.focused.textColor = Color.white;
			normal_button.hover.textColor = normal_button.active.textColor = Color.yellow;
			normal_button.onNormal.textColor = normal_button.onFocused.textColor = normal_button.onHover.textColor = normal_button.onActive.textColor = Color.yellow;
			normal_button.padding = new RectOffset (4, 4, 4, 4);

			red_button = new GUIStyle(normal_button);
			red_button.normal.textColor = red_button.focused.textColor = Color.red;

			dark_red_button = new GUIStyle(normal_button);
			dark_red_button.normal.textColor = dark_red_button.focused.textColor = new Color(0.6f, 0, 0, 1);

			green_button = new GUIStyle(normal_button);
			green_button.normal.textColor = green_button.focused.textColor = Color.green;

			dark_green_button = new GUIStyle(normal_button);
			dark_green_button.normal.textColor = dark_green_button.focused.textColor = new Color(0, 0.6f, 0, 1);

			yellow_button = new GUIStyle(normal_button);
			yellow_button.normal.textColor = yellow_button.focused.textColor = Color.yellow;
			yellow_button.hover.textColor = yellow_button.active.textColor = Color.green;
			yellow_button.onNormal.textColor = yellow_button.onFocused.textColor = yellow_button.onHover.textColor = yellow_button.onActive.textColor = Color.green;

			dark_yellow_button = new GUIStyle(yellow_button);
			dark_yellow_button.normal.textColor = dark_yellow_button.focused.textColor = new Color(0.6f, 0.6f, 0, 1);

			cyan_button = new GUIStyle (normal_button);
			cyan_button.normal.textColor = cyan_button.focused.textColor = Color.cyan;

			magenta_button = new GUIStyle (normal_button);
			magenta_button.normal.textColor = magenta_button.focused.textColor = Color.magenta;

			white = new GUIStyle(GUI.skin.box);
			white.padding = new RectOffset (4, 4, 4, 4);
			white.normal.textColor = white.focused.textColor = Color.white;

			grey = new GUIStyle(white);
			grey.normal.textColor = grey.focused.textColor = Color.grey;

			red = new GUIStyle(white);
			red.normal.textColor = red.focused.textColor = Color.red;

			yellow = new GUIStyle(white);
			yellow.normal.textColor = yellow.focused.textColor = Color.yellow;

			green = new GUIStyle(white);
			green.normal.textColor = green.focused.textColor = Color.green;

			blue = new GUIStyle(white);
			blue.normal.textColor = blue.focused.textColor = new Color(0.6f, 0.6f, 1f, 1f);

			label = new GUIStyle (GUI.skin.label);
			label.normal.textColor = label.focused.textColor = Color.white;
			label.alignment = TextAnchor.MiddleCenter;

			slider = new GUIStyle (GUI.skin.horizontalSlider);
			slider.margin = new RectOffset (0, 0, 0, 0);

			slider_text = new GUIStyle (GUI.skin.label);
			slider_text.alignment = TextAnchor.MiddleCenter;
			slider_text.margin = new RectOffset (0, 0, 0, 0);

			list_item = new GUIStyle(GUI.skin.box);
			var texInit = new Texture2D(1, 1);
			texInit.SetPixel(0, 0, new Color(0.05f, 0.05f, 0.05f, 1f));
			texInit.Apply();
			list_item.normal.background = list_item.onNormal.background = list_item.hover.background = list_item.onHover.background = texInit;
			list_item.normal.textColor = list_item.focused.textColor = Color.white;
			list_item.hover.textColor = list_item.active.textColor = Color.yellow;
			list_item.onNormal.textColor = list_item.onFocused.textColor = list_item.onHover.textColor = list_item.onActive.textColor = Color.yellow;
			list_item.padding = new RectOffset(4, 4, 4, 4);

			list_box = new GUIStyle(GUI.skin.button);
			list_box.normal.textColor = list_box.focused.textColor = Color.yellow;
			list_box.hover.textColor = list_box.active.textColor = Color.green;
			list_box.onNormal.textColor = list_box.onFocused.textColor = list_box.onHover.textColor = list_box.onActive.textColor = Color.green;
			list_box.padding = new RectOffset (4, 4, 4, 4);
		}

		public static void Init()
		{
			Styles.InitSkin();
			GUI.skin = Styles.skin;
			Styles.InitGUI();
		}

		public static GUIStyle fracStyle(float frac)
		{
			if(frac < 0.1) return Styles.red;
			if(frac < 0.5) return Styles.yellow;
			if(frac < 0.8) return Styles.white;
			return Styles.green;
		}
	}
}

