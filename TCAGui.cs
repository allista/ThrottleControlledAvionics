/* The GUI for ThrottleControlledAvionics.
 * Authors: Quinten Feys, Willem van Vliet, Allis Tauri
 * License: BY: Attribution-ShareAlike 3.0 Unported (CC BY-SA 3.0): http://creativecommons.org/licenses/by-sa/3.0/
 */

using System.Linq;
using UnityEngine;

namespace ThrottleControlledAvionics
{
	public class TCAGui
	{
		readonly ThrottleControlledAvionics TCA;

		#region GUI
		bool showEngines;
		bool showHelp;
		bool showHUD = true;
		//named configs
		NamedConfig selected_config;
		string config_name = string.Empty;
		GUIContent[] namedConfigsList;
		readonly ComboBox namedConfigsListBox = new ComboBox();
		//dimensions
		Vector2 positionScrollViewEngines;
		public const int controlsWidth = 500, controlsHeight = 100;
		public const int helpWidth = 500, helpHeight = 100;
		//icons
		const string ICON_ON  = "ThrottleControlledAvionics/Icons/icon_button_on";
		const string ICON_OFF = "ThrottleControlledAvionics/Icons/icon_button_off";
		const string ICON_NC  = "ThrottleControlledAvionics/Icons/icon_button_noCharge";
		//buttons
		IButton TCAToolbarButton;
		ApplicationLauncherButton TCAButton;
		Texture textureOn;
		Texture textureOff;
		Texture textureNoCharge;
		#endregion

		//constructor
		public TCAGui(ThrottleControlledAvionics _TCA)
		{
			TCA = _TCA;

			if(ToolbarManager.ToolbarAvailable)
			{
				TCAToolbarButton = ToolbarManager.Instance.add("ThrottleControlledAvionics", "ThrottleControlledAvionicsButton");
				TCAToolbarButton.TexturePath = ICON_OFF;
				TCAToolbarButton.ToolTip     = "Throttle Controlled Avionics";
				TCAToolbarButton.Visibility  = new GameScenesVisibility(GameScenes.FLIGHT);
				TCAToolbarButton.Visible     = true;
				TCAToolbarButton.OnClick    += e => TCA.CFG.GUIVisible = !TCA.CFG.GUIVisible;
			}
			else 
			{
				textureOn = GameDatabase.Instance.GetTexture(ICON_ON, false);
				textureOff = GameDatabase.Instance.GetTexture(ICON_OFF, false);
				textureNoCharge = GameDatabase.Instance.GetTexture(ICON_NC, false);
				GameEvents.onGUIApplicationLauncherReady.Add(OnGUIAppLauncherReady);
			}
			GameEvents.onHideUI.Add(onHideUI);
			GameEvents.onShowUI.Add(onShowUI);
			updateConfigs();
		}

		void updateConfigs()
		{ namedConfigsList = TCAConfiguration.NamedConfigs.Keys.Select(s => new GUIContent(s)).ToArray(); }

		#region GUI methods
		public void UpdateToolbarIcon() 
		{
			if(TCAToolbarButton != null)
			{
				if(TCA.CFG.Enabled) TCAToolbarButton.TexturePath = TCA.haveEC ? ICON_ON : ICON_NC;
				else TCAToolbarButton.TexturePath = ICON_OFF;
			}
			if(TCAButton != null) 
			{
				if(TCA.CFG.Enabled) TCAButton.SetTexture(TCA.haveEC? textureOn : textureNoCharge);
				else TCAButton.SetTexture(textureOff);
			}
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
					GameDatabase.Instance.GetTexture(ICON_OFF, false));
				if(TCA.CFG.GUIVisible) TCAButton.SetTrue();
				else TCAButton.SetFalse();
			}
		}

		void onAppLaunchToggleOn() { TCA.CFG.GUIVisible = true; }
		void onAppLaunchToggleOff() { TCA.CFG.GUIVisible = false; }
		void DummyVoid() {}

		void onShowUI() { showHUD = true; }
		void onHideUI() { showHUD = false; }

		public void OnDestroy() 
		{ 
			GameEvents.onGUIApplicationLauncherReady.Remove(OnGUIAppLauncherReady);
			if(TCAButton != null)
				ApplicationLauncher.Instance.RemoveModApplication(TCAButton);
			if(TCAToolbarButton != null)
				TCAToolbarButton.Destroy();
			GameEvents.onHideUI.Remove(onHideUI);
			GameEvents.onShowUI.Remove(onShowUI);
		}

		void TCA_Window(int windowID)
		{
			if(GUI.Button(new Rect(TCAConfiguration.Globals.ControlsPos.width - 23f, 2f, 20f, 18f), "?"))
				showHelp = !showHelp;
			GUILayout.BeginVertical();
			GUILayout.BeginHorizontal();
			TCA.ActivateTCA(GUILayout.Toggle(TCA.CFG.Enabled, "Toggle TCA"));
			if(!TCA.haveEC)	GUILayout.Label("WARNING! no electric charge!", GUILayout.ExpandWidth(false));
			GUILayout.EndHorizontal();
			ConfigsGUI();
			ControllerProperties();
			EnginesInfo();
			GUILayout.EndVertical();
			GUI.DragWindow();
		}

		void ControllerProperties()
		{
			GUILayout.BeginHorizontal();
			GUILayout.Label("Smoothness: ", GUILayout.ExpandWidth(false));
			GUILayout.Label(TCA.CFG.StabilityCurve.ToString("F1"), GUILayout.ExpandWidth(false));
			TCA.CFG.StabilityCurve = GUILayout.HorizontalSlider(TCA.CFG.StabilityCurve, 0f, 2f);
			GUILayout.EndHorizontal();
			//PI Controllers
			TCA.CFG.Torque.DrawPIControls("Torque");
			TCA.CFG.Steering.DrawPIControls("Steering");
			TCA.CFG.Engines.DrawPIControls("Engines");
			//speed limit
			GUILayout.BeginHorizontal();
			GUILayout.Label("Vertical Speed Limit: ", GUILayout.ExpandWidth(false));
			GUILayout.Label(TCA.CFG.VerticalCutoff >= TCAConfiguration.Globals.MaxCutoff? "inf." :
			                TCA.CFG.VerticalCutoff.ToString("F1"), GUILayout.ExpandWidth(false));
			TCA.CFG.VerticalCutoff = GUILayout.HorizontalSlider(TCA.CFG.VerticalCutoff, 
			                                                    -TCAConfiguration.Globals.MaxCutoff, 
			                                                    TCAConfiguration.Globals.MaxCutoff);
			GUILayout.EndHorizontal();
		}

		void ConfigsGUI()
		{
			GUILayout.BeginHorizontal();
			GUILayout.Label("Name:", GUILayout.Width(50));
			config_name = GUILayout.TextField(config_name, GUILayout.ExpandWidth(true), GUILayout.MinWidth(50));
			if(TCAConfiguration.NamedConfigs.ContainsKey(config_name))
			{
				if(GUILayout.Button("Overwrite", GUILayout.Width(70)))
				{
					TCAConfiguration.SaveNamedConfig(config_name, TCA.CFG, true);
					TCAConfiguration.Save();
				}
			}
			else if(GUILayout.Button("Add", GUILayout.Width(50))) 
			{
				TCAConfiguration.SaveNamedConfig(config_name, TCA.CFG);
				TCAConfiguration.Save();
				updateConfigs();
			}
			selected_config = null;
			if(namedConfigsList.Length > 0) 
			{
				var i = namedConfigsListBox.GetSelectedItemIndex();
				i = namedConfigsListBox.List(namedConfigsList[i].text, namedConfigsList, "Box");
				selected_config = TCAConfiguration.GetConfig(namedConfigsList[i].text);
			}
			else GUILayout.Label("[Nothing Saved]", GUILayout.ExpandWidth(true));
			if(GUILayout.Button("Load", GUILayout.Width(50)) && selected_config != null) 
				TCA.CFG.CopyFrom(selected_config);
			if(GUILayout.Button("Delete", GUILayout.Width(50)) && selected_config != null)
			{ 
				TCAConfiguration.NamedConfigs.Remove(selected_config.Name);
				TCAConfiguration.Save();
				updateConfigs();
				selected_config = null;
			}
			GUILayout.EndHorizontal();
		}

		void EnginesInfo()
		{
			showEngines = GUILayout.Toggle(showEngines, "Show Engines Information");
			if(showEngines)
			{
				positionScrollViewEngines = GUILayout.BeginScrollView(positionScrollViewEngines, GUILayout.Height(controlsHeight*4));
				foreach(var e in TCA.engines)
				{
					if(!e.Valid) continue;
					GUILayout.Label(e.getName() + "\n" +
					                string.Format(
						                "Torque: {0}\n" +
						                "Thrust Dir: {1}\n" +
						                "Efficiency: {2:P1}\n" +
						                "Thrust: {3:F1}%",
						                e.currentTorque, e.thrustDirection,
						                e.efficiency, e.thrustPercentage));
				}
				GUILayout.EndScrollView();
			}
		}

		void windowHelp(int windowID)
		{
			GUILayout.BeginVertical();
			GUILayout.Label(TCAConfiguration.Globals.Instructions, GUILayout.MaxWidth(helpWidth));
			if(GUILayout.Button("Close")) showHelp = !showHelp;
			GUILayout.EndVertical();
			GUI.DragWindow();
		}

		public void DrawGUI()
		{
			if(!TCA.CFG.GUIVisible || !showHUD) return;
			TCAConfiguration.Globals.ControlsPos = 
				GUILayout.Window(1, 
				                 TCAConfiguration.Globals.ControlsPos, 
				                 TCA_Window, 
				                 "Throttle Controlled Avionics",
				                 GUILayout.Width(controlsWidth),
				                 GUILayout.Height(controlsHeight));
			Utils.CheckRect(ref TCAConfiguration.Globals.ControlsPos);
			if(showHelp) 
			{
				TCAConfiguration.Globals.HelpPos = 
					GUILayout.Window(2, 
					                 TCAConfiguration.Globals.HelpPos, 
					                 windowHelp, 
					                 "Instructions",
					                 GUILayout.Width(helpWidth),
					                 GUILayout.Height(helpHeight));
				Utils.CheckRect(ref TCAConfiguration.Globals.HelpPos);
			}
		}
		#endregion
	}
}

