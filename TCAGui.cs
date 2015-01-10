/* The GUI class of THCA.
 * Author: Quinten Feys & Willem van Vliet
 * License: BY: Attribution-ShareAlike 3.0 Unported (CC BY-SA 3.0): http://creativecommons.org/licenses/by-sa/3.0/
 */
using System;
using UnityEngine;

namespace ThrottleControlledAvionics
{
	public class TCAGui
	{
		ThrottleControlledAvionics TCA;

		#region GUI
		bool showEngines;
		bool showHelp;
		bool showHUD = true;

		Vector2 positionScrollViewEngines;

		public const int controlsWidth = 400, controlsHeight = 100;
		public const int helpWidth = 500, helpHeight = 100;
		const string ICON_ON  = "ThrottleControlledAvionics/Icons/icon_button_on";
		const string ICON_OFF = "ThrottleControlledAvionics/Icons/icon_button_off";
		const string ICON_NC  = "ThrottleControlledAvionics/Icons/icon_button_noCharge";

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
				TCAToolbarButton.OnClick    += e => TCAConfiguration.Globals.GUIVisible = !TCAConfiguration.Globals.GUIVisible;
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
		}

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
				if(TCAConfiguration.Globals.GUIVisible) TCAButton.SetTrue();
				else TCAButton.SetFalse();
			}
		}

		void onAppLaunchToggleOn() { TCAConfiguration.Globals.GUIVisible = true; }
		void onAppLaunchToggleOff() { TCAConfiguration.Globals.GUIVisible = false; }
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

			GUILayout.BeginHorizontal();
			GUILayout.Label("Vertical Speed Limit: ", GUILayout.ExpandWidth(false));
			GUILayout.Label(TCA.CFG.VerticalCutoff >= TCAConfiguration.Globals.MaxCutoff? "inf." :
			                TCA.CFG.VerticalCutoff.ToString("F1"), GUILayout.ExpandWidth(false));
			TCA.CFG.VerticalCutoff = GUILayout.HorizontalSlider(TCA.CFG.VerticalCutoff, 
			                                                    -TCAConfiguration.Globals.MaxCutoff, 
			                                                    TCAConfiguration.Globals.MaxCutoff);
			GUILayout.EndHorizontal();
			//engines info
			showEngines = GUILayout.Toggle(showEngines, "Show/Hide Engines Information");
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
			GUILayout.EndVertical();
			GUI.DragWindow();
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
			if(!TCAConfiguration.Globals.GUIVisible || !showHUD) return;
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

