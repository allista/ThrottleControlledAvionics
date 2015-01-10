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
		bool showAny;
		bool showEngines;
		bool showHelp;
		bool showHUD = true;

		Vector2 positionScrollViewEngines;

		const int controlsWidth = 400, controlsHeight = 100;
		const int helpWidth = 500, helpHeight = 600;
		protected Rect controlsPos = new Rect(50, 50, controlsWidth, controlsHeight);
		protected Rect helpPos = new Rect(500, 100, helpWidth, helpHeight);
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
				TCAToolbarButton.ToolTip = "Throttle Controlled Avionics";
				TCAToolbarButton.Visibility = new GameScenesVisibility(GameScenes.FLIGHT);
				TCAToolbarButton.Visible = true;
				TCAToolbarButton.OnClick += e => showAny = !showAny;
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
				if(TCA.isActive) TCAToolbarButton.TexturePath = TCA.haveEC ? ICON_ON : ICON_NC;
				else TCAToolbarButton.TexturePath = ICON_OFF;
			}
			if(TCAButton != null) 
			{
				if(TCA.isActive) TCAButton.SetTexture(TCA.haveEC? textureOn : textureNoCharge);
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
			}
		}

		void onAppLaunchToggleOn() { showAny = true; }
		void onAppLaunchToggleOff() { showAny = false; }
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
			if(GUI.Button(new Rect(controlsPos.width - 23f, 2f, 20f, 18f), "?"))
				showHelp = !showHelp;

			GUILayout.BeginVertical();
			if(!TCA.haveEC)	GUILayout.Label("WARNING! no electric charge!");

			GUILayout.BeginHorizontal();
			TCA.ActivateTCA(GUILayout.Toggle(TCA.isActive, "Toggle TCA", GUILayout.ExpandWidth(false)));
			GUILayout.Label("Steering Threshold: ", GUILayout.ExpandWidth(false)); //redundant?
			GUILayout.Label(TCA.steeringThreshold.ToString("P1"), GUILayout.ExpandWidth(false));
			TCA.steeringThreshold = GUILayout.HorizontalSlider(TCA.steeringThreshold, 0f, 0.5f); //TODO: the boundaries should be in config
			GUILayout.EndHorizontal();

			GUILayout.BeginHorizontal();
			GUILayout.Label("Vertical Speed Limit: ", GUILayout.ExpandWidth(false));
			GUILayout.Label(TCA.verticalCutoff >= ThrottleControlledAvionics.maxCutoff? "inf." :
			                TCA.verticalCutoff.ToString("F1"), GUILayout.ExpandWidth(false));
			TCA.verticalCutoff = GUILayout.HorizontalSlider(TCA.verticalCutoff, 
			                                                -ThrottleControlledAvionics.maxCutoff, 
			                                                ThrottleControlledAvionics.maxCutoff);
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

		static void windowHelp(int windowID)
		{
			GUILayout.Label(TCAGlobals.Instructions, GUILayout.MaxWidth(helpWidth));
			GUI.DragWindow();
		}

		public void DrawGUI()
		{
			if(!showAny || !showHUD) return;
			controlsPos = GUILayout.Window(1, 
										   controlsPos, 
										   TCA_Window, 
										   "Throttle Controlled Avionics",
			                               GUILayout.Width(controlsWidth),
			                               GUILayout.Height(controlsHeight));
			Utils.CheckRect(ref controlsPos);
			if(showHelp) 
			{
				helpPos = GUILayout.Window(2, 
				                           helpPos, 
				                           windowHelp, 
				                           "Instructions",
				                           GUILayout.Width(helpWidth),
				                           GUILayout.Height(helpHeight));
				Utils.CheckRect(ref helpPos);
			}
		}
		#endregion
	}
}

