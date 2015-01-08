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

		GUIContent[] saveList;
		ComboBox saveListBox;
		Vector2 positionScrollViewEngines;

		protected Rect windowPos = new Rect(50, 50, 450, 200);
		protected Rect windowPosHelp = new Rect(500, 100, 400, 50);
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
			saveList = new GUIContent[3];
			saveList[0] = new GUIContent(TCA.save.GetName(0));
			saveList[1] = new GUIContent(TCA.save.GetName(1));
			saveList[2] = new GUIContent(TCA.save.GetName(2));
			saveListBox = new ComboBox();
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

		void InsertDropdownboxSave()
		{
			int i = saveListBox.GetSelectedItemIndex();
			i = saveListBox.List(saveList[i].text, saveList, "Box");
			TCA.save.SetActiveSave(i);
		}

		void TCA_Window(int windowID)
		{
			if(GUI.Button(new Rect(windowPos.width - 23f, 2f, 20f, 18f), "?"))
				showHelp = !showHelp;

			GUILayout.BeginVertical();
			if(!TCA.haveEC)	GUILayout.Label("WARNING! Electric charge has run out!");

			GUILayout.BeginHorizontal();
			TCA.ActivateTCA(GUILayout.Toggle(TCA.isActive, "Toggle TCA"));
			GUILayout.Label("Settings: ", GUILayout.ExpandWidth(true));
			InsertDropdownboxSave();
			GUILayout.EndHorizontal();

			GUILayout.BeginHorizontal();
			GUILayout.Label("Steering Threshold: ", GUILayout.ExpandWidth(false)); //redundant?
			GUILayout.Label(TCA.steeringThreshold.ToString("P1"), GUILayout.ExpandWidth(false));
			TCA.steeringThreshold = GUILayout.HorizontalSlider(TCA.steeringThreshold, 0f, 0.5f); //TODO: the boundaries should be in config
			GUILayout.EndHorizontal();

			GUILayout.BeginHorizontal();
			GUILayout.Label("Vertical Cutoff: ", GUILayout.ExpandWidth(false));
			GUILayout.Label(TCA.verticalCutoff >= ThrottleControlledAvionics.maxCutoff? "inf." :
			                TCA.verticalCutoff.ToString("F1"), GUILayout.ExpandWidth(false));
			TCA.verticalCutoff = GUILayout.HorizontalSlider(TCA.verticalCutoff, 
			                                                -ThrottleControlledAvionics.maxCutoff, 
			                                                ThrottleControlledAvionics.maxCutoff);
			GUILayout.EndHorizontal();

			showEngines = GUILayout.Toggle(showEngines, "Show/hide engine information");
			if(showEngines)
			{
				positionScrollViewEngines = GUILayout.BeginScrollView(positionScrollViewEngines, GUILayout.Height(300));
				foreach(var eng in TCA.engines)
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

		public void DrawGUI()
		{
			if(!showAny || !showHUD) return;
			windowPos = GUILayout.Window(1, windowPos, TCA_Window, "Throttle Controlled Avionics");
			if(showHelp) windowPosHelp = GUILayout.Window(2, windowPosHelp, windowHelp, "Instructions");
		}
		#endregion
	}
}

