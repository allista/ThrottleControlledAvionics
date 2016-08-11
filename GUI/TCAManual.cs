//  Author:
//       Allis Tauri <allista@gmail.com>
//
//  Copyright (c) 2015 Allis Tauri
//
// This work is licensed under the Creative Commons Attribution-ShareAlike 4.0 International License. 
// To view a copy of this license, visit http://creativecommons.org/licenses/by-sa/4.0/ 
// or send a letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.

using UnityEngine;
using AT_Utils;

namespace ThrottleControlledAvionics
{
	[KSPAddon(KSPAddon.Startup.EveryScene, false)]
	public class TCAManual : AddonWindowBase<TCAManual>
	{
		const string LockName = "TCAMnualEditor";

		static MDSection Manual { get { return Globals.Instance.Manual; } }
		static bool show_status;
		static MDSection current_section;
		static string current_text = "";
		static Vector2 sections_scroll;
		static Vector2 content_scroll;

		public TCAManual() { width = 800; height = 600; }

		void Update()
		{
			if(Manual == null) return;
			if(window_enabled)
			{
				if(current_section == null)
					change_section(Manual.NoText && Manual.Subsections.Count > 0? 
					               Manual.Subsections[0] : Manual);
			}
		}

		void change_section(MDSection sec)
		{
			show_status = false;
			current_section = sec;
			current_text = sec.Text;
		}

		static void PartsInfo()
		{
			if(TCAScenario.Parts.Count == 0) 
			{
				GUILayout.Label("No modules installed.");
				return;
			}
			GUILayout.BeginVertical(Styles.white);
			for(int i = 0, partsCount = TCAScenario.Parts.Count; i < partsCount; i++)
			{
				var part = TCAScenario.Parts[i];
				GUILayout.BeginHorizontal();
				GUILayout.Label(part.Title);
				GUILayout.FlexibleSpace();
				if(part.Active) GUILayout.Label("Active", Styles.green);
				else GUILayout.Label(new GUIContent("Dependencies Unsatisfied", 
				                                    "Consult R&D tree to see what modules are required for this to work."), 
				                     Styles.red);
				GUILayout.EndHorizontal();
			}
			GUILayout.EndVertical();
		}

		public static void ShowStatus()
		{
			if(HighLogic.CurrentGame == null) return;
			show_status = true;
			Show(true);
		}

		protected override void DrawMainWindow(int windowID)
		{
			GUILayout.BeginVertical();
			sections_scroll = GUILayout.BeginScrollView(sections_scroll, GUILayout.Height(60));
			GUILayout.BeginHorizontal();
			if(HighLogic.CurrentGame != null &&
			   GUILayout.Button("Status", show_status? Styles.green_button : Styles.normal_button, GUILayout.ExpandWidth(false)))
				show_status = true;
			for(int i = 0, count = Manual.Subsections.Count; i < count; i++)
			{
				var ss = Manual.Subsections[i];
				if(GUILayout.Button(ss.Title, (!show_status && current_section == ss)? 
				                    Styles.green_button : Styles.normal_button, GUILayout.ExpandWidth(false)))
					change_section(ss);
			}
			GUILayout.EndHorizontal();
			GUILayout.EndScrollView();
			content_scroll = GUILayout.BeginScrollView(content_scroll, Styles.white_on_black, GUILayout.ExpandHeight(true));
			if(show_status)
			{
				GUILayout.BeginVertical();
				GUILayout.Label(Title);
				if(!TCAScenario.ModuleInstalled)
					GUILayout.Label("<color=red><size=30>TCA module was not found in any of the loaded parts.</size></color>\n\n" +
					                    "This probably means you're using an old version of <b>ModuleManager</b> or haven't installed it yet. " +
					                    "<color=yellow><b>ModuleManager</b> is required</color> for TCA to work.", Styles.rich_label);
				else if(HighLogic.CurrentGame.Mode == Game.Modes.CAREER)
				{
					
					if(!TCAScenario.HasTCA)
						GUILayout.Label("<color=yellow><size=30>TCA Subsystem is <b>NOT</b> purchased. Get it in R&D first.</size></color>", Styles.rich_label);
					else
					{
						GUILayout.Label("<color=lime>TCA Subsystem is purchased.</color>\nInstalled TCA modules:", Styles.rich_label);
						PartsInfo();
					}
				}
				else GUILayout.Label("<b>Sandbox Game:</b>\n" +
				                     "<color=lime>TCA should be fully functional</color> " +
				                     "on all vessels with some engines/RCS and a command module (cockpit, probe core, etc).", Styles.rich_label);
				GUILayout.EndVertical();
			}
			else GUILayout.Label(current_text, Styles.rich_label, GUILayout.MaxWidth(width));
			GUILayout.EndScrollView();
			if(GUILayout.Button("Close")) Show(false);
			GUILayout.EndVertical();
			base.DrawMainWindow(windowID);
		}

		public void OnGUI()
		{
			if(Manual == null || !do_show) 
			{
				Utils.LockIfMouseOver(LockName, MainWindow, false);
				return;
			}
			Styles.Init();
			Utils.LockIfMouseOver(LockName, MainWindow);
			MainWindow = 
				GUILayout.Window(GetInstanceID(), 
				                 MainWindow, 
				                 DrawMainWindow, 
				                 Globals.Instance.Manual.Title,
				                 GUILayout.Width(width),
				                 GUILayout.Height(height)).clampToScreen();
		}
	}
}

