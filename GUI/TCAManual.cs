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
			current_section = sec;
			current_text = sec.Text;
		}

		protected override void DrawMainWindow(int windowID)
		{
			GUILayout.BeginVertical();
			if(Manual.Subsections.Count > 0)
			{
				sections_scroll = GUILayout.BeginScrollView(sections_scroll, GUILayout.Height(60));
				GUILayout.BeginHorizontal();
				if(GUILayout.Button("All", Styles.normal_button, GUILayout.ExpandWidth(false)))
					change_section(Manual);
				for(int i = 0, count = Manual.Subsections.Count; i < count; i++)
				{
					var ss = Manual.Subsections[i];
					if(GUILayout.Button(ss.Title, Styles.normal_button, GUILayout.ExpandWidth(false)))
						change_section(ss);
				}
				GUILayout.EndHorizontal();
				GUILayout.EndScrollView();
			}
			content_scroll = GUILayout.BeginScrollView(content_scroll, Styles.white_on_black, GUILayout.ExpandHeight(true));
			GUILayout.Label(current_text, Styles.rich_label, GUILayout.MaxWidth(width));
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

