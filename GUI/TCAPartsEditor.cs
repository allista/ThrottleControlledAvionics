//   TCAPartsEditor.cs
//
//  Author:
//       Allis Tauri <allista@gmail.com>
//
//  Copyright (c) 2017 Allis Tauri

using System.Collections.Generic;
using UnityEngine;
using AT_Utils;

namespace ThrottleControlledAvionics
{
	public class TCAPartsEditor : GUIWindowBase
	{
		public VesselConfig CFG;
		List<TCAPart> parts;
		List<GUIContent> titles;

		public TCAPartsEditor()
		{
			width = 500;
			height = 315;
		}

		public override void Awake()
		{
			base.Awake();
			update_part_status();
		}

		void update_part_status()
		{
			if(parts == null)
				parts = TCAModulesDatabase.GetPurchasedParts();
			parts.ForEach(p => p.UpdateInfo(CFG));
			TCAGuiEditor.UpdateModules();
			titles = new List<GUIContent>();
			for(int i = 0, count = parts.Count; i < count; i++)
			{
				var part = parts[i];
				//remove Module from module title. Maybe drop it altogether from the TechTree?
				titles.Add(new GUIContent(part.Title.Replace("Module", "").Trim(), 
				                          //remove the "Updates TCA (tm)..." sentence.
				                          part.Description.Substring(part.Description.IndexOf('.')+1)));
			}
		}

		public override void Show(bool show)
		{
			base.Show(show);
			if(show) update_part_status();
		}

		protected override bool can_draw() { return CFG != null; }

		Vector2 scroll = Vector2.zero;
		void MainWindow(int windowID)
		{
			GUILayout.BeginVertical();
			scroll = GUILayout.BeginScrollView(scroll);
			GUILayout.BeginVertical();
			for(int i = 0, count = parts.Count; i < count; i++)
			{
				if(i%3 == 0)
				{
					if(i > 0) GUILayout.EndHorizontal();
					GUILayout.BeginHorizontal();
				}
				var part = parts[i];
				if(CFG.EnabledTCAParts.Contains(part.Name))
				{
					if(!part.Active)
						GUILayout.Label(titles[i], Styles.inactive_button);
					else if(GUILayout.Button(titles[i], Styles.enabled_button, GUILayout.ExpandWidth(true)))
					{
						CFG.EnabledTCAParts.Remove(part.Name);
						update_part_status();
					}
				}
				else
				{
					if(GUILayout.Button(titles[i], Styles.active_button, GUILayout.ExpandWidth(true)))
					{
						CFG.EnabledTCAParts.Add(part.Name);
						update_part_status();
					}
				}
			}
			GUILayout.EndHorizontal();
			GUILayout.EndVertical();
			GUILayout.EndScrollView();
			if(GUILayout.Button(new GUIContent("Enable All", "Enable all disabled modules"), 
			                    Styles.active_button, GUILayout.ExpandWidth(true)))
			{
				parts.ForEach(p => CFG.EnabledTCAParts.Add(p.Name));
				update_part_status();
			}
			if(GUILayout.Button("Close", Styles.close_button, GUILayout.ExpandWidth(true))) Show(false);
			GUILayout.EndVertical();
			TooltipsAndDragWindow();
		}

		public void Draw()
		{
			if(doShow)
			{
				LockControls();
				WindowPos = GUILayout.Window(GetInstanceID(), 
				                             WindowPos, MainWindow,
				                             "Select TCA Modules", 
				                             GUILayout.Width(width),
				                             GUILayout.Height(height)).clampToScreen();
			}
			else UnlockControls();
		}
	}
}

