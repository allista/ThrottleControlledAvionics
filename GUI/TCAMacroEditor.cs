//   ActionNode.cs
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
using UnityEngine;

namespace ThrottleControlledAvionics
{
	[KSPAddon(KSPAddon.Startup.EveryScene, false)]
	public class TCAMacroEditor : AddonWindowBase<TCAMacroEditor>
	{
		const string LockName = "TCAMacroEditor";

		static TCAMacro Macro;
		static VesselConfig CFG;
		static Vector2 scroll;

		static bool SelectingCondition, SelectingAction, LoadMacro;
		static Action<Condition> condition_selected;
		static Action<MacroNode> action_selected;

		public TCAMacroEditor()
		{ width = 800; height = 600; }

		static void SelectCondition(Action<Condition> callback)
		{
			condition_selected = callback;
			SelectingCondition = condition_selected != null;
		}

		static void SelectAction(Action<MacroNode> callback)
		{
			action_selected = callback;
			SelectingAction = action_selected != null;
		}

		static public void Edit(VesselConfig cfg)
		{ 
			if(cfg == null) return;
			CFG = cfg;
			Edit(CFG.SelectedMacro);
		}

		static public void Edit(TCAMacro macro)
		{
			if(macro == null)
			{
				Macro = new TCAMacro();
				Macro.Name = "Empty Macro";
			}
			else Macro = (TCAMacro)macro.GetCopy();
			Macro.SetSelector(SelectAction);
			Macro.SetConditionSelector(SelectCondition);
			Macro.Edit = true;
			Macro.Expandable = false;
			Macro.Conditionable = false;
		}

		static public void Exit()
		{ 
			CFG = null;
			Macro = null;  
			SelectAction(null); 
			SelectCondition(null); 
		}

		protected override void DrawMainWindow(int windowID)
		{
			if(Macro == null) return;
			GUILayout.BeginVertical();
			GUILayout.BeginHorizontal();
			LoadMacro |= GUILayout.Button("Load", Styles.green_button, GUILayout.ExpandWidth(false));
			GUILayout.Space(20);
			if(GUILayout.Button("Save", Styles.yellow_button, GUILayout.ExpandWidth(false)))
				CFG.Macros.SaveMacro(Macro, true);
			if(GUILayout.Button("Save Globally", Styles.yellow_button, GUILayout.ExpandWidth(false)))
				TCAScenario.Macros.SaveMacro(Macro, true);
			GUILayout.FlexibleSpace();
			if(GUILayout.Button("Exit", Styles.red_button, GUILayout.ExpandWidth(false)))
				Exit();
			GUILayout.EndHorizontal();
			scroll = GUILayout.BeginScrollView(scroll, GUILayout.Height(height));
			Macro.Draw();
			GUILayout.EndScrollView();
			if(SelectingAction) 
			{
				GUILayout.Label("Select Action", Styles.green, GUILayout.ExpandWidth(true));
				MacroNode action = null;
				if(Components.ActionSelector(out action) ||
					CFG.Macros.Selector(out action) ||
					TCAScenario.Macros.Selector(out action))
				{
					if(action != null) 
						action_selected(action);
					SelectAction(null);
				}
				if(GUILayout.Button("Cancel", Styles.red_button, GUILayout.ExpandWidth(true))) SelectAction(null);
			}
			if(SelectingCondition) 
			{
				GUILayout.Label("Select Condition", Styles.green, GUILayout.ExpandWidth(true));
				Condition cnd = null;
				if(Components.ConditionSelector(out cnd))
				{
					if(cnd != null)
						condition_selected(cnd);
					SelectCondition(null);
				}
				if(GUILayout.Button("Cancel", Styles.red_button, GUILayout.ExpandWidth(true))) SelectCondition(null);
			}
			if(LoadMacro)
			{
				GUILayout.Label("Load Macro form Library", Styles.green, GUILayout.ExpandWidth(true));
				TCAMacro macro = null;
				if(CFG.Macros.Selector(out macro) ||
				   TCAScenario.Macros.Selector(out macro))
				{
					if(macro != null) Edit(macro);
					LoadMacro = false;
				}
				LoadMacro &= !GUILayout.Button("Cancel", Styles.red_button, GUILayout.ExpandWidth(true));
			}
			GUILayout.EndVertical();
			base.DrawMainWindow(windowID);
		}

		public void OnGUI()
		{
			if(Macro == null || CFG == null || !CFG.GUIVisible || !showHUD) 
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
					Macro.Name,
					GUILayout.Width(width),
					GUILayout.Height(height));
			MainWindow.clampToScreen();
		}
	}
}

