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
using System.Collections.Generic;
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
		static bool editing;

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

		static public void Exit() { exit = true; }

		public static bool Editing { get { return Macro != null; } }

		static public void Edit(VesselConfig cfg)
		{ 
			if(cfg == null) return;
			CFG = cfg;
			editing = CFG.SelectedMacro != null;
			EditMacro(CFG.SelectedMacro);
		}

		static void EditMacro(TCAMacro macro)
		{
			if(macro == null)
			{
				Macro = new TCAMacro();
				Macro.Name = "Empty Macro";
			}
			else Macro = (TCAMacro)macro.GetCopy();
			Macro.SetCFG(CFG);
			Macro.SetSelector(SelectAction);
			Macro.SetConditionSelector(SelectCondition);
			Macro.Edit = true;
		}

		void select_action(MacroNode action)
		{
			if(action != null && action_selected != null) 
				action_selected(action);
			SelectAction(null);
		}

		void load_macro(TCAMacro macro)
		{
			if(macro != null) EditMacro(macro);
			LoadMacro = false;
		}

		static bool exit;
		void Update()
		{
			if(exit)
			{
				CFG = null;
				Macro = null;  
				SelectAction(null); 
				SelectCondition(null);
				exit = false;
				editing = false;
			}
		}

		public static bool DrawMacroSelector(VesselConfig cfg, out TCAMacro macro)
		{
			macro = null;
			TCAMacro sel = null;
			bool ret = false;
			GUILayout.BeginVertical(Styles.white);
			GUILayout.Label("Load Macro form Library", Styles.green, GUILayout.ExpandWidth(true));
			GUILayout.BeginHorizontal();
			GUILayout.BeginVertical();
			GUILayout.Label("Current Vessel", Styles.yellow, GUILayout.ExpandWidth(true));
			if(cfg.Macros.Selector(out sel)) { ret = true; macro = sel; }
			GUILayout.EndVertical();
			GUILayout.BeginVertical();
			GUILayout.Label("Global Database", Styles.yellow, GUILayout.ExpandWidth(true));
			if(TCAScenario.Macros.Selector(out sel)) { ret = true; macro = sel; }
			GUILayout.EndVertical();
			GUILayout.EndHorizontal();
			ret |= GUILayout.Button("Cancel", Styles.red_button, GUILayout.ExpandWidth(true));
			GUILayout.EndVertical();
			return ret;
		}

		protected override void DrawMainWindow(int windowID)
		{
			if(Macro == null) return;
			GUILayout.BeginVertical();
			GUILayout.BeginHorizontal();
			LoadMacro |= GUILayout.Button("Load", Styles.green_button, GUILayout.ExpandWidth(false));
			if(GUILayout.Button("New", Styles.yellow_button, GUILayout.ExpandWidth(false))) EditMacro(null);
			GUILayout.Space(20);
			if(editing && GUILayout.Button("Apply", Styles.yellow_button, GUILayout.ExpandWidth(false)))
				CFG.SelectedMacro = (TCAMacro)Macro.GetCopy();
			if(GUILayout.Button("Save to Vessel DB", Styles.yellow_button, GUILayout.ExpandWidth(false)))
				CFG.Macros.SaveMacro(Macro, true);
			if(GUILayout.Button("Save to Global DB", Styles.yellow_button, GUILayout.ExpandWidth(false)))
				TCAScenario.Macros.SaveMacro(Macro, true);
			GUILayout.FlexibleSpace();
			exit |= GUILayout.Button("Exit", Styles.red_button, GUILayout.ExpandWidth(false));
			GUILayout.EndHorizontal();
			scroll = GUILayout.BeginScrollView(scroll, GUILayout.ExpandHeight(false));
			Macro.Draw();
			GUILayout.EndScrollView();
			if(SelectingAction) 
			{
				GUILayout.FlexibleSpace();
				GUILayout.BeginVertical(Styles.white);
				GUILayout.Label("Select Action", Styles.green, GUILayout.ExpandWidth(true));
				MacroNode action = null;
				GUILayout.BeginHorizontal();
				GUILayout.BeginVertical();
				GUILayout.Label("Builtin", Styles.yellow, GUILayout.ExpandWidth(true));
				if(Components.ActionSelector(out action)) select_action(action);
				GUILayout.EndVertical();
				GUILayout.BeginVertical();
				GUILayout.Label("Current Vessel", Styles.yellow, GUILayout.ExpandWidth(true));
				if(CFG.Macros.Selector(out action)) select_action(action);
				GUILayout.EndVertical();
				GUILayout.BeginVertical();
				GUILayout.Label("Global Database", Styles.yellow, GUILayout.ExpandWidth(true));
				if(TCAScenario.Macros.Selector(out action)) select_action(action);
				GUILayout.EndVertical();
				GUILayout.EndHorizontal();
				if(GUILayout.Button("Cancel", Styles.red_button, GUILayout.ExpandWidth(true))) SelectAction(null);
				GUILayout.EndVertical();
			}
			if(SelectingCondition) 
			{
				GUILayout.FlexibleSpace();
				GUILayout.BeginVertical(Styles.white);
				GUILayout.Label("Select Condition", Styles.green, GUILayout.ExpandWidth(true));
				Condition cnd = null;
				if(Components.ConditionSelector(out cnd))
				{
					if(cnd != null && condition_selected != null)
						condition_selected(cnd);
					SelectCondition(null);
				}
				if(GUILayout.Button("Cancel", Styles.red_button, GUILayout.ExpandWidth(true))) SelectCondition(null);
				GUILayout.EndVertical();
			}
			if(LoadMacro)
			{
				GUILayout.FlexibleSpace();
				TCAMacro macro;
				if(DrawMacroSelector(CFG, out macro))
					load_macro(macro);
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

	public static class Components
	{
		public static SortedList<string, ComponentDB<Condition>.Factory> Conditions
		{ get { return ComponentDB<Condition>.Components; } }

		public static SortedList<string, ComponentDB<MacroNode>.Factory> Actions
		{ get { return ComponentDB<MacroNode>.Components; } }

		public static bool ConditionSelector(out Condition condition)
		{ return ComponentDB<Condition>.Selector(out condition); }

		public static bool ActionSelector(out MacroNode action)
		{ return ComponentDB<MacroNode>.Selector(out action); }
	}
}

