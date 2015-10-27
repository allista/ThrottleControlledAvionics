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

		static bool SelectingCondition, SelectingAction;
		static Action<Condition> condition_selected;
		static Action<MacroNode> action_selected;

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
			Macro = macro == null? 
				new TCAMacro() : (TCAMacro)macro.GetCopy();
			Macro.SetSelector(SelectAction);
			Macro.SetConditionSelector(SelectCondition);
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
			GUILayout.BeginVertical(Styles.white);
			if(GUILayout.Button("Save", Styles.yellow_button, GUILayout.ExpandWidth(false)))
				CFG.Macros.SaveMacro(Macro, true);
			if(GUILayout.Button("Save Globally", Styles.yellow_button, GUILayout.ExpandWidth(false)))
				TCAScenario.Macros.SaveMacro(Macro, true);
			if(GUILayout.Button("Exit", Styles.red_button, GUILayout.ExpandWidth(false)))
				Exit();
			scroll = GUILayout.BeginScrollView(scroll, GUILayout.Height(height));
			Macro.Draw();
			GUILayout.EndScrollView();
			if(SelectingAction) 
			{
				MacroNode action = null;
				if(Components.ActionSelector(out action) ||
					CFG.Macros.Selector(out action) ||
					TCAScenario.Macros.Selector(out action))
				{
					action_selected(action);
					SelectAction(null);
				}
			}
			if(SelectingCondition) 
			{
				Condition cnd = null;
				if(Components.ConditionSelector(out cnd))
				{
					condition_selected(cnd);
					SelectCondition(null);
				}
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

