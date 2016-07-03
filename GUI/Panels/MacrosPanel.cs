//  Author:
//       Allis Tauri <allista@gmail.com>
//
//  Copyright (c) 2016 Allis Tauri
//
// This work is licensed under the Creative Commons Attribution-ShareAlike 4.0 International License. 
// To view a copy of this license, visit http://creativecommons.org/licenses/by-sa/4.0/ 
// or send a letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.
//
using UnityEngine;
using AT_Utils;

namespace ThrottleControlledAvionics
{
	public class MacrosPanel : ControlPanel
	{
		public MacrosPanel(ModuleTCA tca) : base(tca) {}

		MacroProcessor MPC;

		bool selecting_macro;
		public override void Draw()
		{
			if(MPC == null) return;
			GUILayout.BeginHorizontal();
			if(CFG.SelectedMacro != null && CFG.MacroIsActive)
			{
				GUILayout.Label(new GUIContent("Macro: "+CFG.SelectedMacro.Title, "The macro is executing..."), 
				                Styles.yellow, GUILayout.ExpandWidth(true));
				CFG.MacroIsActive &= !GUILayout.Button("Pause", Styles.enabled_button, GUILayout.Width(70));
				if(GUILayout.Button("Stop", Styles.danger_button, GUILayout.ExpandWidth(false))) 
					CFG.StopMacro();
				GUILayout.Label("Edit", Styles.inactive_button, GUILayout.ExpandWidth(false));
			}
			else if(CFG.SelectedMacro != null)
			{
				if(GUILayout.Button(new GUIContent("Macro: "+CFG.SelectedMacro.Title, "Select a macro from databases"), 
				                    Styles.normal_button, GUILayout.ExpandWidth(true))) 
					selecting_macro = !selecting_macro;
				CFG.MacroIsActive |= GUILayout.Button(CFG.SelectedMacro.Active? "Resume" : "Execute", 
				                                      Styles.active_button, GUILayout.Width(70));
				if(GUILayout.Button("Stop", CFG.SelectedMacro.Active? 
				                    Styles.danger_button : Styles.inactive_button, GUILayout.ExpandWidth(false))) 
					CFG.SelectedMacro.Rewind();
				if(GUILayout.Button("Edit", Styles.active_button, GUILayout.ExpandWidth(false)))
					TCAMacroEditor.Edit(CFG);
			}
			else 
			{
				if(GUILayout.Button("Select Macro", Styles.normal_button, GUILayout.ExpandWidth(true))) 
					selecting_macro = !selecting_macro;
				if(GUILayout.Button("New Macro", Styles.add_button, GUILayout.ExpandWidth(false)))
					TCAMacroEditor.Edit(CFG);
			}
			GUILayout.EndHorizontal();
			if(selecting_macro)
			{
				TCAMacro macro = null;
				if(TCAMacroEditor.DrawMacroSelector(CFG, out macro)) 
				{
					if(macro != null) 
					{
						CFG.SelectedMacro = (TCAMacro)macro.GetCopy();
						CFG.MacroIsActive = false;
					}
					selecting_macro = false;
				}
			}
		}
	}
}

