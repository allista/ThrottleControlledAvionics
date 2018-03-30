//   MacrosTab.cs
//
//  Author:
//       Allis Tauri <allista@gmail.com>
//
//  Copyright (c) 2017 Allis Tauri

using UnityEngine;
using AT_Utils;

namespace ThrottleControlledAvionics
{
    public class MacrosTab : ControlTab
    {
        public MacrosTab(ModuleTCA tca) : base(tca) {}

        #pragma warning disable 169
        MacroProcessor MPC;
        #pragma warning restore 169

        bool selecting_macro;
        public override void Draw()
        {
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
                        CFG.SelectedMacro = macro.GetCopy() as TCAMacro;
                        CFG.MacroIsActive = false;
                    }
                    selecting_macro = false;
                }
            }
        }
    }
}