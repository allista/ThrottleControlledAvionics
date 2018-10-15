//  Author:
//       Allis Tauri <allista@gmail.com>
//
//  Copyright (c) 2015 Allis Tauri
//
// This work is licensed under the Creative Commons Attribution-ShareAlike 4.0 International License. 
// To view a copy of this license, visit http://creativecommons.org/licenses/by-sa/4.0/ 
// or send a letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.

using System.Collections.Generic;
using UnityEngine;
using AT_Utils;

namespace ThrottleControlledAvionics
{
    [ComponentInfo(Hidden = true)]
    public class ConditionMacroNode : MacroNode
    {
        [Persistent] public PersistentBaseList<Condition> Conditions = new PersistentBaseList<Condition>();
        public bool HasConditions { get { return Conditions.Count > 0; } }
        protected List<Condition> deleted_conditions = new List<Condition>();
        protected string Keyword = "IF";

        public override void SetConditionSelector(Condition.Selector selector)
        { 
            base.SetConditionSelector(selector);
            for(int i = 0, count = Conditions.List.Count; i < count; i++)
                Conditions.List[i].SetSelector(selector);
        }

        public bool ConditionsMet(VesselWrapper VSL)
        {
            var ret = Conditions.Count == 0;
            for(int i = 0, count = Conditions.Count; i < count; i++) 
                ret |= Conditions[i].True(VSL);
            return ret;
        }

        protected override void CleanUp()
        {
            for(int i = 0, count = deleted_conditions.Count; i < count; i++) 
                Conditions.Remove(deleted_conditions[i]);
            deleted_conditions.Clear();
        }

        protected override void DrawThis()
        {
            GUILayout.BeginVertical();
            if(Conditions.Count > 0) 
            {
                for(int i = 0, count = Conditions.Count; i < count; i++) 
                { 
                    var c = Conditions[i];
                    GUILayout.BeginHorizontal();
                    if(i > 0) { if(GUILayout.Button(c.or? "OR" : "AND", Styles.white, GUILayout.Width(50))) c.or = !c.or; }
                    else if(GUILayout.Button(new GUIContent(Keyword, Label.tooltip), Edit? Styles.active_button : Styles.normal_button, GUILayout.ExpandWidth(false)))
                        Edit = !Edit;
                    c.Draw();
                    if(Edit && GUILayout.Button(new GUIContent("X", "Delete"), Styles.close_button, GUILayout.Width(20))) deleted_conditions.Add(c);
                    GUILayout.EndHorizontal();
                }
            }
            else 
            {
                GUILayout.BeginHorizontal();
                if(GUILayout.Button(new GUIContent(Keyword, Label.tooltip), Edit? Styles.active_button : Styles.normal_button, GUILayout.ExpandWidth(false)))
                    Edit = !Edit;
                GUILayout.Label("TRUE", Styles.white, GUILayout.ExpandWidth(true));
                GUILayout.EndHorizontal();
            }
            if(Edit && Parent != null && 
               GUILayout.Button("Add Condition", Styles.active_button, GUILayout.ExpandWidth(true)))
            { 
                if(SelectCondition != null) 
                    SelectCondition(cnd => 
                {
                    cnd.SetSelector(SelectCondition);
                    Conditions.Add(cnd);
                }); 
            }
            GUILayout.EndVertical();
        }
    }
}

