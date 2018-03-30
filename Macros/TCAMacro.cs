//  Author:
//       Allis Tauri <allista@gmail.com>
//
//  Copyright (c) 2015 Allis Tauri
//
// This work is licensed under the Creative Commons Attribution-ShareAlike 4.0 International License. 
// To view a copy of this license, visit http://creativecommons.org/licenses/by-sa/4.0/ 
// or send a letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.

using System;
using System.Collections.Generic;
using UnityEngine;
using AT_Utils;

namespace ThrottleControlledAvionics
{
    [ComponentInfo(Hidden = true)]
    public class TCAMacro : SingleBlockMacroNode
    {
        protected MacroNode ActiveChild;

        protected override bool Action(VesselWrapper VSL)
        { return Block.Execute(VSL); }

        public override void OnChildActivate(MacroNode child)
        {
            ActiveChild = child;
            base.OnChildActivate(this);
        }

        public override void Rewind()
        {
            base.Rewind();
            ActiveChild = null;
        }

        public string Title 
        { 
            get 
            { 
                var title = Name;
                if(ActiveChild != null) title += " ["+ActiveChild.Name+"]";
                return title;
            }
        }

        protected override void DrawThis()
        {
            GUILayout.BeginVertical(Styles.white);
            if(Edit) 
            {
                GUILayout.BeginHorizontal();
                Name = GUILayout.TextField(Name, GUILayout.ExpandWidth(true), GUILayout.MinWidth(50));
                if(Parent != null) Edit &= !GUILayout.Button("Done", Styles.confirm_button, GUILayout.Width (40));
                GUILayout.EndHorizontal();
                Block.Draw();
            }
            else Edit |= GUILayout.Button(Title, Active? Styles.active_button : Styles.normal_button, GUILayout.ExpandWidth(true));
            GUILayout.EndVertical();
        }
    }

    public class TCAMacroLibrary : TypedConfigNodeObject
    {
        [Persistent] public PersistentBaseList<TCAMacro> DB = new PersistentBaseList<TCAMacro>();

        public void SaveMacro(TCAMacro macro, bool overwrite = false)
        {
            var old_macro = DB.List.FindIndex(m => m.Name == macro.Name);
            if(old_macro < 0) 
            { 
                DB.Add((TCAMacro)macro.GetCopy()); 
                DB.List.Sort((a,b) => a.Name.CompareTo(b.Name)); 
            }
            else if(overwrite) DB[old_macro] = (TCAMacro)macro.GetCopy();
        }

        public void Clear() { DB.List.Clear(); }
        public bool Remove(TCAMacro macro) { return DB.List.Remove(macro); }
        public TCAMacro Get(string name) { return DB.List.Find(m => m.Name == name); }

        Vector2 scroll;
        public bool Selector(out TCAMacro macro)
        {
            var ret = false;
            macro = null;
            var del = new List<TCAMacro>();
            scroll = GUILayout.BeginScrollView(scroll, Styles.white, GUILayout.Height(Globals.Instance.ActionListHeight));
            GUILayout.BeginVertical();
            for(int i = 0, count = DB.List.Count; i < count; i++)
            {
                var m = DB.List[i];
                GUILayout.BeginHorizontal();
                if(GUILayout.Button("X", Styles.danger_button, GUILayout.Width(20)))
                    del.Add(m);
                if(GUILayout.Button(m.Name, Styles.normal_button, GUILayout.ExpandWidth(true)))
                {
                    macro = (TCAMacro)m.GetCopy();
                    ret = true;
                }
                GUILayout.EndHorizontal();
            }
            GUILayout.EndVertical();
            GUILayout.EndScrollView();
            for(int i = 0, count = del.Count; i < count; i++)
                DB.List.Remove(del[i]);
            return ret;
        }

        public bool Selector(out MacroNode macro)
        {
            macro = null;
            TCAMacro m = null;
            if(Selector(out m)) 
            { macro = m; return true; }
            return false;
        }
    }
}

