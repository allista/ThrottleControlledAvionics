//  Author:
//       Allis Tauri <allista@gmail.com>
//
//  Copyright (c) 2015 Allis Tauri
//
// This work is licensed under the Creative Commons Attribution-ShareAlike 4.0 International License. 
// To view a copy of this license, visit http://creativecommons.org/licenses/by-sa/4.0/ 
// or send a letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.

using System;
using UnityEngine;
using AT_Utils;

namespace ThrottleControlledAvionics
{
    public class MacroNode : TypedConfigNodeObject
    {
        public delegate void Selector(Action<MacroNode> callback);
        internal static Globals GLB { get { return Globals.Instance; } }
        protected VesselConfig EditedCFG;

        public MacroNode Parent;
        [Persistent] public string Name = "";
        [Persistent] public bool Active;
        [Persistent] public bool Done;
        protected GUIContent Label;

        public virtual bool Edit { get; set; }

        public Selector SelectNode;
        public Condition.Selector SelectCondition;

        public MacroNode()
        { 
            Name = Utils.ParseCamelCase(GetType().Name.Replace(typeof(MacroNode).Name, "")); 
            Label = new GUIContent(Name);
        }

        public MacroNode(ComponentInfo info) : this()
        {
            if(!string.IsNullOrEmpty(info.Name)) Name = info.Name;
            Label = new GUIContent(Name, info.Description);
        }

        public override void Load(ConfigNode node)
        {
            base.Load(node);
            var t = GetType();
            Label = Components.Actions.ContainsKey(t)? 
                Components.Actions[GetType()].Label : new GUIContent(Name);
        }

        public virtual void OnChildRemove(MacroNode child) 
        { child.Parent = null; }

        public virtual bool AddChild(MacroNode child) 
        { return false; }

        public virtual bool AddSibling(MacroNode sibling) 
        { return false; }

        public virtual void OnChildActivate(MacroNode child)
        { if(Parent != null) Parent.OnChildActivate(child); }

        protected virtual void DrawDeleteButton()
        {
            if(Parent != null && Parent.Edit &&
               GUILayout.Button(new GUIContent("X", "Delete"), 
                                Styles.close_button, 
                                GUILayout.Width(20)))
                Parent.OnChildRemove(this);
        }

        protected virtual void DrawThis() 
        { GUILayout.Label(Label, Active? Styles.enabled_button : Styles.normal_button, GUILayout.ExpandWidth(true)); }

        protected virtual void CleanUp() {}

        public virtual void Draw() 
        { 
            GUILayout.BeginHorizontal();
            DrawDeleteButton();
            DrawThis();
            GUILayout.EndHorizontal();
            CleanUp();
        }

        /// <summary>
        /// Perform the Action on a specified VSL.
        /// Returns true if it is not finished and should be called on next update. 
        /// False if it is finished.
        /// </summary>
        /// <param name="VSL">VesselWrapper</param>
        protected virtual bool Action(VesselWrapper VSL) { return false; }

        public bool Execute(VesselWrapper VSL)
        {
            if(Done) return false;
            if(!Active)
            {
                Active = true;
                if(Parent != null) 
                    Parent.OnChildActivate(this);
            }
            Done = !Action(VSL);
            Active &= !Done;
            return !Done;
        }

        public virtual void Rewind() { Active = Done = false; }

        public void CopyFrom(MacroNode mn)
        {
            var node = new ConfigNode();
            mn.Save(node);
            Load(node);
        }

        public MacroNode GetCopy()
        {
            var constInfo = GetType().GetConstructor(Type.EmptyTypes);
            if(constInfo == null) return null;
            var mn = (MacroNode)constInfo.Invoke(null);
            mn.CopyFrom(this);
            mn.Rewind();
            return mn;
        }

        public virtual void SetSelector(Selector selector) 
        { SelectNode = selector; }

        public virtual void SetConditionSelector(Condition.Selector selector) 
        { SelectCondition = selector; }

        public virtual void SetCFG(VesselConfig cfg) 
        { EditedCFG = cfg; }

        protected void Message(string msg)
        { Utils.Message("{0}: {1}", Name, msg); }

        #if DEBUG
        protected void Log(VesselWrapper VSL, string msg, params object[] args)
        { VSL.vessel.Log(msg, args); }
        #endif
    }
}

