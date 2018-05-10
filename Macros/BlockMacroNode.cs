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
    public class BlockMacroNode : MacroNode
    {
        [Persistent] public PersistentBaseList<MacroNode> Subnodes = new PersistentBaseList<MacroNode>();
        [Persistent] public int ActiveSubnode = -1;
        public bool HasSubnodes { get { return Subnodes.Count > 0; } }
        protected List<MacroNode> deleted_nodes = new List<MacroNode>();

        public override void Load(ConfigNode node)
        {
            base.Load(node);
            for(int i = 0, count = Subnodes.Count; i < count; i++) 
                Subnodes[i].Parent= this;
        }

        public override void SetSelector(Selector selector)
        { 
            base.SetSelector(selector);
            for(int i = 0, count = Subnodes.Count; i < count; i++)
                Subnodes[i].SetSelector(selector);
        }

        public override void SetConditionSelector(Condition.Selector selector)
        {
            base.SetConditionSelector(selector);
            for(int i = 0, count = Subnodes.Count; i < count; i++)
                Subnodes[i].SetConditionSelector(selector);
        }

        public override void SetCFG(VesselConfig cfg)
        { 
            base.SetCFG(cfg);
            for(int i = 0, count = Subnodes.Count; i < count; i++)
                Subnodes[i].SetCFG(cfg);
        }

        public override void Rewind()
        {
            base.Rewind();
            ActiveSubnode = -1;
            for(int i = 0, count = Subnodes.Count; i < count; i++) 
                Subnodes[i].Rewind();
        }

        public override void OnChildRemove(MacroNode child)
        { 
            base.OnChildRemove(child);
            deleted_nodes.Add(child); 
        }

        protected override bool Action(VesselWrapper VSL)
        {
            if(ActiveSubnode >= Subnodes.Count) return false;
            if(ActiveSubnode < 0) ActiveSubnode = 0;
            if(!Subnodes[ActiveSubnode].Execute(VSL))
                ActiveSubnode++;
            return true;
        }

        protected override void CleanUp()
        {
            base.CleanUp();
            for(int i = 0, count = deleted_nodes.Count; i < count; i++) 
                Subnodes.Remove(deleted_nodes[i]);
            deleted_nodes.Clear();
        }

        public override bool AddChild(MacroNode child)
        {
            child.Parent = this;
            Subnodes.Add(child);
            child.SetCFG(EditedCFG);
            child.SetSelector(SelectNode);
            child.SetConditionSelector(SelectCondition);
            return true;
        }

        public override bool AddSibling(MacroNode sibling)
        { return AddChild(sibling); }

        public bool MoveLeft(int i)
        {
            if(i < 0 || i >= Subnodes.Count) return false;
            if(Parent == null) return false;
            var child = Subnodes[i];
            if(Parent.AddSibling(child))
            {
                deleted_nodes.Add(child);
                return true;
            }
            return false;
        }

        public bool MoveRight(int i)
        {
            if(i <= 0 || i >= Subnodes.Count) return false;
            var child = Subnodes[i];
            if(Subnodes[i-1].AddChild(child))
            {
                deleted_nodes.Add(child);
                return true;
            }
            return false;
        }

        public bool MoveUp(int i)
        {
            if(i <= 0 || i >= Subnodes.Count) return false;
            var child  = Subnodes[i];
            var child1 = Subnodes[i-1];
            Subnodes[i-1] = child;
            Subnodes[i] = child1;
            return true;
        }

        public bool MoveDown(int i)
        {
            if(i < 0 || i >= Subnodes.Count-1) return false;
            var child  = Subnodes[i];
            var child1 = Subnodes[i+1];
            Subnodes[i+1] = child;
            Subnodes[i] = child1;
            return true;
        }

        protected override void DrawDeleteButton() {}

        protected override void DrawThis()
        {
            GUILayout.BeginHorizontal();
            GUILayout.Space(20);
            GUILayout.BeginVertical();
            for(int i = 0, count = Subnodes.Count; i < count; i++)
            { 
                var child = Subnodes[i];
                GUILayout.BeginHorizontal();
                child.Draw();
                if(Edit)
                {
                    if(GUILayout.Button(new GUIContent("^", "Move upward"), Styles.normal_button, GUILayout.Width(20)))
                        MoveUp(i);
                    if(GUILayout.Button(new GUIContent("<", "Move to the parent block"), Styles.normal_button, GUILayout.Width(20)))
                        MoveLeft(i);
                    if(GUILayout.Button(new GUIContent(">", "Move into the block ABOVE"),  Styles.normal_button, GUILayout.Width(20)))
                        MoveRight(i);
                }
                GUILayout.EndHorizontal();
            }
            if(Edit)
            {
                if(GUILayout.Button("Add Action", Styles.active_button, GUILayout.ExpandWidth(true)))
                { if(SelectNode != null) SelectNode(n => AddChild(n)); }
            }
            GUILayout.EndVertical();
            GUILayout.EndHorizontal();
            CleanUp();
        }
    }
}

