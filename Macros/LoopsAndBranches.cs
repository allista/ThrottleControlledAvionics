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
    [ComponentInfo(Name = "When",
                   Description = "A block of Actions that gets executed when a condition is met")]
    public class TriggeredBlockMacroNode : SingleBlockConditionMacroNode
    {
        public TriggeredBlockMacroNode() { Keyword = "WHEN"; }

        protected override bool Action(VesselWrapper VSL)
        {
            if(Block.Done) return false;
            if(Block.Active || ConditionsMet(VSL)) Block.Execute(VSL);
            return !Block.Done;
        }
    }

    [ComponentInfo(Description = "Two alternate blocks of Actions; which gets executed is defined by a condition")]
    public class IfElseMacroNode : ConditionMacroNode
    {
        [Persistent] public BlockMacroNode IfBlock = new BlockMacroNode();
        [Persistent] public BlockMacroNode ElseBlock = new BlockMacroNode();
        [Persistent] public int Control = -1;

        public IfElseMacroNode() 
        { IfBlock.Parent = this; ElseBlock.Parent = this; }

        public override void Load(ConfigNode node)
        {
            base.Load(node);
            if(IfBlock == null) IfBlock = new BlockMacroNode();
            if(ElseBlock == null) ElseBlock = new BlockMacroNode();
            IfBlock.Parent = this; ElseBlock.Parent = this;
        }

        public override bool Edit
        {    
            get { return base.Edit;    }
            set { base.Edit = value; IfBlock.Edit = value; ElseBlock.Edit = value; }
        }

        public override void Rewind()
        {
            base.Rewind();
            Control = -1;
            IfBlock.Rewind();
            ElseBlock.Rewind();
        }

        public override void SetSelector(Selector selector)
        {
            base.SetSelector(selector);
            IfBlock.SetSelector(selector);
            ElseBlock.SetSelector(selector);
        }

        public override void SetConditionSelector(Condition.Selector selector)
        {
            base.SetConditionSelector(selector);
            IfBlock.SetConditionSelector(selector);
            ElseBlock.SetConditionSelector(selector);
        }

        public override void SetCFG(VesselConfig cfg)
        {
            base.SetCFG(cfg);
            IfBlock.SetCFG(cfg);
            ElseBlock.SetCFG(cfg);
        }

        protected override bool Action(VesselWrapper VSL)
        {
            if(Control > 1) return false;
            if(Control < 0)    Control = ConditionsMet(VSL) ? 0 : 1;
            var ret = Control == 0? IfBlock.Execute(VSL) : ElseBlock.Execute(VSL);
            if(!ret) Control = 2;
            return ret;
        }

        protected override void DrawThis()
        {
            GUILayout.BeginVertical();
            base.DrawThis();
            IfBlock.Draw();
            if(Edit || ElseBlock.HasSubnodes)
            {
                GUILayout.Label("ELSE", Styles.label, GUILayout.ExpandWidth(false));
                ElseBlock.Draw();
            }
            GUILayout.EndVertical();
        }
    }

    [ComponentInfo(Description = "A loop that executes a block of Actions time after time while a condition is met")]
    public class WhileMacroNode : SingleBlockConditionMacroNode
    {
        public WhileMacroNode() { Keyword = "WHILE"; }

        protected override bool Action(VesselWrapper VSL)
        {
            if(Block.Active) { Block.Execute(VSL); return true; }
            if(ConditionsMet(VSL)) 
            { 
                if(Block.Done) Block.Rewind(); 
                Block.Execute(VSL);
                return true; 
            }
            return false;
        }
    }

    [ComponentInfo(Description = "A loop that executes a block of Actions specified number of times")]
    public class RepeatMacroNode : SingleBlockMacroNode
    {
        [Persistent] public int Count = 10;

        protected override bool Action(VesselWrapper VSL)
        {
            if(Count <= 0) return false;
            if(!Block.Execute(VSL)) 
            { Block.Rewind(); Count--; }
            return true; 
        }

        protected override void DrawThis()
        {
            GUILayout.BeginVertical();
            GUILayout.BeginHorizontal();
            GUILayout.Label("REPEAT", Styles.label, GUILayout.ExpandWidth(false));
            Count = Utils.IntSelector(Count, 1);
            GUILayout.FlexibleSpace();
            if(GUILayout.Button("Edit Block", Edit? Styles.active_button : Styles.normal_button, GUILayout.ExpandWidth(false)))
                Edit = !Edit;
            GUILayout.EndHorizontal();
            Block.Draw();
            GUILayout.EndVertical();
        }

        public override bool AddChild(MacroNode child)
        { Block.AddChild(child); return true; }

        public override bool AddSibling(MacroNode sibling)
        { return Parent != null && Parent.AddSibling(sibling); }
    }
}

