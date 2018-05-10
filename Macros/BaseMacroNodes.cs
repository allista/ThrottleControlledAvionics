//  Author:
//       Allis Tauri <allista@gmail.com>
//
//  Copyright (c) 2015 Allis Tauri
//
// This work is licensed under the Creative Commons Attribution-ShareAlike 4.0 International License. 
// To view a copy of this license, visit http://creativecommons.org/licenses/by-sa/4.0/ 
// or send a letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.

using UnityEngine;
using AT_Utils;

namespace ThrottleControlledAvionics
{
    [ComponentInfo(Hidden = true)]
    public class SingleBlockMacroNode : MacroNode
    {
        [Persistent] public BlockMacroNode Block = new BlockMacroNode();

        public SingleBlockMacroNode() { Block.Parent = this; }

        public override void Load(ConfigNode node)
        {
            base.Load(node);
            if(Block == null) Block = new BlockMacroNode();
            Block.Parent = this;
        }

        public override bool Edit
        {    
            get { return base.Edit;    }
            set { base.Edit = value; Block.Edit = value; }
        }

        public override void Rewind()
        {
            base.Rewind();
            Block.Rewind();
        }

        public override void SetSelector(Selector selector)
        {
            base.SetSelector(selector);
            Block.SetSelector(selector);
        }

        public override void SetConditionSelector(Condition.Selector selector)
        {
            base.SetConditionSelector(selector);
            Block.SetConditionSelector(selector);
        }

        public override void SetCFG(VesselConfig cfg)
        {
            base.SetCFG(cfg);
            Block.SetCFG(cfg);
        }

        protected override void DrawThis()
        {
            GUILayout.BeginVertical();
            base.DrawThis();
            Block.Draw();
            GUILayout.EndVertical();
        }
    }

    [ComponentInfo(Hidden = true)]
    public class SingleBlockConditionMacroNode : ConditionMacroNode
    {
        [Persistent] public BlockMacroNode Block = new BlockMacroNode();

        public SingleBlockConditionMacroNode() { Block.Parent = this; }

        public override void Load(ConfigNode node)
        {
            base.Load(node);
            if(Block == null) Block = new BlockMacroNode();
            Block.Parent = this;
        }

        public override bool Edit
        {    
            get { return base.Edit;    }
            set { base.Edit = value; Block.Edit = value; }
        }

        public override void Rewind()
        {
            base.Rewind();
            Block.Rewind();
        }

        public override void SetSelector(Selector selector)
        {
            base.SetSelector(selector);
            Block.SetSelector(selector);
        }

        public override void SetConditionSelector(Condition.Selector selector)
        {
            base.SetConditionSelector(selector);
            Block.SetConditionSelector(selector);
        }

        public override void SetCFG(VesselConfig cfg)
        {
            base.SetCFG(cfg);
            Block.SetCFG(cfg);
        }

        protected override void DrawThis()
        {
            GUILayout.BeginVertical();
            base.DrawThis();
            Block.Draw();
            GUILayout.EndVertical();
        }

        public override bool AddChild(MacroNode child)
        { Block.AddChild(child); return true; }

        public override bool AddSibling(MacroNode sibling)
        { return Parent != null && Parent.AddSibling(sibling); }
    }

    [ComponentInfo(Hidden = true)]
    public class OnOffMacroNode : MacroNode
    {
        [Persistent] public bool On;

        protected override void DrawThis()
        {
            GUILayout.Label(Label, Styles.white, GUILayout.ExpandWidth(false));
            if(GUILayout.Button(On? "On" : "Off", 
                                On? Styles.enabled_button : Styles.active_button, 
                                GUILayout.ExpandWidth(false)))
                On = !On;
        }
    }

    [ComponentInfo(Hidden = true)]
    public class SetFloatMacroNode : MacroNode
    {
        [Persistent] public FloatField Value = new FloatField();
        protected string Suffix;

        protected virtual void OnValueChanged() {}

        protected override void DrawThis()
        {
            GUILayout.BeginHorizontal();
            if(Edit)
            { 
                GUILayout.Label(Label, Styles.white, GUILayout.ExpandWidth(false));
                if(Value.Draw(Suffix))
                { 
                    OnValueChanged();
                    Edit = false; 
                }
            }
            else Edit |= GUILayout.Button(new GUIContent(string.Format("{0} {1}{2}", Name, Value, Suffix), Label.tooltip), Styles.normal_button);
            GUILayout.EndHorizontal();
        }
    }
}

