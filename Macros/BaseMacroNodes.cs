//   BaseMacroNodes.cs
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
	[HiddenComponent]
	public class SingleBlockMacroNode : MacroNode
	{
		[Persistent] public BlockMacroNode Block = new BlockMacroNode();

		public SingleBlockMacroNode() { Block.Parent = this; }

		public override void Load(ConfigNode node)
		{
			base.Load(node);
			if(Block == null) Block = new BlockMacroNode();
		}

		public override bool Edit
		{	
			get { return base.Edit;	}
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

	[HiddenComponent]
	public class SingleBlockConditionMacroNode : ConditionMacroNode
	{
		[Persistent] public BlockMacroNode Block = new BlockMacroNode();

		public SingleBlockConditionMacroNode() { Block.Parent = this; }

		public override void Load(ConfigNode node)
		{
			base.Load(node);
			if(Block == null) Block = new BlockMacroNode();
		}

		public override bool Edit
		{	
			get { return base.Edit;	}
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

	[HiddenComponent]
	public class OnOffMacroNode : MacroNode
	{
		[Persistent] public bool On;

		protected override void DrawThis()
		{
			GUILayout.Label(Name, Styles.white, GUILayout.ExpandWidth(false));
			if(GUILayout.Button(On? "On" : "Off", 
			                    On? Styles.green_button : Styles.yellow_button, 
			                    GUILayout.ExpandWidth(false)))
				On = !On;
		}
	}

	[HiddenComponent]
	public class SetFloatMacroNode : MacroNode
	{
		[Persistent] public float Value;
		protected string Suffix;

		protected readonly FloatField ValueField = new FloatField();
		protected virtual void OnValueChanged() {}

		protected override void DrawThis()
		{
			GUILayout.BeginHorizontal();
			if(Edit)
			{ 
				GUILayout.Label(Name, Styles.white, GUILayout.ExpandWidth(false));
				if(ValueField.Draw(Value))
				{ 
					Value = ValueField.Value; 
					OnValueChanged();
					Edit = false; 
				}
			}
			else Edit |= GUILayout.Button(string.Format("{0} {1:F1}{2}", Name, Value, Suffix), Styles.normal_button);
			GUILayout.EndHorizontal();
		}
	}
}

