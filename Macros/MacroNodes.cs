//   MacroNodes.cs
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
	[HiddenComponent]
	public class SingleBlockMacroNode : MacroNode
	{
		[Persistent] public BlockMacroNode Block = new BlockMacroNode();

		public SingleBlockMacroNode() { Block.Parent = this; }

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

	public class IfElseMacroNode : ConditionMacroNode
	{
		[Persistent] public BlockMacroNode IfBlock = new BlockMacroNode();
		[Persistent] public BlockMacroNode ElseBlock = new BlockMacroNode();
		[Persistent] public int Control = -1;

		public IfElseMacroNode() 
		{ IfBlock.Parent = this; ElseBlock.Parent = this; }

		public override bool Edit
		{	
			get { return base.Edit;	}
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

		protected override bool Action(VesselWrapper VSL)
		{
			if(Control > 1) return false;
			if(Control < 0)	Control = ConditionsMet(VSL) ? 0 : 1;
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
			if(GUILayout.Button("Edit Block", Edit? Styles.yellow_button : Styles.normal_button, GUILayout.ExpandWidth(false)))
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
				GUILayout.Label(Title, Styles.white, GUILayout.ExpandWidth(false));
				if(ValueField.Draw(Value)) 
				{ 
					Value = ValueField.Value; 
					OnValueChanged();
					Edit = false; 
				}
			}
			else if(GUILayout.Button(string.Format("{0} {1:F1}{2}", Title, Value, Suffix), 
			                         Styles.normal_button)) 
				Edit = true;
			GUILayout.EndHorizontal();
		}
	}

	public class SetVerticalSpeed : SetFloatMacroNode
	{
		public SetVerticalSpeed()
		{ Name = "Set vertical speed to:"; Suffix = "m/s"; }

		protected override bool Action(VesselWrapper VSL)
		{
			VSL.CFG.VerticalCutoff = Value;
			return false;
		}
	}

	public class SetAltitude : SetFloatMacroNode
	{
		public SetAltitude()
		{ Name = "Set altitude to:"; Suffix = "m"; }

		protected override bool Action(VesselWrapper VSL)
		{
			VSL.CFG.DesiredAltitude = Value;
			return false;
		}
	}

	public class StopMacroNode : MacroNode
	{
		public StopMacroNode() { Name = "Stop"; }

		protected override bool Action(VesselWrapper VSL)
		{
			VSL.CFG.HF.On(HFlight.Stop);
			return false;
		}
	}

	public class LevelMacroNode : MacroNode
	{
		public LevelMacroNode() { Name = "Level"; }

		protected override bool Action(VesselWrapper VSL)
		{
			VSL.CFG.HF.On(HFlight.Level);
			return false;
		}
	}

	public class HoverMacroNode : MacroNode
	{
		public HoverMacroNode() { Name = "Hover"; }

		protected override bool Action(VesselWrapper VSL)
		{
			VSL.CFG.VF.On(VFlight.AltitudeControl);
			return false;
		}
	}

	public class AnchorMacroNode : MacroNode
	{
		public AnchorMacroNode() { Name = "Anchor"; }

		protected override bool Action(VesselWrapper VSL)
		{
			VSL.CFG.Nav.On(Navigation.AnchorHere);
			return false;
		}
	}

	public class GoToMacroNode : MacroNode
	{
		public GoToMacroNode() { Name = "Go To"; }

		protected override bool Action(VesselWrapper VSL)
		{
			if(!VSL.HasTarget) return true;
			VSL.CFG.Nav.On(Navigation.GoToTarget);
			return false;
		}
	}

	public class WaitMacroNode : SetFloatMacroNode
	{
		protected readonly Timer T = new Timer();

		public WaitMacroNode()
		{ Name = "Wait for"; Suffix = "s"; Value = (float)T.Period; }

		protected override void OnValueChanged()
		{ T.Period = Value; T.Reset(); }

		protected override bool Action(VesselWrapper VSL)
		{ return !T.Check; }
	}
}

