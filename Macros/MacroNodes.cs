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
	public abstract class LoopMacro : ProxyMacro
	{
		protected abstract void DrawLoopCondition();

		protected override void DrawThis()
		{
			GUILayout.BeginVertical();
			GUILayout.BeginHorizontal();
			base.DrawThis();
			DrawLoopCondition();
			GUILayout.EndHorizontal();
			GUILayout.BeginHorizontal();
			if(Macro != null)
			{
				if(GUILayout.Button("X", Styles.red_button, GUILayout.Width(20))) Macro = null;
				else Macro.Draw();
			}
			else 
			{
				GUILayout.Space(20);
				if(Edit && GUILayout.Button("Select Action", Styles.normal_button, GUILayout.ExpandWidth(true)))
				{ if(SelectNode != null) SelectNode(m => Macro = m); }
			}
			GUILayout.EndHorizontal();
			GUILayout.EndVertical();
		}
	}

	public class WhileLoopMacro : LoopMacro
	{
		[Persistent] public Condition LoopCondition;

		public WhileLoopMacro() { Name = "Do While:"; }

		protected override bool Action (VesselWrapper VSL)
		{
			if(Macro == null) return false;
			if(LoopCondition.True(VSL))
			{
				if(!Macro.Execute(VSL))
					Macro.Rewind();
				return true;
			}
			return false;
		}

		protected override void DrawLoopCondition ()
		{ if(LoopCondition != null) LoopCondition.Draw(); }
	}

	public class ForLoopMacro : LoopMacro
	{
		[Persistent] public int Count = 10;

		public ForLoopMacro() { Name = "Repeat:"; }

		protected override bool Action (VesselWrapper VSL)
		{
			if(Macro == null) return false;
			if(Count > 0)
			{
				if(!Macro.Execute(VSL))
				{
					Macro.Rewind();
					Count--;
				}
				return true;
			}
			return false;
		}

		protected override void DrawLoopCondition()
		{ Count = Utils.IntSelector(Count, 0); }
	}

	public abstract class SetFloat : MacroNode
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
				GUILayout.Label(Title, Styles.label);
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

	public class SetVerticalSpeed : SetFloat
	{
		public SetVerticalSpeed()
		{ Name = "Set vertical speed to:"; }

		protected override bool Action(VesselWrapper VSL)
		{
			VSL.CFG.VerticalCutoff = Value;
			return false;
		}
	}

	public class SetAltitude : SetFloat
	{
		public SetAltitude()
		{ Name = "Set altitude to:"; }

		protected override bool Action(VesselWrapper VSL)
		{
			VSL.CFG.DesiredAltitude = Value;
			return false;
		}
	}

	public class StopNode : MacroNode
	{
		public StopNode() { Name = "Stop"; }

		protected override bool Action(VesselWrapper VSL)
		{
			VSL.CFG.HF.On(HFlight.Stop);
			return false;
		}
	}

	public class LevelNode : MacroNode
	{
		public LevelNode() { Name = "Level"; }

		protected override bool Action(VesselWrapper VSL)
		{
			VSL.CFG.HF.On(HFlight.Level);
			return false;
		}
	}

	public class HoverNode : MacroNode
	{
		public HoverNode() { Name = "Hover"; }

		protected override bool Action(VesselWrapper VSL)
		{
			VSL.CFG.VF.On(VFlight.AltitudeControl);
			return false;
		}
	}

	public class AnchorNode : MacroNode
	{
		public AnchorNode() { Name = "Anchor"; }

		protected override bool Action(VesselWrapper VSL)
		{
			VSL.CFG.Nav.On(Navigation.AnchorHere);
			return false;
		}
	}

	public class GoToNode : MacroNode
	{
		public GoToNode() { Name = "Go To"; }

		protected override bool Action(VesselWrapper VSL)
		{
			if(!VSL.HasTarget) return true;
			VSL.CFG.Nav.On(Navigation.GoToTarget);
			return false;
		}
	}
}

