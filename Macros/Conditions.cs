//   Conditions.cs
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
	public class TrueCondition : Condition
	{ 
		protected override void DrawThis()
		{ GUILayout.Label("TRUE", Styles.label, GUILayout.Width(50)); }
		protected override bool Evaluate(VesselWrapper VSL) { return true; } 
	}

	public class FalseCondition : Condition
	{ 
		protected override void DrawThis()
		{ GUILayout.Label("FALSE", Styles.label, GUILayout.Width(50)); }
		protected override bool Evaluate(VesselWrapper VSL) { return false; } 
	}

	public abstract class FloatCondition : Condition
	{ 
		[Persistent] public float Value;
		protected string Title;
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
			else Edit |= GUILayout.Button (string.Format ("{0} {1:F1}{2}", Title, Value, Suffix), Styles.normal_button);
			GUILayout.EndHorizontal();
		}
	}

	public abstract class IntCondition : Condition
	{ 
		[Persistent] public int Value;
		protected string Title;
		protected int Min = 0, Max = int.MaxValue;

		protected virtual void OnValueChanged() {}

		protected override void DrawThis()
		{
			GUILayout.BeginHorizontal();
			if(Edit)
			{ 
				GUILayout.Label(Title, Styles.label);
				var new_value = Utils.IntSelector(Value, Min, Max);
				if(new_value != Value) 
				{ 
					Value = new_value;
					OnValueChanged(); 
					Edit = false;
				} 
			}
			else Edit |= GUILayout.Button(string.Format ("{0} {1:D}", Title, Value), Styles.normal_button);
			GUILayout.EndHorizontal();
		}
	}

	public class AltLower : FloatCondition
	{
		public AltLower() { Title = "Alt <"; Suffix = "m"; }

		protected override bool Evaluate(VesselWrapper VSL)
		{ return VSL.Altitude < Value; }
	}

	public class AltHigher : FloatCondition
	{
		public AltHigher() { Title = "Alt >"; Suffix = "m"; }
		
		protected override bool Evaluate(VesselWrapper VSL)
		{ return VSL.Altitude > Value; }
	}

	public class TimerCondition : FloatCondition
	{
		protected readonly Timer T = new Timer();

		public TimerCondition()
		{ Title = "Wait for"; Suffix = "s"; }

		protected override void OnValueChanged()
		{ T.Period = Value; T.Reset(); }

		protected override bool Evaluate(VesselWrapper VSL)
		{ return T.Check; }
	}
}