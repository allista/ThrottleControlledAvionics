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
		protected override void DrawThis()
		{ GUILayout.Label("Stop", Styles.red); }

		protected override bool Action(VesselWrapper VSL)
		{
			VSL.CFG.HF.On(HFlight.Stop);
			return false;
		}
	}
}

