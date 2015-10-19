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
	{ protected override bool Evaluate(VesselWrapper VSL) { return true; } }

	public abstract class AltitudeCondition : Condition
	{ [Persistent] public float Altitude; }

	public class AltLower : AltitudeCondition
	{
		public override void Draw()
		{ Utils.FloatSlider("Altitude is lower than", Altitude, float.MinValue, float.MaxValue, "0 m"); }

		protected override bool Evaluate(VesselWrapper VSL)
		{ return VSL.Altitude < Altitude; }
	}

	public class AltHigher : AltitudeCondition
	{
		public override void Draw()
		{ Utils.FloatSlider("Altitude is higher than", Altitude, float.MinValue, float.MaxValue, "0 m"); }

		protected override bool Evaluate(VesselWrapper VSL)
		{ return VSL.Altitude > Altitude; }
	}
}