//  Author:
//       Allis Tauri <allista@gmail.com>
//
//  Copyright (c) 2016 Allis Tauri
//
// This work is licensed under the Creative Commons Attribution-ShareAlike 4.0 International License. 
// To view a copy of this license, visit http://creativecommons.org/licenses/by-sa/4.0/ 
// or send a letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.
//

namespace ThrottleControlledAvionics
{
	public class HorizontalSpeedProps : VesselProps
	{
		public HorizontalSpeedProps(VesselWrapper vsl) : base(vsl) {}

		public Vector3d Vector { get; private set; }
		public Vector3d NeededVector { get; private set; }
		public Vector3d normalized { get { return Vector.normalized; } }
		public float    Absolute { get; private set; }
		public bool     MoovingFast;
		public bool     Mooving;

		public static implicit operator float(HorizontalSpeedProps hsp) { return hsp.Absolute; }

		public override void Update()
		{
			Vector = Vector3d.Exclude(VSL.Physics.Up, vessel.srf_velocity);
			Absolute = (float)Vector.magnitude;
			Mooving = Absolute > GLB.HSC.TranslationMinDeltaV;
			MoovingFast = Absolute > GLB.RAD.MinClosingSpeed;
		}

		public Vector3d Predicted(float time) 
		{ return Vector3d.Exclude(VSL.Physics.Up, vessel.srf_velocity+vessel.acceleration*time); }

		public void SetNeeded(Vector3d nV)
		{
			NeededVector = nV;
		}
	}
}

