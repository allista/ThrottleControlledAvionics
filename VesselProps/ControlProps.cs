//  Author:
//       Allis Tauri <allista@gmail.com>
//
//  Copyright (c) 2016 Allis Tauri
//
// This work is licensed under the Creative Commons Attribution-ShareAlike 4.0 International License. 
// To view a copy of this license, visit http://creativecommons.org/licenses/by-sa/4.0/ 
// or send a letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.
//
using UnityEngine;

namespace ThrottleControlledAvionics
{
	public class ControlProps : VesselProps
	{
		public ControlProps(VesselWrapper vsl) : base(vsl) {}

		public Vector3 Steering { get; private set; }
		public Vector3 Translation { get; private set; }
		public bool    TranslationAvailable { get; private set; }
		public bool    RCSAvailable;
		public Vector3 ManualTranslation;
		public Switch  ManualTranslationSwitch = new Switch();
		public float   GimbalLimit = 100;

		public override void Update()
		{
			Steering = new Vector3(vessel.ctrlState.pitch, vessel.ctrlState.roll, vessel.ctrlState.yaw);
			Translation = new Vector3(vessel.ctrlState.X, vessel.ctrlState.Z, vessel.ctrlState.Y);
			if(!Steering.IsZero()) //tune steering if MaxAA has changed drastically
				Steering = Steering*Utils.ClampH(VSL.Torque.MaxAAMod, 1)/Steering.CubeNorm().magnitude;
			if(!Translation.IsZero()) Translation = Translation/Translation.CubeNorm().magnitude;
			TranslationAvailable = VSL.Engines.Maneuver.Count > 0 || VSL.Engines.RCS.Count > 0 && VSL.vessel.ActionGroups[KSPActionGroup.RCS];
		}
	}
}

