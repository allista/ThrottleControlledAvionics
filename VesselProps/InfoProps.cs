//  Author:
//       Allis Tauri <allista@gmail.com>
//
//  Copyright (c) 2016 Allis Tauri
//
// This work is licensed under the Creative Commons Attribution-ShareAlike 4.0 International License. 
// To view a copy of this license, visit http://creativecommons.org/licenses/by-sa/4.0/ 
// or send a letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.
//

using System.Collections.Generic;
using UnityEngine;

namespace ThrottleControlledAvionics
{
	public class InfoProps : VesselProps
	{
		public InfoProps(VesselWrapper vsl) : base(vsl) {}

		const string ElectricChargeName = "ElectricCharge";
		static PartResourceDefinition _electric_charge;
		public static PartResourceDefinition ElectricCharge
		{ 
			get
			{ 
				if(_electric_charge == null)
					_electric_charge = PartResourceLibrary.Instance.GetDefinition(ElectricChargeName);
				return _electric_charge;
			} 
		}

		public bool ElectricChargeAvailible
		{
			get
			{
				var ec = vessel.GetActiveResource(ElectricCharge);
				return ec != null && ec.amount > 0;
			}
		}

		public Vector3 Destination;
		public double Countdown;
		public float  TTB;

		public List<Vector3d> CustomMarkersVec = new List<Vector3d>();
		public List<WayPoint> CustomMarkersWP  = new List<WayPoint>();

		public override void ClearFrameState()
		{
			CustomMarkersWP.Clear();
			CustomMarkersVec.Clear();
			Destination = Vector3.zero;
			Countdown = -1;
			TTB = -1;
		}

		public override void Update() {}
	}
}

