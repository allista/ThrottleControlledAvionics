//  Author:
//       Allis Tauri <allista@gmail.com>
//
//  Copyright (c) 2016 Allis Tauri
//
// This work is licensed under the Creative Commons Attribution-ShareAlike 4.0 International License. 
// To view a copy of this license, visit http://creativecommons.org/licenses/by-sa/4.0/ 
// or send a letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.
//
using System;
using UnityEngine;

namespace ThrottleControlledAvionics
{

	public class OnPlanetProps : VesselProps
	{
		public OnPlanetProps(VesselWrapper vsl) : base(vsl) {}

		public Vector3 Fwd { get; private set; }  //fwd unit vector of the Control module in world space
		public Vector3 FwdL { get; private set; }  //fwd unit vector of the Control module in local space
		public bool    NoseUp { get; private set; }  //if the forward is refT.forward or refT.up
		public Vector3 Bearing { get; private set; }  //bearing unit vector of the Control module in world space

		public float   MaxTWR { get; private set; }
		public float   MaxDTWR { get; private set; }
		public float   DTWR { get; private set; }

		public float   AccelSpeed { get; private set; }
		public float   DecelSpeed { get; private set; }
		public bool    SlowThrust { get; private set; }
		public Vector3 ManualThrust { get; private set; }
		public Vector6 ManualThrustLimits { get; private set; } = Vector6.zero;

		public float   VSF; //vertical speed factor
		public float   MinVSF;
		public float   MinVSFtwr;

		public override void Update()
		{
			AccelSpeed = 0f; DecelSpeed = 0f; SlowThrust = false;
			//calculate total downward thrust and slow engines' corrections
			ManualThrust = Vector3.zero;
			ManualThrustLimits = Vector6.zero;
			var down_thrust = 0f;
			var slow_thrust = 0f;
			var fast_thrust = 0f;
			for(int i = 0; i < VSL.Engines.NumActive; i++)
			{
				var e = VSL.Engines.Active[i];
				e.VSF = 1f;
				if(e.thrustInfo == null) continue;
				if(e.isVSC)
				{
					var dcomponent = -Vector3.Dot(e.wThrustDir, VSL.Physics.Up);
					if(dcomponent <= 0) e.VSF = Utils.Clamp(Vector3.Dot(VSL.Controls.Steering, e.specificTorque), 0, 1);
					else 
					{
						var dthrust = e.nominalCurrentThrust(e.best_limit)*dcomponent;
						if(e.useEngineResponseTime && dthrust > 0) 
						{
							slow_thrust += dthrust;
							AccelSpeed += e.engineAccelerationSpeed*dthrust;
							DecelSpeed += e.engineDecelerationSpeed*dthrust;
						}
						else fast_thrust = dthrust;
						down_thrust += dthrust;
					}
				}
				if(e.Role == TCARole.MANUAL) 
				{
					ManualThrustLimits.Add(e.thrustDirection*e.nominalCurrentThrust(1));
					ManualThrust += e.wThrustDir*e.finalThrust;
				}
			}
			MaxTWR  = VSL.Engines.MaxThrustM/VSL.Physics.M/VSL.Physics.G;
			MaxDTWR = Utils.EWA(MaxDTWR, down_thrust/VSL.Physics.M/VSL.Physics.G, 0.1f);
			DTWR = Vector3.Dot(VSL.Engines.Thrust, VSL.Physics.Up) < 0? 
				Vector3.Project(VSL.Engines.Thrust, VSL.Physics.Up).magnitude/VSL.Physics.M/VSL.Physics.G : 0f;
			if(refT != null)
			{
				Fwd = Vector3.Cross(refT.right, -VSL.Engines.MaxThrust).normalized;
				FwdL = refT.InverseTransformDirection(Fwd);
				NoseUp = Vector3.Dot(Fwd, refT.forward) >= 0.9;
				Bearing = Vector3.ProjectOnPlane(Fwd, VSL.Physics.Up).normalized;
			}
			MinVSFtwr = 1/Utils.ClampL(MaxTWR, 1);
			var mVSFtor = (VSL.Torque.MaxPitchRollAA_m > 0)? 
				Utils.ClampH(GLB.VSC.MinVSFf/VSL.Torque.MaxPitchRollAA_m, GLB.VSC.MaxVSFtwr*MinVSFtwr) : 0;
			MinVSF = Mathf.Lerp(0, mVSFtor, Mathf.Pow(VSL.Controls.Steering.sqrMagnitude, 0.25f));
			var controllable_thrust = slow_thrust+fast_thrust;
			if(controllable_thrust.Equals(0)) return;
			//correct setpoint for current TWR and slow engines
			if(AccelSpeed > 0) AccelSpeed = controllable_thrust/AccelSpeed*GLB.VSC.ASf;
			if(DecelSpeed > 0) DecelSpeed = controllable_thrust/DecelSpeed*GLB.VSC.DSf;
			SlowThrust = AccelSpeed > 0 || DecelSpeed > 0;
		}
	}
}

