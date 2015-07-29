//   VerticalSpeedControl.cs
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
	public class VerticalSpeedControl : TCAModule
	{
		public class Config : ModuleConfig
		{
			new public const string NODE_NAME = "VSC";

			[Persistent] public float K0 = 2f, K1 = 10f, L1 = 1f;    //vertical speed limit control coefficients
			[Persistent] public float MaxSpeed              = 10f;   //max. positive vertical speed m/s (configuration limit)
			[Persistent] public float MinVSF                = 0.05f; //minimum vertical speed factor; so as not to lose control during a rapid descent
			[Persistent] public float BalanceCorrection     = 1.5f;  //multiplier for the vertical speed factor correction; 1.2 means +20% of thrust above the minimal value sufficient for zero balance
			[Persistent] public float TWRf                  = 2.0f;  //factor for the TWR adjustment of VerticalCutoff
			[Persistent] public float UpAf                  = 0.2f;  //factor for the upA adjustment of VerticalCutoff
			[Persistent] public float ASf                   = 2f;    //factor for the acceleration speed adjustment of VerticalCutoff
			[Persistent] public float DSf                   = 1f;    //factor for the deceleration speed adjustment of VerticalCutoff
		}
		static Config VSC { get { return TCAConfiguration.Globals.VSC; } }

		public VerticalSpeedControl(VesselWrapper vsl) { vessel = vsl; }

		public override void UpdateState()
		{ IsActive = (CFG.ControlAltitude || CFG.VerticalCutoff < VSC.MaxSpeed) && vessel.OnPlanet; }

		public void Update()
		{
			vessel.VSF = 1f;
			if(!IsActive) return;
			SetState(TCAState.VerticalSpeedControl);
			var upAF = -vessel.VerticalAccel
				*(vessel.VerticalAccel < 0? vessel.AccelSpeed : vessel.DecelSpeed)*VSC.UpAf;
			var setpoint = CFG.VerticalCutoff;
			if(!vessel.MaxTWR.Equals(0))
				setpoint = CFG.VerticalCutoff+(VSC.TWRf+upAF)/vessel.MaxTWR;
			//calculate new VSF
			var err = setpoint-vessel.VerticalSpeed;
			var K = Mathf.Clamp01(err
			                      /VSC.K0
			                      /Mathf.Pow(Utils.ClampL(vessel.VerticalAccel/VSC.K1+1, VSC.L1), 2f)
			                      +upAF);
			vessel.VSF = vessel.LandedOrSplashed? K : Utils.ClampL(K, VSC.MinVSF);
			//loosing altitude alert
			if(vessel.VerticalSpeed < 0 && vessel.VerticalSpeed < vessel.CFG.VerticalCutoff-0.1f && !vessel.LandedOrSplashed)
				SetState(TCAState.LoosingAltitude);
			//			Utils.CSV(VerticalSpeed, CFG.VerticalCutoff, maxTWR, VerticalAccel, upAF, setpoint-CFG.VerticalCutoff, K);//debug
		}
	}
}

