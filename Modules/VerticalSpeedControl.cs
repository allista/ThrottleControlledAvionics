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
			[Persistent] public float MinVSFf               = 1.2f;  //minimum vertical speed factor; so as not to lose control during a rapid descent
			[Persistent] public float BalanceCorrection     = 1.5f;  //multiplier for the vertical speed factor correction; 1.2 means +20% of thrust above the minimal value sufficient for zero balance
			[Persistent] public float TWRf                  = 2.0f;  //factor for the TWR adjustment of VerticalCutoff
			[Persistent] public float UpAf                  = 0.2f;  //factor for the upA adjustment of VerticalCutoff
			[Persistent] public float ASf                   = 2f;    //factor for the acceleration speed adjustment of VerticalCutoff
			[Persistent] public float DSf                   = 1f;    //factor for the deceleration speed adjustment of VerticalCutoff
			[Persistent] public float MaxDeltaV             = 0.5f;  //maximum VS delta that is considered safe enough not to alarm user about Loosing Altitude
		}
		static Config VSC { get { return TCAConfiguration.Globals.VSC; } }

		public VerticalSpeedControl(VesselWrapper vsl) { VSL = vsl; }

		public override void UpdateState()
		{ IsActive = (CFG.ControlAltitude || CFG.VerticalCutoff < VSC.MaxSpeed) && VSL.OnPlanet; }

		float deltaV;

		public void Update()
		{
			VSL.VSF = 1f;
			if(!IsActive) return;
			SetState(TCAState.VerticalSpeedControl);
			var upAF = -VSL.VerticalAccel
				*(VSL.VerticalAccel < 0? VSL.AccelSpeed : VSL.DecelSpeed)*VSC.UpAf;
			var setpoint = CFG.VerticalCutoff;
			if(!VSL.MaxTWR.Equals(0))
				setpoint = CFG.VerticalCutoff+(VSC.TWRf+upAF)/VSL.MaxTWR;
			//calculate new VSF
			var err = setpoint-VSL.VerticalSpeed;
			var K = Mathf.Clamp01(err
			                      /VSC.K0
			                      /Mathf.Pow(Utils.ClampL(VSL.VerticalAccel/VSC.K1+1, VSC.L1), 2f)
			                      +upAF);
			VSL.VSF = VSL.LandedOrSplashed? K : Utils.ClampL(K, VSL.MinVSF);
			//loosing altitude alert
			if(VSL.LandedOrSplashed) return;
			var dV = VSL.CFG.VerticalCutoff-VSL.VerticalSpeed;
			if(VSL.VerticalSpeed < 0 && dV > 0)
			{
				if(dV < VSC.MaxDeltaV) deltaV += dV*TimeWarp.fixedDeltaTime;
				else SetState(TCAState.LoosingAltitude);
			}
			else deltaV = 0;
//			Utils.CSV(VerticalSpeed, CFG.VerticalCutoff, maxTWR, VerticalAccel, upAF, setpoint-CFG.VerticalCutoff, K);//debug
		}
	}
}

