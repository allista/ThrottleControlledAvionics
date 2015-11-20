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
			[Persistent] public float MaxSpeed          = 10f;   //max. positive vertical speed m/s (configuration limit)
			[Persistent] public float MinVSFf           = 1.2f;  //minimum vertical speed factor; so as not to lose control during a rapid descent
			[Persistent] public float BalanceCorrection = 1.5f;  //multiplier for the vertical speed factor correction; 1.2 means +20% of thrust above the minimal value sufficient for zero balance
			[Persistent] public float TWRf              = 2.0f;  //factor for the TWR adjustment of VerticalCutoff
			[Persistent] public float UpAf              = 0.2f;  //factor for the upA adjustment of VerticalCutoff
			[Persistent] public float ASf               = 2f;    //factor for the acceleration speed adjustment of VerticalCutoff
			[Persistent] public float DSf               = 1f;    //factor for the deceleration speed adjustment of VerticalCutoff
			[Persistent] public float MaxDeltaV         = 0.5f;
			[Persistent] public float FallingTime       = 0.5f;
			[Persistent] public float AccelThreshold    = 0.1f;
			[Persistent] public float MaxVSFtwr         = 0.9f;
		}
		static Config VSC { get { return TCAScenario.Globals.VSC; } }

		public VerticalSpeedControl(ModuleTCA tca) { TCA = tca; }

		float old_accel, accelV;
		readonly EWA setpoint_correction = new EWA();
		readonly Timer Falling = new Timer();

		protected override void UpdateState()
		{ IsActive = VSL.OnPlanet && CFG.VSCIsActive; }

		public override void Init()
		{
			base.Init();
			Falling.Period = VSC.FallingTime;
			if(VSL.LandedOrSplashed && 
			   CFG.VerticalCutoff >= 0 && 
			   CFG.VerticalCutoff < VSC.MaxSpeed)
				CFG.VerticalCutoff = -10;
		}

		protected override void Update()
		{
			VSL.VSF = 1f;
			if(!IsActive) return;
			SetState(TCAState.VerticalSpeedControl);
			var upAF = -VSL.VerticalAccel
				*(VSL.VerticalAccel < 0? VSL.AccelSpeed : VSL.DecelSpeed)*VSC.UpAf;
			var setpoint = CFG.VerticalCutoff;
			if(VSL.MaxDTWR > 0)
				setpoint = CFG.VerticalCutoff+(VSC.TWRf+upAF)/VSL.MaxDTWR;
			//calculate new VSF
			if(!VSL.LandedOrSplashed)
			{
				accelV = (VSL.VerticalAccel-old_accel)/TimeWarp.fixedDeltaTime;
				old_accel = VSL.VerticalAccel;
				var missed = VSL.VerticalSpeed > CFG.VerticalCutoff && VSL.VerticalSpeed < CFG.VerticalCutoff+setpoint_correction && VSL.VerticalAccel > 0 ||
					VSL.VerticalSpeed < CFG.VerticalCutoff && VSL.VerticalSpeed > CFG.VerticalCutoff+setpoint_correction && VSL.VerticalAccel < 0;
				if(missed || Mathf.Abs(VSL.VerticalAccel) < VSC.AccelThreshold && Mathf.Abs(accelV) < VSC.AccelThreshold)
					setpoint_correction.Update(Utils.ClampL(CFG.VerticalCutoff-VSL.VerticalSpeed+setpoint_correction, 0));
			}
			var err = setpoint-VSL.VerticalSpeed+setpoint_correction;
			var K = Mathf.Clamp01(err
			                      /VSC.K0
			                      /Mathf.Pow(Utils.ClampL(VSL.VerticalAccel/VSC.K1+1, VSC.L1), 2f)
			                      +upAF);
			VSL.VSF = VSL.LandedOrSplashed? K : Utils.ClampL(K, VSL.MinVSF);
//			Log("VSP {0}, setpoint {1}, setpoint correction {2}, err {3}, K {4}, VSF {5}, DTWR {6}, MinVSF {7}, G {8}",
//			    CFG.VerticalCutoff, setpoint, setpoint_correction, err, K, VSL.VSF, VSL.DTWR, VSL.MinVSF, VSL.G);//debug
			if(VSL.LandedOrSplashed) return;
			//loosing altitude alert
			if(!CFG.VF) Falling.RunIf(() => SetState(TCAState.LoosingAltitude), 
				        		      VSL.VerticalSpeed < 0 && VSL.CFG.VerticalCutoff-VSL.VerticalSpeed > VSC.MaxDeltaV);
//			CSV(VSL.Altitude, VSL.TerrainAltitude, VSL.RelVerticalSpeed,
//			               VSL.VerticalSpeed, CFG.VerticalCutoff, setpoint, setpoint_correction, 
//			               VSL.VerticalAccel, upAF, K, VSL.VSF);//debug
		}
	}
}

