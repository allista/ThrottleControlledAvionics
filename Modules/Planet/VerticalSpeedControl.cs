//  Author:
//       Allis Tauri <allista@gmail.com>
//
//  Copyright (c) 2015 Allis Tauri
//
// This work is licensed under the Creative Commons Attribution-ShareAlike 4.0 International License. 
// To view a copy of this license, visit http://creativecommons.org/licenses/by-sa/4.0/ 
// or send a letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.

using System;
using UnityEngine;
using AT_Utils;

namespace ThrottleControlledAvionics
{
	[CareerPart]
	public class VerticalSpeedControl : TCAModule
	{
		public class Config : ModuleConfig
		{
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
		static Config VSC { get { return Globals.Instance.VSC; } }

		public VerticalSpeedControl(ModuleTCA tca) : base(tca) {}

		bool overriden;
		float setpoint_override;
		public float SetpointOverride 
		{ 
			get { return setpoint_override; }
			set { setpoint_override = value; overriden = true; }
		}

		float old_accel, accelV;
		readonly EWA setpoint_correction = new EWA();
		readonly Timer Falling = new Timer();

		void set_vspeed(float vspeed)
		{
			TCA.SquadConfigAction(cfg =>
			{
				cfg.VerticalCutoff = vspeed;
				cfg.BlockThrottle |= cfg.VSCIsActive;
			});
		}

        public override void Disable()
        {
            CFG.DisableVSC();
        }

		protected override void UpdateState()
		{ 
			base.UpdateState();
            VSL.OnPlanetParams.VSF = 1f;
			IsActive &= VSL.OnPlanet && CFG.VSCIsActive;
		}

		public override void Init()
		{
			base.Init();
			overriden = false;
			Falling.Period = VSC.FallingTime;
			if(VSL.LandedOrSplashed && 
			   CFG.VerticalCutoff >= 0 && 
			   CFG.VerticalCutoff < VSC.MaxSpeed)
				CFG.VerticalCutoff = -10;
		}

		protected override void Update()
		{
			var raw_setpoint = overriden? SetpointOverride : CFG.VerticalCutoff;
			var setpoint = raw_setpoint;
			SetState(TCAState.VerticalSpeedControl);
            var upAF = -VSL.VerticalSpeed.Derivative * VSC.UpAf *
				(VSL.VerticalSpeed.Derivative < 0? 
                 VSL.OnPlanetParams.CurrentThrustAccelerationTime : 
                 VSL.OnPlanetParams.CurrentThrustDecelerationTime);
			if(VSL.OnPlanetParams.MaxDTWR > 0)
				setpoint += (VSC.TWRf+upAF)/VSL.OnPlanetParams.MaxDTWR;
			//calculate new VSF
			if(!VSL.LandedOrSplashed)
			{
				accelV = (VSL.VerticalSpeed.Derivative-old_accel)/TimeWarp.fixedDeltaTime;
				old_accel = VSL.VerticalSpeed.Derivative;
				var missed = VSL.VerticalSpeed.Absolute > raw_setpoint && VSL.VerticalSpeed.Absolute < raw_setpoint+setpoint_correction && VSL.VerticalSpeed.Derivative > 0 ||
					VSL.VerticalSpeed.Absolute < raw_setpoint && VSL.VerticalSpeed.Absolute > raw_setpoint+setpoint_correction && VSL.VerticalSpeed.Derivative < 0;
				if(missed || Mathf.Abs(VSL.VerticalSpeed.Derivative) < VSC.AccelThreshold && Mathf.Abs(accelV) < VSC.AccelThreshold)
					setpoint_correction.Update(Utils.ClampL(raw_setpoint-VSL.VerticalSpeed.Absolute+setpoint_correction, 0));
			}
			var err = setpoint-VSL.VerticalSpeed.Absolute+setpoint_correction;
			var K = Mathf.Clamp01(err
			                      /VSC.K0
			                      /Mathf.Pow(Utils.ClampL(VSL.VerticalSpeed.Derivative/VSC.K1+1, VSC.L1), 2f)
			                      +upAF);
			VSL.OnPlanetParams.VSF = VSL.LandedOrSplashed? K : Utils.ClampL(K, VSL.OnPlanetParams.MinVSF);
//			Log("VSP {0}, setpoint {1}, setpoint correction {2}, err {3}, K {4}, VSF {5}, DTWR {6}, MinVSF {7}, G {8}",
//			    raw_setpoint, setpoint, setpoint_correction, err, K, VSL.VSF, VSL.OnPlanetParams.DTWR, VSL.MinVSF, VSL.Physics.G);//debug
			overriden = false;
			if(VSL.LandedOrSplashed) return;
			//loosing altitude alert
			if(!CFG.VF) Falling.RunIf(() => SetState(TCAState.LoosingAltitude), 
				        		      VSL.VerticalSpeed.Absolute < 0 && raw_setpoint-VSL.VerticalSpeed.Absolute > VSC.MaxDeltaV);
//			CSV(VSL.Altitude, VSL.Altitude.TerrainAltitude, VSL.VerticalSpeed.Relative,
//			               VSL.VerticalSpeed, raw_setpoint, setpoint, setpoint_correction, 
//			               VSL.VerticalSpeed.Derivative, upAF, K, VSL.VSF);//debug
		}

		public override void ProcessKeys()
		{
			var cutoff = CFG.VerticalCutoff;
			if(GameSettings.THROTTLE_UP.GetKey())
				cutoff = Mathf.Lerp(CFG.VerticalCutoff, 
				                        GLB.VSC.MaxSpeed, 
				                        CFG.ControlSensitivity);
			else if(GameSettings.THROTTLE_DOWN.GetKey())
				cutoff = Mathf.Lerp(CFG.VerticalCutoff, 
				                        -GLB.VSC.MaxSpeed, 
				                        CFG.ControlSensitivity);
			else if(GameSettings.THROTTLE_FULL.GetKeyDown())
				cutoff = GLB.VSC.MaxSpeed;
			else if(GameSettings.THROTTLE_CUTOFF.GetKeyDown())
				cutoff = -GLB.VSC.MaxSpeed;
			if(!cutoff.Equals(CFG.VerticalCutoff)) set_vspeed(cutoff);
		}

		public override void Draw()
		{
			var speed = string.Format("V.Spd. {0}", (CFG.VerticalCutoff < GLB.VSC.MaxSpeed? 
			                                         Utils.formatBigValue(CFG.VerticalCutoff, "m/s", "+0.0;-0.0;+0.0") : "OFF"));
			GUILayout.Label(new GUIContent(speed, "Desired vertical speed"), Styles.boxed_label, GUILayout.ExpandWidth(false));
			var VSP = GUILayout.HorizontalSlider(CFG.VerticalCutoff, -GLB.VSC.MaxSpeed, GLB.VSC.MaxSpeed);
			if(Mathf.Abs(VSP-CFG.VerticalCutoff) > 1e-5) set_vspeed(VSP);
		}
	}
}

