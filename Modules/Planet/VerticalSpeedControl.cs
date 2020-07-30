//  Author:
//       Allis Tauri <allista@gmail.com>
//
//  Copyright (c) 2015 Allis Tauri
//
// This work is licensed under the Creative Commons Attribution-ShareAlike 4.0 International License. 
// To view a copy of this license, visit http://creativecommons.org/licenses/by-sa/4.0/ 
// or send a letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.

using System.Diagnostics.CodeAnalysis;
using AT_Utils;
using UnityEngine;

namespace ThrottleControlledAvionics
{
    [CareerPart]
    public class VerticalSpeedControl : TCAModule
    {
        [SuppressMessage("ReSharper", "FieldCanBeMadeReadOnly.Global"),
         SuppressMessage("ReSharper", "ConvertToConstant.Global")]
        public class Config : ComponentConfig<Config>
        {
            [Persistent] public float K0 = 2f, K1 = 10f, L1 = 1f; //vertical speed limit control coefficients
            [Persistent] public float MaxSpeed = 10f; //max. positive vertical speed m/s (configuration limit)

            /// <summary>
            /// minimum vertical speed factor; so as not to lose control during a rapid descent
            /// </summary>
            [Persistent]
            public float MinVSFf = 1.2f;

            /// <summary>
            /// multiplier for the vertical speed factor correction; 1.2 means +20% of thrust
            /// above the minimal value sufficient for zero balance
            /// </summary>
            [Persistent]
            public float BalanceCorrection = 1.5f;

            /// <summary>
            /// factor for the TWR adjustment of VerticalCutoff
            /// </summary>
            [Persistent]
            public float TWRf = 2.0f;

            /// <summary>
            /// factor for the upA adjustment of VerticalCutoff
            /// </summary>
            [Persistent]
            public float UpAf = 0.2f;

            /// <summary>
            /// factor for the acceleration speed adjustment of VerticalCutoff
            /// </summary>
            [Persistent]
            public float ASf = 2f;

            /// <summary>
            /// factor for the deceleration speed adjustment of VerticalCutoff
            /// </summary>
            [Persistent]
            public float DSf = 1f;

            [Persistent] public float MaxDeltaV = 0.5f;
            [Persistent] public float FallingTime = 0.5f;
            [Persistent] public float AccelThreshold = 0.1f;
            [Persistent] public float MaxVSFtwr = 0.9f;
        }

        public static Config C => Config.INST;

        public VerticalSpeedControl(ModuleTCA tca) : base(tca) { }

        private bool overriden;
        private float setpoint_override;

        public float SetpointOverride
        {
            get => setpoint_override;
            set
            {
                setpoint_override = value;
                overriden = true;
            }
        }

        private float old_accel, accelV;
        private readonly EWA setpoint_correction = new EWA();
        private readonly Timer Falling = new Timer();

        public void SetVerticalCutoff(float cutoff)
        {
            TCA.SquadConfigAction(cfg =>
            {
                cfg.VerticalCutoff = cutoff;
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
            Falling.Period = C.FallingTime;
            if(VSL.LandedOrSplashed && CFG.VerticalCutoff >= 0 && CFG.VerticalCutoff < C.MaxSpeed)
                CFG.VerticalCutoff = -10;
        }

        protected override void Update()
        {
            var raw_setpoint = overriden ? SetpointOverride : CFG.VerticalCutoff;
            var setpoint = raw_setpoint;
            var upAF = -VSL.VerticalSpeed.Derivative
                       * C.UpAf
                       * (VSL.VerticalSpeed.Derivative < 0
                           ? VSL.OnPlanetParams.CurrentThrustAccelerationTime
                           : VSL.OnPlanetParams.CurrentThrustDecelerationTime);
            if(VSL.OnPlanetParams.MaxDTWR > 0)
                setpoint += (C.TWRf + upAF) / VSL.OnPlanetParams.MaxDTWR;
            //calculate new VSF
            if(!VSL.LandedOrSplashed)
            {
                accelV = (VSL.VerticalSpeed.Derivative - old_accel) / TimeWarp.fixedDeltaTime;
                old_accel = VSL.VerticalSpeed.Derivative;
                var missed =
                    VSL.VerticalSpeed.Absolute > raw_setpoint
                    && VSL.VerticalSpeed.Absolute < raw_setpoint + setpoint_correction
                    && VSL.VerticalSpeed.Derivative > 0
                    || VSL.VerticalSpeed.Absolute < raw_setpoint
                    && VSL.VerticalSpeed.Absolute > raw_setpoint + setpoint_correction
                    && VSL.VerticalSpeed.Derivative < 0;
                if(missed
                   || Mathf.Abs(VSL.VerticalSpeed.Derivative) < C.AccelThreshold
                   && Mathf.Abs(accelV) < C.AccelThreshold)
                    setpoint_correction.Update(
                        Utils.ClampL(raw_setpoint - VSL.VerticalSpeed.Absolute + setpoint_correction, 0));
            }
            var err = setpoint - VSL.VerticalSpeed.Absolute + setpoint_correction;
            var K = Mathf.Clamp01(err
                                  / C.K0
                                  / Mathf.Pow(Utils.ClampL(VSL.VerticalSpeed.Derivative / C.K1 + 1, C.L1), 2f)
                                  + upAF);
            VSL.OnPlanetParams.VSF = VSL.LandedOrSplashed ? K : Utils.ClampL(K, VSL.OnPlanetParams.MinVSF);
            overriden = false;
            if(VSL.LandedOrSplashed)
                return;
            //loosing altitude alert
            if(!CFG.VF)
            {
                if(VSL.VerticalSpeed.Absolute < 0
                   && raw_setpoint - VSL.VerticalSpeed.Absolute > C.MaxDeltaV)
                {
                    if(Falling.TimePassed)
                        SetState(TCAState.LoosingAltitude);
                }
                else
                    Falling.Reset();
            }
            // DebugWindowController.PostMessage($"VSC-{GetHashCode():X8}",
            //     $"setpoint: {setpoint:F1}/{raw_setpoint:F1} m/s\n"
            //     + $"error: {err:F1} m/s\n"
            //     + $"corr: {(float)setpoint_correction:F1} m/s\n"
            //     + $"VSF: {VSL.OnPlanetParams.VSF:F3}\n"
            //     + $"minVSF: {VSL.OnPlanetParams.MinVSF:F3}");
//            CSV(VSL.Altitude, VSL.Altitude.TerrainAltitude, VSL.VerticalSpeed.Relative,
//                           VSL.VerticalSpeed, raw_setpoint, setpoint, setpoint_correction, 
//                           VSL.VerticalSpeed.Derivative, upAF, K, VSL.VSF);//debug
        }

        public override void ProcessKeys()
        {
            var cutoff = CFG.VerticalCutoff;
            if(GameSettings.THROTTLE_UP.GetKey())
                cutoff = Mathf.Lerp(CFG.VerticalCutoff,
                    C.MaxSpeed,
                    CFG.ControlSensitivity);
            else if(GameSettings.THROTTLE_DOWN.GetKey())
                cutoff = Mathf.Lerp(CFG.VerticalCutoff,
                    -C.MaxSpeed,
                    CFG.ControlSensitivity);
            else if(GameSettings.THROTTLE_FULL.GetKeyDown())
                cutoff = C.MaxSpeed;
            else if(GameSettings.THROTTLE_CUTOFF.GetKeyDown())
                cutoff = -C.MaxSpeed;
            if(!cutoff.Equals(CFG.VerticalCutoff))
                SetVerticalCutoff(cutoff);
        }
    }
}
