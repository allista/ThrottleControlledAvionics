//  Author:
//       Allis Tauri <allista@gmail.com>
//
//  Copyright (c) 2015 Allis Tauri
//
// This work is licensed under the Creative Commons Attribution-ShareAlike 4.0 International License. 
// To view a copy of this license, visit http://creativecommons.org/licenses/by-sa/4.0/ 
// or send a letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.

using UnityEngine;
using AT_Utils;

namespace ThrottleControlledAvionics
{
	[CareerPart]
	[OptionalModules(typeof(VerticalSpeedControl))]
	[ModuleInputs(typeof(AttitudeControl))]
	public class ThrottleControl : AutopilotModule
	{
		public class Config : ModuleConfig
		{
			[Persistent] public float MinDeltaV        = 0.1f; //m/s
			[Persistent] public float DeltaVThreshold  = 10f;  //sec
			[Persistent] public float AttitudeDeadzone = 1f;   //deg
		}
		static Config THR { get { return Globals.Instance.THR; } }
		public ThrottleControl(ModuleTCA tca) : base(tca) {}

		float throttle = -1;
		public float DeltaV = -1;
		public float Throttle { get { return throttle; } set { throttle = value; DeltaV = -1; } }
		public bool CorrectThrottle = true;

		public float NextThrottle(float dV, float throttle)
		{ return NextThrottle(dV, throttle, VSL.Physics.M, VSL.Engines.MaxThrustM, VSL.Engines.DecelerationTime10); }

		public static float NextThrottle(float dV, float throttle, VesselWrapper VSL)
		{ return NextThrottle(dV, throttle, VSL.Physics.M, VSL.Engines.MaxThrustM, VSL.Engines.DecelerationTime10); }

		public static float NextThrottle(float dV, float throttle, float mass, float thrust, float deceleration_time)
		{
			var dt = Utils.Clamp(dV/THR.DeltaVThreshold, 0.5f, 2f);
			return Utils.Clamp((dV/thrust*mass-throttle*deceleration_time)/dt, 0f, 1f); 
		}

		public void BlockThrottle(bool state)
		{
			if(state == CFG.BlockThrottle) return;
			CFG.BlockThrottle = state;
			if(CFG.BlockThrottle && CFG.VerticalCutoff >= GLB.VSC.MaxSpeed)
				CFG.VerticalCutoff = 0;
		}

        public override void Disable() {}

		protected override void Reset()
		{
			base.Reset();
			Throttle = -1;
			CorrectThrottle = true;
		}

		static void set_engine_limit(EngineWrapper e, float errorF, float max_lim)
		{
			if(e.Role == TCARole.MANEUVER || e.Role == TCARole.MANUAL) return;
			if(e.Role == TCARole.BALANCE || e.Role == TCARole.UNBALANCE) e.preset_limit = errorF;
			else e.preset_limit = Mathf.Lerp(errorF, max_lim, e.torqueRatio);
		}

        void set_throttle(float t)
        {
            CS.mainThrottle = t; 
            VSL.vessel.ctrlState.mainThrottle = CS.mainThrottle;
            if(VSL.IsActiveVessel) 
                FlightInputHandler.state.mainThrottle = CS.mainThrottle;
        }

		protected override void OnAutopilotUpdate()
		{
			if(DeltaV >= 0)
			{
				throttle = DeltaV < THR.MinDeltaV? throttle = 0 :
					NextThrottle(DeltaV, CS.mainThrottle);
				if(CorrectThrottle)
				{
					var errorF = VSL.Controls.OffsetAlignmentFactor(THR.AttitudeDeadzone);
				 	if(errorF < 1) 
					{
						var max_lim = Utils.Clamp(Mathf.Abs(VSL.Controls.Steering.MaxComponentF()), errorF, 1);
						VSL.Engines.Active.ForEach(e => set_engine_limit(e, errorF, max_lim));
					}
				}
			}
			if(Throttle >= 0) 
                set_throttle(throttle);
			else if(CFG.BlockThrottle && VSL.OnPlanet)
                set_throttle(VSL.LandedOrSplashed && CFG.VerticalCutoff <= 0? 0 : 1);
			Reset();
		}

		public override void Draw()
		{
			if(Utils.ButtonSwitch("AutoThrottle", CFG.BlockThrottle, 
			                      CFG.VF[VFlight.AltitudeControl]?
			                      "Change altitude with throttle controls" :
			                      "Set vertical speed with throttle controls",
			                      GUILayout.ExpandWidth(true)))
				BlockThrottle(!CFG.BlockThrottle);
		}
	}
}

