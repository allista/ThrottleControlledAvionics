//  Author:
//       Allis Tauri <allista@gmail.com>
//
//  Copyright (c) 2015 Allis Tauri
//
// This work is licensed under the Creative Commons Attribution-ShareAlike 4.0 International License. 
// To view a copy of this license, visit http://creativecommons.org/licenses/by-sa/4.0/ 
// or send a letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.

using System;

namespace ThrottleControlledAvionics
{
	public class ThrottleControl : AutopilotModule
	{
		public class Config : ModuleConfig
		{
			new public const string NODE_NAME = "THR";

			[Persistent] public float MinDeltaV        = 0.1f; //m/s
			[Persistent] public float DeltaVThreshold  = 10f;  //sec
		}
		static Config THR { get { return TCAScenario.Globals.THR; } }
		public ThrottleControl(ModuleTCA tca) { TCA = tca; }

		public float Throttle = -1;
		public float DeltaV;

		public float NextThrottle(float dV, float thrust, float throttle)
		{ 
			var dt = Utils.Clamp(dV/THR.DeltaVThreshold, 0.5f, 2f);
			return Utils.Clamp((dV/dt/thrust*VSL.M-throttle*VSL.ThrustDecelerationTime/dt), 0f, 1f); 
		}

		public void BlockThrottle(bool state)
		{
			if(state == CFG.BlockThrottle) return;
			CFG.BlockThrottle = state;
			if(CFG.BlockThrottle && CFG.VerticalCutoff >= GLB.VSC.MaxSpeed)
				CFG.VerticalCutoff = 0;
		}

		void reset()
		{
			Throttle = -1;
			DeltaV = 0;
		}

		protected override void OnAutopilotUpdate(FlightCtrlState s)
		{
			if(!CFG.Enabled) return;
			if(DeltaV >= THR.MinDeltaV)
				Throttle = NextThrottle(DeltaV, VSL.MaxThrustM, VSL.ctrlState.mainThrottle) *
					TCA.ATC.AttitudeFactor;
			if(Throttle >= 0) 
			{ 
				s.mainThrottle = Throttle; 
				VSL.ctrlState.mainThrottle = Throttle; 
				if(VSL.IsActiveVessel) FlightInputHandler.state.mainThrottle = Throttle;
			}
			else if(CFG.BlockThrottle)
				s.mainThrottle = VSL.LandedOrSplashed && CFG.VerticalCutoff <= 0? 0f : 1f;
			reset();
		}
	}
}

