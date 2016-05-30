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

namespace ThrottleControlledAvionics
{
	[CareerPart]
	[OptionalModules(typeof(VerticalSpeedControl))]
	[ModuleInputs(typeof(AttitudeControl))]
	public class ThrottleControl : AutopilotModule
	{
		public class Config : ModuleConfig
		{
			new public const string NODE_NAME = "THR";

			[Persistent] public float MinDeltaV        = 0.1f; //m/s
			[Persistent] public float DeltaVThreshold  = 10f;  //sec
		}
		static Config THR { get { return TCAScenario.Globals.THR; } }
		public ThrottleControl(ModuleTCA tca) : base(tca) {}

		AttitudeControl ATC;

		public float Throttle = -1;
		public float DeltaV = -1;

		public float NextThrottle(float dV, float throttle)
		{ 
			var dt = Utils.Clamp(dV/THR.DeltaVThreshold, 0.5f, 2f);
			return Utils.Clamp((dV/VSL.Engines.MaxThrustM*VSL.Physics.M-throttle*VSL.Engines.ThrustDecelerationTime)/dt, 0f, 1f); 
		}

		public void BlockThrottle(bool state)
		{
			if(state == CFG.BlockThrottle) return;
			CFG.BlockThrottle = state;
			if(CFG.BlockThrottle && CFG.VerticalCutoff >= GLB.VSC.MaxSpeed)
				CFG.VerticalCutoff = 0;
		}

		protected override void reset()
		{
			base.reset();
			Throttle = -1;
			DeltaV = -1;
		}

		protected override void OnAutopilotUpdate(FlightCtrlState s)
		{
			if(!CFG.Enabled) return;
			if(DeltaV >= 0)
			{
				if(DeltaV >= THR.MinDeltaV)
				{
					Throttle = NextThrottle(DeltaV, VSL.vessel.ctrlState.mainThrottle) *
						(ATC == null? 1f : ATC.AttitudeFactor);
				}
				else Throttle = 0;
			}
			if(Throttle >= 0) 
			{ 
				s.mainThrottle = Throttle; 
				VSL.vessel.ctrlState.mainThrottle = Throttle; 
				if(VSL.IsActiveVessel) FlightInputHandler.state.mainThrottle = Throttle;
			}
			else if(CFG.BlockThrottle && VSL.OnPlanet)
				s.mainThrottle = VSL.LandedOrSplashed && CFG.VerticalCutoff <= 0? 0f : 1f;
			reset();
		}

		public override void Draw()
		{
			if(Utils.ButtonSwitch("AutoThrottle", CFG.BlockThrottle, 
			                      CFG.VF[VFlight.AltitudeControl]?
			                      "Change altitude with throttle controls" :
			                      "Set vertical speed with throttle controls",
			                      GUILayout.ExpandWidth(false)))
				BlockThrottle(!CFG.BlockThrottle);
		}
	}
}

