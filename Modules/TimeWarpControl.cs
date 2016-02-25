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
	[CareerPart(typeof(ManeuverAutopilot))]
	public class TimeWarpControl : TCAModule
	{
		public class Config : ModuleConfig
		{
			new public const string NODE_NAME = "WRP";

			[Persistent] public float DewarpTime = 20f;  //sec
			[Persistent] public float MaxWarp    = 10000f;
		}
		static Config WRP { get { return TCAScenario.Globals.WRP; } }

		public TimeWarpControl(ModuleTCA tca) : base(tca) {}

		public double WarpToTime = -1;

		protected override void Update()
		{
			if(WarpToTime <= VSL.Physics.UT) return;
			var DewarpTime = WarpToTime-(WRP.DewarpTime+(1+TimeWarp.deltaTime)*(TimeWarp.CurrentRateIndex+1))-VSL.Physics.UT;
			if(TimeWarp.CurrentRateIndex > 0 && DewarpTime < 0)
				TimeWarp.SetRate(TimeWarp.CurrentRateIndex-1, false);
			else if(DewarpTime > TimeWarp.CurrentRate && 
			        TimeWarp.CurrentRateIndex < TimeWarp.fetch.warpRates.Length-1 && 
			        TimeWarp.fetch.warpRates[TimeWarp.CurrentRateIndex+1] <= WRP.MaxWarp &&
			        VSL.Altitude.Absolute > TimeWarp.fetch.GetAltitudeLimit(TimeWarp.CurrentRateIndex+1, VSL.mainBody))
				TimeWarp.SetRate(TimeWarp.CurrentRateIndex+1, false);
		}
	}
}

