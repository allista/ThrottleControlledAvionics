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
	[CareerPart(typeof(ManeuverAutopilot))]
	public class TimeWarpControl : TCAModule
	{
		public class Config : ModuleConfig
		{
			[Persistent] public float DewarpTime = 20f;  //sec
			[Persistent] public float MaxWarp    = 10000f;
		}
		static Config WRP { get { return Globals.Instance.WRP; } }

		public TimeWarpControl(ModuleTCA tca) : base(tca) {}

		public override void Init()
		{
			base.Init();
			GameEvents.onVesselSOIChanged.Add(OnSOIChanged);
		}

		void OnSOIChanged(GameEvents.HostedFromToAction<Vessel, CelestialBody> action)
		{
			if(TCA == null || action.host != VSL.vessel) return;
			CFG.WarpToNode = false;
			VSL.Controls.StopWarp();
		}

		//TimeWarp changes timescale from one rate the next in a second of a real time using Lerp: F+(T-F)*time
		//so the time that will pass from rate F to rate T is: (F-T)/2
		//and from rate F to rate 1: (F-1)/2
		//but due to deltaTime steps it is safer to offset the dewarp time with just F+1
		double TimeToDewarp(int rate_index)
		{ return VSL.Controls.WarpToTime-(WRP.DewarpTime/(VSL.LandedOrSplashed? 2 : 1)+TimeWarp.fetch.warpRates[rate_index]-1)-VSL.Physics.UT; }

		protected override void Update()
		{
			if(VSL.Controls.WarpToTime < 0) return;
			if(VSL.Controls.WarpToTime > 0 && 
			   (!CFG.WarpToNode || TimeWarp.WarpMode == TimeWarp.Modes.LOW)) 
				VSL.Controls.WarpToTime = 0;
			if(VSL.Controls.WarpToTime <= VSL.Physics.UT && TimeWarp.CurrentRate.Equals(1))
			{ VSL.Controls.WarpToTime = -1; return; }
			if(TimeWarp.CurrentRateIndex > 0 && TimeToDewarp(TimeWarp.CurrentRateIndex) < 0)
				TimeWarp.SetRate(TimeWarp.CurrentRateIndex-1, false);
			else if(TimeWarp.CurrentRateIndex < TimeWarp.fetch.warpRates.Length-1 && 
			        TimeWarp.fetch.warpRates[TimeWarp.CurrentRateIndex+1] <= WRP.MaxWarp &&
			        (VSL.LandedOrSplashed || VSL.Altitude.Absolute > TimeWarp.fetch.GetAltitudeLimit(TimeWarp.CurrentRateIndex+1, VSL.Body)) &&
			        TimeToDewarp(TimeWarp.CurrentRateIndex+1) > 0)
				TimeWarp.SetRate(TimeWarp.CurrentRateIndex+1, false);
		}

		public override void Draw()
		{
			if(Utils.ButtonSwitch("Warp", CFG.WarpToNode, "Warp to the burn", GUILayout.ExpandWidth(false)))
			{
				CFG.WarpToNode = !CFG.WarpToNode;
				if(!CFG.WarpToNode) TimeWarp.SetRate(0, false);
			}
		}
	}
}

