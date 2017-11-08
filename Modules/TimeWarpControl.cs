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
			[Persistent] public float DewarpTime   = 20f;  //sec
			[Persistent] public float MaxWarp      = 10000f;
            [Persistent] public int   FramesToSkip = 3;
		}
		static Config WRP { get { return Globals.Instance.WRP; } }

		public TimeWarpControl(ModuleTCA tca) : base(tca) {}

		int last_warp_index;
        int frames_to_skip;

		void AbortWarp(bool instant = false)
		{
            VSL.Controls.AbortWarp(instant);
			Reset();
		}

        public override void Disable()
        {
            AbortWarp();
        }

		protected override void Reset()
		{
			base.Reset();
			last_warp_index = TimeWarp.CurrentRateIndex;
            VSL.Controls.NoDewarpOffset = false;
            frames_to_skip = -1;
		}

		public override void Init()
		{
			base.Init();
            frames_to_skip = -1;
			last_warp_index = TimeWarp.CurrentRateIndex;
			GameEvents.onVesselSOIChanged.Add(OnSOIChanged);
		}

		void OnSOIChanged(GameEvents.HostedFromToAction<Vessel, CelestialBody> action)
		{
			if(TCA == null || action.host != VSL.vessel) return;
            if(TimeWarp.CurrentRate > 1)
                Message("Disengaging Time Warp on SOI change...");
			AbortWarp();
		}

		//TimeWarp changes timescale from one rate the next in a second of a real time using Lerp: F+(T-F)*time
		//so the time that will pass from rate F to rate T is: (F-T)/2
		//and from rate F to rate 1: (F-1)/2
		//but due to deltaTime steps it is safer to offset the dewarp time with F+1
		double TimeToDewarp(int rate_index)
		{ 
            var offset = VSL.Controls.NoDewarpOffset? 0 : WRP.DewarpTime/(VSL.LandedOrSplashed? 2 : 1);
			return VSL.Controls.WarpToTime-(offset+TimeWarp.fetch.warpRates[rate_index]-1)-VSL.Physics.UT;
		}

		public override void ProcessKeys()
		{
			if(GameSettings.TIME_WARP_STOP.GetKey())
			{
				if(CFG.WarpToNode && VSL.Controls.WarpToTime > 0)
					AbortWarp();
			}
		}

        bool can_increase_rate
        { 
            get 
            { 
                return TimeWarp.CurrentRateIndex < 
                    TimeWarp.fetch.GetMaxRateForAltitude(VSL.orbit.radius-VSL.Body.Radius, VSL.Body); 
            } 
        }

		protected override void Update()
		{
			if(VSL.Controls.WarpToTime < 0) goto end;
			//try to catch the moment KSP or some other mod sets warp besides us
            if(TimeWarp.CurrentRateIndex < last_warp_index && can_increase_rate)
			{ 
                if(frames_to_skip < 0)
                    frames_to_skip = TimeWarp.CurrentRateIndex * WRP.FramesToSkip;
//                Log("current index {}, max index at alt {}, frames_to_skip {}",
//                    TimeWarp.CurrentRateIndex, 
//                    TimeWarp.fetch.GetMaxRateForAltitude(VSL.vessel.altitude, VSL.Body),
//                    frames_to_skip);
                if(frames_to_skip-- > 0) return;
				Message("TCA Time Warp was overridden.");
				VSL.Controls.WarpToTime = -1;
				CFG.WarpToNode = false; 
				goto end; 
			}
			//dewarp if the warp was disabled, or LOW mode
			if(VSL.Controls.WarpToTime > 0 && 
			   (!CFG.WarpToNode || 
			    TimeWarp.WarpMode == TimeWarp.Modes.LOW)) 
				VSL.Controls.WarpToTime = 0;
			if(VSL.Controls.WarpToTime <= VSL.Physics.UT && TimeWarp.CurrentRate.Equals(1))
			{ 
				VSL.Controls.WarpToTime = -1;
				goto end;
			}
			if(TimeToDewarp(TimeWarp.CurrentRateIndex) < 0)
			{
				if(TimeWarp.CurrentRateIndex > 0)
					TimeWarp.SetRate(TimeWarp.CurrentRateIndex-1, false);
				else if(TimeWarp.CurrentRate.Equals(1))
					VSL.Controls.WarpToTime = 0;
			}
			else if(TimeWarp.CurrentRateIndex < TimeWarp.fetch.warpRates.Length-1 && 
			        TimeWarp.fetch.warpRates[TimeWarp.CurrentRateIndex+1] <= WRP.MaxWarp &&
                    (VSL.LandedOrSplashed || can_increase_rate) &&
			        TimeToDewarp(TimeWarp.CurrentRateIndex+1) > 0)
                TimeWarp.SetRate(TimeWarp.CurrentRateIndex+1, false, false);
			end: Reset();
		}

		public override void Draw()
		{
			if(Utils.ButtonSwitch("Warp", CFG.WarpToNode, "Warp to the burn", GUILayout.ExpandWidth(false)))
			{
				CFG.WarpToNode = !CFG.WarpToNode;
                if(!CFG.WarpToNode) AbortWarp();
			}
		}
	}
}

