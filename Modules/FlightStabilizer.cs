//   TakeoffAssist.cs
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
	public class FlightStabilizer : TCAModule
	{
		public class Config : ModuleConfig
		{
			new public const string NODE_NAME = "STB";

			[Persistent] public float Timer = 2;
			[Persistent] public float MinAngularVelocity = 0.001f; //(rad/s)^2 ~= 1.8deg/s
		}
		static Config STB { get { return TCAScenario.Globals.STB; } }
		public FlightStabilizer(VesselWrapper vsl) { VSL = vsl; }

		readonly Timer OnTimer = new Timer();
		readonly Timer OffTimer = new Timer();

		public override void Init ()
		{
			base.Init();
			OnTimer.Period = STB.Timer;
			OffTimer.Period = STB.Timer;
		}

		protected override void UpdateState()
		{ 
			IsActive = 
				VSL.OnPlanet && 
				CFG.StabilizeFlight && 
				!VSL.LandedOrSplashed && 
				(Working || !CFG.HF && !CFG.AT && !VSL.ActionGroups[KSPActionGroup.SAS] && !VSL.AutopilotDisabled);
			if(IsActive) return;
			if(Working) CFG.HF.OffIfOn(HFlight.Level);
			Working = false;
			OnTimer.Reset();
			OffTimer.Reset();
		}

		protected override void Update()
		{
			if(!IsActive) return;
			if(Working) 
			{
				SetState(TCAState.StabilizeFlight);
				CFG.HF.OnIfNot(HFlight.Level);
			}
			var omega = Vector3.ProjectOnPlane(VSL.vessel.angularVelocity, VSL.Up);
			if(omega.sqrMagnitude > STB.MinAngularVelocity)
			{ 
				OffTimer.Reset();
				Working |= OnTimer.Check;
			}
			else if(omega.sqrMagnitude < STB.MinAngularVelocity/4)
			{
				OnTimer.Reset();
				OffTimer.RunIf(() =>
				{ 
					Working = false;
					CFG.HF.OffIfOn(HFlight.Level);
					CFG.SASIsControlled = false;
					VSL.ActionGroups.SetGroup(KSPActionGroup.SAS, true);
				}, Working);
			}
		}
	}
}