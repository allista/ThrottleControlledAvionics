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
	[RequireModules(typeof(HorizontalSpeedControl))]
	public class FlightStabilizer : TCAModule
	{
		public class Config : ModuleConfig
		{
			[Persistent] public float Timer = 2;
			[Persistent] public float MinAngularVelocity = 0.001f; //(rad/s)^2 ~= 1.8deg/s
		}
		static Config STB { get { return Globals.Instance.STB; } }
		public FlightStabilizer(ModuleTCA tca) : base(tca) {}

		readonly Timer OnTimer = new Timer();
		readonly Timer OffTimer = new Timer();

		public override void Init ()
		{
			base.Init();
			OnTimer.Period = STB.Timer;
			OffTimer.Period = STB.Timer;
		}

        public override void Disable() {}

		protected override void UpdateState()
		{ 
			base.UpdateState();
			IsActive &= 
				VSL.OnPlanet && 
				CFG.StabilizeFlight && 
				!VSL.LandedOrSplashed && 
				(Working || !VSL.HasUserInput && !CFG.HF && !CFG.AT && 
				 !CFG.CTRL[ControlMode.VTOL] && !VSL.vessel.ActionGroups[KSPActionGroup.SAS]);
			if(IsActive) return;
			if(Working) CFG.HF.OffIfOn(HFlight.Level);
			Working = false;
			OnTimer.Reset();
			OffTimer.Reset();
		}

		protected override void Update()
		{
			if(Working) 
			{
				SetState(TCAState.StabilizeFlight);
				CFG.HF.OnIfNot(HFlight.Level);
			}
			var omega = Vector3.ProjectOnPlane(VSL.vessel.angularVelocity, VSL.Physics.UpL);
			if(omega.sqrMagnitude > STB.MinAngularVelocity)
			{ 
				OffTimer.Reset();
				Working |= OnTimer.TimePassed;
			}
			else if(omega.sqrMagnitude < STB.MinAngularVelocity/4)
			{
				OnTimer.Reset();
				OffTimer.RunIf(() =>
				{ 
					Working = false;
					CFG.HF.OffIfOn(HFlight.Level);
					CFG.SASIsControlled = false;
					VSL.vessel.ActionGroups.SetGroup(KSPActionGroup.SAS, true);
				}, Working);
			}
		}
	}
}