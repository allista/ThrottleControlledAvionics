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

namespace ThrottleControlledAvionics
{
	public class VTOLAssist : TCAModule
	{
		public class Config : ModuleConfig
		{
			new public const string NODE_NAME = "TLA";

			[Persistent] public float MinHSpeed   = 0.1f;
			[Persistent] public float MaxHSpeed   = 10;
			[Persistent] public float GearTimer   = 1;
			[Persistent] public float LandedTimer = 2;
			//liftoff
			[Persistent] public float MinDTWR     = 0.5f;
			[Persistent] public float MinAngularVelocity = 0.001f; //(rad/s)^2 ~= 1.8deg/s
			[Persistent] public float GearOffAngularVelocity = 0.01f; //(rad/s)^2 ~= 1.8deg/s
			//landing
			[Persistent] public float GearOnMaxHSpeed = 1f;
			[Persistent] public float GearOnAtH       = 5f;
			[Persistent] public float GearOnTime      = 5f;
		}
		static Config TLA { get { return TCAScenario.Globals.TLA; } }
		public VTOLAssist(ModuleTCA tca) { TCA = tca; }

		bool last_state, landed, tookoff;
		readonly Timer GearTimer   = new Timer();
		readonly Timer LandedTimer = new Timer();
		readonly SingleAction StopAction = new SingleAction();

		void working(bool state = true)
		{
			if(state) 
			{
				CFG.Nav.Paused = true;
				SetState(TCAState.VTOLAssist);
			}
			else if(state != Working)
			{
				CFG.HF.OffIfOn(HFlight.Level);
				CFG.Nav.Paused = false;
			}
			Working = state;
		}

		public override void Init()
		{
			base.Init();
			last_state = VSL.LandedOrSplashed;
			landed = tookoff = false;
			GearTimer.Period = TLA.GearTimer;
			LandedTimer.Period = TLA.LandedTimer;
			working(false);
		}

		protected override void UpdateState()
		{ 
			IsActive = VSL.OnPlanet && CFG.VTOLAssistON; 
			if(IsActive) return;
			last_state = VSL.LandedOrSplashed;
			landed = tookoff = false;
			working(false);
		}

		protected override void Update()
		{
			if(!IsActive) return;
			//update state
			if(last_state && !VSL.LandedOrSplashed) 
			{ tookoff = true; landed = false; GearTimer.Reset(); StopAction.Reset(); }
			else if(VSL.LandedOrSplashed && !last_state) 
			{ landed = true; tookoff = false; }
			last_state = VSL.LandedOrSplashed;
			//just landed
			if(landed)
			{
				working();
				CFG.HF.OnIfNot(HFlight.Level);
				VSL.ActionGroups.SetGroup(KSPActionGroup.Brakes, true);
				LandedTimer.RunIf(() => landed = false,
				                  VSL.HorizontalSpeed < TLA.MinHSpeed);
			}
			//just took off
			else if(tookoff)
			{
				working();
				StopAction.Run(() => CFG.HF.OnIfNot(HFlight.Stop));
				GearTimer.RunIf(() =>
				{ 
					VSL.ActionGroups.SetGroup(KSPActionGroup.Brakes, false);
					VSL.ActionGroups.SetGroup(KSPActionGroup.Gear, false);
					tookoff = false;
				}, VSL.RelAltitude > TLA.GearOnAtH+VSL.H);
			}
			//moving on the ground
			else if(VSL.LandedOrSplashed)
			{
				var avSqr = VSL.vessel.angularVelocity.sqrMagnitude;
				if(VSL.HorizontalSpeed < TLA.MaxHSpeed &&
				   avSqr > TLA.MinAngularVelocity)
				{
					working();
					CFG.HF.OnIfNot(HFlight.Level);
					if(avSqr > TLA.GearOffAngularVelocity && VSL.DTWR > TLA.MinDTWR)
						VSL.ActionGroups.SetGroup(KSPActionGroup.Gear, false);
				}
				else working(false);
			}
			//if flying, check if trying to land and deploy the gear
			else 
			{
				working(false);
				//if the gear is on, nothing to do; and autopilot takes precedence
				if(!VSL.ActionGroups[KSPActionGroup.Gear] && !CFG.AP[Autopilot.Land])
				{
					//check boundary conditions
					GearTimer.RunIf(() => 
					{
						VSL.ActionGroups.SetGroup(KSPActionGroup.Gear, true);
						VSL.ActionGroups.SetGroup(KSPActionGroup.Brakes, true);
					},
					                VSL.RelVerticalSpeed < 0 &&
					                VSL.HorizontalSpeed < TLA.GearOnMaxHSpeed &&
					                VSL.RelAltitude+VSL.RelVerticalSpeed*(TLA.GearOnTime+TLA.GearTimer) < TLA.GearOnAtH*VSL.H);
				}
				else GearTimer.RunIf(() => 
				{
					VSL.ActionGroups.SetGroup(KSPActionGroup.Gear, false);
					VSL.ActionGroups.SetGroup(KSPActionGroup.Brakes, false);
				},
				                     VSL.RelVerticalSpeed > 0 ||
				                     VSL.HorizontalSpeed > TLA.GearOnMaxHSpeed);
			}
		}
	}
}

