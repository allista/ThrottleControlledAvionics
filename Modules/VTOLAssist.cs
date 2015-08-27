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
			[Persistent] public float MinDTWR     = 0.5f;
			[Persistent] public float MinAngularVelocity = 0.001f; //(rad/s)^2 ~= 1.8deg/s
			[Persistent] public float GearOffAngularVelocity = 0.01f; //(rad/s)^2 ~= 1.8deg/s
		}
		static Config TLA { get { return TCAConfiguration.Globals.TLA; } }
		public VTOLAssist(VesselWrapper vsl) { VSL = vsl; }

		bool last_state, landed, tookoff;
		readonly Timer GearTimer   = new Timer();
		readonly Timer LandedTimer = new Timer();

		public override void Init()
		{
			base.Init();
			last_state = VSL.LandedOrSplashed;
			landed = tookoff = false;
			GearTimer.Period = TLA.GearTimer;
			LandedTimer.Period = TLA.LandedTimer;
		}

		public override void UpdateState()
		{ 
			IsActive = VSL.OnPlanet && CFG.VSCIsActive; 
			if(IsActive) return;
			CFG.HF.OffIfOn(HFlight.Level);
			landed = tookoff = false;
		}

		public void Update()
		{
			if(!IsActive) return;
			//update state
			if(last_state && !VSL.LandedOrSplashed) 
			{ tookoff = true; landed = false; GearTimer.Reset(); }
			else if(VSL.LandedOrSplashed && !last_state) { landed = true; tookoff = false; }
			last_state = VSL.LandedOrSplashed;
			//if flying, nothing to do
//			Log("landed {0}, tookoff {1}, state {2}, gear {3}, brake {4}",
//		          landed, tookoff, last_state, VSL.ActionGroups[KSPActionGroup.Gear], VSL.ActionGroups[KSPActionGroup.Brakes]);//debug
			if(!VSL.LandedOrSplashed && !tookoff) return;
			//just landed
			if(landed)
			{
				CFG.HF.OnIfNot(HFlight.Level);
				VSL.ActionGroups.SetGroup(KSPActionGroup.Brakes, true);
				LandedTimer.RunIf(() => landed = false,
				                  () => VSL.HorizontalSpeed < TLA.MinHSpeed);
				SetState(TCAState.VTOLAssist);
				return;
			}
			//just took off
			if(tookoff)
			{
				CFG.HF.OnIfNot(HFlight.Stop);
				if(GearTimer.Check) 
				{ 
					VSL.ActionGroups.SetGroup(KSPActionGroup.Gear, false);
					tookoff = false;
				}
				SetState(TCAState.VTOLAssist);
				return;
			}
			//moving on the ground
			var avSqr = VSL.vessel.angularVelocity.sqrMagnitude;
			if(VSL.HorizontalSpeed < TLA.MaxHSpeed &&
			   avSqr > TLA.MinAngularVelocity)
			{
				CFG.HF.OnIfNot(HFlight.Level);
				if(avSqr > TLA.GearOffAngularVelocity && VSL.DTWR > TLA.MinDTWR)
					VSL.ActionGroups.SetGroup(KSPActionGroup.Gear, false);
				SetState(TCAState.VTOLAssist);
				return;
			}
			//swithch off otherwise
			CFG.HF.OffIfOn(HFlight.Level);
		}
	}
}

