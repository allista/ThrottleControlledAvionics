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

			[Persistent] public float MaxHSpeed = 10;
			[Persistent] public float GearTimer = 1;
		}
		static Config TLA { get { return TCAConfiguration.Globals.TLA; } }
		public VTOLAssist(VesselWrapper vsl) { VSL = vsl; }

		bool last_state, landed, tookoff;
		readonly Timer GearTimer = new Timer();

		public override void Init()
		{
			base.Init();
			last_state = VSL.LandedOrSplashed;
			landed = tookoff = false;
			GearTimer.Period = TLA.GearTimer;
			if(VSL.LandedOrSplashed) CFG.HF.OnIfNot(HFlight.Level);
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
			if(last_state && !VSL.LandedOrSplashed) tookoff = true;
			else landed |= VSL.LandedOrSplashed && !last_state;
			last_state = VSL.LandedOrSplashed;
			//if flying, nothing to do
//			Log("landed {0}, tookoff {1}, state {2}, gear {3}, brake {4}",
//		          landed, tookoff, last_state, VSL.ActionGroups[KSPActionGroup.Gear], VSL.ActionGroups[KSPActionGroup.Brakes]);//debug
			if(!VSL.LandedOrSplashed && !tookoff) return;
			SetState(TCAState.VTOLAssist);
			if(landed)
			{
				tookoff = false;
				CFG.HF.OnIfNot(HFlight.Level);
				VSL.ActionGroups.SetGroup(KSPActionGroup.Brakes, true);
				landed &= VSL.HorizontalSpeed >= 0.1;
			}
			else if(tookoff)
			{
				landed = false;
				CFG.HF.OnIfNot(HFlight.Stop);
				if(GearTimer.Check) 
				{ 
					VSL.ActionGroups.SetGroup(KSPActionGroup.Gear, false);
					tookoff = false;
				}
			}
			else if(VSL.HorizontalSpeed < TLA.MaxHSpeed)
			{
				CFG.HF.OnIfNot(HFlight.Level);
				GearTimer.Reset();
			}
			else CFG.HF.OffIfOn(HFlight.Level);
		}
	}
}

