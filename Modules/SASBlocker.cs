//  Author:
//       Allis Tauri <allista@gmail.com>
//
//  Copyright (c) 2015 Allis Tauri
//
// This work is licensed under the Creative Commons Attribution-ShareAlike 4.0 International License. 
// To view a copy of this license, visit http://creativecommons.org/licenses/by-sa/4.0/ 
// or send a letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.

namespace ThrottleControlledAvionics
{
	[CareerPart(typeof(AttitudeControl))]
	public class SASBlocker : TCAService
	{
		public SASBlocker(ModuleTCA tca) : base(tca) {}

		protected override void UpdateState()
		{
			if(HasActiveClients)
			{
				//save SAS state
				if(!CFG.SASIsControlled)
					CFG.SASWasEnabled = VSL.vessel.ActionGroups[KSPActionGroup.SAS]; 
				// Disable the new SAS so it won't interfere. But enable it while in timewarp for compatibility with PersistentRotation
				if(TimeWarp.WarpMode != TimeWarp.Modes.HIGH || TimeWarp.CurrentRateIndex == 0)
					VSL.vessel.ActionGroups.SetGroup(KSPActionGroup.SAS, false);
				CFG.SASIsControlled = true;
			}
			else RestoreSAS();
		}

		public override void OnEnable(bool enabled) { if(!enabled) RestoreSAS(); }

		public void RestoreSAS()
		{
			if(CFG.SASIsControlled) 
				VSL.vessel.ActionGroups.SetGroup(KSPActionGroup.SAS, CFG.SASWasEnabled);
			CFG.SASIsControlled = false;
		}

		public void EnableSAS()
		{
			VSL.vessel.ActionGroups.SetGroup(KSPActionGroup.SAS, true);
			CFG.SASIsControlled = false;
		}
	}
}

