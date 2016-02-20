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
	[CareerPart]
	public class SASBlocker : TCAService
	{
		public SASBlocker(ModuleTCA tca) : base(tca) {}

		protected override void UpdateState()
		{
			if(HasActiveClients)
			{
				if(!CFG.SASIsControlled)
					CFG.SASWasEnabled = VSL.vessel.ActionGroups[KSPActionGroup.SAS]; 
				CFG.SASIsControlled = true;
			}
			else UnblockSAS();
		}

		public override void OnEnable(bool enabled)
		{ if(!enabled) UnblockSAS(); }

		public void UnblockSAS(bool set_flag = true)
		{
			if(CFG.SASIsControlled) 
				VSL.vessel.ActionGroups.SetGroup(KSPActionGroup.SAS, CFG.SASWasEnabled);
			CFG.SASIsControlled &= !set_flag;
		}

		public void EnableSAS()
		{
			VSL.vessel.ActionGroups.SetGroup(KSPActionGroup.SAS, true);
			CFG.SASIsControlled = false;
		}
	}
}

