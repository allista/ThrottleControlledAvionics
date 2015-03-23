//   TCAEngineInfo.cs
//
//  Author:
//       Allis Tauri <allista@gmail.com>
//
//  Copyright (c) 2015 Allis Tauri
//
// This work is licensed under the Creative Commons Attribution-ShareAlike 3.0 International License. 
// To view a copy of this license, visit http://creativecommons.org/licenses/by-sa/3.0/ 
// or send a letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.

using System;

namespace ThrottleControlledAvionics
{
	public enum TCARole { MAIN, MANEUVER, MANUAL }

	public class TCAEngineInfo : PartModule
	{
		readonly int num_roles = Enum.GetValues(typeof(TCARole)).Length;
		public TCARole Role = TCARole.MAIN;
		[KSPField(isPersistant = true)] int role;

		public override void OnLoad(ConfigNode node)
		{
			base.OnLoad(node);
			Role = (TCARole)role;
			update_status();
		}

		public override void OnStart(StartState state) { update_status(); }

		public void SetRole(TCARole R)
		{
			Role = R;
			update_status();
		}

		[KSPEvent(guiActive = true, guiActiveEditor = true, guiName = "TCA Role", active = true)]
		public void SwitchRole() 
		{ 
			Role = (TCARole)(((int)Role+1) % num_roles);
			update_status();
			//set the role of symmetry counterparts, if needed
			if(!TCAConfiguration.Globals.RoleSymmetryInFlight 
			   && HighLogic.LoadedSceneIsFlight) return;
			foreach(var cp in part.symmetryCounterparts)
			{
				var einfo = cp.GetModule<TCAEngineInfo>();
				if(einfo != null) einfo.SetRole(Role);
			}
		}

		void update_status()
		{
			role = (int)Role;
			switch(Role)
			{
			case TCARole.MAIN:
				Events["SwitchRole"].guiName = "TCA: Main Engine";
				break;
			case TCARole.MANEUVER:
				Events["SwitchRole"].guiName = "TCA: Maneuver Engine";
				break;
			default:
				Events["SwitchRole"].guiName = "TCA: Manual Control";
				break;
			}
		}
	}
}

