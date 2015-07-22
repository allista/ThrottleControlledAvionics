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
	public enum TCARole { MAIN, MANEUVER, MANUAL, BALANCE }

	public class TCAEngineInfo : PartModule
	{
		public static readonly string[] RoleNames = 
		{
			"TCA: Main Engine",
			"TCA: Maneuver Engine",
			"TCA: Manual Control",
			"TCA: Balanced Thrust",
		};

		public static readonly TCARole[] RolesOrder = { TCARole.MAIN, TCARole.BALANCE, TCARole.MANEUVER, TCARole.MANUAL };

		readonly int num_roles = Enum.GetValues(typeof(TCARole)).Length;
		public TCARole Role = TCARole.MAIN;
		[KSPField(isPersistant = true)] int role;
		int index;

		public override void OnLoad(ConfigNode node)
		{
			base.OnLoad(node);
			Role = (TCARole)role;
			index = Array.FindIndex(RolesOrder, r => r == Role);
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
			Role = RolesOrder[(++index) % num_roles];
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
			Events["SwitchRole"].guiName = role > num_roles ? "TCA: Unknown" : RoleNames[role];
		}
	}
}

