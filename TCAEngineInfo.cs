//  Author:
//       Allis Tauri <allista@gmail.com>
//
//  Copyright (c) 2015 Allis Tauri
//
// This work is licensed under the Creative Commons Attribution-ShareAlike 4.0 International License. 
// To view a copy of this license, visit http://creativecommons.org/licenses/by-sa/4.0/ 
// or send a letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.

using System;
using UnityEngine;

namespace ThrottleControlledAvionics
{
	public enum TCARole { MAIN, MANEUVER, MANUAL, BALANCE }

	public class TCAEngineInfo : PartModule
	{
		public static readonly string[] RoleNames = 
		{
			"Main Engine",
			"Maneuver Engine",
			"Manual Control",
			"Balanced Thrust",
		};

		public static readonly TCARole[] RolesOrder = { TCARole.MAIN, TCARole.BALANCE, TCARole.MANEUVER, TCARole.MANUAL };
		public static readonly int NumRoles = RolesOrder.Length;

		public static TCARole NextRole(TCARole cur)
		{ return RolesOrder[(Array.FindIndex(RolesOrder, r => r == cur)+1) % NumRoles]; }

		public static TCARole PrevRole(TCARole cur)
		{ 
			var i = Array.FindIndex(RolesOrder, r => r == cur)-1;
			if(i < 0) i = NumRoles-1;
			return RolesOrder[i]; 
		}

		[KSPField(isPersistant = true)]
		int role;
		int role_index;

		[UI_ChooseOption]
		[KSPField(isPersistant = true, guiActive = true, guiActiveEditor = true, guiName = "Engine Group:")]
		public int group;

		public TCARole Role = TCARole.MAIN;
		public int Group { get { return group; } }

		public override void OnLoad(ConfigNode node)
		{
			base.OnLoad(node);
			Role = (TCARole)role;
			role_index = Array.FindIndex(RolesOrder, r => r == Role);
			update_status();
		}

		static void setup_groups(UI_ChooseOption chooser)
		{
			if(chooser == null) return;
			chooser.options = new string[TCAScenario.Globals.MaxManualGroups];
			chooser.options[0] = "OFF";
			for(int i = 1; i<TCAScenario.Globals.MaxManualGroups; i++)
				chooser.options[i] = string.Format("G{0:D}", i);
		}

		static void setup_roles(UI_ChooseOption chooser)
		{
			if(chooser == null) return;
			chooser.options = new string[NumRoles];
			for(int i = 0; i<NumRoles; i++) chooser.options[i] = RoleNames[i];
		}

		public override void OnStart(StartState state) 
		{ 
			setup_groups(Fields["group"].uiControlEditor as UI_ChooseOption);
			setup_groups(Fields["group"].uiControlFlight as UI_ChooseOption);
			update_status();
		}

		public void SetRole(TCARole R)
		{
			Role = R;
			update_status();
		}

		[KSPEvent(guiActive = true, guiActiveEditor = true, guiName = "TCA Role", active = true)]
		public void SwitchRole() 
		{ 
			if(!HighLogic.LoadedSceneIsEditor && group > 0) 
			{
				ScreenMessages.PostScreenMessage(
					"Cannot change the role of an engine belonging to a group.\n" +
					"Use in-flight group controls  instead.", 
					5, ScreenMessageStyle.UPPER_CENTER);
				return;
			}
			Role = RolesOrder[(++role_index) % NumRoles];
			update_status();
			//set the role of symmetry counterparts, if needed
			if(!TCAScenario.Globals.RoleSymmetryInFlight 
			   && HighLogic.LoadedSceneIsFlight) return;
			foreach(var cp in part.symmetryCounterparts)
			{
				var einfo = cp.GetModule<TCAEngineInfo>();
				if(einfo != null && 
				   (HighLogic.LoadedSceneIsEditor || einfo.Group == 0)) 
					einfo.SetRole(Role);
			}
		}

		void update_status()
		{
			role = (int)Role;
			Events["SwitchRole"].guiName = role > NumRoles ? "TCA: Unknown" : ("TCA: "+RoleNames[role]);
		}
	}
}

