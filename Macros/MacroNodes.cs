//   MacroNodes.cs
//
//  Author:
//       Allis Tauri <allista@gmail.com>
//
//  Copyright (c) 2015 Allis Tauri
//
// This work is licensed under the Creative Commons Attribution 4.0 International License. 
// To view a copy of this license, visit http://creativecommons.org/licenses/by/4.0/ 
// or send a letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.

using UnityEngine;

namespace ThrottleControlledAvionics
{
	public class SetVerticalSpeedMacroNode : SetFloatMacroNode
	{
		public SetVerticalSpeedMacroNode()
		{ Name = "Set vertical speed to:"; Suffix = "m/s"; }
		protected override bool Action(VesselWrapper VSL)
		{ VSL.CFG.VerticalCutoff = Value; return false; }
	}

	public class SetAltitudeMacroNode : SetFloatMacroNode
	{
		public SetAltitudeMacroNode()
		{ Name = "Set altitude to:"; Suffix = "m"; }
		protected override bool Action(VesselWrapper VSL)
		{ VSL.CFG.DesiredAltitude = Value; return false; }
	}

	public class StopMacroNode : MacroNode
	{
		protected override bool Action(VesselWrapper VSL)
		{ VSL.CFG.HF.On(HFlight.Stop); return false; }
	}

	public class LevelMacroNode : MacroNode
	{
		protected override bool Action(VesselWrapper VSL)
		{ VSL.CFG.HF.On(HFlight.Level); return false; }
	}

	public class AnchorMacroNode : MacroNode
	{
		protected override bool Action(VesselWrapper VSL)
		{ VSL.CFG.Nav.On(Navigation.AnchorHere); return false; }
	}

	public class LandMacroNode : MacroNode
	{
		protected override bool Action(VesselWrapper VSL)
		{ VSL.CFG.AP.On(Autopilot.Land); return false; }
	}

	public class GoToMacroNode : MacroNode
	{
		protected override bool Action(VesselWrapper VSL)
		{
			if(!VSL.HasTarget) return true;
			VSL.CFG.Nav.OnIfNot(Navigation.GoToTarget);
			return VSL.CFG.Nav[Navigation.GoToTarget];
		}
	}

	public class FollowPathMacroNode : MacroNode
	{
		protected override bool Action(VesselWrapper VSL)
		{
			if(VSL.CFG.Waypoints.Count == 0) return true;
			VSL.CFG.Nav.OnIfNot(Navigation.FollowPath);
			return VSL.CFG.Nav[Navigation.FollowPath];
		}
	}

	public class FollowTargetMacroNode : MacroNode
	{
		protected override bool Action(VesselWrapper VSL)
		{
			if(!VSL.HasTarget) return true;
			VSL.CFG.Nav.On(Navigation.FollowTarget);
			return false;
		}
	}

	public class FlyForwardMacroNode : SetFloatMacroNode
	{
		public FlyForwardMacroNode() { Name = "Fly forward at:"; Suffix = "m/s"; }
		protected override bool Action(VesselWrapper VSL)
		{
			VSL.CFG.HF.On(HFlight.CruiseControl);
			VSL.NeededHorVelocity = VSL.HFwd*Value;
			VSL.CFG.Starboard = VSL.GetStarboard(VSL.NeededHorVelocity);
			return false;
		}
	}

	public class WaitMacroNode : SetFloatMacroNode
	{
		protected readonly Timer T = new Timer();

		public WaitMacroNode()
		{ Name = "Wait for"; Suffix = "s"; Value = (float)T.Period; }

		public override void Load(ConfigNode node)
		{
			base.Load(node);
			T.Period = Value; T.Reset();
		}

		protected override void OnValueChanged()
		{ T.Period = Value; T.Reset(); }

		protected override bool Action(VesselWrapper VSL)
		{ return !T.Check; }
	}

	public class ActivateProfileMacroNode : MacroNode
	{
		[Persistent] public string Profile = "";
		Vector2 scroll;

		protected override void DrawThis()
		{
			GUILayout.BeginHorizontal();
			if(Edit)
			{ 
				Edit &= !GUILayout.Button(Name, Styles.yellow_button, GUILayout.ExpandWidth(false));
				if(CFG != null)
				{
					scroll = GUILayout.BeginScrollView(scroll, Styles.white, GUILayout.ExpandWidth(true), GUILayout.Height(70));
					GUILayout.BeginVertical();
					for(int i = 0, CFGEnginesProfilesDBCount = CFG.EnginesProfiles.DB.Count; i < CFGEnginesProfilesDBCount; i++)
					{
						var p = CFG.EnginesProfiles.DB[i];
						if(GUILayout.Button(p.Name, p.Name == Profile ? Styles.green_button : Styles.normal_button, GUILayout.ExpandWidth(true)))
							Profile = p.Name;
					}
					GUILayout.EndVertical();
					GUILayout.EndScrollView();
				}
				else Profile = GUILayout.TextField(Profile, GUILayout.ExpandWidth(true));
			}
			else Edit |= GUILayout.Button(Name+": "+Profile, Styles.normal_button);
			GUILayout.EndHorizontal();
		}

		protected override bool Action(VesselWrapper VSL)
		{ VSL.CFG.EnginesProfiles.Activate(Profile); return false; }
	}

	public class HoverMacroNode : OnOffMacroNode
	{
		protected override bool Action(VesselWrapper VSL)
		{ VSL.CFG.VF[VFlight.AltitudeControl] = On; return false; }
	}

	public class FollowTerrainMacroNode : OnOffMacroNode
	{
		protected override bool Action(VesselWrapper VSL)
		{ VSL.CFG.AltitudeAboveTerrain = On; return false; }
	}

	public class LightsMacroNode : OnOffMacroNode
	{
		protected override bool Action(VesselWrapper VSL)
		{ VSL.ActionGroups.SetGroup(KSPActionGroup.Light, On); return false; }
	}

	public class BrakesMacroNode : OnOffMacroNode
	{
		protected override bool Action(VesselWrapper VSL)
		{ VSL.ActionGroups.SetGroup(KSPActionGroup.Brakes, On); return false; }
	}

	public class GearMacroNode : OnOffMacroNode
	{
		protected override bool Action(VesselWrapper VSL)
		{ VSL.ActionGroups.SetGroup(KSPActionGroup.Gear, On); return false; }
	}
}

