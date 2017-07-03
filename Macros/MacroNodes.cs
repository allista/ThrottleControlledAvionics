//  Author:
//       Allis Tauri <allista@gmail.com>
//
//  Copyright (c) 2015 Allis Tauri
//
// This work is licensed under the Creative Commons Attribution-ShareAlike 4.0 International License. 
// To view a copy of this license, visit http://creativecommons.org/licenses/by-sa/4.0/ 
// or send a letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.

using System;
using System.Linq;
using UnityEngine;
using AT_Utils;

namespace ThrottleControlledAvionics
{
	[RequireModules(typeof(VerticalSpeedControl))]
    [ComponentInfo(Description = "Set desired vertical speed. If Follow Terrain mode is active, the speed relative to the ground is used.")]
	public class SetVerticalSpeedMacroNode : SetFloatMacroNode
	{
		public SetVerticalSpeedMacroNode()
		{ Name = "Set Vertical Speed to"; Suffix = "m/s"; }

		protected override void OnValueChanged ()
		{ Value.Value = Utils.Clamp(Value, -GLB.VSC.MaxSpeed, GLB.VSC.MaxSpeed); }

		protected override bool Action(VesselWrapper VSL)
		{ 
			VSL.CFG.BlockThrottle = true;
			VSL.CFG.VF.XOffIfOn(VFlight.AltitudeControl);
			VSL.CFG.VerticalCutoff = Value; 
			return false;
		}
	}

	[RequireModules(typeof(AltitudeControl))]
    [ComponentInfo(Description = "Set desired altitude to specified value. If Follow Terrain mode is active, height above the ground is used.")]
	public class SetAltitudeMacroNode : SetFloatMacroNode
	{
		public SetAltitudeMacroNode()
		{ Name = "Set Altitude to"; Suffix = "m"; }
		protected override bool Action(VesselWrapper VSL)
		{ 
			VSL.CFG.BlockThrottle = true;
			VSL.CFG.VF.XOnIfNot(VFlight.AltitudeControl);
			VSL.CFG.DesiredAltitude = Value;
			return false; 
		}
	}

	[RequireModules(typeof(ThrottleControl))]
    [ComponentInfo(Description = "Set main throttle to specified value")]
	public class SetThrottleMacroNode : SetFloatMacroNode
	{
		public SetThrottleMacroNode() 
		{ 
			Name += ":"; Suffix = "%";
			Value.Min = 0; Value.Max = 100;
		}

		protected override bool Action(VesselWrapper VSL)
		{ 
			var THR = VSL.TCA.GetModule<ThrottleControl>();
			if(THR != null) THR.Throttle = Value/100;
			return false; 
		}
	}

	[RequireModules(typeof(HorizontalSpeedControl))]
    [ComponentInfo(Description = "Kill horizontal speed")]
	public class StopMacroNode : MacroNode
	{
		protected override bool Action(VesselWrapper VSL)
		{ VSL.CFG.HF.XOn(HFlight.Stop); return false; }
	}

	[RequireModules(typeof(HorizontalSpeedControl))]
    [ComponentInfo(Description = "Point total thrust directly downward")]
	public class LevelMacroNode : MacroNode
	{
		protected override bool Action(VesselWrapper VSL)
		{ VSL.CFG.HF.XOn(HFlight.Level); return false; }
	}

	[RequireModules(typeof(Anchor))]
    [ComponentInfo(Description = "Stop and maintain current position above the ground, correcting lateral drift")]
	public class AnchorMacroNode : MacroNode
	{
		protected override bool Action(VesselWrapper VSL)
		{ VSL.CFG.Nav.XOn(Navigation.AnchorHere); return false; }
	}

    [RequireModules(typeof(ManeuverAutopilot))]
    [ComponentInfo(Description = "Execute maneuver node")]
    public class ExecuteManeuverMacroNode : MacroNode
    {
        protected override bool Action(VesselWrapper VSL)
        {
            if(!VSL.HasManeuverNode) { Message("No Maneuver Node"); return false; }
            VSL.CFG.AP1.XOnIfNot(Autopilot1.Maneuver);
            return VSL.CFG.AP1[Autopilot1.Maneuver];
        }
    }

	[RequireModules(typeof(MatchVelocityAutopilot))]
    [ComponentInfo(Description = "Continiously match orbital velocity with the target")]
	public class MatchVelocityMacroNode : MacroNode
	{
		protected override bool Action(VesselWrapper VSL)
		{
			VSL.CFG.AP1.XOn(Autopilot1.MatchVel);
			return false;
		}
	}

	[RequireModules(typeof(MatchVelocityAutopilot))]
    [ComponentInfo(Description = "Match orbital speed with the target on next approach")]
	public class BrakeNearTargetMacroNode : MacroNode
	{
		protected override bool Action(VesselWrapper VSL)
		{
			if(!VSL.HasTarget) { Message("No Target"); return false; }
			VSL.CFG.AP1.XOnIfNot(Autopilot1.MatchVelNear);
			return VSL.CFG.AP1[Autopilot1.MatchVelNear];
		}
	}

	[RequireModules(typeof(AutoLander))]
    [ComponentInfo(Description = "Search for the nearest flat surface and land on it VTOL-style")]
	public class LandMacroNode : MacroNode
	{
		protected override bool Action(VesselWrapper VSL)
		{ 
			VSL.CFG.AP1.XOnIfNot(Autopilot1.Land); 
			return VSL.CFG.AP1[Autopilot1.Land];
		}
	}

	[RequireModules(typeof(BallisticJump))]
    [ComponentInfo(Description = "Get to the target using minimum-energy suborbital trajectory and precise landing")]
	public class JumpToTargetMacroNode : MacroNode
	{
		protected override bool Action(VesselWrapper VSL)
		{
			if(!VSL.HasTarget) { Message("No Target"); return false; }
			VSL.CFG.AP2.XOnIfNot(Autopilot2.BallisticJump);
			return VSL.CFG.AP2[Autopilot2.BallisticJump];
		}
	}

	[RequireModules(typeof(ToOrbitAutopilot))]
    [ComponentInfo(Description = "Take off and achive circular orbit of specified radius, direction and inclination")]
	public class ToOrbitNode : MacroNode
	{
		[Persistent] public TargetOrbitInfo OrbitInfo = new TargetOrbitInfo();

		protected override void DrawThis()
		{
			GUILayout.BeginHorizontal();
			if(Edit)
			{ 
				GUILayout.Label(Name, Styles.label, GUILayout.ExpandWidth(false));
				OrbitInfo.Draw();
				if(GUILayout.Button("Done", Styles.confirm_button, GUILayout.ExpandWidth(false)))
				{ 
					OrbitInfo.UpdateValues();
					Edit = false; 
				}
			}
			else Edit |= GUILayout.Button(Name, Styles.normal_button);
			GUILayout.EndHorizontal();
		}

		protected override bool Action(VesselWrapper VSL)
		{
			var ToOrbit = VSL.TCA.GetModule<ToOrbitAutopilot>();
			if(ToOrbit == null) return false;
			if(!VSL.CFG.AP2[Autopilot2.ToOrbit])
			{
				ToOrbit.TargetOrbit.Copy(OrbitInfo);
				VSL.CFG.AP2.XOnIfNot(Autopilot2.ToOrbit);
			}
			return VSL.CFG.AP2[Autopilot2.ToOrbit];
		}
	}

	[RequireModules(typeof(DeorbitAutopilot))]
    [ComponentInfo(Description = "Deorbit and land at specified target location")]
	public class DeorbitMacroNode : MacroNode
	{
		protected override bool Action(VesselWrapper VSL)
		{
			if(!VSL.HasTarget) { Message("No Target"); return false; }
			VSL.CFG.AP2.XOnIfNot(Autopilot2.Deorbit);
			return VSL.CFG.AP2[Autopilot2.Deorbit];
		}
	}

	[RequireModules(typeof(RendezvousAutopilot))]
    [ComponentInfo(Description = "Rendezvous with current target. Works both for orbit-to-orbit and for planet-to-orbit rendezvous.")]
	public class RendezvousMacroNode : MacroNode
	{
		protected override bool Action(VesselWrapper VSL)
		{
			if(!VSL.HasTarget) { Message("No Target"); return false; }
			VSL.CFG.AP2.XOnIfNot(Autopilot2.Rendezvous);
			return VSL.CFG.AP2[Autopilot2.Rendezvous];
		}
	}

	[RequireModules(typeof(PointNavigator))]
    [ComponentInfo(Description = "Fly to current target and Stop")]
	public class GoToTargetMacroNode : MacroNode
	{
		protected override bool Action(VesselWrapper VSL)
		{
			if(!VSL.HasTarget) { Message("No Target"); return false; }
			VSL.CFG.Nav.XOnIfNot(Navigation.GoToTarget);
			return VSL.CFG.Nav[Navigation.GoToTarget];
		}
	}

	[RequireModules(typeof(PointNavigator))]
    [ComponentInfo(Description = "Follow current target")]
	public class FollowTargetMacroNode : MacroNode
	{
		protected override bool Action(VesselWrapper VSL)
		{
			if(!VSL.HasTarget) { Message("No Target"); return false; }
			VSL.CFG.Nav.XOn(Navigation.FollowTarget);
			return false;
		}
	}

	[RequireModules(typeof(PointNavigator))]
    [ComponentInfo(Description = "Activate waypoint navigation")]
	public class FollowPathMacroNode : MacroNode
	{
		protected NavPath Path = new NavPath();
		protected bool path_loaded;

		public override void Rewind()
		{ base.Rewind(); path_loaded = false; }

		public override void Save(ConfigNode node)
		{
			base.Save(node);
			Path.Save(node.AddNode(NavPath.NODE_NAME));
		}

		public override void Load(ConfigNode node)
		{
			base.Load(node);
			var path = node.GetNode(NavPath.NODE_NAME) ?? node.GetNode("Waypoints"); //deprecated: old config conversion
			if(path != null) Path.Load(path);
		}

		protected override void DrawThis ()
		{
			var title = Name;
			if(Path.Count > 0) title += " (waypoints stored)";
			GUILayout.BeginHorizontal();
			if(Edit)
			{ 
				Edit &= !GUILayout.Button(title, Styles.active_button, GUILayout.ExpandWidth(false));
				if(EditedCFG != null && GUILayout.Button("Copy waypoints from Vessel", 
				                                         Styles.active_button, GUILayout.ExpandWidth(false)))
					Path = EditedCFG.Path.Copy();
			}
			else Edit |= GUILayout.Button(title, Styles.normal_button) && EditedCFG != null;
			GUILayout.EndHorizontal();
		}

		protected override bool Action(VesselWrapper VSL)
		{
			if(!path_loaded)
			{
				if(Path.Count > 0)
					VSL.CFG.Path = Path.Copy();
				path_loaded = true;
			}
			if(VSL.CFG.Path.Count == 0) { Message("No Waypoints"); return false; }
			VSL.CFG.Nav.XOnIfNot(Navigation.FollowPath);
			return VSL.CFG.Nav[Navigation.FollowPath];
		}
	}

	[RequireModules(typeof(HorizontalSpeedControl))]
    [ComponentInfo(Description = "Activate Cruis Control, set desired bearing and speed")]
	public class FlyMacroNode : SetFloatMacroNode
	{
		public enum Mode { Forward, Backward, Right, Left, Bearing, Off }

		[Persistent] public Mode mode;
		[Persistent] public FloatField Bearing = new FloatField("F1", 0, 360);

		public FlyMacroNode() { Name += ":"; Suffix = "m/s"; }

		protected override void DrawThis()
		{
			GUILayout.BeginHorizontal();
			if(Edit)
			{ 
				GUILayout.Label(Name, Styles.label, GUILayout.ExpandWidth(false));
				if(GUILayout.Button(mode.ToString(), Styles.normal_button, GUILayout.ExpandWidth(false)))
					mode = (Mode)(((int)mode+1)%6);
				if(mode == Mode.Bearing) Bearing.Draw("°", 10);
				if(mode != Mode.Off) Value.Draw(Suffix);
				if(GUILayout.Button("Done", Styles.confirm_button, GUILayout.ExpandWidth(false)))
				{ 
					Bearing.UpdateValue();
					Value.UpdateValue();
					OnValueChanged();
					Edit = false; 
				}
			}
			else 
			{
				var title = Name+" "+mode+" ";
				if(mode == Mode.Bearing) title += Bearing+"°, ";
				if(mode != Mode.Off) title += Value+Suffix;
				Edit |= GUILayout.Button(title, Styles.normal_button);
			}
			GUILayout.EndHorizontal();
		}

		protected override bool Action(VesselWrapper VSL)
		{
			var nv = Vector3d.zero;
			switch(mode)
			{
			case Mode.Forward:
				nv = VSL.OnPlanetParams.Heading;
				break;
			case Mode.Backward:
				nv = -VSL.OnPlanetParams.Heading;
				break;
			case Mode.Right:
				nv = Vector3.ProjectOnPlane(VSL.refT.right, VSL.Physics.Up).normalized;
				break;
			case Mode.Left:
				nv = -Vector3.ProjectOnPlane(VSL.refT.right, VSL.Physics.Up).normalized;
				break;
			case Mode.Bearing:
				nv = VSL.Physics.Direction(Bearing);
				break;
			case Mode.Off:
				VSL.CFG.HF.XOff();
				return false;
			}
			VSL.CFG.HF.XOn(HFlight.CruiseControl);
			VSL.HorizontalSpeed.SetNeeded(nv*Value);
			return false;
		}
	}

    [ComponentInfo(Description = "Wait specified number of seconds")]
	public class WaitMacroNode : SetFloatMacroNode
	{
		protected readonly Timer T = new Timer();

		public WaitMacroNode()
		{ Name = "Wait for"; Suffix = "s"; Value.Value = (float)T.Period; }

		public override void Load(ConfigNode node)
		{ base.Load(node); T.Period = Value; T.Reset(); }

		protected override void OnValueChanged()
		{ T.Period = Value; T.Reset(); }

		public override void Rewind()
		{ base.Rewind(); T.Reset(); }

		protected override bool Action(VesselWrapper VSL)
		{ 
			VSL.Info.Countdown = T.Remaining;
			return !T.TimePassed; 
		}
	}

	[RequireModules(typeof(TimeWarpControl))]
    [ComponentInfo(Description = "Toggle the Follow Terrain mode")]
	public class TimeWarpMacroNode : SetFloatMacroNode
	{
		[Persistent] public double StopUT = -1;

		public TimeWarpMacroNode()
		{ Name = "Warp for"; Suffix = "s"; }

		public override void Rewind()
		{ base.Rewind(); StopUT = -1; }

		protected override bool Action(VesselWrapper VSL)
		{
			if(StopUT < 0) StopUT = VSL.Physics.UT+Value;
			else if(VSL.Physics.UT >= StopUT) return false;
			VSL.Controls.WarpToTime = StopUT;
			VSL.Info.Countdown = VSL.Physics.UT-StopUT;
			return true;
		}
	}

    [ComponentInfo(Description = "Activate selected engine profile")]
	public class ActivateProfileMacroNode : MacroNode
	{
		[Persistent] public string Profile = "";
		Vector2 scroll;

		protected override void DrawThis()
		{
			GUILayout.BeginHorizontal();
			if(Edit)
			{ 
				Edit &= !GUILayout.Button(Name, Styles.active_button, GUILayout.ExpandWidth(false));
				if(EditedCFG != null)
				{
					scroll = GUILayout.BeginScrollView(scroll, Styles.white, GUILayout.ExpandWidth(true), GUILayout.Height(70));
					GUILayout.BeginVertical();
					for(int i = 0, CFGEnginesProfilesDBCount = EditedCFG.EnginesProfiles.DB.Count; i < CFGEnginesProfilesDBCount; i++)
					{
						var p = EditedCFG.EnginesProfiles.DB[i];
						if(GUILayout.Button(p.Name, p.Name == Profile ? Styles.enabled_button : Styles.normal_button, GUILayout.ExpandWidth(true)))
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

    [ComponentInfo(Description = "Apply custom engine settings to the currently active profile")]
	public class SetEngineParamsMacroNode : MacroNode
	{
		[Persistent] public EngineConfig Config = new EngineConfig();
		[Persistent] public int Group;
		Vector2 scroll;
		bool show_single;

		protected override void DrawThis()
		{
			GUILayout.BeginHorizontal();
			if(Edit)
			{ 
				Edit &= !GUILayout.Button(Name, Styles.active_button, GUILayout.ExpandWidth(false));
				Group = Utils.IntSelector(Group, 0, tooltip: "Group ID");
				if(EditedCFG != null && EditedCFG.ActiveProfile != null && EditedCFG.ActiveProfile.Single.Count > 0)
				{
					GUILayout.BeginVertical();
					if(GUILayout.Button("Show Single Engines", Styles.normal_button, GUILayout.ExpandWidth(true)))
						show_single = !show_single;
					if(show_single)
					{
						scroll = GUILayout.BeginScrollView(scroll, Styles.white, GUILayout.ExpandWidth(true), GUILayout.Height(70));
						GUILayout.BeginVertical();
						foreach(var k in EditedCFG.ActiveProfile.Single.DB.Keys)
						{
							var p = EditedCFG.ActiveProfile.Single[k];
							if(GUILayout.Button(p.Name, Styles.normal_button, GUILayout.ExpandWidth(true)))
								Config.Name = p.Name;
						}
						GUILayout.EndVertical();
						GUILayout.EndScrollView();
					}
					GUILayout.EndVertical();
				}
				Config.Draw();
			}
			else Edit |= GUILayout.Button(Name+": "+Config.Name, Styles.normal_button);
			GUILayout.EndHorizontal();
		}

		protected override bool Action(VesselWrapper VSL)
		{ 
			if(VSL.CFG.ActiveProfile == null) return false;
			if(Group > 0 && Group < VSL.CFG.ActiveProfile.Groups.Count)
			{
				VSL.CFG.ActiveProfile.Groups[Group].Update(Config);
				VSL.CFG.ActiveProfile.Changed = true;
			}
			else
			{
				var c = VSL.CFG.ActiveProfile.Single.DB.FirstOrDefault(it => it.Value.Name == Config.Name).Value;
				if(c != null) 
				{
					c.Update(Config);
					VSL.CFG.ActiveProfile.Changed = true;
				}
			}
			return false; 
		}
	}

	[RequireModules(typeof(AltitudeControl))]
    [ComponentInfo(Description = "Toggle the Follow Terrain mode")]
	public class FollowTerrainMacroNode : OnOffMacroNode
	{
		protected override bool Action(VesselWrapper VSL)
		{ VSL.CFG.AltitudeAboveTerrain = On; return false; }
	}

    [ComponentInfo(Description = "Toggle lights")]
	public class LightsMacroNode : OnOffMacroNode
	{
		protected override bool Action(VesselWrapper VSL)
		{ VSL.vessel.ActionGroups.SetGroup(KSPActionGroup.Light, On); return false; }
	}

    [ComponentInfo(Description = "Toggle brakes")]
	public class BrakesMacroNode : OnOffMacroNode
	{
		protected override bool Action(VesselWrapper VSL)
		{ VSL.vessel.ActionGroups.SetGroup(KSPActionGroup.Brakes, On); return false; }
	}

    [ComponentInfo(Description = "Toggle landing gear")]
	public class GearMacroNode : OnOffMacroNode
	{
		protected override bool Action(VesselWrapper VSL)
		{ VSL.vessel.ActionGroups.SetGroup(KSPActionGroup.Gear, On); return false; }
	}

    [ComponentInfo(Description = "Toggle RCS")]
	public class RCSMacroNode : OnOffMacroNode
	{
		protected override bool Action(VesselWrapper VSL)
		{ VSL.vessel.ActionGroups.SetGroup(KSPActionGroup.RCS, On); return false; }
	}

	[RequireModules(typeof(VerticalSpeedControl), 
	                typeof(ThrottleControl))]
    [ComponentInfo(Description = "Enable automatic throttle control")]
	public class AutoThrottleMacroNode : OnOffMacroNode
	{
		protected override bool Action(VesselWrapper VSL)
		{ VSL.CFG.BlockThrottle = On; return false; }
	}

	[RequireModules(typeof(AttitudeControl))]
    [ComponentInfo(Description = "Activate specified mode of T-SAS")]
	public class TSASMacroNode : MacroNode
	{
		[Persistent] public Attitude attitude;

		protected override void DrawThis()
		{
			GUILayout.BeginHorizontal();
			if(Edit)
			{ 
				Edit &= !GUILayout.Button(Name, Styles.active_button, GUILayout.ExpandWidth(false));
				if(GUILayout.Button(attitude.ToString(), Styles.normal_button, GUILayout.ExpandWidth(false)))
					attitude = (Attitude)(((int)attitude+1)%10);
			}
			else Edit |= GUILayout.Button(Name+": "+attitude, Styles.normal_button);
			GUILayout.EndHorizontal();
		}

		protected override bool Action(VesselWrapper VSL)
		{ 
			if(attitude.Equals(Attitude.None)) VSL.CFG.AT.XOff();
			else VSL.CFG.AT.XOnIfNot(attitude);
			return false; 
		}
	}

	[RequireModules(typeof(VerticalSpeedControl))]
    [ComponentInfo(Description = "Disable both Altitude Control and Vertical Speed Control")]
	public class DisableVerticalControlMacroNode : MacroNode
	{
		protected override bool Action(VesselWrapper VSL)
		{ VSL.CFG.DisableVSC(); return false; }
	}

    [ComponentInfo(Description = "Activate SAS. Deactivate T-SAS and all depending autopilots.")]
	public class StockSASMacroNode : OnOffMacroNode
	{
		protected override bool Action(VesselWrapper VSL)
		{ 
			if(On)
			{
				VSL.CFG.HF.XOff();
				VSL.CFG.AT.XOff();
				var SAS = VSL.TCA.GetModule<SASBlocker>();
				if(SAS != null) SAS.EnableSAS();
			}
			else VSL.vessel.ActionGroups.SetGroup(KSPActionGroup.SAS, false);
			return false; 
		}
	}

    [ComponentInfo(Description = "Activate next stage")]
	public class StageMacroNode : MacroNode
	{
		protected override bool Action(VesselWrapper VSL)
		{ VSL.ActivateNextStage(); return false; }
	}

    [ComponentInfo(Description = "Toggle selected action groups, do not change the reset")]
	public class ToggleActionGroupsMacroNode : MacroNode
	{
		[Persistent] public KSPActionGroup Group = KSPActionGroup.None;
		Vector2 scroll;

//		protected static string group_name(KSPActionGroup g)
//		{ return Enum.GetName(typeof(KSPActionGroup), g); }

		protected bool group_is_set(KSPActionGroup g)
		{ return (g & Group) == g; }

		protected override void DrawThis()
		{
			GUILayout.BeginHorizontal();
			if(Edit)
			{ 
                Edit &= !GUILayout.Button(Label, Styles.active_button, GUILayout.ExpandWidth(false));
				var new_group = KSPActionGroup.None;
				scroll = GUILayout.BeginScrollView(scroll, Styles.white, GUILayout.ExpandWidth(true), GUILayout.Height(100));
				foreach(KSPActionGroup g in Enum.GetValues(typeof(KSPActionGroup)))
				{
					if(g == KSPActionGroup.None || g == KSPActionGroup.REPLACEWITHDEFAULT) continue;
					var is_set = group_is_set(g);
					if(Utils.ButtonSwitch(g.ToString(), is_set) && !is_set) new_group |= g;
					else if(is_set) new_group |= g;
				}
				GUILayout.EndScrollView();
				Group = new_group;
			}
            else Edit |= GUILayout.Button(new GUIContent(Name+": "+Group, Label.tooltip), Styles.normal_button);
			GUILayout.EndHorizontal();
		}

		protected override bool Action(VesselWrapper VSL)
		{ 
			if(VSL.vessel != null)
			{
				foreach(KSPActionGroup g in Enum.GetValues(typeof(KSPActionGroup)))
				{ if(group_is_set(g)) VSL.vessel.ActionGroups.ToggleGroup(g); }
			}
			return false; 
		}
	}

    [ComponentInfo(Description = "Activate selected action groups, deactivate the rest")]
	public class SetActionGroupsMacroNode : ToggleActionGroupsMacroNode
	{
		protected override bool Action(VesselWrapper VSL)
		{ 
			if(VSL.vessel != null)
			{
				foreach(KSPActionGroup g in Enum.GetValues(typeof(KSPActionGroup)))
				{ VSL.vessel.ActionGroups.SetGroup(g, group_is_set(g)); }
			}
			return false; 
		}
	}
}

