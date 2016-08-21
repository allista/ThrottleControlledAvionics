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
using System.Collections.Generic;
using UnityEngine;
using AT_Utils;

namespace ThrottleControlledAvionics
{
	[RequireModules(typeof(VerticalSpeedControl))]
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
	public class StopMacroNode : MacroNode
	{
		protected override bool Action(VesselWrapper VSL)
		{ VSL.CFG.HF.XOn(HFlight.Stop); return false; }
	}

	[RequireModules(typeof(HorizontalSpeedControl))]
	public class LevelMacroNode : MacroNode
	{
		protected override bool Action(VesselWrapper VSL)
		{ VSL.CFG.HF.XOn(HFlight.Level); return false; }
	}

	[RequireModules(typeof(Anchor))]
	public class AnchorMacroNode : MacroNode
	{
		protected override bool Action(VesselWrapper VSL)
		{ VSL.CFG.Nav.XOn(Navigation.AnchorHere); return false; }
	}

	[RequireModules(typeof(MatchVelocityAutopilot))]
	public class MatchVelocityMacroNode : MacroNode
	{
		protected override bool Action(VesselWrapper VSL)
		{
			VSL.CFG.AP1.XOn(Autopilot1.MatchVel);
			return false;
		}
	}

	[RequireModules(typeof(MatchVelocityAutopilot))]
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
	public class LandMacroNode : MacroNode
	{
		protected override bool Action(VesselWrapper VSL)
		{ 
			VSL.CFG.AP1.XOnIfNot(Autopilot1.Land); 
			return VSL.CFG.AP1[Autopilot1.Land];
		}
	}

	[RequireModules(typeof(BallisticJump))]
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
	public class ToOrbitNode : MacroNode
	{
		[Persistent] public TargetOrbitInfo OrbitInfo = new TargetOrbitInfo();

		protected override void DrawThis()
		{
			GUILayout.BeginHorizontal();
			if(Edit)
			{ 
				GUILayout.Label(Name, Styles.label, GUILayout.ExpandWidth(false));
				OrbitInfo.Draw(false);
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
	public class FollowPathMacroNode : MacroNode
	{
		protected Queue<WayPoint> Waypoints = new Queue<WayPoint>();
		protected bool waypoints_loaded;

		public override void Rewind()
		{ base.Rewind(); waypoints_loaded = false; }

		public override void Save(ConfigNode node)
		{
			base.Save(node);
			var wpn = node.AddNode("Waypoints");
			foreach(var wp in Waypoints)
				wp.Save(wpn.AddNode(WayPoint.NODE_NAME));
		}

		public override void Load(ConfigNode node)
		{
			base.Load(node);
			Waypoints.Clear();
			var wpn = node.GetNode("Waypoints");
			if(wpn == null) return;
			foreach(var n in wpn.GetNodes(WayPoint.NODE_NAME))
				Waypoints.Enqueue(ConfigNodeObject.FromConfig<WayPoint>(n));
		}

		protected override void DrawThis ()
		{
			var title = Name;
			if(Waypoints.Count > 0) title += " (waypoints stored)";
			GUILayout.BeginHorizontal();
			if(Edit)
			{ 
				Edit &= !GUILayout.Button(title, Styles.active_button, GUILayout.ExpandWidth(false));
				if(EditedCFG != null && GUILayout.Button("Copy waypoints from Vessel", 
				                                         Styles.active_button, GUILayout.ExpandWidth(false)))
					Waypoints = new Queue<WayPoint>(EditedCFG.Waypoints);
			}
			else Edit |= GUILayout.Button(title, Styles.normal_button) && EditedCFG != null;
			GUILayout.EndHorizontal();
		}

		protected override bool Action(VesselWrapper VSL)
		{
			if(!waypoints_loaded)
			{
				if(Waypoints.Count > 0)
					VSL.CFG.Waypoints = new Queue<WayPoint>(Waypoints);
				waypoints_loaded = true;
			}
			if(VSL.CFG.Waypoints.Count == 0) { Message("No Waypoints"); return false; }
			VSL.CFG.Nav.XOnIfNot(Navigation.FollowPath);
			return VSL.CFG.Nav[Navigation.FollowPath];
		}
	}

	[RequireModules(typeof(HorizontalSpeedControl))]
	public class FlyMacroNode : SetFloatMacroNode
	{
		public enum Mode { Forward, Backward, Right, Left, Bearing, Off }

		[Persistent] public Mode mode;
		[Persistent] public FloatField Bearing = new FloatField("F1", 0, 360);

		[Obsolete("Only needed for legacy config conversion")]
		public override void Load(ConfigNode node)
		{
			base.Load(node);
			if(node.HasValue("Bearing"))
			{
				float val;
				if(float.TryParse(node.GetValue("Bearing"), out val))
					Bearing.Value = val;
			}
		}

		public FlyMacroNode() { Name += ":"; Suffix = "m/s"; }

		protected override void DrawThis()
		{
			GUILayout.BeginHorizontal();
			if(Edit)
			{ 
				GUILayout.Label(Name, Styles.label, GUILayout.ExpandWidth(false));
				if(GUILayout.Button(mode.ToString(), Styles.normal_button, GUILayout.ExpandWidth(false)))
					mode = (Mode)(((int)mode+1)%6);
				if(mode == Mode.Bearing) Bearing.Draw("°", false, 10);
				if(mode != Mode.Off) Value.Draw(Suffix, false);
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
	public class FollowTerrainMacroNode : OnOffMacroNode
	{
		protected override bool Action(VesselWrapper VSL)
		{ VSL.CFG.AltitudeAboveTerrain = On; return false; }
	}

	public class LightsMacroNode : OnOffMacroNode
	{
		protected override bool Action(VesselWrapper VSL)
		{ VSL.vessel.ActionGroups.SetGroup(KSPActionGroup.Light, On); return false; }
	}

	public class BrakesMacroNode : OnOffMacroNode
	{
		protected override bool Action(VesselWrapper VSL)
		{ VSL.vessel.ActionGroups.SetGroup(KSPActionGroup.Brakes, On); return false; }
	}

	public class GearMacroNode : OnOffMacroNode
	{
		protected override bool Action(VesselWrapper VSL)
		{ VSL.vessel.ActionGroups.SetGroup(KSPActionGroup.Gear, On); return false; }
	}

	public class RCSMacroNode : OnOffMacroNode
	{
		protected override bool Action(VesselWrapper VSL)
		{ VSL.vessel.ActionGroups.SetGroup(KSPActionGroup.RCS, On); return false; }
	}

	[RequireModules(typeof(VerticalSpeedControl), 
	                typeof(ThrottleControl))]
	public class AutoThrottleMacroNode : OnOffMacroNode
	{
		protected override bool Action(VesselWrapper VSL)
		{ VSL.CFG.BlockThrottle = On; return false; }
	}

	[RequireModules(typeof(AttitudeControl))]
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
	public class DisableVerticalControlMacroNode : MacroNode
	{
		protected override bool Action(VesselWrapper VSL)
		{ VSL.CFG.DisableVSC(); return false; }
	}

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

	public class StageMacroNode : MacroNode
	{
		protected override bool Action(VesselWrapper VSL)
		{ VSL.ActivateNextStage(); return false; }
	}
}

