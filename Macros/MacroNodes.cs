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

using System;
using System.Collections.Generic;
using UnityEngine;

namespace ThrottleControlledAvionics
{
	public class SetVerticalSpeedMacroNode : SetFloatMacroNode
	{
		public SetVerticalSpeedMacroNode()
		{ Name = "Set Vertical Speed to"; Suffix = "m/s"; }

		protected override void OnValueChanged ()
		{ Value = Utils.Clamp(Value, -GLB.VSC.MaxSpeed, GLB.VSC.MaxSpeed); }

		protected override bool Action(VesselWrapper VSL)
		{ 
			VSL.CFG.BlockThrottle = true;
			VSL.CFG.VF.OffIfOn(VFlight.AltitudeControl);
			VSL.CFG.VerticalCutoff = Value; 
			return false;
		}
	}

	public class SetAltitudeMacroNode : SetFloatMacroNode
	{
		public SetAltitudeMacroNode()
		{ Name = "Set Altitude to"; Suffix = "m"; }
		protected override bool Action(VesselWrapper VSL)
		{ 
			VSL.CFG.BlockThrottle = true;
			VSL.CFG.VF.OnIfNot(VFlight.AltitudeControl);
			VSL.CFG.DesiredAltitude = Value;
			return false; 
		}
	}

	public class SetThrottleMacroNode : SetFloatMacroNode
	{
		public SetThrottleMacroNode() { Name += ":"; Suffix = "%"; }

		protected override void OnValueChanged ()
		{ Value = Utils.Clamp(Value, 0, 100); }

		protected override bool Action(VesselWrapper VSL)
		{ 
			VSL.ThrottleRequest = Value/100;
			return false; 
		}
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
		{ 
			VSL.CFG.AP.OnIfNot(Autopilot.Land); 
			return VSL.CFG.AP[Autopilot.Land];
		}
	}

	public class GoToTargetMacroNode : MacroNode
	{
		protected override bool Action(VesselWrapper VSL)
		{
			if(!VSL.HasTarget)
			{
				ScreenMessages.PostScreenMessage(Name+": No Target", 
				                                 5, ScreenMessageStyle.UPPER_CENTER);
				return false;
			}
			VSL.CFG.Nav.OnIfNot(Navigation.GoToTarget);
			return VSL.CFG.Nav[Navigation.GoToTarget];
		}
	}

	public class FollowTargetMacroNode : MacroNode
	{
		protected override bool Action(VesselWrapper VSL)
		{
			if(!VSL.HasTarget) 
			{
				ScreenMessages.PostScreenMessage(Name+": No Target", 
				                                 5, ScreenMessageStyle.UPPER_CENTER);
				return false;
			}
			VSL.CFG.Nav.On(Navigation.FollowTarget);
			return false;
		}
	}

	public class FollowPathMacroNode : MacroNode
	{
		protected Queue<WayPoint> Waypoints = new Queue<WayPoint>();
		protected bool waypoints_loaded;

		public override void Rewind()
		{ base.Rewind(); waypoints_loaded = false; }

		protected override void DrawThis ()
		{
			var title = Name;
			if(Waypoints.Count > 0) title += " (waypoints stored)";
			GUILayout.BeginHorizontal();
			if(Edit)
			{ 
				Edit &= !GUILayout.Button(Name, Styles.yellow_button, GUILayout.ExpandWidth(false));
				if(CFG != null && GUILayout.Button("Copy waypoints from Vessel", 
					Styles.yellow_button, GUILayout.ExpandWidth(false)))
					Waypoints = new Queue<WayPoint>(CFG.Waypoints);
			}
			else Edit |= GUILayout.Button(title, Styles.normal_button) && CFG != null;
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
			if(VSL.CFG.Waypoints.Count == 0) 
			{
				ScreenMessages.PostScreenMessage(Name+": No Waypoints", 
				                                 5, ScreenMessageStyle.UPPER_CENTER);
				return true;
			}
			VSL.CFG.Nav.OnIfNot(Navigation.FollowPath);
			return VSL.CFG.Nav[Navigation.FollowPath];
		}
	}

	public class FlyMacroNode : SetFloatMacroNode
	{
		public enum Mode { Forward, Backward, Right, Left, Bearing, Off }

		[Persistent] public Mode mode;
		[Persistent] public float Bearing;
		readonly FloatField BearingField = new FloatField();

		public FlyMacroNode() { Name += ":"; Suffix = "m/s"; }

		protected override void DrawThis()
		{
			GUILayout.BeginHorizontal();
			if(Edit)
			{ 
				GUILayout.Label(Name, Styles.label, GUILayout.ExpandWidth(false));
				if(GUILayout.Button(mode.ToString(), Styles.normal_button, GUILayout.ExpandWidth(false)))
					mode = (Mode)(((int)mode+1)%6);
				if(mode == Mode.Bearing) BearingField.Draw(Bearing, "°", false);
				if(mode != Mode.Off) ValueField.Draw(Value, Suffix, false);
				if(GUILayout.Button("Done", Styles.green_button, GUILayout.ExpandWidth(false)))
				{ 
					if(BearingField.UpdateValue(Bearing)) Bearing = BearingField.Value;
					if(ValueField.UpdateValue(Value)) Value = ValueField.Value;
					OnValueChanged();
					Edit = false; 
				}
			}
			else 
			{
				var title = Name+" "+mode+" ";
				if(mode == Mode.Bearing) title += Bearing+"°, ";
				if(mode != Mode.Off) title += Value.ToString("F1")+Suffix;
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
				nv = VSL.HFwd;
				break;
			case Mode.Backward:
				nv = -VSL.HFwd;
				break;
			case Mode.Right:
				nv = Vector3.ProjectOnPlane(VSL.refT.right, VSL.Up).normalized;
				break;
			case Mode.Left:
				nv = -Vector3.ProjectOnPlane(VSL.refT.right, VSL.Up).normalized;
				break;
			case Mode.Bearing:
				nv = Quaternion.AngleAxis(Bearing, VSL.Up) * 
					Vector3.ProjectOnPlane(VSL.mainBody.position+VSL.mainBody.transform.up*(float)VSL.mainBody.Radius-VSL.wCoM, VSL.Up).normalized;
				break;
			case Mode.Off:
				VSL.CFG.HF.Off();
				return false;
			}
			VSL.CFG.HF.On(HFlight.CruiseControl);
			VSL.SetNeededHorVelocity(nv*Value);
			return false;
		}
	}

	public class WaitMacroNode : SetFloatMacroNode
	{
		protected readonly Timer T = new Timer();

		public WaitMacroNode()
		{ Name = "Wait for"; Suffix = "s"; Value = (float)T.Period; }

		public override void Load(ConfigNode node)
		{ base.Load(node); T.Period = Value; T.Reset(); }

		protected override void OnValueChanged()
		{ T.Period = Value; T.Reset(); }

		public override void Rewind()
		{ base.Rewind(); T.Reset(); }

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

	public class RCSMacroNode : OnOffMacroNode
	{
		protected override bool Action(VesselWrapper VSL)
		{ VSL.ActionGroups.SetGroup(KSPActionGroup.RCS, On); return false; }
	}

	public class AutoThrottleMacroNode : OnOffMacroNode
	{
		protected override bool Action(VesselWrapper VSL)
		{ VSL.CFG.BlockThrottle = On; return false; }
	}

	public class TSASMacroNode : MacroNode
	{
		[Persistent] public Attitude attitude;

		protected override void DrawThis()
		{
			GUILayout.BeginHorizontal();
			if(Edit)
			{ 
				Edit &= !GUILayout.Button(Name, Styles.yellow_button, GUILayout.ExpandWidth(false));
				if(GUILayout.Button(attitude.ToString(), Styles.normal_button, GUILayout.ExpandWidth(false)))
					attitude = (Attitude)(((int)attitude+1)%10);
			}
			else Edit |= GUILayout.Button(Name+": "+attitude, Styles.normal_button);
			GUILayout.EndHorizontal();
		}

		protected override bool Action(VesselWrapper VSL)
		{ 
			if(attitude.Equals(Attitude.None)) VSL.CFG.AT.Off();
			else VSL.CFG.AT.OnIfNot(attitude);
			return false; 
		}
	}

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
				VSL.CFG.HF.Off();
				VSL.CFG.AT.Off();
				VSL.UnblockSAS();
			}
			else VSL.ActionGroups.SetGroup(KSPActionGroup.SAS, false);
			return false; 
		}
	}

//	public class StageMacroNode : MacroNode
//	{
//		protected override bool Action(VesselWrapper VSL)
//		{ VSL.vessel.stag; return false; }
//	}
}

