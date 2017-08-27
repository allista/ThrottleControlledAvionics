//  Author:
//       Allis Tauri <allista@gmail.com>
//
//  Copyright (c) 2015 Allis Tauri
//
// This work is licensed under the Creative Commons Attribution-ShareAlike 4.0 International License. 
// To view a copy of this license, visit http://creativecommons.org/licenses/by-sa/4.0/ 
// or send a letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.

﻿using System;
using System.Linq;
using System.Reflection;
using System.Collections.Generic;
using UnityEngine;
using AT_Utils;

namespace ThrottleControlledAvionics
{
	public enum Attitude 
	{ 
		None, 
		KillRotation, HoldAttitude, 
		Prograde, Retrograde, 
		Radial, AntiRadial, 
		Normal, AntiNormal, 
		Target, AntiTarget, 
		RelVel, AntiRelVel, 
		TargetCorrected, 
		ManeuverNode, 
		Custom 
	}
	public enum SmartEnginesMode { None, Closest, Fastest, Best }
	public enum BearingMode { None, User, Auto }
	public enum ControlMode { None, VTOL }
	public enum HFlight { None, Stop, Move, Level, NoseOnCourse, CruiseControl }
	public enum VFlight { None, AltitudeControl }
	public enum Navigation { None, GoToTarget, FollowTarget, FollowPath, Anchor, AnchorHere }
	public enum Autopilot1 { None, Land, Maneuver, MatchVel, MatchVelNear }
	public enum Autopilot2 { None, Deorbit, BallisticJump, Rendezvous, ToOrbit }

	public class VesselConfig : ConfigNodeObject, IComparable<VesselConfig>
	{
		new public const string NODE_NAME = "VSLCONFIG";
		public ConfigNode LoadedConfig { get; private set; }

		//modules
		public HashSet<string> EnabledTCAParts = new HashSet<string>();
		public SortedList<string,ConfigNode> ModuleConfigs = new SortedList<string, ConfigNode>();
		//common
		[Persistent] public Guid    VesselID;
		[Persistent] public bool    Enabled;
		[Persistent] public bool    GUIVisible;
		[Persistent] public int     ActiveTab;
		[Persistent] public KSPActionGroup ActionGroup = KSPActionGroup.None;
		//attitude control
		[Persistent] public Multiplexer<Attitude> AT = new Multiplexer<Attitude>();
		[Persistent] public bool WarpToNode = true;
		//vertical speed and altitude
		[Persistent] public Multiplexer<VFlight> VF = new Multiplexer<VFlight>();
		[Persistent] public bool    AltitudeAboveTerrain;
		[Persistent] public float   DesiredAltitude; //desired altitude m (configurable)
		[Persistent] public float   VerticalCutoff; //desired positive vertical speed m/s (configurable)
		[Persistent] public bool    BlockThrottle;
		[Persistent] public float   ControlSensitivity = 0.01f;

		public bool VSCIsActive { get { return VF || VerticalCutoff < Globals.Instance.VSC.MaxSpeed; } }
		public void DisableVSC() { VF.Off(); VerticalCutoff = Globals.Instance.VSC.MaxSpeed; BlockThrottle = false; }
		public void SmoothSetVSC(float spd) { VerticalCutoff = Mathf.Lerp(VerticalCutoff, spd, TimeWarp.fixedDeltaTime); }
		public void SmoothSetVSC(float spd, float min, float max) { VerticalCutoff = Utils.Clamp(Mathf.Lerp(VerticalCutoff, spd, TimeWarp.fixedDeltaTime), min, max); }
		//steering
		[Persistent] public Multiplexer<ControlMode> CTRL = new Multiplexer<ControlMode>();
		[Persistent] public uint    ControlTransform = 0;
		[Persistent] public float   SteeringGain     = 1f;          //steering vector is scaled by this
		[Persistent] public Vector3 SteeringModifier = Vector3.one; //steering vector is scaled by this (pitch, roll, yaw); needed to prevent too fast roll on vtols and oscilations in wobbly ships
		[Persistent] public bool    PitchYawLinked   = true;        //if true, pitch and yaw sliders will be linked
		[Persistent] public bool    AutoTune         = true;        //if true, engine PI coefficients and steering modifier will be tuned automatically
		//horizontal velocity
		[Persistent] public Multiplexer<HFlight> HF = new Multiplexer<HFlight>();
		[Persistent] public bool     SASIsControlled;
		[Persistent] public bool     SASWasEnabled;
		[Persistent] public WayPoint Anchor;
		//cruise control
		[Persistent] public Multiplexer<BearingMode> BR = new Multiplexer<BearingMode>();
		//waypoint navigation
		[Persistent] public Multiplexer<Navigation> Nav = new Multiplexer<Navigation>();
		[Persistent] public float    MaxNavSpeed = 100;
		[Persistent] public bool     ShowPath;
		[Persistent] public WayPoint Target;
		[Persistent] public NavPath  Path = new NavPath();
		//autopilot
		[Persistent] public Multiplexer<Autopilot1> AP1 = new Multiplexer<Autopilot1>();
		[Persistent] public Multiplexer<Autopilot2> AP2 = new Multiplexer<Autopilot2>();
		[Persistent] public bool VTOLAssistON = true;
		[Persistent] public bool AutoGear = true;
		[Persistent] public bool AutoBrakes = true;
		[Persistent] public bool StabilizeFlight = true;
		[Persistent] public bool UseCPS = true;
		[Persistent] public bool CorrectWithTranslation = true;
		//engines
		[Persistent] public PI_Controller Engines = new PI_Controller();
		[Persistent] public EnginesProfileDB EnginesProfiles = new EnginesProfileDB();
		[Persistent] public bool ShowManualLimits;
		[Persistent] public bool AutoStage = true;
		[Persistent] public bool AutoParachutes = true;
		[Persistent] public bool UseSmartEngines = false;
		[Persistent] public Multiplexer<SmartEnginesMode> SmartEngines = new Multiplexer<SmartEnginesMode>();
		public EnginesProfile ActiveProfile { get { return EnginesProfiles.Active; } }
		//squad
		[Persistent] public int Squad;
		//macros
		[Persistent] public TCAMacroLibrary Macros = new TCAMacroLibrary();
		[Persistent] public TCAMacro SelectedMacro;
		[Persistent] public bool MacroIsActive;
		public void StopMacro()
		{
			if(SelectedMacro != null && MacroIsActive)
			{
				MacroIsActive = false;
				SelectedMacro.Rewind();
			}
		}
		//automation
		static List<FieldInfo> multiplexer_fields = typeof(VesselConfig).GetFields(BindingFlags.Public|BindingFlags.Instance)
			.Where(fi => fi.FieldType.IsSubclassOf(typeof(Multiplexer))).ToList();
		readonly List<Multiplexer> multiplexers = new List<Multiplexer>();

		public ConfigNode Configuration 
		{ get { var node = new ConfigNode(); Save(node); return node; } }

		public VesselConfig()
		{
			//set defaults
			VerticalCutoff = Globals.Instance.VSC.MaxSpeed;
			Engines.setPI(Globals.Instance.ENG.EnginesPI);
			//explicitly set multiplexer conflicts
			AT.AddConflicts(HF, Nav, AP1, AP2);
			HF.AddConflicts(AT, Nav, AP1, AP2);
			Nav.AddConflicts(AT, HF, AP1, AP2);
			AP1.AddConflicts(AT, Nav, HF, AP2);
			AP2.AddConflicts(AT, Nav, HF, AP1);
			//get all multiplexers
			multiplexer_fields.ForEach(fi => multiplexers.Add((Multiplexer)fi.GetValue(this)));
			//init TCAParts list
			EnabledTCAParts = new HashSet<string>(TCAModulesDatabase.GetAllParts()
			                                      //comment the next line to enables OTA upgrades
			                                      .Where(p => p.Active)
			                                      .Select(p => p.Name));
		}
		public VesselConfig(Vessel vsl) : this() { VesselID = vsl.id; }
		public VesselConfig(Guid vid) : this() { VesselID = vid; }

		public override void Load(ConfigNode node)
		{
			LoadedConfig = node;
			base.Load(node);
			//restore vessel ID
			var val = node.GetValue(Utils.PropertyName(new {VesselID}));
			if(!string.IsNullOrEmpty(val)) VesselID = new Guid(val);
			//restore module configs
			var mcn = node.GetNode("ModuleConfigs");
			if(mcn != null)
			{
				ModuleConfigs.Clear();
				foreach(var n in mcn.GetNodes())
					ModuleConfigs.Add(n.name, n);
			}

			//deprecated: old config conversion
			var path = node.GetNode("Waypoints"); 
			if(path != null) Path.Load(path);
			///////////////////////////////////

			//initialize saved waypoints
			if(Anchor != null && string.IsNullOrEmpty(Anchor.Name))
				Anchor = null;
			if(Target != null && string.IsNullOrEmpty(Target.Name))
				Target = null;
			//initialize saved selected macro
			if(SelectedMacro != null && !SelectedMacro.Block.HasSubnodes)
				SelectedMacro = null;
			//FIXME: have to save/load HashSet manually until Squad fixes http://bugs.kerbalspaceprogram.com/issues/13670
			var parts_node = node.GetNode("EnabledTCAParts");
			if(parts_node != null)
			{
				EnabledTCAParts.Clear();
				parts_node.GetValues().ForEach(pname => EnabledTCAParts.Add(pname));
			}
		}

		public override void Save(ConfigNode node)
		{
			node.AddValue(Utils.PropertyName(new {VesselID}), VesselID.ToString());
			if(ModuleConfigs.Count > 0)
			{
				var mcn = node.AddNode("ModuleConfigs");
				foreach(var mc in ModuleConfigs)
					mcn.AddNode(mc.Key, mc.Value);
			}
			//FIXME: have to save/load HashSet manually until Squad fixes http://bugs.kerbalspaceprogram.com/issues/13670
			var parts_node = node.AddNode("EnabledTCAParts");
			EnabledTCAParts.ForEach(pname => parts_node.AddValue("item", pname));
			base.Save(node);
		}

		public int CompareTo(VesselConfig other)
		{ return VesselID.CompareTo(other.VesselID); }

		public static VesselConfig Clone(VesselConfig config)
		{ return ConfigNodeObject.FromConfig<VesselConfig>(config.Configuration); }

		public void CopyFrom(VesselConfig other)
		{
			var vid = VesselID;
			Load(other.Configuration);
			VesselID = vid;
		}

		public static VesselConfig FromVesselConfig(Vessel vsl, VesselConfig other)
		{
			var c = new VesselConfig(vsl);
			c.CopyFrom(other);
			return c;
		}

		public void ClearCallbacks()
		{ multiplexers.ForEach(m => m.ClearCallbacks()); }

		public void Resume(ModuleTCA TCA)
		{ 
			if(Target != null) 
			{
				Target.Update(TCA.VSL);
				TCA.VSL.SetTarget(null, Target);
			}
			if(Anchor != null) Anchor.Update(TCA.VSL);
			Path.ForEach(wp => wp.Update(TCA.VSL));
			multiplexers.ForEach(m => m.Resume()); 
		}
	}

	public class NamedConfig : VesselConfig
	{ 
		[Persistent] public string Name = "Config"; 

		public NamedConfig() {}
		public NamedConfig(string name) : this() { Name = name; }

		public static NamedConfig FromVesselConfig(string name, VesselConfig other)
		{
			var nc = new NamedConfig();
			nc.CopyFrom(other);
			nc.Name = name;
			return nc;
		}
	}
}