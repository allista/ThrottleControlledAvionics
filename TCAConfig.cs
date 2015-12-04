//  Author:
//       Allis Tauri <allista@gmail.com>
//
//  Copyright (c) 2015 Allis Tauri
//
// This work is licensed under the Creative Commons Attribution-ShareAlike 4.0 International License. 
// To view a copy of this license, visit http://creativecommons.org/licenses/by-sa/4.0/ 
// or send a letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.

﻿using System;
using System.IO;
using System.Linq;
using System.Reflection;
using System.Collections.Generic;
using UnityEngine;

namespace ThrottleControlledAvionics
{
	public class TCAGlobals : ConfigNodeObject
	{
		new public const string NODE_NAME = "TCAGLOBALS";

		public const string TCA_PART = "ThrottleControlledAvionics";
		public const string INSTRUCTIONS = "INSTRUCTIONS.txt";

		[Persistent] public bool  IntegrateIntoCareer  = true;
		[Persistent] public bool  RoleSymmetryInFlight = true;
		[Persistent] public bool  UseStockAppLauncher  = false;
		[Persistent] public float InputDeadZone        = 0.01f; //1% of steering or translation control
		[Persistent] public int   MaxManualGroups      = 10; //maximum number of manual control groups
		[Persistent] public float KeyRepeatTime        = 0.1f;
		[Persistent] public float WaypointHeight       = 3f;
		[Persistent] public float UnpackDistance       = 5000f;
		[Persistent] public float ActionListHeight     = 110f;
		[Persistent] public float MaxAAFilter          = 1f;

		[Persistent] public EngineOptimizer.Config           ENG = new EngineOptimizer.Config();
		[Persistent] public VerticalSpeedControl.Config      VSC = new VerticalSpeedControl.Config();
		[Persistent] public AltitudeControl.Config           ALT = new AltitudeControl.Config();
		[Persistent] public AttitudeControl.Config           ATC = new AttitudeControl.Config();
		[Persistent] public HorizontalSpeedControl.Config    HSC = new HorizontalSpeedControl.Config();
		[Persistent] public RCSOptimizer.Config              RCS = new RCSOptimizer.Config();
		[Persistent] public CruiseControl.Config             CC  = new CruiseControl.Config();
		[Persistent] public Anchor.Config                    ANC = new Anchor.Config();
		[Persistent] public PointNavigator.Config            PN  = new PointNavigator.Config();
		[Persistent] public Radar.Config                     RAD = new Radar.Config();
		[Persistent] public AutoLander.Config                LND = new AutoLander.Config();
		[Persistent] public VTOLAssist.Config                TLA = new VTOLAssist.Config();
		[Persistent] public CollisionPreventionSystem.Config CPS = new CollisionPreventionSystem.Config();
		[Persistent] public FlightStabilizer.Config          STB = new FlightStabilizer.Config();
		[Persistent] public ThrottleControl.Config           THR = new ThrottleControl.Config();
		[Persistent] public TranslationControl.Config        TRA = new TranslationControl.Config();
		[Persistent] public TimeWarpControl.Config           WRP = new TimeWarpControl.Config();
		[Persistent] public MatchVelocityAutopilot.Config    MVA = new MatchVelocityAutopilot.Config();

		public string Instructions = string.Empty;

		public void Init()
		{ 
			try
			{
				using(var file = new StreamReader(TCAScenario.PluginFolder(INSTRUCTIONS)))
					Instructions = file.ReadToEnd();
			}
			catch { Instructions = string.Format("{0} file cannot be read", TCAScenario.PluginFolder(INSTRUCTIONS)); }
			InputDeadZone *= InputDeadZone; //it is compared with the sqrMagnitude
			//init all module configs
			var mt = typeof(TCAModule.ModuleConfig);
			foreach(var fi in GetType().GetFields())
			{
				if(!fi.FieldType.IsSubclassOf(mt)) continue;
				var method = fi.FieldType.GetMethod("Init");
				if(method == null) continue;
				method.Invoke(fi.GetValue(this), null);
			}
		}

		//Globals are readonly
		public override void Save(ConfigNode node) {}
	}

	public enum Attitude { None, KillRotation, HoldAttitude, Prograde, Retrograde, Radial, AntiRadial, Normal, AntiNormal, Target, AntiTarget, RelVel, AntiRelVel, ManeuverNode, Custom }
	public enum HFlight { None, Stop, Move, Level, NoseOnCourse, CruiseControl }
	public enum VFlight { None, AltitudeControl }
	public enum Navigation { None, GoToTarget, FollowTarget, FollowPath, Anchor, AnchorHere }
	public enum Autopilot { None, Land, Maneuver, MatchVel, MatchVelNear }

	public class VesselConfig : ConfigNodeObject, IComparable<VesselConfig>
	{
		new public const string NODE_NAME = "VSLCONFIG";

		[Persistent] public Guid    VesselID;
		[Persistent] public bool    Enabled;
		[Persistent] public bool    GUIVisible;
		//attitude control
		[Persistent] public Multiplexer<Attitude> AT = new Multiplexer<Attitude>();
		[Persistent] public bool WarpToNode;
		//vertical speed and altitude
		[Persistent] public Multiplexer<VFlight> VF = new Multiplexer<VFlight>();
		[Persistent] public bool    AltitudeAboveTerrain;
		[Persistent] public float   DesiredAltitude; //desired altitude m (configurable)
		[Persistent] public float   VerticalCutoff; //desired positive vertical speed m/s (configurable)
		[Persistent] public bool    BlockThrottle;
		[Persistent] public float   VSControlSensitivity = 0.01f;
		public bool VSCIsActive { get { return VF || VerticalCutoff < TCAScenario.Globals.VSC.MaxSpeed; } }
		public void DisableVSC() { VF.Off(); VerticalCutoff = TCAScenario.Globals.VSC.MaxSpeed; }
		//steering
		[Persistent] public float   SteeringGain     = 1f;          //steering vector is scaled by this
		[Persistent] public Vector3 SteeringModifier = Vector3.one; //steering vector is scaled by this (pitch, roll, yaw); needed to prevent too fast roll on vtols and oscilations in wobbly ships
		[Persistent] public bool    PitchYawLinked   = true;        //if true, pitch and yaw sliders will be linked
		[Persistent] public bool    AutoTune         = true;        //if true, engine PI coefficients and steering modifier will be tuned automatically
		//horizontal velocity
		[Persistent] public Multiplexer<HFlight> HF = new Multiplexer<HFlight>();
		[Persistent] public bool    SASIsControlled;
		[Persistent] public bool    SASWasEnabled;
		[Persistent] public WayPoint Anchor;
		//cruise control
		[Persistent] public Vector3  SavedUp;
		[Persistent] public Vector3  NeededHorVelocity;
		//waypoint navigation
		[Persistent] public Multiplexer<Navigation> Nav = new Multiplexer<Navigation>();
		[Persistent] public float    MaxNavSpeed = 100;
		[Persistent] public bool     ShowWaypoints;
		public Queue<WayPoint>       Waypoints = new Queue<WayPoint>();
		[Persistent] public WayPoint Target;
		//autopilot
		[Persistent] public Multiplexer<Autopilot> AP = new Multiplexer<Autopilot>();
		[Persistent] public bool VTOLAssistON = true;
		[Persistent] public bool StabilizeFlight = true;
		//engines
		[Persistent] public PI_Controller Engines = new PI_Controller();
		[Persistent] public EnginesProfileDB EnginesProfiles = new EnginesProfileDB();
		[Persistent] public bool ShowEnginesProfiles;
		[Persistent] public bool ShowManualLimits;
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
			VerticalCutoff = TCAScenario.Globals.VSC.MaxSpeed;
			Engines.setPI(TCAScenario.Globals.ENG.EnginesPI);
			//explicitly set multiplexer conflicts
			AT.AddConflicts(HF, Nav, AP);
			HF.AddConflicts(AT, Nav, AP);
			Nav.AddConflicts(AT, HF, AP);
			AP.AddConflicts(AT, Nav, HF);
			//get all multiplexers
			multiplexer_fields.ForEach(fi => multiplexers.Add((Multiplexer)fi.GetValue(this)));
		}
		public VesselConfig(Vessel vsl) : this() { VesselID = vsl.id; }
		public VesselConfig(Guid vid) : this() { VesselID = vid; }

		public override void Load(ConfigNode node)
		{
			base.Load(node);
			var val = node.GetValue(Utils.PropertyName(new {VesselID}));
			if(!string.IsNullOrEmpty(val)) VesselID = new Guid(val);
			foreach(var n in node.GetNodes(WayPoint.NODE_NAME))
				Waypoints.Enqueue(ConfigNodeObject.FromConfig<WayPoint>(n));
			if(Anchor != null && string.IsNullOrEmpty(Anchor.Name))
				Anchor = null;
			if(Target != null && string.IsNullOrEmpty(Target.Name))
				Target = null;
			if(SelectedMacro != null && !SelectedMacro.Block.HasSubnodes)
				SelectedMacro = null;
		}

		public override void Save(ConfigNode node)
		{
			node.AddValue(Utils.PropertyName(new {VesselID}), VesselID.ToString());
			foreach(var wp in Waypoints)
				wp.Save(node.AddNode(WayPoint.NODE_NAME));
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

		public void Resume()
		{ multiplexers.ForEach(m => m.Resume()); }
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