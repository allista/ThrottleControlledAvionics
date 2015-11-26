//  Author:
//       Allis Tauri <allista@gmail.com>
//
//  Copyright (c) 2015 Allis Tauri
//
// This work is licensed under the Creative Commons Attribution-ShareAlike 4.0 International License. 
// To view a copy of this license, visit http://creativecommons.org/licenses/by-sa/4.0/ 
// or send a letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.

﻿using System;
using System.Reflection;
using System.Collections.Generic;
using UnityEngine;

namespace ThrottleControlledAvionics
{
	public class TCAGlobals : ConfigNodeObject
	{
		new public const string NODE_NAME = "TCAGLOBALS";

		public const string TCA_PART = "ThrottleControlledAvionics";

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

		//help text
		public string Instructions = string.Empty;
		const string instructions = 
@"Welcome to TCA instructions manual.

For simple use:
    1) Turn TCA on ('{0}')
    2) Turn SAS on ('t')
    3) Activate the engines and throttle them up!

Engine Modes:
    * In editor or in flight (through the part menu) you may set any engine to work in one of the three modes: 
        1) Main Engine (default). TCA tries to maximize the thrust of these engines. In a perfectly balanced ship all Main Engines should have 100% thrust in the absence of control input. These engines are also used to control vertical speed.
        2) Maneuver Engine. TCA tries to minimize the thrust of these engines. In a perfectly balanced ship these engines produce thrust only in response to control input
        3) Manual Control. TCA does not change the thrust of these engines, but includes them in calculations.

Autotuning Parameters:
    * If this option is enabled, TCA calculates Steering Gains and PI coefficients as functions of maximum possible angular acceleration along each of the principal axes of a craft. Note, that calculations are based on the predefined response curves tuned for the stock SAS.
    * If you have already tuned these parameters for the ship, save its configuration before enabling, as autotuning will overwrite previous parameters.

Steering Gains:
    * Master Gain modifies the magnitude of all steering input after other modifications. If the craft reacts slowly, increase it a little. If the craft vibrates after a maneuver, decrease it.
    * Pitch, Yaw and Roll modify only corresponding inputs.
    * Linking Pitch and Yaw useful if the ship's engines are symmetrical along main axis. In that case Pitch&Yaw slider may be used in stead of Master Gain.

Engines PI-controller tuning:
    * P (proportional) parameter controls the response speed of the engines. If the craft reacts slowly, increase it a little. If the craft vibrates after a maneuver, decrease it.
    * I (integral) parameter controls the smoothing. It prevents rapid changes in thrust. If engines's trust jitters, increase it.

Vertical Speed Control, hovering and horizontal flight:
    * If you're using TCA to control VTOL or during vertical landing of a rocket, you may enable the Vertical Speed Control System.
    * The desired vertical speed may be set with the scroll bar in the (configurable) interval from -{1:F1}m/s to {1:F1}m/s (not including). Then the total thrust of all controllable engines is modified in an attempt to reach the specified vertical speed. This speed, however, is never achieved, but approached asymptotically, so you need to set it a little higher (0.1-0.5m/s) than desired.
    * To completely disable the VSC, just set it the desired speed to its maximum value ({1}m/s).
    * VSC is also very useful to maintain stable horizontal flight. Consider a VTOL that has lifted off, reached some altitude and started to pitch to get some forward momentum. If the thrust of its engines will remain constant, it will start to loose altitude as it moves forward. But with the VSC the thrust will be adjusted, and the VTOL will move more or less in a plane.

Set vertical speed with throttle controls:
    * When this option is enabled, the throttle is locked at 100% and throttle controls are used to set desired vertical speed instead. If VSC system was switched off it is automatically enabled and the desired speed is set to 0.

Kill Horizontal Velocity:
    * Enables an autopilot that tries to maneuver the craft so that the horizontal component of its velocity became zero. It includes flip-over prevention system, so whatever the speed, the autopilot decrease it carefully and steady.

Notes:
    * For safety reasons the Vertical and Horizontal speed controls are disabled in orbit, but not on suborbital trajectories, so be carefull.
    * If your ship wobbles and oscillates with TCA and SAS enabled, rebuild it with more struts, or decrease appropriate Steering Gains.
    * Thrust of jets and turbofan engines changes very slowly. Thus using them as attitude controllers is impractical. If you want to use them, switch them to Manual Control mode.
    * Solid boosters have constant thrust and thus cannot be controlled by TCA. But they are still accounted for, if present.";

		public void Init()
		{ 
			Instructions = string.Format(instructions, ThrottleControlledAvionics.TCA_Key, VSC.MaxSpeed);
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

		public ConfigNode Configuration 
		{ get { var node = new ConfigNode(); Save(node); return node; } }

		public VesselConfig() //set defaults
		{
			VerticalCutoff = TCAScenario.Globals.VSC.MaxSpeed;
			Engines.setPI(TCAScenario.Globals.ENG.EnginesPI);
			AT.AddConflicts(HF, Nav, AP);
			HF.AddConflicts(AT, Nav, AP);
			Nav.AddConflicts(AT, HF, AP);
			AP.AddConflicts(AT, Nav, HF);
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
		{
			HF.ClearCallbacks();
			VF.ClearCallbacks();
			Nav.ClearCallbacks();
			AP.ClearCallbacks();
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