//   TCAGlobals.cs
//
//  Author:
//       Allis Tauri <allista@gmail.com>
//
//  Copyright (c) 2016 Allis Tauri

﻿using System;
using System.IO;
using System.Reflection;
using AT_Utils;

namespace ThrottleControlledAvionics
{
	class Globals : PluginGlobals<Globals>
	{
		public const string TCA_PART = "ThrottleControlledAvionics";
		public const string INSTRUCTIONS = "INSTRUCTIONS.md";

		public const string RADIATION_ICON = "ThrottleControlledAvionics/Icons/waypoint";
		public const string CIRCLE_ICON = "ThrottleControlledAvionics/Icons/path-node";

		[Persistent] public bool  IntegrateIntoCareer   = true;
		[Persistent] public bool  RoleSymmetryInFlight  = true;
		[Persistent] public bool  UseStockAppLauncher   = false;
		[Persistent] public bool  AutosaveBeforeLanding = true;

		[Persistent] public float InputDeadZone        = 0.01f; //1% of steering or translation control
		[Persistent] public int   MaxManualGroups      = 10; //maximum number of manual control groups
		[Persistent] public float KeyRepeatTime        = 0.1f;
		[Persistent] public float ClickDuration        = 0.05f;
		[Persistent] public float WaypointFadoutDist   = 10000f;
		[Persistent] public float CameraFadeinPower    = 0.3f;
		[Persistent] public float UnpackDistance       = 5000f;
		[Persistent] public float ActionListHeight     = 110f;
		[Persistent] public float MaxAAFilter          = 1f;
		[Persistent] public float ExhaustSafeDist      = 1.1f;

		[Persistent] public string PersistentRotationName = "PersistentRotation";
		[Persistent] public float PersistentRotationThreshold = 5e-7f;
		[Persistent] public float NoPersistentRotationThreshold = 5e-7f;

		[Persistent] public EngineOptimizer.Config           ENG = new EngineOptimizer.Config();
		[Persistent] public VerticalSpeedControl.Config      VSC = new VerticalSpeedControl.Config();
		[Persistent] public AltitudeControl.Config           ALT = new AltitudeControl.Config();
		[Persistent] public AttitudeControlBase.Config       ATCB = new AttitudeControlBase.Config();
		[Persistent] public AttitudeControl.Config           ATC = new AttitudeControl.Config();
		[Persistent] public BearingControl.Config            BRC = new BearingControl.Config();
		[Persistent] public ThrustDirectionControl.Config    TDC = new ThrustDirectionControl.Config();
		[Persistent] public HorizontalSpeedControl.Config    HSC = new HorizontalSpeedControl.Config();
		[Persistent] public RCSOptimizer.Config              RCS = new RCSOptimizer.Config();
		[Persistent] public CruiseControl.Config             CC  = new CruiseControl.Config();
		[Persistent] public Anchor.Config                    ANC = new Anchor.Config();
		[Persistent] public PointNavigator.Config            PN  = new PointNavigator.Config();
		[Persistent] public Radar.Config                     RAD = new Radar.Config();
		[Persistent] public AutoLander.Config                LND = new AutoLander.Config();
		[Persistent] public VTOLAssist.Config                TLA = new VTOLAssist.Config();
		[Persistent] public VTOLControl.Config               VTOL = new VTOLControl.Config();
		[Persistent] public CollisionPreventionSystem.Config CPS = new CollisionPreventionSystem.Config();
		[Persistent] public FlightStabilizer.Config          STB = new FlightStabilizer.Config();
		[Persistent] public ThrottleControl.Config           THR = new ThrottleControl.Config();
		[Persistent] public TranslationControl.Config        TRA = new TranslationControl.Config();
		[Persistent] public TimeWarpControl.Config           WRP = new TimeWarpControl.Config();
		[Persistent] public ManeuverAutopilot.Config         MAN = new ManeuverAutopilot.Config();
		[Persistent] public MatchVelocityAutopilot.Config    MVA = new MatchVelocityAutopilot.Config();

		[Persistent] public TrajectoryCalculator.Config      TRJ = new TrajectoryCalculator.Config();
		[Persistent] public LandingTrajectoryAutopilot.Config LTRJ = new LandingTrajectoryAutopilot.Config();
		[Persistent] public DeorbitAutopilot.Config          DEO = new DeorbitAutopilot.Config();
		[Persistent] public BallisticJump.Config             BJ  = new BallisticJump.Config();
		[Persistent] public RendezvousAutopilot.Config       REN = new RendezvousAutopilot.Config();
		[Persistent] public ToOrbitAutopilot.Config          ORB = new ToOrbitAutopilot.Config();

		public MDSection Manual;

        #if DEBUG
        public ConfigNodeObjectGUI UI;
        #endif

		public override void Init()
		{ 
			try
			{
				using(var file = new StreamReader(PluginFolder(INSTRUCTIONS)))
				{
					Manual = MD2Unity.Parse(file);
					if(Manual.NoTitle) Manual.Title = "TCA Reference Manual";
				}
			}
			catch(Exception ex) { Utils.Log("Error loading {} file:\n{}", PluginFolder(INSTRUCTIONS), ex); }
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
            #if DEBUG
            UI = ConfigNodeObjectGUI.FromObject(this);
            #endif
		}
	}
}

