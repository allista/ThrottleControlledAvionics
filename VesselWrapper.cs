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
using System.Reflection;
using UnityEngine;

namespace ThrottleControlledAvionics
{
	public class VesselWrapper
	{
		public ModuleTCA    TCA { get; private set; }
		public VesselConfig CFG { get; private set; }
		public TCAGlobals   GLB { get { return TCAScenario.Globals; } }

		public TCAState State;
		public void     SetState(TCAState state) { State |= state; }
		public bool     IsStateSet(TCAState state) { return (State & state) == state; }

		public Vessel vessel { get; private set; }
		public Transform refT; //transform of the controller-part
		public bool ForceUpdateParts;

		public PhysicalProps        Physics;
		public AltitudeProps        Altitude;
		public VerticalSpeedProps   VerticalSpeed;
		public HorizontalSpeedProps HorizontalSpeed;
		public EnginesProps         Engines;
		public ControlProps         Controls;
		public InfoProps            Info;
		public OnPlanetProps		OnPlanetParams;
		public TorqueProps          Torque;
		public GeometryProps        Geometry;
		static List<FieldInfo> AllProps = typeof(VesselWrapper)
			.GetFields(BindingFlags.DeclaredOnly|BindingFlags.Instance|BindingFlags.Public)
			.Where(fi => fi.FieldType.IsSubclassOf(typeof(VesselProps))).ToList();
		
		//state
		public CelestialBody mainBody { get { return vessel.mainBody; } }
		public Orbit orbit { get { return vessel.orbit; } }
		public bool OnPlanet { get; private set; }
		public bool InOrbit { get; private set; }
		public bool IsActiveVessel { get; private set; }
		public bool LandedOrSplashed { get { return vessel.LandedOrSplashed; } }
		public ITargetable Target { get { return vessel.targetObject; } set { vessel.targetObject = value; } }
		public bool HasTarget { get { return vessel.targetObject != null && !(vessel.targetObject is CelestialBody); } }
		public bool HasManeuverNode { get { return vessel.patchedConicSolver != null && vessel.patchedConicSolver.maneuverNodes.Count > 0; } }
		public Vessel.Situations Situation { get { return vessel.situation; } }


		#region Utils
		public void Log(string msg, params object[] args) { vessel.Log(msg, args); }
		public Vector3 LocalDir(Vector3 worldV) { return refT.InverseTransformDirection(worldV); }
		public Vector3 WorldDir(Vector3 localV) { return refT.TransformDirection(localV); }
		public Vector3d PredictedSrfVelocity(float time) { return vessel.srf_velocity+vessel.acceleration*time; }
		#endregion

		#region VesselRanges
		static List<FieldInfo> situation_ranges = typeof(VesselRanges)
			.GetFields(BindingFlags.Public|BindingFlags.Instance)
			.Where(fi => fi.FieldType.Equals(typeof(VesselRanges.Situation))).ToList();
		VesselRanges saved_ranges;

		public void SetUnpackDistance(float distance)
		{
			if(saved_ranges == null) 
				saved_ranges = new VesselRanges(vessel.vesselRanges);
			foreach(var fi in situation_ranges)
			{
				var sit = fi.GetValue(vessel.vesselRanges) as VesselRanges.Situation;
				if(sit == null) continue;
				sit.pack   = distance*1.5f;
				sit.unpack = distance;
				sit.unload = distance*2.5f;
				sit.load   = distance*2f;
				Log("{0}: unload {1}, load {2}, pack {3}, unpack {4}", fi.Name, sit.unload, sit.load, sit.pack, sit.unpack);//debug
			}
		}

		public void RestoreUnpackDistance()
		{
			if(saved_ranges == null) return;
			vessel.vesselRanges = new VesselRanges(saved_ranges);
			saved_ranges = null;
			Log("VesselRanges restored");//debug
		}
		#endregion

		public VesselWrapper(ModuleTCA tca)
		{
			TCA = tca; 
			CFG = tca.CFG;
			vessel = tca.vessel;
			AllProps.ForEach(fi => TCA.CreateComponent(this, fi));
			OnPlanet = _OnPlanet();
			InOrbit  = _InOrbit();
			UpdateState();
			UpdatePhysics();
			UpdateParts();
		}

		public void Init()
		{
			UpdateCommons();
			OnPlanetParams.Update();
			Geometry.Update();
			vessel.OnAutopilotUpdate += UpdateAutopilotInfo;
		}

		public void Reset()
		{
			vessel.OnAutopilotUpdate -= UpdateAutopilotInfo;
			RestoreUnpackDistance();
		}

		bool _OnPlanet() 
		{ 
			return (vessel.situation != Vessel.Situations.DOCKED   &&
			        vessel.situation != Vessel.Situations.ORBITING &&
			        vessel.situation != Vessel.Situations.ESCAPING); 
		}

		bool _InOrbit()
		{
			return vessel.situation == Vessel.Situations.ORBITING ||
				vessel.situation == Vessel.Situations.SUB_ORBITAL;
		}

		public bool AutopilotDisabled { get; private set; }
		public void UpdateAutopilotInfo(FlightCtrlState s)
		{
			if(!CFG.Enabled) return;
			AutopilotDisabled = 
				!Mathfx.Approx(s.pitch, s.pitchTrim, 0.1f) ||
				!Mathfx.Approx(s.roll, s.rollTrim, 0.1f) ||
				!Mathfx.Approx(s.yaw, s.yawTrim, 0.1f);
		}

		public void UpdateState()
		{
			//update onPlanet state
			var on_planet = _OnPlanet();
			var in_orbit  = _InOrbit();
			if(on_planet != OnPlanet) 
			{
				CFG.EnginesProfiles.OnPlanetChanged(on_planet);
				if(!on_planet) 
				{ 
					if(CFG.BlockThrottle) 
					{
						var THR = TCA.GetModule<ThrottleControl>();
						if(THR != null ) THR.Throttle = 0f;
					}
					CFG.DisableVSC();
					CFG.Nav.Off(); 
					CFG.HF.Off();
				}
			}
			OnPlanet = on_planet;
			InOrbit = in_orbit;
			IsActiveVessel = vessel != null && vessel == FlightGlobals.ActiveVessel;
		}

		public void UpdatePhysics()
		{
			Physics.Update();
			Altitude.Update();
		}

		public void UpdateCommons()
		{
			Engines.Update();
			Torque.Update();
			Controls.Update();
		}

		public void ClearFrameState()
		{
			TCA.ClearFrameState();
			Info.ClearFrameState();
		}

		public void UpdateOnPlanetStats()
		{
			if(!OnPlanet) return;
			VerticalSpeed.Update();
			HorizontalSpeed.Update();
			OnPlanetParams.Update();
		}

		public void UpdateParts()
		{
			EngineWrapper.ThrustPI.setMaster(CFG.Engines);
			Engines.Clear(); Torque.Wheels.Clear();
			for(int i = 0, vesselPartsCount = vessel.Parts.Count; i < vesselPartsCount; i++)
			{
				Part p = vessel.Parts[i];
				for(int j = 0, pModulesCount = p.Modules.Count; j < pModulesCount; j++)
				{
					//engines
					var module = p.Modules[j];
					var engine = module as ModuleEngines;
					if(engine != null)
					{
						Engines.Add(new EngineWrapper(engine));
						continue;
					}
					//reaction wheels
					var rwheel = module as ModuleReactionWheel;
					if(rwheel != null)
					{
						Torque.Wheels.Add(rwheel);
						continue;
					}
					//rcs
					var rcs = module as ModuleRCS;
					if(rcs != null)
					{
						Engines.RCS.Add(new RCSWrapper(rcs));
						continue;
					}
				}
			}
			if(CFG.EnginesProfiles.Empty) CFG.EnginesProfiles.AddProfile(Engines.All);
			else if(CFG.Enabled && Engines.ProfileSyncAllowed) CFG.ActiveProfile.Update(Engines.All);
		}
	}

	/// <summary>
	/// Binary flags of TCA state.
	/// They should to be checked in this particular order, as they are set sequentially:
	/// If a previous flag is not set, the next ones are not either.
	/// </summary>
	[Flags] public enum TCAState 
	{ 
		//basic state
		Disabled 			   = 0,
		Enabled 			   = 1 << 0,
		HaveEC 				   = 1 << 1, 
		HaveActiveEngines 	   = 1 << 2,
		Unoptimized			   = 1 << 3,
		//vertical flight
		VerticalSpeedControl   = 1 << 4,
		AltitudeControl        = 1 << 5,
		LoosingAltitude 	   = 1 << 6,
		//cruise radar
		ObstacleAhead	 	   = 1 << 7,
		GroundCollision	 	   = 1 << 8,
		Ascending		 	   = 1 << 9,
		//autopilot
		Scanning               = 1 << 10,
		Searching              = 1 << 11,
		CheckingSite           = 1 << 12,
		Landing                = 1 << 13,
		VTOLAssist             = 1 << 14,
		StabilizeFlight        = 1 << 15,
		//composite
		Nominal				   = Enabled | HaveEC | HaveActiveEngines,
		NoActiveEngines        = Enabled | HaveEC,
		NoEC                   = Enabled,
	}
}