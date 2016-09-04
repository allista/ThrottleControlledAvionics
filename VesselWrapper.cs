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
using KSP.UI.Screens;
using AT_Utils;

namespace ThrottleControlledAvionics
{
	public class VesselWrapper
	{
		public ModuleTCA    TCA { get; private set; }
		public VesselConfig CFG { get; private set; }
		internal Globals    GLB { get { return Globals.Instance; } }

		public TCAState State;
		public void     SetState(TCAState state) { State |= state; }
		public bool     IsStateSet(TCAState state) { return (State & state) == state; }

		public Vessel vessel { get; private set; }
		public Transform refT; //transform of the controller-part

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
		List<VesselProps> AllPros = new List<VesselProps>();
		static List<FieldInfo> AllPropFields = typeof(VesselWrapper)
			.GetFields(BindingFlags.DeclaredOnly|BindingFlags.Instance|BindingFlags.Public)
			.Where(fi => fi.FieldType.IsSubclassOf(typeof(VesselProps))).ToList();
		
		//state
		public CelestialBody Body { get { return vessel.mainBody; } }
		public Orbit orbit { get { return vessel.orbit; } }
		public bool OnPlanet { get; private set; }
		public bool InOrbit { get; private set; }
		public bool IsActiveVessel { get; private set; }
		public bool LandedOrSplashed { get { return vessel.LandedOrSplashed; } }
		public ITargetable Target { get { return vessel.targetObject; } set { vessel.targetObject = value; } }
		public Vessel TargetVessel { get { return vessel.targetObject == null? null : vessel.targetObject.GetVessel(); } }
		public bool HasTarget { get { return vessel.targetObject != null && !(vessel.targetObject is CelestialBody); } }
		public bool HasManeuverNode 
		{ 
			get 
			{ 
				return vessel.patchedConicSolver != null && 
					vessel.patchedConicSolver.maneuverNodes.Count > 0 && 
					vessel.patchedConicSolver.maneuverNodes[0] != null; 
			} 
		}
		public ManeuverNode FirstManeuverNode { get { return vessel.patchedConicSolver.maneuverNodes[0]; } }
		public Vessel.Situations Situation { get { return vessel.situation; } }

		public void SetTarget(WayPoint wp = null)
		{
			CFG.Target = wp;
			var t = wp == null? null : wp.GetTarget();
			if(t != null && IsActiveVessel) 
				Utils.Message("Target: {0}", t.GetName());
			Target = t;
		}

		#if DEBUG
		public void LogF(string msg, params object[] args) 
		{ vessel.Log("{0}", Utils.Format(msg, args)); }

		public void LogFST(string msg, params object[] args) 
		{ vessel.Log("{0}\n{1}", Utils.Format(msg, args), DebugUtils.getStacktrace(1)); }
		#endif

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
			}
		}

		public void RestoreUnpackDistance()
		{
			if(saved_ranges == null) return;
			vessel.vesselRanges = new VesselRanges(saved_ranges);
			saved_ranges = null;
		}
		#endregion

		void create_props()
		{
			AllPros.Clear();
			foreach(var fi in AllPropFields)
			{
				var constructor = fi.FieldType.GetConstructor(new [] {typeof(VesselWrapper)});
				if(constructor == null)
					throw new MissingMemberException(string.Format("No suitable constructor found for {0}", fi.FieldType.Name));
				var prop = constructor.Invoke(new [] {this}) as VesselProps;
				if(prop != null) AllPros.Add(prop);
				fi.SetValue(this, prop);
			}
		}

		public VesselWrapper(ModuleTCA tca)
		{
			TCA = tca; 
			CFG = tca.CFG;
			vessel = TCA.vessel;
			create_props();
			OnPlanet = _OnPlanet();
			InOrbit  = _InOrbit();
			UpdateState();
			UpdatePhysics();
			UpdateParts();
		}

		public void Init()
		{
			Controls.UpdateRefTransform();
			UpdateCommons();
			OnPlanetParams.Update();
			Geometry.Update();
			vessel.OnAutopilotUpdate += UpdateAutopilotInfo;
		}

		public void ConnectAutopilotOutput()
		{ vessel.OnAutopilotUpdate += ApplyAutopilotSteering; }

		public void Reset()
		{
			vessel.OnAutopilotUpdate -= UpdateAutopilotInfo;
			vessel.OnAutopilotUpdate -= ApplyAutopilotSteering;
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

		public bool AutopilotDisabled;
		public bool HasUserInput;
		public void UpdateAutopilotInfo(FlightCtrlState s)
		{
			if(!CFG.Enabled) return;
			Controls.GimbalLimit = 100;
			HasUserInput = 
				!Mathfx.Approx(s.pitch, s.pitchTrim, 0.01f) ||
				!Mathfx.Approx(s.roll, s.rollTrim, 0.01f) ||
				!Mathfx.Approx(s.yaw, s.yawTrim, 0.01f);
			AutopilotDisabled = HasUserInput;
		}

		public void ApplyAutopilotSteering(FlightCtrlState s)
		{
			if(!CFG.Enabled || Controls.AutopilotSteering.IsZero()) return;
			s.pitch = Utils.Clamp(Controls.AutopilotSteering.x, -1, 1);
			s.roll  = Utils.Clamp(Controls.AutopilotSteering.y, -1, 1);
			s.yaw   = Utils.Clamp(Controls.AutopilotSteering.z, -1, 1);
			Controls.AutopilotSteering = Vector3.zero;
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
						if(THR != null) THR.Throttle = 0f;
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
			if(CFG.Target != null) 
				CFG.Target.Update(this);
		}

		public void UpdateCommons()
		{
			Engines.Update();
			Torque.Update();
			Controls.Update();
		}

		public void ClearFrameState()
		{
			AllPros.ForEach(p => p.ClearFrameState());
		}

		public void UpdateOnPlanetStats()
		{
			if(!OnPlanet) return;
			VerticalSpeed.Update();
			HorizontalSpeed.Update();
			OnPlanetParams.Update();
		}

		public static bool AddModule<M>(PartModule m, List<M> db)  
			where M : PartModule
		{
			var module = m as M;
			if(module == null) return false;
			db.Add(module);
			return true;
		}

		public void UpdateParts()
		{
			EngineWrapper.ThrustPI.setMaster(CFG.Engines);
			Engines.Clear(); Torque.Wheels.Clear();
			OnPlanetParams.Parachutes.Clear();
			Physics.AngularDrag = 0;
			var drag_parts = 0;
			var parts_count = vessel.Parts.Count;
			for(int i = 0; i < parts_count; i++)
			{
				Part p = vessel.Parts[i];
				if(p.angularDragByFI) { Physics.AngularDrag += p.angularDrag; drag_parts += 1; }
				for(int j = 0, pModulesCount = p.Modules.Count; j < pModulesCount; j++)
				{
					var module = p.Modules[j];
					if(Engines.Add(module as ModuleEngines)) continue;
					if(Engines.Add(module as ModuleRCS)) continue;
					if(AddModule(module, Torque.Wheels)) continue;
					if(AddModule(module, OnPlanetParams.Parachutes)) continue;
				}
			}
			Physics.AngularDrag /= drag_parts;
			if(CFG.EnginesProfiles.Empty) CFG.EnginesProfiles.AddProfile(Engines.All);
			else if(CFG.Enabled && TCA.ProfileSyncAllowed) CFG.ActiveProfile.Update(Engines.All);
		}

		bool stage_is_empty(int stage)
		{
			var cur_parts = vessel.parts.Where(p => p.hasStagingIcon && p.inverseStage == stage).ToList();
//			LogF("{} stagable parts at stage {}", cur_parts.Count, stage);//debug
			return cur_parts.Count == 0 || cur_parts.All(p => p.State == PartStates.ACTIVE);
		}

		void activate_next_stage()
		{
			var next_stage = vessel.currentStage;
			while(next_stage >= 0 && stage_is_empty(next_stage)) next_stage--;
			if(next_stage < 0) return;
//			Log(vessel.parts.Aggregate("\n", (s, p) => s+Utils.Format("{}: {}, stage {}\n", p.Title(), p.State, p.inverseStage)));//debug
//			LogF("current stage {}, next stage {}, next engines {}", vessel.currentStage, next_stage, Engines.NearestEnginedStage);//debug
			if(IsActiveVessel)
			{
				StageManager.ActivateStage(next_stage);
				vessel.ActionGroups.ToggleGroup(KSPActionGroup.Stage);
				if(ResourceDisplay.Instance != null)
					ResourceDisplay.Instance.isDirty = true;
			}
			else
			{
				GameEvents.onStageActivate.Fire(next_stage);
				vessel.parts.ForEach(p => p.activate(next_stage, vessel));
				vessel.currentStage = next_stage;
				vessel.ActionGroups.ToggleGroup(KSPActionGroup.Stage);
			}
		}
		readonly ActionDamper next_cooldown = new ActionDamper(0.5);
		public void ActivateNextStage() { next_cooldown.Run(activate_next_stage); }

		public void GearOn(bool enable = true)
		{
			if(!CFG.AutoGear) return;
			if(vessel.ActionGroups[KSPActionGroup.Gear] != enable)
				vessel.ActionGroups.SetGroup(KSPActionGroup.Gear, enable);
		}

		public void BrakesOn(bool enable = true)
		{
			if(!CFG.AutoBrakes) return;
			if(vessel.ActionGroups[KSPActionGroup.Brakes] != enable)
				vessel.ActionGroups.SetGroup(KSPActionGroup.Brakes, enable);
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
		VTOLAssist             = 1 << 10,
		StabilizeFlight        = 1 << 11,
		//composite
		Nominal				   = Enabled | HaveEC | HaveActiveEngines,
		NoActiveEngines        = Enabled | HaveEC,
		NoEC                   = Enabled,
	}
}