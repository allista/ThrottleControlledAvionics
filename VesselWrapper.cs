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

        public FlightCtrlState PreUpdateControls;
        public FlightCtrlState PostUpdateControls;

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
			.GetFields(BindingFlags.Instance|BindingFlags.Public)
			.Where(fi => fi.FieldType.IsSubclassOf(typeof(VesselProps))).ToList();
		
		//state
		public CelestialBody Body { get { return vessel.mainBody; } }
		public Orbit orbit { get { return vessel.orbit; } }
		public bool OnPlanet { get; private set; }
		public bool InOrbit { get; private set; }
		public bool IsActiveVessel { get; private set; }
		public bool LandedOrSplashed { get { return vessel.LandedOrSplashed; } }
        public Vessel.Situations Situation { get { return vessel.situation; } }

        public bool AutopilotDisabled;
        public bool HasUserInput;

		public bool HasManeuverNode 
		{ 
			get 
			{ 
				return vessel.patchedConicSolver != null && 
					vessel.patchedConicSolver.maneuverNodes.Count > 0 && 
					vessel.patchedConicSolver.maneuverNodes[0] != null; 
			} 
		}
		public ManeuverNode FirstManeuverNode 
        { get { return vessel.patchedConicSolver.maneuverNodes[0]; } }
		

        #region Target
        public HashSet<TCAModule> TargetUsers = new HashSet<TCAModule>();

        public ITargetable Target 
        { 
            get 
            { 
                var t = vessel.targetObject;
                if(t == null || t is CelestialBody)
                {
                    t = NavWayPoint ?? CFG.Target;
                    if(t == null && CFG.Path.Count > 0)
                        t = CFG.Path.Peek();
                }
                return t;
            } 
            set 
            { 
                vessel.targetObject = value; 
            } 
        }

        public bool HasTarget 
        { 
            get 
            { 
                return 
                    vessel.targetObject != null && 
                    !(vessel.targetObject is CelestialBody) || 
                    NavWaypoint.fetch != null && NavWaypoint.fetch.IsActive ||
                    CFG.Target || 
                    CFG.Path.Count > 0;
            } 
        }

        public Vessel TargetVessel 
        { 
            get 
            { 
                var t = Target;
                return t == null? null : t.GetVessel();
            } 
        }

        public bool TargetIsNavPoint 
        { 
            get 
            { 
                return 
                    vessel.targetObject == null && 
                    NavWaypoint.fetch != null && 
                    NavWaypoint.fetch.IsActive; 
            } 
        }

        public bool TargetIsWayPoint { get { return Target is WayPoint; } }

        public void UpdateTarget(WayPoint wp)
        {
            if(wp != null && CFG.Target && wp != CFG.Target)
            {
                var t = wp.GetTarget();
                if(IsActiveVessel)
                    FlightGlobals.fetch.SetVesselTarget(t, true);
                else Target = t;
                CFG.Target = wp;
                CFG.Target.Update(this);
            }
        }

		public void SetTarget(TCAModule user, WayPoint wp = null)
		{
			if(wp == null)
			{
				TargetUsers.Remove(user);
				if(TargetUsers.Count == 0)
				{
					CFG.Target = null;
					if(vessel.targetObject is WayPoint)
						Target = null;
				}
			}
			else
			{
				if(user != null)
					TargetUsers.Add(user);
				var t = wp.GetTarget();
				if(IsActiveVessel && wp != CFG.Target)
                    FlightGlobals.fetch.SetVesselTarget(t, true);
				else Target = t;
				CFG.Target = wp;
                CFG.Target.Update(this);
			}
		}

		public WayPoint TargetAsWP
		{
			get
			{
				var t = Target;
				if(t == null) return null;
				return t as WayPoint ?? new WayPoint(t);
			}
		}

		public WayPoint NavWayPoint
		{
			get
			{
				var nvp = NavWaypoint.fetch;
				if(nvp == null || !nvp.IsActive) return null;
				var wp = new WayPoint(nvp.Latitude, nvp.Longitude, nvp.Altitude+nvp.Height);
				wp.Name = nvp.name;
				return wp;
			}
		}
        #endregion

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
			OnPlanet = vessel.OnPlanet();
			InOrbit = vessel.InOrbit();
            PreUpdateState(vessel.ctrlState);
			UpdatePhysics();
			UpdateParts();
		}

		public void Init()
		{
			UpdateCommons();
			OnPlanetParams.Update();
			Geometry.Update();
		}

		public void ConnectAutopilotOutput()
		{ vessel.OnAutopilotUpdate += ApplyAutopilotSteering; }

		public void Reset()
		{
			vessel.OnAutopilotUpdate -= ApplyAutopilotSteering;
			RestoreUnpackDistance();
		}

		public void ApplyAutopilotSteering(FlightCtrlState s)
		{
			if(!CFG.Enabled || Controls.AutopilotSteering.IsZero()) return;
			s.pitch = Utils.Clamp(Controls.AutopilotSteering.x, -1, 1);
			s.roll  = Utils.Clamp(Controls.AutopilotSteering.y, -1, 1);
			s.yaw   = Utils.Clamp(Controls.AutopilotSteering.z, -1, 1);
		}

        public void PreUpdateState(FlightCtrlState s)
		{
            //update control info
            PreUpdateControls = s;
            IsActiveVessel = vessel != null && vessel == FlightGlobals.ActiveVessel;
            HasUserInput = 
                !Mathfx.Approx(PreUpdateControls.pitch, PreUpdateControls.pitchTrim, 0.01f) ||
                !Mathfx.Approx(PreUpdateControls.roll, PreUpdateControls.rollTrim, 0.01f) ||
                !Mathfx.Approx(PreUpdateControls.yaw, PreUpdateControls.yawTrim, 0.01f);
            AutopilotDisabled = HasUserInput;
			//update onPlanet state
			var on_planet = vessel.OnPlanet();
			var in_orbit = vessel.InOrbit();
            if(on_planet != OnPlanet) 
			{
                if(CFG.Enabled)
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
					if(IsActiveVessel && TCAGui.Instance.ORB != null)
						TCAGui.Instance.ActiveTab = TCAGui.Instance.ORB;
				}
			}
			OnPlanet = on_planet;
			InOrbit = in_orbit;
		}

        public void PostUpdateState(FlightCtrlState s)
        {
            PostUpdateControls = s;
            Controls.Update(s);
        }

		public void UpdatePhysics()
		{
			Physics.Update();
			Altitude.Update();
			if(CFG.Target)
            {
				CFG.Target.Update(this);
                if(!CFG.Target.Valid)
                {
                    CFG.Target = null;
                    TargetUsers.Clear();
                }
            }
		}

		public void UpdateCommons()
		{
			Engines.Sort();
			Engines.Update();
			Torque.Update();
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

		public void OnModulesUpdated()
		{
			if(Controls.PauseWhenStopped)
			{
				if(!(CFG.AP1 || CFG.AP2 || HorizontalSpeed.Mooving))
				{
                    Controls.PauseWhenStopped = false;
					PauseMenu.Display();
				}
			}
		}

		public void FinalUpdate() {}

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
			Engines.Clear(); 
			Torque.Wheels.Clear();
			Physics.Clear();
			OnPlanetParams.Clear();
			var drag_parts = 0;
			var parts_count = vessel.Parts.Count;
			var max_active_stage = -1;
			for(int i = 0; i < parts_count; i++)
			{
				Part p = vessel.Parts[i];
				if(p.State == PartStates.ACTIVE && p.inverseStage > max_active_stage)
					max_active_stage = p.inverseStage;
				Physics.UpdateMaxTemp(p);
				if(p.angularDragByFI) { Physics.AngularDrag += p.angularDrag; drag_parts += 1; }
				for(int j = 0, pModulesCount = p.Modules.Count; j < pModulesCount; j++)
				{
					var module = p.Modules[j];
					if(Engines.AddEngine(module)) continue;
					if(Engines.AddRCS(module)) continue;
					if(OnPlanetParams.AddLaunchClamp(module)) continue;
					if(OnPlanetParams.AddParachute(module)) continue;
					if(OnPlanetParams.AddGear(module)) continue;
					if(AddModule(module, Torque.Wheels)) continue;
				}
			}
			Physics.AngularDrag /= drag_parts;
			Engines.Clusters.Update();
			if(max_active_stage >= 0 && 
			   (vessel.currentStage < 0 || vessel.currentStage > max_active_stage))
				vessel.currentStage = StageManager.RecalculateVesselStaging(vessel);
			if(CFG.EnginesProfiles.Empty) CFG.EnginesProfiles.AddProfile(Engines.All);
			else if(CFG.Enabled && TCA.ProfileSyncAllowed) CFG.ActiveProfile.Update(Engines.All);
		}

		bool stage_is_empty(int stage)
		{ return !vessel.parts.Any(p => p.hasStagingIcon && p.inverseStage == stage); }

		public void ActivateNextStageImmidiate()
		{
			var next_stage = vessel.currentStage;
			while(next_stage >= 0 && stage_is_empty(next_stage)) next_stage--;
			if(next_stage == vessel.currentStage) next_stage--;
			if(next_stage < 0) return;
//			Log(vessel.parts.Aggregate("\n", (s, p) => s+Utils.Format("{}: {}, stage {}\n", p.Title(), p.State, p.inverseStage)));//debug
//			Log("current stage {}, next stage {}, next engines {}", vessel.currentStage, next_stage, Engines.NearestEnginedStage);//debug
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
		public void ActivateNextStage() { next_cooldown.Run(ActivateNextStageImmidiate); }

        public void XToggleWithEngines<T>(Multiplexer<T> mp, T cmd) where T : struct
        {
            if(mp[cmd]) mp.XOff();
            else Engines.ActivateEnginesAndRun(() => mp.XOn(cmd));
        }

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