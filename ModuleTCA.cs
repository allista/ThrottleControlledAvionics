//   ModuleTCA.cs
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
using System.Reflection;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace ThrottleControlledAvionics
{
	public class ModuleTCA : PartModule, ITCAModule
	{
		#if DEBUG
		internal static Profiler prof = new Profiler();
		#endif

		public static TCAGlobals GLB { get { return TCAScenario.Globals; } }
		public VesselWrapper VSL { get; private set; }
		public VesselConfig CFG { get; set; }
		public TCAState State { get { return VSL.State; } set { VSL.State = value; } }
		public void SetState(TCAState state) { VSL.State |= state; }
		public bool IsStateSet(TCAState state) { return Available && VSL.IsStateSet(state); }

		#region Modules
		EngineOptimizer eng;
		VerticalSpeedControl vsc;
		HorizontalSpeedControl hsc;
		Anchor anc;
		AltitudeControl alt;
		RCSOptimizer rcs;
		CruiseControl cc;
		PointNavigator pn;
		Radar rad;
		AutoLander lnd;
		VTOLAssist tla;
		FlightStabilizer stb;
		CollisionPreventionSystem cps;
		List<TCAModule> modules;
		FieldInfo[] mod_fields;
		#endregion

		#region Public Info
		public float TorqueError { get { return eng == null? 0f : eng.TorqueError; } }
		public bool  Available { get { return enabled && VSL != null; } }
		public bool  Controllable { get { return Available && vessel.IsControllable; } }
		public static bool HasTCA { get { return !GLB.IntegrateIntoCareer || Utils.PartIsPurchased(TCAGlobals.TCA_PART); } }
		#endregion

		#region Initialization
		public void OnReloadGlobals() { VSL.Init(); invoke_in_modules("Init"); }

		public override string GetInfo()
		{ return HasTCA? "Software Installed" : "Not Available"; }

		public override void OnAwake()
		{
			base.OnAwake();
			GameEvents.onVesselWasModified.Add(onVesselModify);
			GameEvents.onStageActivate.Add(onStageActive);
		}

		internal void OnDestroy() 
		{ 
			GameEvents.onVesselWasModified.Remove(onVesselModify);
			GameEvents.onStageActivate.Remove(onStageActive);
			reset();
		}

		public override void OnSave(ConfigNode node)
		{
			if((enabled || HighLogic.LoadedSceneIsEditor) && CFG != null)
				CFG.Save(node.AddNode(VesselConfig.NODE_NAME));
			base.OnSave(node);
		}

		public override void OnLoad(ConfigNode node)
		{
			base.OnLoad(node);
			var SavedCFG = node.GetNode(VesselConfig.NODE_NAME);
			if(SavedCFG != null)
			{
				CFG = ConfigNodeObject.FromConfig<VesselConfig>(SavedCFG);
				if(vessel != null) CFG.VesselID = vessel.id;
			}
		}

		public override void OnStart(StartState state)
		{
			base.OnStart(state);
			if(state == StartState.Editor || state == StartState.None)
			{ enabled = isEnabled = false; return; }
			check_priority();
			check_career_part();
			init();
		}

		#if DEBUG
		public 
		#endif
		void onVesselModify(Vessel vsl)
		{ 
			if(vsl != vessel) return;
			reset();
			check_priority();
			init();
		}

		void onStageActive(int stage)
		{ 
			if(VSL == null) return;
			if(!CFG.EnginesProfiles.ActivateOnStage(stage, VSL.Engines))
				StartCoroutine(onStageUpdate());
		}

		IEnumerator<YieldInstruction> onStageUpdate()
		{
			VSL.CanUpdateEngines = false;
			yield return new WaitForSeconds(0.5f);
			CFG.ActiveProfile.Update(VSL.Engines, true);
			VSL.CanUpdateEngines = true;
		}

		void check_priority()
		{
			if(vessel == null || vessel.parts == null) goto disable;
			var TCA_part = vessel.parts.FirstOrDefault(p => p.HasModule<ModuleTCA>());
			if(TCA_part != part) goto disable;
			var TCA = TCA_part.Modules.OfType<ModuleTCA>().FirstOrDefault();
			if(TCA != this) goto disable;
			enabled = isEnabled = true;
			return;
			disable: enabled = isEnabled = false;
		}

		void check_career_part()
		{ if(enabled) enabled = isEnabled = HasTCA; }

		void create_modules()
		{
			var mt = typeof(TCAModule);
			var vt = typeof(VesselWrapper);
			if(mod_fields == null)
				mod_fields = GetType()
					.GetFields(BindingFlags.DeclaredOnly|BindingFlags.NonPublic|BindingFlags.Instance)
					.Where(fi => fi.FieldType.IsSubclassOf(mt)).ToArray();
			modules = new List<TCAModule>(mod_fields.Length);
			foreach(var fi in mod_fields)
			{
				if(!fi.FieldType.IsSubclassOf(mt)) continue;
				var method = fi.FieldType.GetConstructor(new [] {vt});
				if(method != null)
				{
					var m = (TCAModule)method.Invoke(fi.GetValue(this), new [] {VSL});
					if(m != null)
					{
						fi.SetValue(this, m);
						modules.Add(m);
					}
					else this.Log("Failed to create {0}. Constructor returned null.", fi.FieldType);
				}
				else this.Log("Failed to create {0}. No constructor found.", fi.FieldType);
			}
		}

		void delete_modules()
		{
			if(mod_fields == null) return;
			mod_fields.ForEach(mf => mf.SetValue(this, null));
		}

		void invoke_in_modules(string m)
		{
			if(modules == null) return;
			var mt = typeof(TCAModule);
			var method = mt.GetMethod(m);
			if(method == null) this.Log("No {0} method in {1} found.", m, mt.Name);
			for(int i = 0; i<modules.Count; i++) method.Invoke(modules[i], null);
		}

		public static List<ModuleTCA> AllTCA(IShipconstruct ship)
		{
			//get all ModuleTCA instances in the vessel
			var TCA_Modules = new List<ModuleTCA>();
			(from p in ship.Parts select p.Modules.OfType<ModuleTCA>())
				.ForEach(TCA_Modules.AddRange);
			return TCA_Modules;
		}

		public static ModuleTCA EnabledTCA(IShipconstruct ship)
		{
			ModuleTCA tca = null;
			for(int i = 0, shipPartsCount = ship.Parts.Count; i < shipPartsCount; i++) 
			{
				tca = ship.Parts[i].Modules
					.OfType<ModuleTCA>()
					.FirstOrDefault(m => m.enabled);
				if(tca != null) break;
			}
			return tca;
		}

		void UpdateCFG()
		{
			//get all ModuleTCA instances in the vessel
			var TCA_Modules = AllTCA(vessel);
			//try to get saved CFG from other modules, if needed
			if(CFG == null)
				foreach(var tca in TCA_Modules)
				{
					if(tca.CFG == null) continue;
					CFG = tca.CFG;
					break;
				}
			//if it is found in one of the modules, use it
			//else, get it from common database or create a new one
			if(vessel != null)
			{
				if(CFG != null)
				{
					if(CFG.VesselID == Guid.Empty)
						CFG.VesselID = vessel.id;
					else if(CFG.VesselID != vessel.id)
						CFG = VesselConfig.FromVesselConfig(vessel, CFG);
					TCAScenario.Configs[CFG.VesselID] = CFG;
				}
				else CFG = TCAScenario.GetConfig(vessel);
			}
			else CFG = new VesselConfig();
			//finally, update references in other modules
			TCA_Modules.ForEach(m => m.CFG = CFG);
		}

		void init()
		{
			if(!enabled) return;
			UpdateCFG();
			VSL = new VesselWrapper(vessel, CFG);
			VSL.Init();
			VSL.UpdateState();
			VSL.UpdateEngines();
			enabled = isEnabled = VSL.Engines.Count > 0 || VSL.RCS.Count > 0;
			if(!enabled) { VSL = null; return; }
			VSL.UpdateCommons();
			VSL.UpdateOnPlanetStats();
			VSL.UpdateBounds();
			create_modules();
			modules.ForEach(m => m.Init());
			vessel.OnAutopilotUpdate += block_throttle;
			if(CFG.AP[Autopilot.Land] && VSL.LandedOrSplashed) CFG.AP.Off();
			if(CFG.Nav.Any(Navigation.GoToTarget, Navigation.FollowTarget)) 
				pn.GoToTarget(VSL.vessel.targetObject != null);
			else if(CFG.Nav[Navigation.FollowPath]) 
				pn.FollowPath(CFG.Waypoints.Count > 0);
			ThrottleControlledAvionics.AttachTCA(this);
			VSL.SetUnpackDistance(GLB.UnpackDistance);
			part.force_activate(); //need to activate the part for OnFixedUpdate to work
		}

		void reset()
		{
			if(VSL != null)
			{
				VSL.OnAutopilotUpdate -= block_throttle;
				modules.ForEach(m => m.Reset());
				CFG.ClearCallbacks();
			}
			delete_modules();
			VSL = null; 
		}
		#endregion

		#region Controls
		public void ToggleTCA()
		{
			CFG.Enabled = !CFG.Enabled;
			if(!CFG.Enabled) //reset engine limiters
			{
				VSL.Engines.ForEach(e => e.forceThrustPercentage(100));
				State = TCAState.Disabled;
				VSL.UnblockSAS(false);
			}
		}

		public void BlockThrottle(bool state)
		{
			if(state == CFG.BlockThrottle) return;
			CFG.BlockThrottle = state;
			if(CFG.BlockThrottle && CFG.VerticalCutoff >= GLB.VSC.MaxSpeed)
				CFG.VerticalCutoff = 0;
		}

		public void AltitudeAboveTerrain(bool state) { alt.SetAltitudeAboveTerrain(state); }
		#endregion
		void block_throttle(FlightCtrlState s)
		{ 
			if(CFG.Enabled && CFG.BlockThrottle && VSL.OnPlanet) 
				s.mainThrottle = VSL.LandedOrSplashed && CFG.VerticalCutoff <= 0? 0f : 1f;
		}

		public override void OnUpdate()
		{
			//update vessel config if needed
			if(CFG != null && vessel != null && CFG.VesselID == Guid.Empty) UpdateCFG();
			//update heavy to compute parameters
			if(IsStateSet(TCAState.HaveActiveEngines)) VSL.UpdateMoI();
			if(rad.IsActive || lnd.IsActive) VSL.UpdateBounds();
		}

		public override void OnFixedUpdate() 
		{
			//initialize systems
			VSL.UpdateState();
			if(!CFG.Enabled) return;
			State = TCAState.Enabled;
			if(!VSL.ElectricChargeAvailible) return;
			SetState(TCAState.HaveEC);
			if(VSL.CheckEngines()) 
				SetState(TCAState.HaveActiveEngines);
			//update state
			VSL.UpdateCommons();
			if(VSL.NumActive > 0)
			{
				for(int i = 0; i < modules.Count; i++) modules[i].UpdateState();
				VSL.UpdateOnPlanetStats();
				//these follow specific order
				rad.Update();//sets AltitudeAhead
				lnd.Update();//sets VerticalCutoff, sets DesiredAltitude
				alt.Update();//uses AltitudeAhead, uses DesiredAltitude, sets VerticalCutoff
				cps.Update();//updates VerticalCutoff
				vsc.Update();//uses VerticalCutoff
				anc.Update();
				tla.Update();
				stb.Update();
				pn.Update();
			}
			//handle engines
			VSL.TuneEngines();
			if(VSL.NumActive > 0)
			{
				VSL.SortEngines();
				//:preset manual limits for translation if needed
				if(VSL.ManualTranslationSwitch.On)
					eng.PresetLimitsForTranslation(VSL.ManualEngines, VSL.ManualTranslation);
				//:balance-only engines
				if(VSL.BalancedEngines.Count > 0)
				{
					VSL.UpdateTorque(VSL.ManualEngines);
					eng.OptimizeLimitsForTorque(VSL.BalancedEngines, Vector3.zero);
				}
				VSL.UpdateTorque(VSL.ManualEngines, VSL.BalancedEngines);
				//:optimize limits for steering
				eng.PresetLimitsForTranslation(VSL.ManeuverEngines, VSL.Translation);
				eng.Steer();
			}
			rcs.Steer();
			VSL.SetThrustLimiters();
		}
	}
}



