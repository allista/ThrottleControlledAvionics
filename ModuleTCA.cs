//  Author:
//       Allis Tauri <allista@gmail.com>
//
//  Copyright (c) 2015 Allis Tauri
//
// This work is licensed under the Creative Commons Attribution-ShareAlike 4.0 International License. 
// To view a copy of this license, visit http://creativecommons.org/licenses/by-sa/4.0/ 
// or send a letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.

using System;
using System.Reflection;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace ThrottleControlledAvionics
{
	public class ModuleTCA : PartModule, ITCAModule, IModuleInfo
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
		//core modules
		public EngineOptimizer ENG;
		public RCSOptimizer RCS;
		public static List<FieldInfo> CoreModuleFields = typeof(ModuleTCA)
			.GetFields(BindingFlags.DeclaredOnly|BindingFlags.Public|BindingFlags.Instance)
			.Where(fi => fi.FieldType.IsSubclassOf(typeof(TCAModule))).ToList();
		//optional modules
		public Dictionary<Type, TCAModule> ModulesDB = new Dictionary<Type, TCAModule>();
		public List<TCAModule> AllModules = new List<TCAModule>();
		public List<TCAModule> ModulePipeline = new List<TCAModule>();
		public List<TCAModule> AutopilotPipeline = new List<TCAModule>();

		public M GetModule<M>() where M : TCAModule 
		{ 
			TCAModule module = null;
			return ModulesDB.TryGetValue(typeof(M), out module)? module as M : null;
		}

		public TCAModule GetModule(Type T)
		{
			TCAModule module = null;
			return ModulesDB.TryGetValue(T, out module)? module : null;
		}

		public object CreateComponent(Type T)
		{
			var constructor = T.GetConstructor(new [] {typeof(ModuleTCA)});
			if(constructor == null)
				throw new MissingMemberException(string.Format("No suitable constructor found for {0}", T.Name));
			return constructor.Invoke(new [] {this});
		}

		public void CreateComponent(object obj, FieldInfo fi)
		{ fi.SetValue(obj, CreateComponent(fi.FieldType)); }
		#endregion

		#region Public Info
		public bool Valid { get { return vessel != null && part != null && Available; } }
		public bool Available { get { return enabled && VSL != null; } }
		public bool Controllable { get { return Available && vessel.IsControllable; } }
		#endregion

		#region Initialization
		public void OnReloadGlobals() 
		{ AllModules.ForEach(m => m.Init()); }

		public override string GetInfo() 
		{ return "Software can be installed"; }

		internal const string TCA_NAME = "TCA";
		public string GetModuleTitle() { return TCA_NAME; }

		public string GetPrimaryField()
		{ return "<b>TCA:</b> "+TCAScenario.ModuleStatusString(); }

		public Callback<Rect> GetDrawModulePanelCallback() { return null; }

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

		void onVesselModify(Vessel vsl)
		{ 
			
			check_priority();
			if(!enabled) reset();
			else if(VSL == null || VSL.vessel == null ||
			        vsl.id != VSL.vessel.id)
			{ reset(); init(); }
			else 
			{
				VSL.ForceUpdateParts = true;
				StartCoroutine(onVesselModifiedUpdate());
			}
		}

		void onStageActive(int stage)
		{ 
			if(VSL == null || !CFG.Enabled) return;
			if(!CFG.EnginesProfiles.ActivateOnStage(stage, VSL.Engines.All))
				StartCoroutine(onStageUpdate());
		}

		IEnumerator<YieldInstruction> onStageUpdate()
		{
			VSL.Engines.ProfileSyncAllowed = false;
			yield return new WaitForSeconds(0.5f);
			VSL.UpdateParts();
			CFG.ActiveProfile.Update(VSL.Engines.All, true);
			VSL.Engines.ProfileSyncAllowed = true;
		}

		IEnumerator<YieldInstruction> onVesselModifiedUpdate()
		{
			if(!CFG.Enabled) yield break;
			yield return new WaitForSeconds(0.5f);
			if(VSL != null) VSL.SetUnpackDistance(GLB.UnpackDistance);
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
		{ if(enabled) enabled = isEnabled = TCAScenario.HasTCA; }

		public void DeleteModules()
		{
			ModulesDB.Clear();
			AllModules.Clear();
			ModulePipeline.Clear();
			AutopilotPipeline.Clear();
			foreach(var core_field in CoreModuleFields)
				core_field.SetValue(this, null);
		}

		public static List<ModuleTCA> AllTCA(IShipconstruct ship)
		{
			//get all ModuleTCA instances in the vessel
			var TCA_Modules = new List<ModuleTCA>();
			(from p in ship.Parts select p.Modules.GetModules<ModuleTCA>())
				.ForEach(TCA_Modules.AddRange);
			return TCA_Modules;
		}

		public static ModuleTCA AvailableTCA(IShipconstruct ship)
		{
			ModuleTCA tca = null;
			for(int i = 0, shipPartsCount = ship.Parts.Count; i < shipPartsCount; i++) 
			{
				tca = ship.Parts[i].Modules
					.GetModules<ModuleTCA>()
					.FirstOrDefault(m => m.Available);
				if(tca != null) break;
			}
			return tca;
		}

		public static ModuleTCA EnabledTCA(IShipconstruct ship)
		{ 
			var tca = AvailableTCA(ship);
			return tca != null && tca.CFG != null && tca.CFG.Enabled? tca : null;
		}

		void updateCFG()
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
			if(CFG != null)
			{
				if(CFG.VesselID == Guid.Empty)
					CFG.VesselID = vessel.id;
				else if(CFG.VesselID != vessel.id)
					CFG = VesselConfig.FromVesselConfig(vessel, CFG);
				TCAScenario.Configs[CFG.VesselID] = CFG;
			}
			else CFG = TCAScenario.GetConfig(vessel);
			//finally, update references in other modules
			TCA_Modules.ForEach(m => m.CFG = CFG);
		}

		void init()
		{
			if(!enabled) return;
			updateCFG();
			VSL = new VesselWrapper(this);
			enabled = isEnabled = VSL.Engines.All.Count > 0 || VSL.Engines.RCS.Count > 0;
			if(!enabled) { VSL = null; return; }
			VSL.Init();
			TCAModulesDatabase.InitModules(this);
			ThrottleControlledAvionics.AttachTCA(this);
			part.force_activate(); //need to activate the part for OnFixedUpdate to work
			StartCoroutine(onVesselModifiedUpdate());
			CFG.Resume();
		}

		void reset()
		{
			if(VSL != null)
			{
				VSL.Reset();
				AllModules.ForEach(m => m.Reset());
				CFG.ClearCallbacks();
			}
			DeleteModules();
			VSL = null;
		}
		#endregion

		#region Controls
		public void ToggleTCA()
		{
			CFG.Enabled = !CFG.Enabled;
			if(CFG.Enabled) //test
			{
				CFG.ActiveProfile.Update(VSL.Engines.All, true);
				VSL.SetUnpackDistance(GLB.UnpackDistance);
				AllModules.ForEach(m => m.OnEnable(true));
			}
			else
			{
				VSL.Engines.All.ForEach(e => e.forceThrustPercentage(100));
				VSL.RestoreUnpackDistance();
				State = TCAState.Disabled;
				AllModules.ForEach(m => m.OnEnable(false));
			}
		}
		#endregion

		public void ClearFrameState()
		{ AllModules.ForEach(m => m.ClearFrameState()); }

		public override void OnUpdate()
		{
			//update vessel config if needed
			if(CFG != null && vessel != null && CFG.VesselID == Guid.Empty) updateCFG();
			if(CFG.Enabled)
			{
				//update heavy to compute parameters
				VSL.Physics.UpdateMoI();
				VSL.Geometry.Update();
			}
		}

		public override void OnFixedUpdate() 
		{
			//initialize systems
			VSL.UpdateState();
			if(!CFG.Enabled) return;
			State = TCAState.Enabled;
			if(!VSL.Info.ElectricChargeAvailible) return;
			//update vessel info
			SetState(TCAState.HaveEC);
			VSL.UpdatePhysics();
			if(VSL.Engines.Check()) 
				SetState(TCAState.HaveActiveEngines);
			VSL.UpdateCommons();
			VSL.ClearFrameState();
			VSL.UpdateOnPlanetStats();
			//update modules
			ModulePipeline.ForEach(m => m.OnFixedUpdate());
			//handle engines
			VSL.Engines.Tune();
			if(VSL.Engines.NumActive > 0)
			{
				VSL.Engines.Sort();
				//:preset manual limits for translation if needed
				if(VSL.Controls.ManualTranslationSwitch.On)
				{
					ENG.PresetLimitsForTranslation(VSL.Engines.Manual, VSL.Controls.ManualTranslation);
					if(CFG.VSCIsActive) ENG.LimitInDirection(VSL.Engines.Manual, VSL.Physics.UpL);
				}
				//:balance-only engines
				if(VSL.Engines.Balanced.Count > 0)
				{
					VSL.Torque.UpdateTorque(VSL.Engines.Manual);
					ENG.OptimizeLimitsForTorque(VSL.Engines.Balanced, Vector3.zero);
				}
				VSL.Torque.UpdateTorque(VSL.Engines.Manual, VSL.Engines.Balanced);
				//:optimize limits for steering
				ENG.PresetLimitsForTranslation(VSL.Engines.Maneuver, VSL.Controls.Translation);
				ENG.Steer();
			}
			RCS.Steer();
			VSL.Engines.SetControls();
		}
	}
}



