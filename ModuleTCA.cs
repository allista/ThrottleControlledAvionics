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
using AT_Utils;

namespace ThrottleControlledAvionics
{
	public class ModuleTCA : PartModule, ITCAComponent, IModuleInfo
	{
		#if DEBUG
		internal static Profiler prof = new Profiler();
		#endif

		internal static Globals GLB { get { return Globals.Instance; } }
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
		public bool ProfileSyncAllowed { get; private set; } = true;

		[KSPField(guiActive = true, guiActiveEditor = true, guiName = "TCA Active")]
		public bool TCA_Active;

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
		{ 
			AllModules.ForEach(m => m.Reset()); 
			VSL.Reset();
			VSL.Init();
			AllModules.ForEach(m => m.Init()); 
			VSL.ConnectAutopilotOutput();
		}

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
			{
				AllModules.ForEach(m => m.SaveToConfig());
				CFG.Save(node.AddNode(VesselConfig.NODE_NAME));
			}
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
			Actions["onActionUpdate"].active = false;
			if(state == StartState.None || !TCAScenario.HasTCA) 
			{ enable_module(false); return; }
			check_priority();
			if(state == StartState.Editor) return;
			enable_module(TCA_Active);
			StartCoroutine(delayed_init());
		}

		void onVesselModify(Vessel vsl)
		{ 
			if(vsl != vessel) return;
			AllModules.ForEach(m => m.SaveToConfig());
			check_priority();
			enable_module(TCA_Active);
			if(!enabled) reset();
			else if(VSL == null || VSL.vessel == null || vsl.id != VSL.vessel.id)
			{ reset(); init(); }
			else 
			{
				VSL.Engines.ForceUpdateEngines = true;
				StartCoroutine(updateUnpackDistance());
			}
		}

		void onStageActive(int stage)
		{ 
			if(VSL == null || !CFG.Enabled || !VSL.IsActiveVessel) return;
			if(!CFG.EnginesProfiles.ActivateOnStage(stage, VSL.Engines.All))
				StartCoroutine(activeProfileUpdate());
		}

		IEnumerator<YieldInstruction> delayed_init()
		{
			if(!vessel.loaded) yield return null;
			yield return new WaitForSeconds(0.5f);
			init();
		}

		IEnumerator<YieldInstruction> activeProfileUpdate()
		{
			ProfileSyncAllowed = false;
			yield return new WaitForSeconds(0.5f);
			VSL.UpdateParts();
			CFG.ActiveProfile.Update(VSL.Engines.All, true);
			ProfileSyncAllowed = true;
		}

		IEnumerator<YieldInstruction> updateUnpackDistance()
		{
			if(!CFG.Enabled) yield break;
			yield return new WaitForSeconds(0.5f);
			if(VSL != null) VSL.SetUnpackDistance(GLB.UnpackDistance);
		}

		[KSPAction("Update TCA Profile")]
		void onActionUpdate(KSPActionParam param) { StartCoroutine(activeProfileUpdate()); }

		void check_priority()
		{
			if(HighLogic.LoadedSceneIsEditor) 
				check_priority(EditorLogic.fetch.ship);
			else check_priority(vessel);
		}

		void check_priority(IShipconstruct ship)
		{
			TCA_Active = 
				ship != null && ship.Parts != null &&
				part == ship.Parts.FirstOrDefault(p => p.HasModule<ModuleTCA>()) &&
				this == part.Modules.OfType<ModuleTCA>().FirstOrDefault();
			Actions["ToggleTCA"].active = TCA_Active;
		}

		void enable_module(bool enable = true)
		{
			TCA_Active &= enable;
			enabled = isEnabled = enable;
			Fields["TCA_Active"].guiActive = enable;
			Fields["TCA_Active"].guiActiveEditor = enable;
		}

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
			enable_module(VSL.Engines.All.Count > 0 || VSL.Engines.RCS.Count > 0);
			if(!enabled) { VSL = null; return; }
			VSL.Init();
			TCAModulesDatabase.InitModules(this);
			VSL.ConnectAutopilotOutput();//should follow module initialization
			TCAGui.AttachTCA(this);
			StartCoroutine(updateUnpackDistance());
			Actions["onActionUpdate"].active = true;
			Actions["ToggleTCA"].actionGroup = CFG.ActionGroup;
			CFG.Resume(this);
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
		[KSPAction("Toggle TCA")]
		public void ToggleTCA(KSPActionParam param = null)
		{
			CFG.Enabled = !CFG.Enabled;
			if(CFG.Enabled)
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

		void ClearFrameState()
		{ 
			VSL.ClearFrameState();
			AllModules.ForEach(m => m.ClearFrameState());
		}

		void Update() //works both in Editor and in flight
		{
			if(!TCA_Active) return;
			if(CFG != null) CFG.ActionGroup = Actions["ToggleTCA"].actionGroup;
		}

		public override void OnUpdate()
		{
			if(VSL == null) return;
			//update vessel config if needed
			if(CFG != null && vessel != null && CFG.VesselID == Guid.Empty) updateCFG();
			if(CFG.Enabled)
			{
				//update heavy to compute parameters
				VSL.Physics.UpdateCoM();
				VSL.Physics.UpdateMoI();
				VSL.Geometry.Update();
//				Utils.Log("{}: MoI {}, Srf {}, MaxPossibleTorque {}, M {}, MaxThrust {}\n" +
//				           "atmDensity {}, Torq.Ang.DragRes {}, NoEng.Torq.Ang.DragRes {}, Max.Pos.Ang.DragRes {}\n",
//				           this.Title(), VSL.Physics.MoI, VSL.Geometry.BoundsSideAreas, VSL.Torque.MaxTorquePossible,
//				           VSL.Physics.M, VSL.Engines.MaxThrustM, VSL.Body.atmDensityASL,
//				           VSL.Torque.MaxAngularDragResistance, VSL.Torque.NoEnginesAngularDragResistance, VSL.Torque.MaxPossibleAngularDragResistance
//				          );//debug
			}
		}

		public void FixedUpdate() 
		{
			if(VSL == null) return;
			//initialize systems
			VSL.UpdateState();
			State = TCAState.Disabled;
			if(!CFG.Enabled) return;
			State = TCAState.Enabled;
			if(!VSL.Info.ElectricChargeAvailible) return;
			SetState(TCAState.HaveEC);
			ClearFrameState();
			//update VSL
			VSL.UpdatePhysics();
			if(VSL.Engines.Check()) SetState(TCAState.HaveActiveEngines);
			Actions["onActionUpdate"].actionGroup = VSL.Engines.ActionGroups;
			VSL.UpdateCommons();
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
					VSL.Torque.UpdateTorque(VSL.Engines.Manual, VSL.Engines.UnBalanced);
					ENG.OptimizeLimitsForTorque(VSL.Engines.Balanced, Vector3.zero);
				}
				VSL.Torque.UpdateTorque(VSL.Engines.Manual, VSL.Engines.UnBalanced, VSL.Engines.Balanced);
				//:optimize limits for steering
				ENG.PresetLimitsForTranslation(VSL.Engines.Maneuver, VSL.Controls.Translation);
				ENG.Steer();
			}
			RCS.Steer();
			VSL.Engines.SetControls();
		}
	}
}



