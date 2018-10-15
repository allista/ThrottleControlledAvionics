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
using CommNet;

namespace ThrottleControlledAvionics
{
    public class ModuleTCA : PartModule, ITCAComponent, IModuleInfo, ICommNetControlSource
    {
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
            .GetFields(BindingFlags.Public|BindingFlags.Instance)
            .Where(fi => fi.FieldType.IsSubclassOf(typeof(TCAModule))).ToList();
        //optional modules
        public Dictionary<Type, TCAModule> ModulesDB = new Dictionary<Type, TCAModule>();
        public List<TCAModule> AllModules = new List<TCAModule>();
        public List<TCAModule> ModulePipeline = new List<TCAModule>();
        public List<TCAModule> AutopilotPipeline = new List<TCAModule>();
        public bool ProfileSyncAllowed { get; private set; } = true;

        [KSPField(guiActive = true, guiActiveEditor = true, guiName = "TCA Active")]
        public bool TCA_Active;

        [KSPField(guiActive = true, guiActiveEditor = true, guiName = "TCA Group")]
        public string GID_Display = "";

        [KSPField(isPersistant = true)] public string GID = "";
        [KSPField(isPersistant = true)] public bool GroupMaster;

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

        public List<T> CreateComponents<T>(IList<FieldInfo> fields, object obj = null)
                where T : TCAComponent
        {
            var components = new List<T>();
            foreach(var fi in fields)
            {
                var component = CreateComponent(fi.FieldType) as T;
                if(component != null) components.Add(component);
                if(obj != null) fi.SetValue(obj, component);
            }
            return components;
        }

        public static void SetTCAField(object obj, ModuleTCA tca)
        {
            var tca_fi = obj.GetType().GetField("TCA");
            if(tca_fi != null && tca_fi.FieldType == typeof(ModuleTCA))
                tca_fi.SetValue(obj, tca);
        }

        public void SetTCAField(object obj)
        { SetTCAField(obj, this); }

        public void InitModuleFields(object obj)
        {
            var ModuleFields = TCAModulesDatabase.GetAllModuleFields(obj.GetType());
            ModuleFields.ForEach(fi => fi.SetValue(obj, GetModule(fi.FieldType)));
        }

        public static void ResetModuleFields(object obj)
        {
            var ModuleFields = TCAModulesDatabase.GetAllModuleFields(obj.GetType());
            ModuleFields.ForEach(fi => fi.SetValue(obj, null));
        }

        public void SquadAction(Action<ModuleTCA> action)
        {
            var SQD = GetModule<SquadControl>();
            if(SQD == null) action(this);
            else SQD.Apply(action);
        }

        public void SquadConfigAction(Action<VesselConfig> action)
        {
            var SQD = GetModule<SquadControl>();
            if(SQD == null) action(CFG);
            else SQD.ApplyCFG(action);
        }
        #endregion

        #region Public Info
        public bool Valid { get { return vessel != null && part != null && Available; } }
        public bool Available { get { return TCA_Active && VSL != null; } }
        public bool IsControllable 
        { 
            get 
            { 
                return Available && (vessel.CurrentControlLevel == Vessel.ControlLevel.FULL || 
                                     vessel.CurrentControlLevel == Vessel.ControlLevel.PARTIAL_MANNED); 
            } 
        }
        #endregion

        #region Initialization
        public void OnReloadGlobals() 
        { 
            AllModules.ForEach(m => m.Cleanup()); 
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
            GameEvents.onEditorShipModified.Add(onEditorShipModified);
            GameEvents.onVesselWasModified.Add(onVesselModify);
            GameEvents.onStageActivate.Add(onStageActive);
            GameEvents.onVesselGoOffRails.Add(onVesselGoOffRails);
        }

        internal void OnDestroy() 
        { 
            if(vessel != null && vessel.connection != null)
                vessel.connection.UnregisterCommandSource(this);
            GameEvents.CommNet.OnNetworkInitialized.Remove(OnNetworkInitialised);
            GameEvents.onEditorShipModified.Remove(onEditorShipModified);
            GameEvents.onVesselWasModified.Remove(onVesselModify);
            GameEvents.onStageActivate.Remove(onStageActive);
            GameEvents.onVesselGoOffRails.Remove(onVesselGoOffRails);
            reset();
        }

        public override void OnSave(ConfigNode node)
        {
            if(GroupMaster && CFG != null)
            {
                AllModules.ForEach(m => m.SaveToConfig());
                CFG.SaveInto(node);
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
                GroupMaster = true;
                //this.Log("GroupMaster: {}", GroupMaster);//debug
            }
            if(!string.IsNullOrEmpty(GID))
                SetGID(GID);
            //deprecated config conversion
            enabled = isEnabled = true;
        }

        public override void OnStart(StartState state)
        {
            base.OnStart(state);
            Actions["onActionUpdate"].active = false;
            if(state == StartState.None || !TCAScenario.HasTCA) 
            { 
                EnableTCA(false); 
                return; 
            }
            check_priority();
            EnableTCA(TCA_Active);
            if(state != StartState.Editor) 
                StartCoroutine(delayed_init());
        }

        void onVesselGoOffRails(Vessel vsl)
        {
            if(vsl != vessel) return;
            if(vessel.situation == Vessel.Situations.PRELAUNCH)
            {
                if(VSL != null && CFG != null && 
                   CFG.ActiveProfile != null &&
                   CFG.ActiveProfile.HasActiveEngines)
                {
                    vessel.ctrlState.mainThrottle = 0;
                    if(VSL.IsActiveVessel)
                        FlightInputHandler.state.mainThrottle = 0;
                }
            }
        }

        void onVesselModify(Vessel vsl)
        { 
            if(vsl == null || vsl != vessel) return;
            //this.Log("onVesselModify: vsl.id {}, old.id {}", vsl.id, 
                     //VSL != null && VSL.vessel != null? VSL.vessel.id.ToString() : "null");//debug
            AllModules.ForEach(m => m.SaveToConfig());
            check_priority();
            if(GroupMaster)
                change_GID();
            EnableTCA(TCA_Active);
            if(!TCA_Active) reset();
            else if(VSL == null || VSL.vessel == null || vsl.id != VSL.vessel.id)
            { 
                reset(); 
                if(CFG != null) 
                    CFG = CFG.Clone<VesselConfig>();
                init(); 
            }
            else 
            {
                VSL.Engines.ForceUpdateParts = true;
                StartCoroutine(updateUnpackDistance());
            }
        }

        void onEditorShipModified(IShipconstruct ship) =>
        set_TCA_Active(ship);

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
                set_TCA_Active(EditorLogic.fetch.ship);
            else set_TCA_Active(vessel);
        }

        void init_group(List<ModuleTCA> all_tca)
        {
            if(string.IsNullOrEmpty(GID))
            {
                var other = all_tca.FirstOrDefault(tca => !string.IsNullOrEmpty(tca.GID));
                SetGID(other != null? other.GID : new_GID());
                //this.Log("init group: {}", GID);//debug
            }
        }

        ModuleTCA select_group_master(List<ModuleTCA> all_tca)
        {
            var master = all_tca.FirstOrDefault(tca => tca.GID == GID);
            GroupMaster = master == this;
            //this.Log("select group master: {}, {}", master, GroupMaster);//debug
            return master;
        }

        string new_GID() => Guid.NewGuid().ToString("N");

        void SetGID(string gid)
        {
            GID = gid;
            GID_Display = gid.Substring(gid.Length-6);
        }

        void change_GID()
        {
            var gid = new_GID();
            GetGroup().ForEach(tca => tca.SetGID(gid));
            //this.Log("change group: {}, {}", GID, GetGroup());//debug
        }

        void set_TCA_Active(IShipconstruct ship)
        {
            TCA_Active = false;
            var all_tca = AllTCA(ship);
            init_group(all_tca);
            var masters = all_tca
                .Where(tca => tca.GroupMaster)
                .ToDictionary(tca => tca.GID);
            if(!masters.ContainsKey(GID))
            {
                var master = select_group_master(all_tca);
                if(master != null)
                    masters[GID] = master;
            }
            if(GroupMaster)
                TCA_Active = (masters.Count == 1 || 
                              masters.Values.SelectMax(m => 
            {
                try { return -m.part.Modules.IndexOf(m)-ship.Parts.IndexOf(m.part); }
                catch(NullReferenceException) { return float.NegativeInfinity; }
            }) == this);
            Actions["ToggleTCA"].active = TCA_Active;
            //this.Log("TCA Active: {}, GroupMaster {}, masters {}, top master {}", 
                     //TCA_Active, GroupMaster, masters,
                     //masters.Values.SelectMax(m => -m.part.Modules.IndexOf(m)-ship.Parts.IndexOf(m.part)));//debug
        }

        public void EnableTCA(bool enable = true)
        {
            TCA_Active = enable;
            Actions["ToggleTCA"].active = enable;
            Actions["onActionUpdate"].active = enable;
            //this.Log("TCA Module enabled: {}", enable);//debug
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
            if(ship.Parts != null)
            {
                (from p in ship.Parts where p.Modules != null 
                 select p.Modules.GetModules<ModuleTCA>())
                    .ForEach(TCA_Modules.AddRange);
            }
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

        public List<ModuleTCA> GetGroup() =>
        vessel != null? AllTCA(vessel).Where(tca => tca.GID == GID).ToList() : null;

        void updateCFG()
        {
            var group = GetGroup();
            var master = group.FirstOrDefault(tca => tca.GroupMaster);
            if(this == master)
            {
                if(CFG == null)
                    CFG = new VesselConfig();
                group.ForEach(tca => tca.CFG = CFG);
            }
        }

        void init()
        {
            if(!TCA_Active) return;
            updateCFG();
            VSL = new VesselWrapper(this);
            EnableTCA(VSL.Engines.All.Count > 0 || 
                          VSL.Engines.RCS.Count > 0 || 
                          VSL.Torque.Wheels.Count > 0);
            if(!TCA_Active) { VSL = null; return; }
            VSL.Init();
            TCAModulesDatabase.InitModules(this);
            VSL.ConnectAutopilotOutput();//should follow module initialization
            vessel.OnPreAutopilotUpdate += OnPreAutopilotUpdate;
            vessel.OnPostAutopilotUpdate += OnPostAutopilotUpdate;
            TCAGui.Reinitialize(this);
            StartCoroutine(updateUnpackDistance());
            Actions["onActionUpdate"].active = true;
            Actions["ToggleTCA"].actionGroup = CFG.ActionGroup;
            CFG.Resume(this);
        }

        void reset()
        {
            if(VSL != null)
            {
                vessel.OnPreAutopilotUpdate -= OnPreAutopilotUpdate;
                vessel.OnPostAutopilotUpdate -= OnPostAutopilotUpdate;
                VSL.Reset();
                AllModules.ForEach(m => m.Cleanup());
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
                AllModules.ForEach(m => m.OnEnableTCA(true));
            }
            else
            {
                AllModules.ForEach(m => m.OnEnableTCA(false));
                VSL.Engines.All.ForEach(e => e.forceThrustPercentage(100));
                VSL.Engines.RCS.ForEach(e => e.forceThrustPercentage(100));
                VSL.RestoreUnpackDistance();
            }
        }

        [KSPEvent(guiName ="Activate TCA", guiActive = true, active = true)]
        public void ActivateTCA()
        {
            if(TCA_Active) return;
            var all_tca = AllTCA(vessel);
            all_tca.ForEach(tca => 
            { 
                if(tca.TCA_Active)
                    tca.AllModules.ForEach(m => m.SaveToConfig());
                tca.EnableTCA(false); 
                tca.reset(); 
            });
            if(!GroupMaster)
            {
                all_tca
                    .Where(tca => tca.GID == GID)
                    .ForEach(tca => tca.GroupMaster = false);
                GroupMaster = true;
            }
            EnableTCA(true);
            init();
            ShowGroup();
        }

        [KSPEvent(guiName ="Show TCA Group", guiActive = true, active = true)]
        public void ShowGroup()
        {
            var group = GetGroup();
            group.ForEach(m => m.part.HighlightAlways(m.TCA_Active? 
                                                      Color.green : 
                                                      (m.GroupMaster? 
                                                       Color.magenta : 
                                                       Color.cyan)));
            StartCoroutine(CallbackUtil.DelayedCallback(3.0f, () => group.ForEach(m => 
            { 
                if(m != null && m.part != null) 
                    m.part.SetHighlightDefault();
            })));
        }
        #endregion

        #region ICommNetControlSource
        public void UpdateNetwork() {}
        VesselControlState localControlState;
        public VesselControlState GetControlSourceState() { return localControlState; }
        public bool IsCommCapable() { return false; }

        //this code adapted from the ModuleCommand
        //why not do it in OnStart?
        public virtual void Start()
        {
            if(HighLogic.LoadedSceneIsGame && !HighLogic.LoadedSceneIsEditor && 
               HighLogic.LoadedScene != GameScenes.SPACECENTER)
                {
                    if(CommNetNetwork.Initialized)
                    { if(vessel.Connection != null) OnNetworkInitialised(); }
                    GameEvents.CommNet.OnNetworkInitialized.Add(OnNetworkInitialised);
                }
        }
        protected virtual void OnNetworkInitialised()
        { vessel.connection.RegisterCommandSource(this); }
        #endregion

        void ClearFrameState()
        { 
            VSL.ClearFrameState();
            AllModules.ForEach(m => m.ClearFrameState());
            #if DEBUG
            if(VSL.IsActiveVessel) TCAGui.ClearDebugMessage();
            #endif
        }

        void Update() //works both in Editor and in flight
        {
            if(!TCA_Active) return;
            if(CFG != null) 
                CFG.ActionGroup = Actions["ToggleTCA"].actionGroup;
        }

        public override void OnUpdate()
        {
            if(Valid && CFG.Enabled)
            {
                //update heavy to compute parameters
                VSL.Geometry.Update();
                var ATC = GetModule<AttitudeControl>();
                if(ATC != null) ATC.UpdateCues();
            }
        }

        public void OnPreAutopilotUpdate(FlightCtrlState s) 
        {
            if(!Valid) return;
            //initialize systems
            VSL.PreUpdateState(s);
            State = TCAState.Disabled;
            if(CFG.Enabled) 
            {
                State = TCAState.Enabled;
                localControlState = VesselControlState.None;
                if(!VSL.Info.ElectricChargeAvailible) 
                {
                    if(VSL.Controls.WarpToTime > 0)
                        VSL.Controls.AbortWarp();
                    return;
                }
                localControlState = VesselControlState.ProbePartial;
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
                VSL.OnModulesUpdated();
            }
        }

        public void OnPostAutopilotUpdate(FlightCtrlState s)
        {
            if(!Valid || !CFG.Enabled) return;
            //handle engines
            VSL.PostUpdateState(s);
            VSL.Engines.Tune();
            if(VSL.Engines.NumActive > 0)
            {
                //:preset manual limits for translation if needed
                if(VSL.Controls.ManualTranslationSwitch.On)
                {
                    ENG.PresetLimitsForTranslation(VSL.Engines.Active.Manual, VSL.Controls.ManualTranslation);
                    if(CFG.VSCIsActive) ENG.LimitInDirection(VSL.Engines.Active.Manual, VSL.Physics.UpL);
                }
                //:optimize limits for steering
                ENG.PresetLimitsForTranslation(VSL.Engines.Active.Maneuver, VSL.Controls.Translation);
                ENG.Steer();
            }
            RCS.Steer();
            VSL.Engines.SetControls();
            VSL.FinalUpdate();
        }

        void FixedUpdate()
        {
            if(Valid &&
               TimeWarp.CurrentRate > 1 &&
               TimeWarp.WarpMode == TimeWarp.Modes.HIGH)
            {
                OnPreAutopilotUpdate(vessel.ctrlState);
                OnPostAutopilotUpdate(vessel.ctrlState);
            }
        }
    }
}
