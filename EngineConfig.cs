//  Author:
//       Allis Tauri <allista@gmail.com>
//
//  Copyright (c) 2015 Allis Tauri
//
// This work is licensed under the Creative Commons Attribution-ShareAlike 4.0 International License. 
// To view a copy of this license, visit http://creativecommons.org/licenses/by-sa/4.0/ 
// or send a letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.

using System;
using System.Collections.Generic;
using UnityEngine;
using AT_Utils;
using System.Linq;

namespace ThrottleControlledAvionics
{
    public class EngineConfig : ConfigNodeObject
    {
        new public const string NODE_NAME = "ENGINECFG";
        const float lim_eps = 1e-5f;

        [Persistent] public string Name = "";
        [Persistent] public bool On;
        [Persistent] public float Limit;
        [Persistent] string role;
        public TCARole Role;
        public bool Changed, Edit;

//        bool changed; //debug
//        public bool Changed
//        { 
//            get { return changed; } 
//            set { 
//                if(value != changed) DebugUtils.Log("{}.Changed: {}->{}", Name, changed, value);
//                changed = value; 
//            } 
//        }
//        public bool Edit;

        public EngineConfig() {}
        public EngineConfig(EngineWrapper e) 
        { Name = e.Group > 0? ("Group "+e.Group) : e.name; Update(e, true); }
        public EngineConfig(EngineConfig c)    { Update(c); }

        public void Update(EngineConfig c)
        {
            Name  = c.Name;
            On    = c.On;
            Limit = c.Limit;
            Role  = c.Role;
        }

        public void Update(EngineWrapper e, bool with_On = false)
        {
            Limit = e.thrustLimit;
            Role  = e.Role;
            if(with_On) On = e.engine.EngineIgnited;
            else Changed |= On != e.engine.EngineIgnited;
        }

        public void Update(IList<EngineWrapper> engines, bool with_On = false)
        {
            var cfg = new EngineConfig(this);
            for(int i = 0, enginesCount = engines.Count; i < enginesCount; i++)
            {
                var e = engines[i];
                if(cfg.DiffersFrom(e)) Update(e, with_On);
            }
        }

        public override void Load (ConfigNode node)
        {
            base.Load(node);
            try { Role = (TCARole)Enum.Parse(typeof(TCARole), role); }
            catch { Role = default(TCARole); }
        }

        public override void Save (ConfigNode node)
        {
            role = Enum.GetName(typeof(TCARole), Role);
            base.Save(node);
        }

        public void Apply(EngineWrapper e)
        {
            if(e == null || e.info == null) return;
            e.SetRole(Role);
            if(HighLogic.LoadedSceneIsFlight)
            {
                if(On && !e.engine.EngineIgnited) e.engine.Activate();
                else if(!On && e.engine.EngineIgnited) e.engine.Shutdown();
            }
            if(Role == TCARole.MANUAL) e.forceThrustPercentage(Limit*100);
            Changed = false;
        }

        public bool DiffersFrom(EngineWrapper e)
        {
            return On != e.engine.EngineIgnited || Role != e.Role ||
                (Role == TCARole.MANUAL && Mathf.Abs(e.thrustLimit-Limit) > lim_eps);
        }

        void RoleControl()
        {
            if(GUILayout.Button("<", Styles.normal_button, GUILayout.Width(15)))
            { Role = TCAEngineInfo.PrevRole(Role); Changed = true; }
            GUILayout.Label(TCAEngineInfo.RoleNames[(int)Role], GUILayout.Width(120));
            if(GUILayout.Button(">", Styles.normal_button, GUILayout.Width(15)))
            { Role = TCAEngineInfo.NextRole(Role); Changed = true; }
        }

        void NameControl(string comment)
        {
            if(Edit)
            {
                Name = GUILayout.TextField(Name, GUILayout.Width(80));
                Edit &= !GUILayout.Button("Done", Styles.confirm_button, GUILayout.Width(50));
            }
            else 
            {
                var name = Name+(comment ?? "");
                Edit |= GUILayout.Button(new GUIContent(name, name), 
                                         Styles.normal_button, GUILayout.Width(130));
            }
        }

        public bool Draw(string comment = null, bool with_role = true)
        {
            GUILayout.BeginHorizontal();
            NameControl(comment);
            if(GUILayout.Button(On? "On" : "Off", On? Styles.enabled_button : Styles.close_button, GUILayout.Width(30)))
            { On = !On; Changed = true; }
            if(with_role) RoleControl();
            GUILayout.EndHorizontal();
            if(Role == TCARole.MANUAL)
            {
                GUILayout.BeginHorizontal();
                var lim = Utils.FloatSlider("", Limit, 0f, 1f, "P1", 50, "Throttle");
                if(lim <= lim_eps) lim = 0;
                if(Mathf.Abs(lim-Limit) > lim_eps) { Limit = lim; Changed = true; }
                GUILayout.EndHorizontal();
            }
            return Changed;
        }

        public override string ToString()
        { return string.Format("[{0}]: Role {1}, Limit {2}, On: {3}", Name, Role, Limit, On); }
    }

    public abstract class EngineConfigDB<K> : ConfigNodeObject
    {
        public Dictionary<K, EngineConfig> DB = new Dictionary<K, EngineConfig>();

        protected abstract bool TryParseK(string k, out K K);

        public override void Save (ConfigNode node)
        {
            foreach(var k in DB.Keys) 
                DB[k].Save(node.AddNode(k.ToString()));
            base.Save(node);
        }

        public override void Load (ConfigNode node)
        {
            base.Load(node);
            DB.Clear();
            if(node == null) return;
            foreach(var n in node.GetNodes())
            {
                K k;
                if(TryParseK(n.name, out k))
                    DB[k] = ConfigNodeObject.FromConfig<EngineConfig>(n);
            }
        }

        #region reduced dict interface
        public EngineConfig this[K k] { get { return DB[k]; } set { DB[k] = value; } }
        public bool TryGetValue(K k, out EngineConfig c) { return DB.TryGetValue(k, out c); }
        public void Add(K k, EngineConfig c) { DB.Add(k, c); }
        public bool Remove(K k) { return DB.Remove(k); }
        public void Clear() { DB.Clear(); }
        public bool ContainsKey(K k) { return DB.ContainsKey(k); }
        public bool ContainsValue(EngineConfig c) { return DB.ContainsValue(c); }
        public int  Count { get { return DB.Count; } }
        public Dictionary<K, EngineConfig>.KeyCollection Keys { get { return DB.Keys; } }
        public Dictionary<K, EngineConfig>.ValueCollection Values { get { return DB.Values; } }
        #endregion
    }

    public class EngineConfigIntDB : EngineConfigDB<int>
    {
        protected override bool TryParseK (string k, out int K)
        { return int.TryParse(k, out K); }
    }

    public class EngineConfigUintDB : EngineConfigDB<uint>
    {
        protected override bool TryParseK (string k, out uint K)
        { return uint.TryParse(k, out K); }
    }

    public class EnginesProfile : ConfigNodeObject
    {
        new public const string NODE_NAME = "ENGINESPROF";
        protected static readonly string[] OnPlanetStates = { "On Planets", "In Space", "Always" };

        [Persistent] public string Name;
        [Persistent] public bool Active;
        [Persistent] public bool Activated;
        [Persistent] public bool Default;
        [Persistent] public int  NumManual;
        [Persistent] public int  OnPlanet = 2;
        [Persistent] public int  Stage = -1;
        [Persistent] public bool Level;
        [Persistent] public int  SmartEngines;

        [Persistent] public EngineConfigIntDB  Groups = new EngineConfigIntDB();
        [Persistent] public EngineConfigUintDB Single = new EngineConfigUintDB();
        public bool Changed, Edit;

        public bool HasActiveEngines
        { get { return Single.DB.Any(e => e.Value.On) || Groups.DB.Any(e => e.Value.On); } }

        public EnginesProfile() {}
        public EnginesProfile(EnginesProfile p)
        {
            Name = p.Name+" (Copy)";
            OnPlanet = p.OnPlanet;
            foreach(var c in p.Groups.DB) 
                Groups[c.Key] = new EngineConfig(c.Value);
            foreach(var c in p.Single.DB) 
                Single[c.Key] = new EngineConfig(c.Value);
            NumManual = p.NumManual;
            SmartEngines = p.SmartEngines;
            Level = p.Level;
        }
        public EnginesProfile(string name, IList<EngineWrapper> engines)
        { Name = name; Init(engines); }

        public void Init(IList<EngineWrapper> engines)
        {
            Single.Clear();
            Groups.Clear();
            NumManual = 0;
            foreach(var e in engines)
            {
                if(e.Group > 0)
                {
                    if(!Groups.ContainsKey(e.Group))
                    {
                        if(e.Role == TCARole.MANUAL) NumManual++;
                        Groups[e.Group] = new EngineConfig(e);
                    }
                }
                else if(!Single.ContainsKey(e.ID))
                {
                    Single[e.ID] = new EngineConfig(e);
                    if(e.Role == TCARole.MANUAL) NumManual++;
                }
            }
        }

        public void Update(IList<EngineWrapper> engines, bool with_On = false)
        {
//            DebugUtils.Log("Updating {0}", Name);//debug
            var group_engines = new ListDict<int, EngineWrapper>();
            var groups = new EngineConfigIntDB();
            var single = new EngineConfigUintDB();
            NumManual = 0;
            Changed = false;
            //sort configs and engines, update single configs
            for(int i = 0, enginesCount = engines.Count; i < enginesCount; i++)
            {
                var e = engines[i];
                var c = GetConfig(e);
                if(c == null)
                {
                    if(e.Group > 0)
                    {
                        if(!groups.ContainsKey(e.Group))
                        {
                            if(e.Role == TCARole.MANUAL) NumManual++;
                            groups[e.Group] = new EngineConfig(e);
                            group_engines.Add(e.Group, e);
                        }
                    }
                    else if(!single.ContainsKey(e.ID))
                    {
                        if(e.Role == TCARole.MANUAL) NumManual++;
                        single[e.ID] = new EngineConfig(e);
                    }
                }
                else if(e.Group > 0)
                { 
                    if(e.Role == TCARole.MANUAL && !groups.ContainsKey(e.Group)) NumManual++;
                    group_engines.Add(e.Group, e);
                    groups[e.Group] = c;
                }
                else 
                { 
                    if(e.Role == TCARole.MANUAL) NumManual++;
                    c.Update(e, with_On);
                    Changed |= c.Changed;
                    single[e.ID] = c;
                }
            }
            //update groups
            foreach(var g in group_engines)
            {
                var c = groups[g.Key];
                c.Update(g.Value, with_On);
                Changed |= c.Changed;
            }
            Changed |= Groups.Count != groups.Count || Single.Count != single.Count;
            Groups = groups; Single = single;
            if(Changed) Apply(engines);
        }

        public void OnActivated(VesselWrapper VSL)
        {
            if(Level && VSL.OnPlanet) 
                VSL.CFG.HF.OnIfNot(HFlight.Level);
            if(SmartEngines != 0)
                VSL.CFG.UseSmartEngines = SmartEngines > 0;
            Activated = false;
        }

        public void Apply(IList<EngineWrapper> engines)
        {
//            DebugUtils.Log("Applying {}", Name);//debug
            for(int i = 0, enginesCount = engines.Count; i < enginesCount; i++) 
            {
                var e = engines[i];
                var c = GetConfig(e);
//                Utils.Log("Applying {0} to {1}, {2}: engine ignited {3}", c, e.ID, e.part.flightID, e.engine.EngineIgnited);//debug
                if(c != null) c.Apply(e);
//                Utils.Log("Applyed {0}: engine ignited {1}", c, e.engine.EngineIgnited);//debug
            }
            Changed = false;
        }

        public EngineConfig GetConfig(EngineWrapper e)
        {
            EngineConfig c; 
            if(e.Group > 0) { if(!Groups.TryGetValue(e.Group, out c)) return null; }
            else if(!Single.TryGetValue(e.ID, out c)) return null;
            return c;
        }

        public bool Usable(bool on_planet)
        {
            if(Default) return true;
            if(on_planet) return OnPlanet == 0 || OnPlanet == 2;
            return OnPlanet == 1 || OnPlanet == 2;
        }

        public bool ShouldBeActive(bool on_planet)
        {
            if(Default) return false;
            return on_planet ? OnPlanet == 0 : OnPlanet == 1;
        }

        void StageControl()
        { 
            GUILayout.Label(new GUIContent("Stage:", "Automatically activate at stage"), GUILayout.ExpandWidth(false));
            Stage = Utils.IntSelector(Stage, 0); 
        }

        void OnPlanetControl()
        {
            GUILayout.Label("Active:", GUILayout.ExpandWidth(false));
            if(GUILayout.Button(new GUIContent(OnPlanetStates[OnPlanet], "When this profile should be active"), 
                                Styles.normal_button, GUILayout.Width(80)))
                OnPlanet = (OnPlanet+1)%3;
        }

        void TitleControl()
        {
            if(Active) GUILayout.Toggle(Active, "", GUILayout.Width(15));
            else 
            {
                Active = GUILayout.Toggle(Active, new GUIContent("", "Activate"), GUILayout.Width(15));
                Changed |= Active;
            }
            if(Edit) Name = GUILayout.TextField(Name, GUILayout.ExpandWidth(true), GUILayout.MinWidth(50));
            else GUILayout.Label(Name, Active? Styles.green : Styles.white, GUILayout.ExpandWidth(true), GUILayout.MinWidth(50));
        }

        void Switches()
        { 
            GUILayout.Label("Smart Engines:", GUILayout.ExpandWidth(false));
            switch(SmartEngines)
            {
            case 0:
                if(GUILayout.Button("No Change", Styles.normal_button, GUILayout.Width(80)))
                    SmartEngines = 1;
                break;
            case 1:
                if(GUILayout.Button("Enable", Styles.enabled_button, GUILayout.Width(80)))
                    SmartEngines = -1;
                break;
            case -1:
                if(GUILayout.Button("Disable", Styles.active_button, GUILayout.Width(80)))
                    SmartEngines = 0;
                break;
            default: 
                SmartEngines = 0;
                break;
            }
            Utils.ButtonSwitch("AutoLevel", ref Level, "Level the craft when this profile is activated", GUILayout.ExpandWidth(false)); 
        }

        public bool Draw()
        {
            GUILayout.BeginVertical();
            GUILayout.BeginHorizontal();
            //header controls
            TitleControl();
            //default switch
            if(Default) GUILayout.Label("Default", Styles.green, GUILayout.ExpandWidth(false));
            else { Default = GUILayout.Toggle(Default, "Default", GUILayout.ExpandWidth(false)); }
            //edit button
            if(GUILayout.Button(Edit? "Done" : "Edit", 
                Edit? Styles.confirm_button : Styles.normal_button, GUILayout.Width(50)))
                Edit = !Edit;
            //delete button
            var delete = !Default && 
                !GUILayout.Button(new GUIContent("X", "Delete profile"), Styles.close_button, GUILayout.Width(20));
            GUILayout.EndHorizontal();
            if(Edit)
            {
                GUILayout.BeginVertical(Styles.white);
                GUILayout.BeginHorizontal();
                GUILayout.FlexibleSpace();
                Switches();
                GUILayout.FlexibleSpace();
                GUILayout.EndHorizontal();
                if(!Default)
                {
                    GUILayout.BeginHorizontal();
                    GUILayout.FlexibleSpace();
                    OnPlanetControl();
                    StageControl();
                    GUILayout.FlexibleSpace();
                    GUILayout.EndHorizontal();
                }
                foreach(var k in Groups.Keys)
                    Changed |= Groups[k].Draw(string.Format(" (G{0})", k));
                foreach(var k in Single.Keys)
                    Changed |= Single[k].Draw();
                GUILayout.EndVertical();
            }
            GUILayout.EndVertical();
            return delete;
        }

        public void DrawManual()
        {
            GUILayout.BeginVertical();
            foreach(var k in Groups.Keys)
            {
                var c = Groups[k];
                if(c.Role != TCARole.MANUAL) continue;
                Changed |= c.Draw(string.Format(" (G{0})", k), false);
            }
            foreach(var k in Single.Keys)
            {
                var c = Single[k];
                if(c.Role != TCARole.MANUAL) continue;
                Changed |= c.Draw(with_role:false);
            }
            GUILayout.EndVertical();
        }
    }

    public class EnginesProfileDB : ConfigNodeObject
    {
        new public const string NODE_NAME = "ENGPROFILES";

        public List<EnginesProfile> DB = new List<EnginesProfile>();

        public EnginesProfile Default { get; private set; }
        public EnginesProfile Active { get; private set; }
        public bool Empty { get { return DB.Count == 0; } }

        Vector2 enginesScroll, manualScroll;

        public override void Load(ConfigNode node)
        {
            base.Load(node);
            DB.Clear();
            foreach(var n in node.GetNodes())
            {
                var c = ConfigNodeObject.FromConfig<EnginesProfile>(n);
                if(c.Active) Active = c;
                if(c.Default) Default = c;
                DB.Add(c);
            }
            if(!Empty)
            {
                if(Default == null) 
                { Default = DB[0]; Default.Default = true; }
                if(Active == null)
                { Active = Default; Active.Active = true; }
            }
        }

        public override void Save(ConfigNode node)
        {
            DB.ForEach(p => p.SaveInto(node));
            base.Save(node);
        }

        public void AddProfile(IList<EngineWrapper> engines, string name = "Default")
        { 
            DB.Add(new EnginesProfile(name, engines));
            if(DB.Count == 1) 
            {
                DB[0].Active  = true;
                DB[0].Default = true;
                DB[0].Changed = true;
                Active = Default = DB[0];
            }
        }

        public void CopyActive()
        { if(Active != null) DB.Add(new EnginesProfile(Active)); }

        void Activate(EnginesProfile p)
        {
            Active.Active  = false;
            Active.Changed = false;
            Active = p;
            Active.Activated = true;
            Active.Changed = true;
            Active.Active  = true;
        }

        void SetDefault(EnginesProfile p)
        {
            Default.Default = false; 
            Default = p;
            Default.Default = true;
        }

        public bool ActivateOnStage(int stage, IList<EngineWrapper> engines)
        {
            var activated = false;
            foreach(var p in DB)
            {
                if(p.Stage < 0 || p.Stage != stage) continue;
                Activate(p);
                Active.Apply(engines);
                activated = true;
                break;
            }
            return activated;
        }

        public void OnPlanetChanged(bool on_planet)
        {
            if(DB.Count < 2) return;
            bool found = false;
            foreach(var p in DB)
            {
                if(p == Active || p == Default || 
                   !p.ShouldBeActive(on_planet)) continue;
                Activate(p);
                found = true;
                break;
            }
            if(!found) 
            {
                if(Active.Usable(on_planet)) return;
                Activate(Default);
            }
        }

        public bool Activate(string name)
        {
            foreach(var p in DB)
            {
                if(p.Name != name) continue;
                Activate(p);
                return true;
            }
            return false;
        }

        public void Draw(int height)
        {
            if(DB.Count == 0) return;
            GUILayout.BeginVertical();
            enginesScroll = GUILayout.BeginScrollView(enginesScroll, GUILayout.Height(height));
            GUILayout.BeginVertical();
            var num_profs = DB.Count ;
            var del = new List<EnginesProfile>(num_profs);
            for(int i = 0; i < num_profs; i++)
            {
                var p = DB[i];
                if(!p.Draw() && p != Default)
                {
                    del.Add(p);
                    if(p == Active) Activate(Default);
                    continue;
                }
                if(p.Active && p != Active)    Activate(p);
                if(p.Default && p != Default) SetDefault(p);
            }
            if(del.Count > 0) foreach(var p in del) DB.Remove(p);
            GUILayout.EndVertical();
            GUILayout.EndScrollView();
            if(GUILayout.Button("Add Profile", Styles.add_button, GUILayout.ExpandWidth(true)))
                CopyActive();
            GUILayout.EndVertical();
        }

        public void DrawManual(int height)
        {
            if(Active == null) return;
            GUILayout.BeginVertical(Styles.white);
            manualScroll = GUILayout.BeginScrollView(manualScroll, GUILayout.Height(height));
            Active.DrawManual();
            GUILayout.EndScrollView();
            GUILayout.EndVertical();
        }
    }
}

