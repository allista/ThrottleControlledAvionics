//  Author:
//       Allis Tauri <allista@gmail.com>
//
//  Copyright (c) 2016 Allis Tauri
//
// This work is licensed under the Creative Commons Attribution-ShareAlike 4.0 International License. 
// To view a copy of this license, visit http://creativecommons.org/licenses/by-sa/4.0/ 
// or send a letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.
//
using System;
using System.Linq;
using System.Collections.Generic;
using System.Reflection;
using System.IO;
using UnityEngine;
using AT_Utils;

namespace ThrottleControlledAvionics
{
    [KSPAddon(KSPAddon.Startup.Instantly, false)]
    public class TCAModulesDatabase : MonoBehaviour
    {
        static Type[] tca_modules = null;
        public static Type[] ValidModules
        {
            get 
            {
                if(tca_modules == null)
                    tca_modules = Assembly.GetExecutingAssembly().GetTypes().Where(ValidModule).ToArray();
                return tca_modules;
            }
        }

        public static readonly Dictionary<Type, ModuleMeta> RegisteredModules = new Dictionary<Type, ModuleMeta>();
        static readonly List<Type> Pipeline = new List<Type>();
        static readonly Dictionary<string, TCAPart> TechTreeParts = new Dictionary<string, TCAPart>();

        static void sort_module(Type m, List<Type> route = null)
        {
            if(route == null) route = new List<Type>();
            if(route.Contains(m))
                throw new ApplicationException(string.Format("Circular TCAModule dependency detected:\n{0}", 
                                                             route.Aggregate("", (s, t) => s+t.Name+"->")+m.Name));
            if(RegisteredModules.ContainsKey(m) && !Pipeline.Contains(m))
            {
                route.Add(m);
                var meta = GetModuleMeta(m);
                meta.Requires.ForEach(req => sort_module(req, route));
                meta.Optional.ForEach(opt => sort_module(opt, route));
                Pipeline.Insert(0, m);
                route.Remove(m);
            }
        }

        static void SortModules()
        {
            Pipeline.Clear();
            var modules = new Queue<Type>(RegisteredModules.Keys);
            while(modules.Count > 0) sort_module(modules.Dequeue());
        }

        static void add_optional(Type optional, Type user)
        {
            var user_meta = GetModuleMeta(user);
            if(user_meta != null) user_meta.Optional.Add(optional);
        }
        
        void Awake()
        {
            //register modules
            foreach(var module in ValidModules)
            {
                var meta = new ModuleMeta(module);
                if(!meta.Valid) continue;
                RegisteredModules.Add(module, meta);
            }
            //add inputs
            RegisteredModules.ForEach(m => m.Value.Input.ForEach(inp => add_optional(m.Key, inp)));
            //topo-sort modules
            SortModules();
            //register techtree parts
            foreach(var module in RegisteredModules.Values)
            {
                if(string.IsNullOrEmpty(module.PartName)) continue;
                TCAPart part = null;
                if(TechTreeParts.TryGetValue(module.PartName, out part))
                    part.AddModule(module);
                else
                {
                    part = new TCAPart(module.PartName);
                    part.AddModule(module);
                    TechTreeParts.Add(part.Name, part);
                }
            }

            #if DEBUG
            Utils.Log("\nTCA Modules in the ModulesDatabase:\n{}", 
                      RegisteredModules.Aggregate("", (s, t) => s + t.Value + "\n\n"));
            Utils.Log("Pipeline: {}", Pipeline.Aggregate("", (s, t) => s + t.Name + "->"));
            Utils.Log("AP Pipeline: {}", Pipeline.Where(m => m.IsSubclassOf(typeof(AutopilotModule))).Aggregate("", (s, t) => s + t.Name + "->"));
            File.WriteAllText("ModuleDatabase.csv",
                              RegisteredModules.Aggregate("", (s, t) => s + t.Value.ToCSV() + "\n"));
            #endif
        }

        public static bool ValidModule(Type module)
        { return module.IsSubclassOf(typeof(TCAModule)) && !module.IsAbstract; }

        public static ModuleMeta GetModuleMeta(Type t)
        { 
            ModuleMeta meta = null;
            return RegisteredModules.TryGetValue(t, out meta)? meta : null;
        }

        public static bool ModuleAvailable(Type mtype, VesselConfig CFG) 
        { 
            if(!ValidModule(mtype)) return false;
            var meta = GetModuleMeta(mtype);
            if(meta == null) return true;
            if(CFG != null && !CFG.EnabledTCAParts.Contains(meta.PartName)) return false;
            if(!Globals.Instance.IntegrateIntoCareer) return true;
            return (string.IsNullOrEmpty(meta.PartName) || 
                    Utils.PartIsPurchased(meta.PartName))
                && meta.Requires.All(m => ModuleAvailable(m, CFG));
        }

        static TCAModule create_module(Type mtype, ModuleTCA TCA)
        { return ModuleAvailable(mtype, TCA.CFG)? TCA.CreateComponent(mtype) as TCAModule : null; }

        public static void InitModules(ModuleTCA TCA)
        {
            TCA.DeleteModules();
            foreach(var core_field in ModuleTCA.CoreModuleFields)
            {
                var module = create_module(core_field.FieldType, TCA);
                if(module == null) continue;
                core_field.SetValue(TCA, module);
                TCA.AllModules.Add(module);
            }
            foreach(var mtype in Pipeline)
            {
                var module = create_module(mtype, TCA);
                if(module == null) continue;
                TCA.ModulesDB.Add(mtype, module);
                if(mtype.IsSubclassOf(typeof(AutopilotModule)))
                    TCA.AutopilotPipeline.Add(module);
                else TCA.ModulePipeline.Add(module);
                TCA.AllModules.Add(module);
            }
            TCA.AllModules.ForEach(m => m.Init());
        }

        public static List<TCAPart> GetAllParts()
        {
            if(HighLogic.CurrentGame == null) return new List<TCAPart>();
            var list = TechTreeParts.Values.Select(p => p.Clone()).ToList();
            list.Sort((a, b) => a.Title.CompareTo(b.Title));
            list.ForEach(p => p.UpdateInfo());
            return list;
        }

        public static List<TCAPart> GetPurchasedParts()
        {
            if(HighLogic.CurrentGame == null) return new List<TCAPart>();
            var list = TechTreeParts.Values.Select(p => p.Clone()).ToList();
            list.ForEach(p => p.UpdateInfo());
            list = list.Where(p => p.Purchased).ToList();
            list.Sort((a, b) => a.Title.CompareTo(b.Title));
            return list;
        }

        public static List<FieldInfo> GetAllModuleFields(Type t, List<FieldInfo> list = null)
        {
            if(list == null) list = new List<FieldInfo>();
            list.AddRange(t.GetFields(BindingFlags.Instance|BindingFlags.NonPublic|BindingFlags.FlattenHierarchy)
                          .Where(fi => fi.FieldType.IsSubclassOf(typeof(TCAModule))));
            if(t.BaseType != null) GetAllModuleFields(t.BaseType, list);
            return list;
        }

        public static List<TCAModule> GetAllModules(object obj)
        {
            var AllModules = new List<TCAModule>();
            foreach(var fi in TCAModulesDatabase.GetAllModuleFields(obj.GetType()))
            {
                var module = fi.GetValue(obj) as TCAModule;
                if(module != null) AllModules.Add(module);
            }
            return AllModules;
        }
    }

    [AttributeUsage(AttributeTargets.Class, 
                    Inherited = false, 
                    AllowMultiple = true)]
    public abstract class Relation : Attribute
    {
        public HashSet<Type> Modules;

        protected static HashSet<Type> toValidSet(Type[] array)
        { return new HashSet<Type>(array.Where(TCAModulesDatabase.ValidModule)); }

        protected Relation(params Type[] modules)
        { if(modules != null) Modules = toValidSet(modules); }
    }

    public class RequireModules : Relation
    { public RequireModules(params Type[] modules) : base(modules) {} }
    public class OptionalModules : Relation
    { public OptionalModules(params Type[] modules) : base(modules) {} }

    public class ModuleInputs : Relation
    { public ModuleInputs(params Type[] modules) : base(modules) {} }
    public class OverrideModules : ModuleInputs
    { public OverrideModules(params Type[] modules) : base(modules) {} }

    public class CareerPart : Attribute
    {
        readonly string PartName = "";
        public CareerPart(string part_name = "") { PartName = part_name; }
        public CareerPart(Type module) { PartName = module.Name; }
        public static implicit operator string(CareerPart cp) { return cp.PartName; }
    }

    public class ModuleMeta
    {
        public Type Module;
        public readonly string PartName = "";

        public HashSet<Type> Requires = new HashSet<Type>();
        public HashSet<Type> Optional = new HashSet<Type>();
        public HashSet<Type> Input    = new HashSet<Type>();

        public bool Valid { get { return HasMeta(Module); } }

        public static A GetAttr<A>(Type t) where A : Attribute
        { return Attribute.GetCustomAttribute(t, typeof(A)) as A; }

        public static A[] GetAttrs<A>(Type t) where A : Attribute
        { return Attribute.GetCustomAttributes(t, typeof(A)) as A[]; }

        public static bool HasMeta(Type t)
        { 
            if(GetAttr<CareerPart>(t) != null) return true;
            var rels = GetAttrs<Relation>(t); 
            return rels != null && rels.Length > 0;
        }

        public void AddToSet<A>(ref HashSet<Type> S) where A : Relation
        {
            var rels = GetAttrs<A>(Module);
            if(rels == null) return;
            foreach(var rel in rels)
            {
                if(rel.Modules == null)
                {
                    rel.Modules = new HashSet<Type>(TCAModulesDatabase.ValidModules);
                    rel.Modules.Remove(Module);
                }
                S.UnionWith(rel.Modules);
            }
        }

        public ModuleMeta(Type module)
        {
            Module = module;
            var partname = GetAttr<CareerPart>(Module);
            if(partname != null) 
            {
                PartName = "TCAModule";
                PartName += string.IsNullOrEmpty(partname)? Module.Name : partname;
            }
            AddToSet<RequireModules>(ref Requires);
            AddToSet<OptionalModules>(ref Optional);
            AddToSet<ModuleInputs>(ref Input);
        }

        public string ToCSV()
        {
            return string.Format("{0},{1},{2}|,{3}",
                                 PartName,
                                 Module.Name ?? "null",
                                 Requires.Aggregate("", (s, t) => s + t.Name + ","),
                                 Optional.Aggregate("", (s, t) => s + t.Name + ","));
        }

        public override string ToString()
        {
            return string.Format("{0} [{1}]\nRequires: {2}\nOptional: {3}\nInput: {4}",
                                 Module.Name ?? "null",
                                 string.IsNullOrEmpty(PartName)? "always available" : PartName,
                                 Requires.Aggregate("", (s, t) => s + t.Name + " "),
                                 Optional.Aggregate("", (s, t) => s + t.Name + " "),
                                 Input.Aggregate("", (s, t) => s + t.Name + " "));
        }
    }

    public class TCAPart
    {
        public readonly string Name = "";
        public string Title = "";
        public string Description = "";
        public bool Purchased;
        public bool Active;

        public AvailablePart info { get; private set; }
        public readonly HashSet<ModuleMeta> Modules = new HashSet<ModuleMeta>();

        public TCAPart(string partname) { Name = partname; }

        protected TCAPart(TCAPart part)
        {
            Name = part.Name;
            Modules = part.Modules;
            info = part.info;
            Title = part.Title;
            Description = part.Description;
        }

        public TCAPart Clone() { return new TCAPart(this); }

        public void AddModule(ModuleMeta module)
        {
            if(module.PartName != Name)
            {
                Utils.Log("PartMeta[{}]: trying to add {} that belongs to the {}", 
                          Name, module.Module.Name, module.PartName);
                return;
            }
            Modules.Add(module);
        }

        public void UpdateInfo(VesselConfig CFG = null)
        { 
            if(info == null)
            {
                info = PartLoader.getPartInfoByName(Name); 
                if(info != null)
                {
                    Title = info.title;
                    Description = info.description;
                }
                if(string.IsNullOrEmpty(Title)) 
                    Title = Utils.ParseCamelCase(Name);
            }
            Purchased = Utils.PartIsPurchased(Name);
            Active = Modules.All(m => TCAModulesDatabase.ModuleAvailable(m.Module, CFG));
        }
    }


    public class TCAPartGraphNode
    {
        public TCAPart part { get; private set; }
        public HashSet<TCAPartGraphNode> inputs = new HashSet<TCAPartGraphNode>();
        public HashSet<TCAPartGraphNode> outputs = new HashSet<TCAPartGraphNode>();
        public int tier = 1;

        public virtual void SetPart(TCAPart part)
        { this.part = part; }

        public void AddInput(TCAPartGraphNode input)
        {
            inputs.Add(input);
            input.outputs.Add(this);
        }

        public void RemoveInput(TCAPartGraphNode input)
        {
            inputs.Remove(input);
            input.outputs.Remove(this);
        }

        public override string ToString() { return part.Name; }
    }

    public class TCAPartGraph<T> where T : TCAPartGraphNode, new()
    {
        public List<T> roots = new List<T>();
        public List<T> leafs = new List<T>();
        public SortedList<int,List<T>> tiers;

        static void sort_into_tiers(T node, HashSet<T> nodes, int tier = 1)
        {
            if(node.outputs.Count == 0)
                node.tier = Math.Max(node.tier, tier);
            tier += 1;
            node.outputs.ForEach(o => sort_into_tiers(o as T, nodes, tier));
            nodes.Add(node);
        }

        public void UpdateTiers()
        {
            var nodes = new HashSet<T>();
            tiers = new SortedList<int, List<T>>();
            roots.ForEach(r => sort_into_tiers(r, nodes));
            foreach(var node in nodes)
            {
                if(node.outputs.Count > 0)
                {
                    node.tier = -1;
                    node.outputs.ForEach(o => { if(node.tier < 0 || o.tier < node.tier) node.tier = o.tier; });
                    node.tier -= 1;
                }
                List<T> tier;
                if(tiers.TryGetValue(node.tier, out tier))
                    tier.Add(node);
                else tiers.Add(node.tier, new List<T>{node});
            }
            tiers.ForEach(t => t.Value.Sort((a, b) => b.outputs.Count.CompareTo(a.outputs.Count)));
        }

        public static G BuildGraph<G>(IEnumerable<TCAPart> parts)
            where G : TCAPartGraph<T>, new()
        {
            var nodes  = new List<T>();
            var lookup = new Dictionary<Type, T>();
            foreach(var part in parts)
            {
                var node = new T();
                node.SetPart(part);
                part.Modules.ForEach(m => lookup.Add(m.Module, node));
                nodes.Add(node);
            }
            foreach(var node in nodes) 
            {
                foreach(var module in node.part.Modules)
                {
                    foreach(var req in module.Requires)
                    {
                        T input;
                        if(lookup.TryGetValue(req, out input))
                        { if(input != node) node.AddInput(input); }
//                        Utils.Log("Node: {}, {} requires {}, input part {}\ninputs {}\noutputs {}", 
//                                  node, module.Module, req, input, node.inputs, node.outputs);//debug
                    }
                }
            }
            var graph = new G();
            graph.roots = nodes.Where(n => n.inputs.Count == 0).ToList();
            graph.leafs = nodes.Where(n => n.outputs.Count == 0).ToList();
            graph.UpdateTiers();
            return graph;
        }
    }
}

