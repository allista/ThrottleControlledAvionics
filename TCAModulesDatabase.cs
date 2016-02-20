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
using UnityEngine;

namespace ThrottleControlledAvionics
{
	[KSPAddon(KSPAddon.Startup.Instantly, false)]
	public class TCAModulesDatabase : MonoBehaviour
	{
		public static readonly Type[] ValidModules = Assembly.GetExecutingAssembly().GetTypes().Where(ValidModule).ToArray();
		public static readonly Dictionary<Type, ModuleMeta> RegisteredModules = new Dictionary<Type, ModuleMeta>();
		public static readonly List<Type> Pipeline = new List<Type>();

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
			foreach(var module in ValidModules)
			{
				var meta = new ModuleMeta(module);
				if(!meta.Valid) continue;
				RegisteredModules.Add(module, meta);
				meta.Input.ForEach(inp => add_optional(module, inp));
			}
			SortModules();

			#if DEBUG
			Utils.Log("\nTCA Modules in the ModulesDatabase:\n{0}", 
			          RegisteredModules.Aggregate("", (s, t) => s + t.Value + "\n\n"));
			Utils.Log("Pipeline: {0}", Pipeline.Aggregate("", (s, t) => s + t.Name + "->"));
			#endif
		}

		public static bool ValidModule(Type module)
		{ return module.IsSubclassOf(typeof(TCAModule)) && !module.IsAbstract; }

		public static ModuleMeta GetModuleMeta(Type t)
		{ 
			ModuleMeta meta = null;
			return RegisteredModules.TryGetValue(t, out meta)? meta : null;
		}

		public static bool ModuleAvailable(Type m) 
		{ 
			if(!ValidModule(m)) return false;
			if(!TCAScenario.Globals.IntegrateIntoCareer) return true;
			var meta = GetModuleMeta(m);
			return meta == null || 
				((string.IsNullOrEmpty(meta.PartName) || 
				  Utils.PartIsPurchased(meta.PartName))
				 && meta.Requires.All(ModuleAvailable));
		}

		static TCAModule create_module(Type mtype, ModuleTCA TCA)
		{ return !ModuleAvailable(mtype) ? null : TCA.CreateComponent(mtype) as TCAModule; }

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
			if(partname != null) PartName = partname;
			AddToSet<RequireModules>(ref Requires);
			AddToSet<OptionalModules>(ref Optional);
			AddToSet<ModuleInputs>(ref Input);
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
}

