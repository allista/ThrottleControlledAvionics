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
using System.Reflection;
using UnityEngine;
using AT_Utils;

namespace ThrottleControlledAvionics
{
	[AttributeUsage(AttributeTargets.Class, Inherited = false)]
	public class HiddenComponent : Attribute {}

	public static class ComponentDB<T> where T : class, new()
	{
		public class ComponentFactory
		{
			static ST CreateComponent<ST>() where ST : T, new() { return new ST(); }

			public delegate T Factory();
			public readonly Type Component;
			public readonly Factory Create;
			public ComponentFactory(Type component)
			{
				Component = component;
				Create = (Factory)Delegate
					.CreateDelegate(typeof(Factory), 
					                GetType()
					                .GetMethod("CreateComponent", BindingFlags.Static|BindingFlags.NonPublic)
					                .MakeGenericMethod(Component));
			}
		}

		/// <summary>
		/// Gets all types defined in all loaded assemblies.
		/// </summary>
		static IEnumerable<Type> get_all_types()
		{
			foreach (var assembly in AppDomain.CurrentDomain.GetAssemblies())
			{
				Type[] types;
				try	{ types = assembly.GetTypes(); }
				catch(Exception) { types = Type.EmptyTypes; }
				foreach(var type in types) yield return type;
			}
		}

		static SortedList<string, ComponentFactory> components;
		public static SortedList<string, ComponentFactory> Components
		{
			get
			{
				if(components == null)
				{
					components = new SortedList<string, ComponentFactory>();
					foreach(var t in get_all_types())
					{
						if(!t.IsAbstract && t.IsSubclassOf(typeof(T)) &&
						   t.GetCustomAttributes(typeof(HiddenComponent), true).Length == 0)
							components.Add(Utils.ParseCamelCase(t.Name.Replace(typeof(T).Name, "")), 
							               new ComponentFactory(t));
					}
				}
				return components;
			}
		}

		public static SortedList<string, ComponentFactory> AvailableComponents = new SortedList<string, ComponentFactory>();

		public static void UpdateAvailableComponents() 
		{
			AvailableComponents.Clear();
			foreach(var c in Components.Keys)
			{
				var cmp = Components[c];
				var reqs = cmp.Component.GetCustomAttributes(typeof(RequireModules), true) as RequireModules[];
				var available = true;
				if(reqs != null && reqs.Length > 0)
				{
					foreach(var req in reqs)
					{
						foreach(var m in req.Modules)
						{
							available &= TCAModulesDatabase.ModuleAvailable(m);
							if(!available) break;
						}
						if(!available) break;
					}
				}
				if(available) AvailableComponents[c] = cmp;
			}
		}

		static Vector2 scroll;
		public static bool Selector(out T component)
		{
			var ret = false;
			component = null;
			scroll = GUILayout.BeginScrollView(scroll, Styles.white, GUILayout.Height(Globals.Instance.ActionListHeight));
			GUILayout.BeginVertical();
			for(int i = 0, count = AvailableComponents.Keys.Count; i < count; i++)
			{
				var c = AvailableComponents.Keys[i];
				if(GUILayout.Button(c, Styles.normal_button, GUILayout.ExpandWidth(true)))
				{
					component = AvailableComponents[c].Create();
					ret = true;
				}
			}
			GUILayout.EndVertical();
			GUILayout.EndScrollView();
			return ret;
		}
	}
}

