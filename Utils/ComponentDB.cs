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

namespace ThrottleControlledAvionics
{
	[AttributeUsage(AttributeTargets.Class, Inherited = false)]
	public class HiddenComponent : Attribute {}

	public static class ComponentDB<T> where T : class, new()
	{
		public static TT Create<TT>() where TT : T, new() { return new TT(); }

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


		public delegate T Factory();
		static SortedList<string, Factory> components;
		public static SortedList<string, Factory> Components
		{
			get
			{
				if(components == null)
				{
					var creator = typeof(ComponentDB<T>).GetMethod("Create", BindingFlags.Static | BindingFlags.Public);
					components = new SortedList<string, Factory>();
					foreach(var t in get_all_types())
					{
						if(!t.IsAbstract && t.IsSubclassOf(typeof(T)) &&
						   t.GetCustomAttributes(typeof(HiddenComponent), true).Length == 0)
						{
							var constInfo = creator.MakeGenericMethod(t);
							if(constInfo == null) continue;
							components.Add(Utils.ParseCamelCase(t.Name.Replace(typeof(T).Name, "")), 
							               (Factory)Delegate.CreateDelegate(typeof(Factory), constInfo));
						}
					}
				}
				return components;
			}
		}

		static Vector2 scroll;
		public static bool Selector(out T component)
		{
			var ret = false;
			component = null;
			scroll = GUILayout.BeginScrollView(scroll, Styles.white, GUILayout.Height(TCAScenario.Globals.ActionListHeight));
			GUILayout.BeginVertical();
			for(int i = 0, count = Components.Keys.Count; i < count; i++)
			{
				var c = Components.Keys[i];
				if(GUILayout.Button(c, Styles.normal_button, GUILayout.ExpandWidth(true)))
				{
					component = Components[c]();
					ret = true;
				}
			}
			GUILayout.EndVertical();
			GUILayout.EndScrollView();
			return ret;
		}
	}
}

