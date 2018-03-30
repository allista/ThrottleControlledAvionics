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
using System.Linq;

namespace ThrottleControlledAvionics
{
    [AttributeUsage(AttributeTargets.Class, Inherited = false, AllowMultiple = false)]
    public class ComponentInfo : Attribute 
    {
        public string Name = "";
        public string Description = "";
        public bool Hidden;
    }

    public static class ComponentDB<T> where T : class, new()
    {
        public class ComponentFactory
        {
            ST CreateComponent<ST>() where ST : class, T, new() 
            { 
                if(Info == null) return new ST();
                var constructor = typeof(ST).GetConstructor(new []{typeof(ComponentInfo)});
                return constructor == null ? new ST() : constructor.Invoke(new[] {Info}) as ST;
            }

            public delegate T Factory();
            public readonly ComponentInfo Info;
            public readonly string Name;
            public readonly Type Component;
            public readonly Factory Create;
            public readonly GUIContent Label;

            public ComponentFactory(Type component, ComponentInfo info)
            {
                Info = info;
                Component = component;
                //parse name and create GUIContent
                Name = Info == null || string.IsNullOrEmpty(Info.Name)? 
                    Utils.ParseCamelCase(Component.Name.Replace(typeof(T).Name, "")) : Info.Name;
                if(Info != null && !string.IsNullOrEmpty(Info.Description))
                   Label = new GUIContent(Name, Info.Description);
                else Label = new GUIContent(Name);
                //make generic factory method
                Create = (Factory)Delegate
                    .CreateDelegate(typeof(Factory), this,
                                    GetType()
                                    .GetMethod("CreateComponent", BindingFlags.Instance|BindingFlags.NonPublic)
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
                try    { types = assembly.GetTypes(); }
                catch(Exception) { types = Type.EmptyTypes; }
                foreach(var type in types) yield return type;
            }
        }

        class TypeComparer : IComparer<Type>
        {
            public int Compare(Type x, Type y)
            { return x.Name.CompareTo(y.Name); }
        }
        static SortedList<Type, ComponentFactory> components;
        public static SortedList<Type, ComponentFactory> Components
        {
            get
            {
                if(components == null)
                {
                    components = new SortedList<Type, ComponentFactory>(new TypeComparer());
                    foreach(var t in get_all_types())
                    {
                        if(t.IsAbstract || !t.IsSubclassOf(typeof(T))) continue;
                        var info = t.GetCustomAttributes(typeof(ComponentInfo), true).FirstOrDefault() as ComponentInfo;
                        if(info != null && info.Hidden) continue;
                        var c = new ComponentFactory(t, info);
                        components.Add(t, c);
                    }
                }
                return components;
            }
        }

        public static SortedList<Type, ComponentFactory> AvailableComponents = new SortedList<Type, ComponentFactory>(new TypeComparer());

        public static void UpdateAvailableComponents(VesselConfig CFG) 
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
                            available &= TCAModulesDatabase.ModuleAvailable(m, CFG);
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
            foreach(var c in AvailableComponents.Values)
            {
                if(GUILayout.Button(c.Label, Styles.normal_button, GUILayout.ExpandWidth(true)))
                {
                    component = c.Create();
                    ret = true;
                }
            }
            GUILayout.EndVertical();
            GUILayout.EndScrollView();
            return ret;
        }
    }
}

