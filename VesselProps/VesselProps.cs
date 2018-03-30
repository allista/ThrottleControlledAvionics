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
using System.Reflection;
using System.Collections.Generic;
using UnityEngine;

namespace ThrottleControlledAvionics
{
    public abstract class VesselProps
    {
        protected VesselProps(VesselWrapper vsl) { VSL = vsl; InitChildProps(); }

        protected readonly VesselWrapper VSL;
        protected VesselConfig CFG { get { return VSL.CFG; } }
        internal static Globals GLB { get { return Globals.Instance; } }
        protected Vessel vessel { get { return VSL.vessel; } }
        protected Transform refT { get { return VSL.refT; } set { VSL.refT = value; } }

        public virtual void Clear() {}
        public virtual void ClearFrameState() {}
        public virtual void Update() {}

        protected void Log(string msg, params object[] args) { VSL.Log(msg, args); }

        protected List<FieldInfo> get_all_props_fields(Type t, List<FieldInfo> list = null)
        {
            if(list == null) list = new List<FieldInfo>();
            list.AddRange(t.GetFields(BindingFlags.Instance|BindingFlags.NonPublic|BindingFlags.Public|BindingFlags.FlattenHierarchy)
                          .Where(fi => fi.FieldType.IsSubclassOf(typeof(VesselProps))));
            if(t.BaseType != null) get_all_props_fields(t.BaseType, list);
            return list;
        }

        protected void create_props(FieldInfo fi)
        {
            var m = fi.FieldType.GetConstructor(new []{typeof(VesselWrapper)});
            if(m != null) 
            {
                try { fi.SetValue(this, m.Invoke(new []{VSL})); }
                catch(Exception ex) { Log("Error while creating child props object {}:\n{}", fi.Name, ex); }
            }
            else Log("{} has no constructor {}(VesselWrapper)", fi.Name, fi.FieldType.Name);
        }

        public void InitChildProps()
        {
            var ModuleFields = get_all_props_fields(GetType());
            ModuleFields.ForEach(fi => create_props(fi));
        }
    }
}

