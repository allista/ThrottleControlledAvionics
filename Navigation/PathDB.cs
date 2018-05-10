//   PathDB.cs
//
//  Author:
//       Allis Tauri <allista@gmail.com>
//
//  Copyright (c) 2016 Allis Tauri

using System;
using System.Linq;
using System.Collections.Generic;
using AT_Utils;

namespace ThrottleControlledAvionics
{
    public class PathDB : ConfigNodeObject
    {
        public new const string NODE_NAME = "PATH_DB";

        readonly SortedList<string, NavPath> DB = new SortedList<string, NavPath>();

        public void Clear() { DB.Clear(); }
        public bool Contains(string name) { return DB.ContainsKey(name); }
        public void SavePath(NavPath path) { DB[path.Name] = path.Copy(); }
        public bool Remove(string name) { return DB.Remove(name); }
        public int Count { get { return DB.Count; } }
        public bool Empty { get { return DB.Count == 0; } }
        public IList<string> Names { get { return DB.Keys; } }

        public NavPath GetPath(string name)
        {
            NavPath path;
            return DB.TryGetValue(name, out path)? path.Copy() : null;
        }

        public override void Load(ConfigNode node)
        {
            DB.Clear();
            base.Load(node);
            foreach(var n in node.GetNodes(NavPath.NODE_NAME))
            {
                var path = new NavPath();
                path.Load(n);
                DB[path.Name] = path;
            }
        }

        public override void Save(ConfigNode node)
        {
            base.Save(node);
            DB.ForEach(p => p.Value.Save(node.AddNode(NavPath.NODE_NAME)));
        }
    }
}

