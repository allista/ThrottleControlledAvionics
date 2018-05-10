//   NavPath.cs
//
//  Author:
//       Allis Tauri <allista@gmail.com>
//
//  Copyright (c) 2016 Allis Tauri

using System.Collections.Generic;
using AT_Utils;

namespace ThrottleControlledAvionics
{
    public class NavPath : Queue<WayPoint>, IConfigNode
    {
        public const string NODE_NAME = "PATH";
        public string Name = "";

        public NavPath Copy()
        {
            var node = new ConfigNode(NODE_NAME);
            var path = new NavPath();
            Save(node);
            path.Load(node);
            return path;
        }

        public virtual void Save(ConfigNode node)
        {
            node.AddValue("Name", Name);
            this.ForEach(wp => wp.SaveInto(node));
        }

        public virtual void Load(ConfigNode node)
        {
            Clear();
            Name = node.GetValue("Name") ?? "";
            foreach(var n in node.GetNodes(WayPoint.NODE_NAME))
            {
                var wp = ConfigNodeObject.FromConfig<WayPoint>(n);
                if(wp != null) Enqueue(wp);
            }
        }
    }
}

