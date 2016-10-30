//   NavPath.cs
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
	public class NavPath : Queue<WayPoint>, IConfigNode
	{
		public const string NODE_NAME = "PATH";
		public string Name = "";

		public void FillFrom(IEnumerable<WayPoint> waypoints)
		{ Clear(); waypoints.ForEach(Enqueue); }

		public NavPath Copy()
		{
			var node = new ConfigNode(NODE_NAME);
			var path = new NavPath();
			Save(node);
			path.Load(node);
			return path;
		}

		public bool Remove(WayPoint waypoint) 
		{ 
			var count = Count;
			var waypoints = this.Where(wp => wp != waypoint).ToList();
			Clear(); waypoints.ForEach(Enqueue);
			return Count != count;
		}

		public bool MoveUp(WayPoint up)
		{
			if(up == Peek()) return false;
			var waypoints = this.ToList();
			var upi = waypoints.IndexOf(up);
			if(upi < 0) return false;
			waypoints[upi] = waypoints[upi-1];
			waypoints[upi-1] = up;
			Clear(); waypoints.ForEach(Enqueue);
			return true;
		}

		public virtual void Save(ConfigNode node)
		{
			node.AddValue("Name", Name);
			this.ForEach(wp => wp.Save(node.AddNode(WayPoint.NODE_NAME)));
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

