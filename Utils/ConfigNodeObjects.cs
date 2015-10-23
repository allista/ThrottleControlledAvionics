//   ConfigNodeObjects.cs
//
//  Author:
//       Allis Tauri <allista@gmail.com>
//
//  Copyright (c) 2015 Allis Tauri
//
// This work is licensed under the Creative Commons Attribution 4.0 International License. 
// To view a copy of this license, visit http://creativecommons.org/licenses/by/4.0/ 
// or send a letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.

using System;
using System.Reflection;
using System.Collections.Generic;

namespace ThrottleControlledAvionics
{
	public class ConfigNodeObject : IConfigNode
	{
		public const string NODE_NAME = "NODE";

		static readonly string cnode_name = typeof(IConfigNode).Name;

		virtual public void Load(ConfigNode node)
		{ 
			ConfigNode.LoadObjectFromConfig(this, node);
			foreach(var fi in GetType().GetFields())
			{
				if(fi.FieldType.GetInterface(cnode_name) == null) continue;
				var n = node.GetNode(fi.Name);
				if(n == null) continue;
				var method = fi.FieldType.GetMethod("Load", new [] {typeof(ConfigNode)});
				if(method == null) continue;
				var f = fi.GetValue(this);
				if(f == null) continue;
				method.Invoke(f, new [] {n});
			}
		}

		virtual public void Save(ConfigNode node)
		{ 
			ConfigNode.CreateConfigFromObject(this, node); 
			foreach(var fi in GetType().GetFields())
			{
				if(fi.FieldType.GetInterface(cnode_name) == null) continue;
				var method = fi.FieldType.GetMethod("Save", new [] {typeof(ConfigNode)});
				if(method == null) continue;
				var f = fi.GetValue(this);
				var n = node.GetNode(fi.Name);
				if(n == null) n = node.AddNode(fi.Name);
				else n.ClearData();
				if(f == null) continue;
				method.Invoke(f, new [] {n});
			}
		}

		public static CNO FromConfig<CNO>(ConfigNode node)
			where CNO : ConfigNodeObject, new()
		{
			var cno = new CNO();
			cno.Load(node);
			return cno;
		}

		public override string ToString()
		{
			var n = new ConfigNode(GetType().Name);
			Save(n);
			return n.ToString();
		}
	}

	public class TypedConfigNodeObject : ConfigNodeObject
	{
		public override void Save(ConfigNode node)
		{
			node.AddValue("type", GetType().Name);
			base.Save(node);
		}

		public override void Load(ConfigNode node)
		{
			base.Load(node);
			foreach(var fi in GetType().GetFields())
			{
				if(fi.FieldType == typeof(TypedConfigNodeObject) || 
				   fi.FieldType.IsSubclassOf(typeof(TypedConfigNodeObject)))
				{
					var n = node.GetNode(fi.Name);
					if(n == null) continue;
					var ctype = Assembly.GetCallingAssembly().GetType(node.GetValue("type"));
					if(ctype == null) continue;
					var cconst = ctype.GetConstructor(null);
					if(cconst == null) continue;
					var obj = cconst.Invoke(null) as TypedConfigNodeObject;
					if(obj != null) obj.Load(node);
					fi.SetValue(this, obj);
				}
			}
		}
	}

	public class PersistentBaseList<T> : ConfigNodeObject where T : TypedConfigNodeObject, new()
	{
		readonly public List<T> List = new List<T>();

		public int Count { get { return List.Count; } }
		public T this[int i]
		{
			get { return List[i]; }
			set { List[i] = value; }
		}
		public void Add(T it) { List.Add(it); }
		public bool Remove(T it) { return List.Remove(it); }
		public int IndexOf(T it) { return List.IndexOf(it); }

		public override void Save(ConfigNode node)
		{
			base.Save(node);
			for(int i = 0, count = List.Count; i < count; i++) 
				List[i].Save(node.AddNode(i.ToString()));
		}

		public override void Load(ConfigNode node)
		{
			base.Load(node);
			foreach(var n in node.GetNodes())
			{
				var it = ConfigNodeObject.FromConfig<T>(n);
				if(it != null) List.Add(it);
			}
		}
	}
}

