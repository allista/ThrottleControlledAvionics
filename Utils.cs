/* Name: Throttle Controlled Avionics, Fork by Allis Tauri
 *
 * Authors: Quinten Feys & Willem van Vliet & Allis Tauri
 * License: BY: Attribution-ShareAlike 3.0 Unported (CC BY-SA 3.0): 
 * http://creativecommons.org/licenses/by-sa/3.0/
 * 
 */

using System;
using System.Reflection;
using System.Collections.Generic;
using System.Text.RegularExpressions;
using UnityEngine;

namespace ThrottleControlledAvionics
{
	public static class Utils
	{
		/// <summary>
		/// The camel case components matching regexp.
		/// From: http://stackoverflow.com/questions/155303/net-how-can-you-split-a-caps-delimited-string-into-an-array
		/// </summary>
		const string CamelCaseRegexp = "([a-z](?=[A-Z])|[A-Z](?=[A-Z][a-z]))";

		public static string ParseCamelCase(string s) { return Regex.Replace(s, CamelCaseRegexp, "$1 "); }

		public static string formatVector(Vector3 v)
		{ return string.Format("({0}, {1}, {2}); |v| = {3}", v.x, v.y, v.z, v.magnitude); }

		public static string formatVector(Vector3d v)
		{ return string.Format("({0}, {1}, {2}); |v| = {3}", v.x, v.y, v.z, v.magnitude); }

		public static void Log(string msg, params object[] args)
		{ 
			for(int i = 0; i < args.Length; i++) 
			{
				if(args[i] is Vector3) args[i] = formatVector((Vector3)args[i]);
				else if(args[i] is Vector3d) args[i] = formatVector((Vector3d)args[i]);
				else if(args[i] == null) args[i] = "null";
			}
			Debug.Log(string.Format("[TCA] "+msg, args)); 
		}

		public static void CSV(params object[] args)
		{ 
			var row = "tag: ";
			for(int i = 0; i < args.Length-1; i++) 
			{ row += "{"+i+"}, "; }
			row += "{"+(args.Length-1)+"}\n";
			Log(row, args);
		}

		public static float Asymptote01(float x, float k=1) { return 1-1/(x/k+1); }
		public static float ClampL(float x, float low)  { return x < low  ? low  : x;  }
		public static float ClampH(float x, float high) { return x > high ? high : x;  }
		public static float CenterAngle(float a) { return a > 180? a-360 : a; }

		public static float WAverage(float old, float cur, float ratio = 0.7f)
		{ return (1-ratio)*old + ratio*cur; }

		public static Vector3 WAverage(Vector3 old, Vector3 cur, float ratio = 0.7f)
		{ return (1-ratio)*old + ratio*cur; }

		#region Vector3
		public static Vector3 Inverse(this Vector3 v) { return new Vector3(1f/v.x, 1f/v.y, 1f/v.z); }

		public static Vector3 ClampComponents(this Vector3 v, float min, float max) 
		{ 
			return new Vector3(Mathf.Clamp(v.x, min, max), 
			                   Mathf.Clamp(v.y, min, max), 
			                   Mathf.Clamp(v.z, min, max)); 
		}

		public static Vector3 Sign(this Vector3 v)
		{ return new Vector3(Mathf.Sign(v.x), Mathf.Sign(v.y), Mathf.Sign(v.z)); }

		public static Vector3 AbsComponents(this Vector3 v)
		{ return new Vector3(Mathf.Abs(v.x), Mathf.Abs(v.y), Mathf.Abs(v.z)); }
		#endregion

		public static void CheckRect(ref Rect R)
		{
			//check size
			if(R.width > Screen.width) R.width = Screen.width;
			if(R.height > Screen.height) R.height = Screen.height;
			//check position
			if(R.xMin < 0) R.x -= R.xMin;
			else if(R.xMax > Screen.width) R.x -= R.xMax-Screen.width;
			if(R.yMin < 0) R.y -= R.yMin;
			else if(R.yMax > Screen.height) R.y -= R.yMax-Screen.height;
		}

		//from http://stackoverflow.com/questions/716399/c-sharp-how-do-you-get-a-variables-name-as-it-was-physically-typed-in-its-dec
		//second answer
		public static string PropertyName<T>(T obj) { return typeof(T).GetProperties()[0].Name; }

		public static float FloatSlider(string name, float value, float min, float max, string format="F1")
		{
			var label = name.Length > 0? string.Format("{0}: {1}", name, value.ToString(format)) : value.ToString(format);
			GUILayout.Label(label, GUILayout.ExpandWidth(false));
			return GUILayout.HorizontalSlider(value, min, max, GUILayout.ExpandWidth(true));
		}

		//ResearchAndDevelopment.PartModelPurchased is broken and always returns 'true'
		public static bool PartIsPurchased(string name)
		{
			var info = PartLoader.getPartInfoByName(name);
			if(info == null) return false;
			if(HighLogic.CurrentGame.Mode != Game.Modes.CAREER) return true;
			var tech = ResearchAndDevelopment.Instance.GetTechState(info.TechRequired);
			return tech != null && tech.state == RDTech.State.Available && tech.partsPurchased.Contains(info);
		}

		public static void ForEach<T>(this IEnumerable<T> E, Action<T> action)
		{
			var en = E.GetEnumerator();
			while(en.MoveNext()) action(en.Current);
		}
	}

	public class ConfigNodeObject : IConfigNode
	{
		public const string NODE_NAME = "NODE";

		static readonly string iface_name = typeof(IConfigNode).Name;
		static readonly Type cnode_type = typeof(ConfigNode);

		virtual public void Load(ConfigNode node)
		{ 
			ConfigNode.LoadObjectFromConfig(this, node);
			foreach(var fi in GetType().GetFields())
			{
				if(fi.FieldType.GetInterface(iface_name) == null) continue;
				var n = node.GetNode(fi.Name);
				if(n == null) continue;
				var method = fi.FieldType.GetMethod("Load", new [] {cnode_type});
				if(method == null) continue;
				method.Invoke(fi.GetValue(this), new [] {n});
			}
		}

		virtual public void Save(ConfigNode node)
		{ 
			ConfigNode.CreateConfigFromObject(this, node); 
			foreach(var fi in GetType().GetFields())
			{
				if(fi.FieldType.GetInterface(iface_name) == null) continue;
				var method = fi.FieldType.GetMethod("Save", new [] {cnode_type});
				if(method == null) continue;
				var n = node.GetNode(fi.Name);
				if(n == null) n = node.AddNode(fi.Name);
				else n.ClearData();
				method.Invoke(fi.GetValue(this), new [] {n});
			}
		}
	}

	public abstract class PersistentDict<K, V> : ConfigNodeObject
	{
		readonly Dictionary<K, V> dict = new Dictionary<K, V>();

		protected abstract K parseK(string k);
		protected abstract V parseV(string v);

		public override void Save(ConfigNode node)
		{
			base.Save(node);
			foreach(var i in dict)
				node.AddValue(i.Key.ToString(), i.Value.ToString());
		}

		public override void Load(ConfigNode node)
		{
			base.Load(node);
			foreach(ConfigNode.Value v in node.values)
				dict[parseK(v.name)] = parseV(v.value);
		}

		#region dict interface
		public V this[K k] { get { return dict[k]; } set { dict[k] = value; } }
		public bool TryGetValue(K k, out V v) { return dict.TryGetValue(k, out v); }

		public void Add(K k, V v) { dict.Add(k, v); }
		public bool Remove(K k) { return dict.Remove(k); }

		public bool ContainsKey(K k) { return dict.ContainsKey(k); }
		public bool ContainsValue(V v) { return dict.ContainsValue(v); }

		public int  Count { get { return dict.Count; } }
		#endregion
	}

	public abstract class PDictKFloat<K> : PersistentDict<K, float>
	{
		protected override float parseV(string v)
		{
			var V = 0f;
			return float.TryParse (v, out V)? V : 0f;
		}
	}

	public class PDictIntFloat : PDictKFloat<int>
	{
		protected override int parseK(string k)
		{
			var K = 0;
			return int.TryParse (k, out K)? K : 0;
		}
	}

	public class PDictUIntFloat : PDictKFloat<uint>
	{
		protected override uint parseK(string k)
		{
			uint K = 0;
			return uint.TryParse (k, out K)? K : 0;
		}
	}

	//convergent with Anatid's Vector6, but not taken from it
	public class Vector6 
	{
		public Vector3 positive = Vector3.zero, negative = Vector3.zero;

		public static Vector6 operator+(Vector6 first, Vector6 second)
		{ 
			var sum = new Vector6();
			sum.positive = first.positive+second.positive; 
			sum.negative = first.negative+second.negative; 
			return sum;
		}

		public void Add(Vector6 vec)
		{
			positive += vec.positive; 
			negative += vec.negative; 
		}

		public void Add(Vector3 vec)
		{
			for(int i = 0; i < 3; i++)
			{
				if(vec[i] >= 0) positive[i] = positive[i]+vec[i];
				else negative[i] = negative[i]+vec[i];
			}
		}

		public void Add(List<Vector3> vecs) { vecs.ForEach(Add); }

		public Vector3 Clamp(Vector3 vec)
		{
			var cvec = Vector3.zero;
			for(int i = 0; i < 3; i++)
				cvec[i] = vec[i] >= 0 ? 
					Mathf.Min(positive[i], vec[i]) : 
					Mathf.Max(negative[i], vec[i]);
			return cvec;
		}

		public Vector3 Max
		{
			get
			{
				var mvec = Vector3.zero;
				for(int i = 0; i < 3; i++)
					mvec[i] = Mathf.Max(-negative[i], positive[i]);
				return mvec;
			}
		}
	}

	//from MechJeb2
	public class Matrix3x3f
	{
		//row index, then column index
		float[,] e = new float[3, 3];
		public float this[int i, int j]
		{
			get { return e[i, j]; }
			set { e[i, j] = value; }
		}

		public Matrix3x3f transpose()
		{
			var ret = new Matrix3x3f();
			for (int i = 0; i < 3; i++)
			{
				for (int j = 0; j < 3; j++)
					ret.e[i, j] = e[j, i];
			}
			return ret;
		}

		public static Vector3 operator *(Matrix3x3f M, Vector3 v)
		{
			Vector3 ret = Vector3.zero;
			for(int i = 0; i < 3; i++) {
				for(int j = 0; j < 3; j++) {
					ret[i] += M.e[i, j] * v[j];
				}
			}
			return ret;
		}
	}

	#if DEBUG
	public class DebugCounter
	{
		int count = 0;
		string name = "";
		public DebugCounter(string name = "Debug") { this.name = name; }
		public void Log() { Utils.Log("{0}: {1}", name, count++); }
		public void Reset() { count = 0; }
	}
	#endif
}