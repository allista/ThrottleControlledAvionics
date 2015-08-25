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

		#region Logging
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
		#endregion

		#region Math
		public static float Asymptote01(float x, float k=1) { return 1-1/(x/k+1); }

		public static float ClampL(float x, float low)  { return x < low  ? low  : x;  }
		public static double ClampL(double x, double low)  { return x < low  ? low  : x;  }

		public static float ClampH(float x, float high) { return x > high ? high : x;  }
		public static double ClampH(double x, double high) { return x > high ? high : x;  }

		public static float CenterAngle(float a) { return a > 180? a-360 : a; }
		public static double CenterAngle(double a) { return a > 180? a-360 : a; }

		public static float EWA(float old, float cur, float ratio = 0.7f)
		{ return (1-ratio)*old + ratio*cur; }

		public static float Gauss(float old, float cur, float ratio = 0.7f, int poles = 2)
		{ 
			for(int i = 0; i < poles; i++) 
				old = (1-ratio)*old + ratio*cur; 
			return old;
		}

		public static Vector3 EWA(Vector3 old, Vector3 cur, float ratio = 0.7f)
		{ return (1-ratio)*old + ratio*cur; }

		public static double Haversine(double a) { return (1-Math.Cos(a))/2; }
		#endregion

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

		#region GUI
		public static float FloatSlider(string name, float value, float min, float max, string format="F1", int label_width = -1)
		{
			var label = name.Length > 0? string.Format("{0}: {1}", name, value.ToString(format)) : value.ToString(format);
			GUILayout.Label(label, label_width > 0? GUILayout.Width(label_width) : GUILayout.ExpandWidth(false));
			return GUILayout.HorizontalSlider(value, min, max, GUILayout.ExpandWidth(true));
		}
		#endregion

		//from http://stackoverflow.com/questions/716399/c-sharp-how-do-you-get-a-variables-name-as-it-was-physically-typed-in-its-dec
		//second answer
		public static string PropertyName<T>(T obj) { return typeof(T).GetProperties()[0].Name; }

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

		public static Vector3[] BoundCorners(Bounds b)
		{
			var edges = new Vector3[8];
			Vector3 min = b.min;
			Vector3 max = b.max;
			edges[0] = new Vector3(min.x, min.y, min.z); //left-bottom-back
			edges[1] = new Vector3(min.x, min.y, max.z); //left-bottom-front
			edges[2] = new Vector3(min.x, max.y, min.z); //left-top-back
			edges[3] = new Vector3(min.x, max.y, max.z); //left-top-front
			edges[4] = new Vector3(max.x, min.y, min.z); //right-bottom-back
			edges[5] = new Vector3(max.x, min.y, max.z); //right-bottom-front
			edges[6] = new Vector3(max.x, max.y, min.z); //right-top-back
			edges[7] = new Vector3(max.x, max.y, max.z); //right-top-front
			return edges;
		}

		public static string DistanceToStr(double d)
		{
			var k = d/1000;
			return k < 1? string.Format("{0:F0}m", d) : string.Format("{0:F1}km", k);
		}

		#region From MechJeb
		public static double TerrainAltitude(CelestialBody body, double Lat, double Lon)
		{
			if (body.pqsController == null) return 0;
			//			var pqsRadialVector = QuaternionD.AngleAxis(longitude, Vector3d.down) * QuaternionD.AngleAxis(latitude, Vector3d.forward) * Vector3d.right;
			var alt = body.pqsController.GetSurfaceHeight(body.GetRelSurfaceNVector(Lat, Lon)) - body.pqsController.radius;
			return body.ocean && alt < 0? 0 : alt;
		}

		public static Vector2 GetMousePosition(Rect window) 
		{
			var mouse_pos = Input.mousePosition;
			return new Vector2(mouse_pos.x-window.x, Screen.height-mouse_pos.y-window.y).clampToScreen();
		}

		public static Coordinates GetMouseCoordinates(CelestialBody body)
		{
			var mouseRay = PlanetariumCamera.Camera.ScreenPointToRay(Input.mousePosition);
			mouseRay.origin = ScaledSpace.ScaledToLocalSpace(mouseRay.origin);
			Vector3d relOrigin = mouseRay.origin - body.position;
			Vector3d relSurfacePosition;
			double curRadius = body.pqsController.radiusMax;
			double lastRadius = 0;
			double error = 0;
			double threshold = (body.pqsController.radiusMax - body.pqsController.radiusMin)/100;
			int loops = 0;
			while(loops < 50)
			{
				if(PQS.LineSphereIntersection(relOrigin, mouseRay.direction, curRadius, out relSurfacePosition))
				{
					var surfacePoint = body.position + relSurfacePosition;
					var alt = TerrainAltitude(body, body.GetLongitude(surfacePoint), body.GetLatitude(surfacePoint))+body.Radius;
					error = Math.Abs(curRadius - alt);
					if(error < threshold)
						return new Coordinates(body.GetLatitude(surfacePoint), body.GetLongitude(surfacePoint));
					else
					{
						lastRadius = curRadius;
						curRadius = alt;
						loops++;
					}
				}
				else
				{
					if(loops > 0)
					{ // Went too low, needs to try higher
						curRadius = (lastRadius * 9 + curRadius) / 10;
						loops++;
					} else break;
				}
			}
			return null;
		}
		#endregion
	}

	//adapted from MechJeb
	public class Coordinates
	{
		public double Lat, Lon;
		public Coordinates(double lat, double lon) { Lat = lat; Lon = lon; }
		public static string AngleToDMS(double angle)
		{
			var d = (int)Math.Floor(Math.Abs(angle));
			var m = (int)Math.Floor(60 * (Math.Abs(angle) - d));
			var s = (int)Math.Floor(3600 * (Math.Abs(angle) - d - m / 60.0));
			return String.Format("{0:0}°{1:00}'{2:00}\"", d, m, s);
		}
		public override string ToString()
		{ return string.Format("Lat: {0} Lon: {1}", AngleToDMS(Lat), AngleToDMS(Lon)); }
	}

	public class TimerBase
	{
		protected double next_time;
		public double Period;

		public TimerBase(double period) { Period = period; next_time = -1; }
		public void Reset() { next_time = -1; }

		public override string ToString()
		{
			return string.Format("time: {0} < next time {1}: {2}", 
			                     Planetarium.GetUniversalTime(), next_time, Planetarium.GetUniversalTime() < next_time);
		}
	}

	public class ActionDamper : TimerBase
	{
		public ActionDamper(double period = 0.1) : base(period) {}

		public void Run(Action action)
		{
			var time = Planetarium.GetUniversalTime();
			if(next_time > time) return;
			next_time = time+Period;
			action();
		}
	}

	public class Timer : TimerBase
	{
		public Timer(double period = 1) : base(period) {}

		public bool Check
		{
			get 
			{
				if(next_time < 0)
				{
					next_time = Planetarium.GetUniversalTime()+Period; 
					return false;
				}
				else return next_time < Planetarium.GetUniversalTime();
			}
		}
	}

	public class TimeAverage
	{
		public float dT;
		protected int n;
		protected float t, val, new_val;
		public float Value { get; protected set; }
		public float Speed { get; protected set; }
		public TimeAverage(float dt = 0.05f) { dT = dt; }

		public bool Update(float v)
		{
			val += v;
			t += TimeWarp.fixedDeltaTime;
			n += 1;
			if(t < dT) return false;
			new_val = val/n;
			Speed = (new_val-Value)/t;
			Value = new_val;
			val = t = n = 0;
			return true;
		}

		public void Reset() { Value = Speed = val = t = n = 0; }

		public static implicit operator float(TimeAverage td) { return td.Value; }
		public override string ToString() { return string.Format("Value: {0}, Speed: {1}", Value, Speed); }
	}

	public class EWA
	{
		float old, value;

		public EWA(float v = 0) { old = v; value = v; }

		public float Update(float v, float ratio = 0.1f)
		{
			value = Utils.EWA(old, v, ratio);
			old = value;
			return value;
		}
		public static implicit operator float(EWA ewa) { return ewa.value; }
		public override string ToString() { return value.ToString(); }
		public string ToString(string F) { return value.ToString(F); }
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
				if(fi.FieldType.GetInterface(iface_name) == null) continue;
				var method = fi.FieldType.GetMethod("Save", new [] {cnode_type});
				if(method == null) continue;
				var f = fi.GetValue(this);
				var n = node.GetNode(fi.Name);
				if(n == null) n = node.AddNode(fi.Name);
				else n.ClearData();
				if(f == null) continue;
				method.Invoke(f, new [] {n});
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

	public class Multiplexer<T> : ConfigNodeObject where T : struct
	{
		[Persistent] public string State;
		public T state;

		readonly Dictionary<T, Action<bool>> callbacks = new Dictionary<T, Action<bool>>();

		public Multiplexer() 
		{ if(!typeof(T).IsEnum) throw new ArgumentException("Multiplexer<T> T must be an enumerated type"); }

		public bool this[T key] 
		{ 
			get { return key.Equals(state); } 
			set 
			{ 
				if(value) On(key);
				else if(key.Equals(state)) Off();
          	}
		}
		public void Toggle(T key) { this[key] = !this[key]; }
		public void OnIfNot(T key) { if(!this[key]) On(key); }
		public void OffIfOn(T key) { if(this[key]) Off(); }
		public void On(T key) 
		{ 
			if(!key.Equals(state)) Off();
			state = key;
			Action<bool> callback;
			if(callbacks.TryGetValue(state, out callback))
			{ if(callback != null) callback(true); }
		}
		public void Off() 
		{ 
			if(state.Equals(default(T))) return;
			var old_state = state; //prevents recursion
			state = default(T);
			Action<bool> callback;
			if(callbacks.TryGetValue(old_state, out callback))
			{ if(callback != null) callback(false); }
		}

		public static implicit operator bool(Multiplexer<T> m) { return !m[default(T)]; }

		public void ClearCallbacks() { callbacks.Clear(); }

		public void AddCallback(T key, Action<bool> callback)
		{
			if(callbacks.ContainsKey(key))
				callbacks[key] += callback;
			else callbacks[key] = callback;
		}

		public override void Load(ConfigNode node)
		{
			base.Load(node);
			try { state = (T)Enum.Parse(typeof(T), State); }
			catch { state = default(T); }
		}

		public override void Save(ConfigNode node)
		{
			State = Enum.GetName(typeof(T), state);
			base.Save(node);
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
}