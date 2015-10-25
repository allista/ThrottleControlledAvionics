/* Name: Throttle Controlled Avionics, Fork by Allis Tauri
 *
 * Authors: Quinten Feys & Willem van Vliet & Allis Tauri
 * License: BY: Attribution-ShareAlike 3.0 Unported (CC BY-SA 3.0): 
 * http://creativecommons.org/licenses/by-sa/3.0/
 * 
 */

using System;
using System.Collections.Generic;
using System.Text.RegularExpressions;
using System.Reflection;
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

		public static void Log(this MonoBehaviour mb, string msg, params object[] args)
		{ Utils.Log(string.Format("{0}: {1}", mb.name, msg), args); }

		public static void Log(this PartModule pm, string msg, params object[] args)
		{ 
			var vn = pm.vessel == null? "_vessel" : (string.IsNullOrEmpty(pm.vessel.vesselName)? pm.vessel.id.ToString() : pm.vessel.vesselName);
			Utils.Log(string.Format("{0}:{1}:{2}: {3}", vn, pm.part == null? "_part" : pm.part.Title(), pm.moduleName, msg), args); 
		}
		#endregion

		#region Math
		public static float Clamp(float x, float low, float high)  
		{ return x < low ? low : (x > high? high : x); }

		public static double Clamp(double x, double low, double high)  
		{ return x < low ? low : (x > high? high : x); }

		public static int Clamp(int x, int low, int high)  
		{ return x < low ? low : (x > high? high : x); }

		public static float ClampL(float x, float low)  { return x < low  ? low  : x;  }
		public static double ClampL(double x, double low)  { return x < low  ? low  : x;  }

		public static float ClampH(float x, float high) { return x > high ? high : x;  }
		public static double ClampH(double x, double high) { return x > high ? high : x;  }

		public static int ClampL(int x, int low)  { return x < low  ? low  : x; }
		public static int ClampH(int x, int high) { return x > high ? high : x; }

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

		#region GUI
		public static float FloatSlider(string name, float value, float min, float max, string format="F1", int label_width = -1, string tooltip = "")
		{
			var label = name.Length > 0? string.Format("{0}: {1}", name, value.ToString(format)) : value.ToString(format);
			GUILayout.Label(new GUIContent(label, tooltip), label_width > 0? GUILayout.Width(label_width) : GUILayout.ExpandWidth(false));
			return GUILayout.HorizontalSlider(value, min, max, GUILayout.ExpandWidth(true));
		}

		public static int IntSelector(int value, int min, int max=int.MaxValue, string format="D", string tooltip = "")
		{
			if(GUILayout.Button("<", Styles.normal_button, GUILayout.Width(15)))
			{ if(value >= min) value--; }
			GUILayout.Label(new GUIContent(value < min? "Off" : value.ToString(format), tooltip), 
			                GUILayout.Width(20));
			if(GUILayout.Button(">", Styles.normal_button, GUILayout.Width(15)))
			{ if(value <= max) value++; }
			return value;
		}
		#endregion

		//from http://stackoverflow.com/questions/716399/c-sharp-how-do-you-get-a-variables-name-as-it-was-physically-typed-in-its-dec
		//second answer
		public static string PropertyName<T>(T obj) { return typeof(T).GetProperties()[0].Name; }

		//ResearchAndDevelopment.PartModelPurchased is broken and always returns 'true'
		public static bool PartIsPurchased(string name)
		{
			var info = PartLoader.getPartInfoByName(name);
			if(info == null || HighLogic.CurrentGame == null) return false;
			if(HighLogic.CurrentGame.Mode != Game.Modes.CAREER) return true;
			var tech = ResearchAndDevelopment.Instance.GetTechState(info.TechRequired);
			return tech != null && tech.state == RDTech.State.Available && tech.partsPurchased.Contains(info);
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

		static Coordinates SearchCoordinates(CelestialBody body, Ray mouseRay)
		{
			Vector3d relSurfacePosition;
			Vector3d relOrigin = mouseRay.origin - body.position;
			double curRadius = body.pqsController.radiusMax;
			double lastRadius = 0;
			double error = 0;
			int loops = 0;
			float st = Time.time;
			while(loops < 50)
			{
				if(PQS.LineSphereIntersection(relOrigin, mouseRay.direction, curRadius, out relSurfacePosition))
				{
					var surfacePoint = body.position + relSurfacePosition;
					var alt = TerrainAltitude(body, body.GetLongitude(surfacePoint), body.GetLatitude(surfacePoint))+body.Radius;
					error = Math.Abs(curRadius - alt);
					if(error < (body.pqsController.radiusMax - body.pqsController.radiusMin) / 100)
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
					if(loops == 0) break;
					else
					{ // Went too low, needs to try higher
						curRadius = (lastRadius * 9 + curRadius) / 10;
						loops++;
					}
				}
			}
			return null;
		}

		public static Coordinates GetMouseCoordinates(CelestialBody body)
		{
			var mouseRay = PlanetariumCamera.Camera.ScreenPointToRay(Input.mousePosition);
			mouseRay.origin = ScaledSpace.ScaledToLocalSpace(mouseRay.origin);
			return SearchCoordinates(body, mouseRay);
		}

		public static Coordinates GetMouseFlightCoordinates()
		{
			var body = FlightGlobals.currentMainBody;
			var mouseRay = FlightCamera.fetch.mainCamera.ScreenPointToRay(Input.mousePosition);
			RaycastHit raycast;
			return Physics.Raycast(mouseRay, out raycast, (float)body.Radius * 4f, 1 << 15)? 
				new Coordinates(body.GetLatitude(raycast.point), CenterAngle(body.GetLongitude(raycast.point))) : 
				SearchCoordinates(body, mouseRay);
		}
		#endregion

		#region ControlLock
		//modified from Kerbal Alarm Clock mod
		public static void LockEditor(string LockName, bool Lock=true)
		{
			if(Lock && InputLockManager.GetControlLock(LockName) != ControlTypes.EDITOR_LOCK)
			{
				#if DEBUG
				Log("AddingLock: {0}", LockName);
				#endif
				InputLockManager.SetControlLock(ControlTypes.EDITOR_LOCK, LockName);
				return;
			}
			if(!Lock && InputLockManager.GetControlLock(LockName) == ControlTypes.EDITOR_LOCK) 
			{
				#if DEBUG
				Log("RemovingLock: {0}", LockName);
				#endif
				InputLockManager.RemoveControlLock(LockName);
			}
		}

		public static void LockIfMouseOver(string LockName, Rect WindowRect, bool Lock=true)
		{
			Lock &= WindowRect.Contains(Event.current.mousePosition);
			LockEditor(LockName, Lock);
		}
		#endregion
	}

	public class FloatField
	{
		string svalue;
		public float Value;

		public bool Draw(float cvalue)
		{
			if(string.IsNullOrEmpty(svalue) || !Value.Equals(cvalue))
			{
				Value = cvalue;
				svalue = Value.ToString("F1");
			}
			svalue = GUILayout.TextField(svalue, GUILayout.ExpandWidth(true), GUILayout.MinWidth(70));
			if(GUILayout.Button("Set", Styles.normal_button, GUILayout.Width(50))) 
			{
				if(float.TryParse(svalue, out Value)) return true;
				Value = cvalue;
			}
			return false;
		}
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
		protected DateTime next_time;
		public double Period;

		public TimerBase(double period) { Period = period; next_time = DateTime.MinValue; }
		public void Reset() { next_time = DateTime.MinValue; }

		public override string ToString()
		{
			var time = DateTime.Now;
			return string.Format("time: {0} < next time {1}: {2}", 
			                     time, next_time, time < next_time);
		}
	}

	public class ActionDamper : TimerBase
	{
		public ActionDamper(double period = 0.1) : base(period) {}

		public void Run(Action action)
		{
			var time = DateTime.Now;
			if(next_time > time) return;
			next_time = time.AddSeconds(Period);
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
				var time = DateTime.Now;
				if(next_time == DateTime.MinValue)
				{
					next_time = time.AddSeconds(Period); 
					return false;
				}
				return next_time < time;
			}
		}

		public void RunIf(Action action, bool predicate)
		{
			if(predicate) { if(Check) { action(); Reset(); } }
			else Reset();
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

	public class Switch
	{
		bool state, prev_state;

		public void Set(bool s)
		{ prev_state = state; state = s; }

		public bool WasSet { get { return state != prev_state; } }
		public bool On { get { return state; } }
		public void Checked() { prev_state = state; }

		public static implicit operator bool(Switch s) { return s.state; }
	}

	public static class ComponentDB<T> where T : class, new()
	{
		public static TT Create<TT>() where TT : T, new() { return new TT(); }

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
					foreach(var t in Assembly.GetCallingAssembly().GetTypes())
					{
						if(t == typeof(T) || t.IsSubclassOf(typeof(T)))
						{
							var constInfo = creator.MakeGenericMethod(t);
							if(constInfo == null) continue;
							components.Add(Utils.ParseCamelCase(t.Name), 
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
			scroll = GUILayout.BeginScrollView(scroll, Styles.white, GUILayout.Height(110));
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