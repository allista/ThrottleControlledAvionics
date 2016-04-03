//  Author:
//       Allis Tauri <allista@gmail.com>
//
//  Copyright (c) 2015 Allis Tauri
//
// This work is licensed under the Creative Commons Attribution-ShareAlike 4.0 International License. 
// To view a copy of this license, visit http://creativecommons.org/licenses/by-sa/4.0/ 
// or send a letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.

using System;
using System.Text.RegularExpressions;
using UnityEngine;

namespace ThrottleControlledAvionics
{
	public static class Utils
	{
		public const double TwoPI = 6.2831853;

		/// <summary>
		/// The camel case components matching regexp.
		/// From: http://stackoverflow.com/questions/155303/net-how-can-you-split-a-caps-delimited-string-into-an-array
		/// </summary>
		const string CamelCaseRegexp = "([a-z](?=[A-Z])|[A-Z](?=[A-Z][a-z]))";
		static Regex CCR = new Regex(CamelCaseRegexp);
		public static string ParseCamelCase(string s) { return CCR.Replace(s, "$1 "); }

		#region Logging
		public static string Format(string s, params object[] args)
		{
			if(args == null || args.Length == 0) return s;
			convert_args(args);
			for(int i = 0, argsLength = args.Length; i < argsLength; i++)
			{
				var ind = s.IndexOf("{}"); 
				if(ind >= 0) s = s.Substring(0, ind)+"{"+i+"}"+s.Substring(ind+2);
				else s += string.Format(" arg{0}: {{{0}}}", i);
			}
			return string.Format(s.Replace("{}", "[no arg]"), args);
		}

		public static string formatVector(Vector3 v)
		{ return string.Format("({0}, {1}, {2}); |v| = {3}", v.x, v.y, v.z, v.magnitude); }

		public static string formatVector(Vector3d v)
		{ return string.Format("({0}, {1}, {2}); |v| = {3}", v.x, v.y, v.z, v.magnitude); }

		public static string formatOrbit(Orbit o)
		{
			return string.Format(
				"Body R: {0} m\n" +
				"PeR:    {1} m\n" +
				"ApR:    {2} m\n" +
				"Ecc:    {3}\n" +
				"Inc:    {4} deg\n" +
				"Period: {5} s\n" +
				"Vel: {6} m/s\n",
				o.referenceBody.Radius, o.PeR, o.ApR, 
				o.eccentricity, o.inclination, o.period, 
				formatVector(o.vel));
		}

		static void convert_args(object[] args)
		{
			for(int i = 0, argsL = args.Length; i < argsL; i++) 
			{
				var arg = args[i];
				if(arg is Vector3) args[i] = formatVector((Vector3)arg);
				else if(arg is Vector3d) args[i] = formatVector((Vector3d)arg);
				else if(arg is Orbit) args[i] = formatOrbit((Orbit)arg);
				else if(arg == null) args[i] = "null";
				else args[i] = arg.ToString();
			}
		}

		public static void Log(string msg, params object[] args)
		{ 
			
			msg = string.Format("[TCA: {0:HH:mm:ss.fff}] {1}", DateTime.Now, msg);
			if(args.Length > 0)
			{
				convert_args(args);
				Debug.Log(string.Format(msg, args)); 
			}
			else Debug.Log(msg);
		}

		public static void Log(this MonoBehaviour mb, string msg, params object[] args)
		{ Utils.Log(string.Format("{0}: {1}", mb.name, msg), args); }

		public static void Log(this Vessel v, string msg, params object[] args)
		{ Utils.Log(string.Format("{0}: {1}", v.vesselName, msg), args); }

		public static void Log(this PartModule pm, string msg, params object[] args)
		{ 
			var vn = pm.vessel == null? "_vessel" : (string.IsNullOrEmpty(pm.vessel.vesselName)? pm.vessel.id.ToString() : pm.vessel.vesselName);
			Utils.Log(string.Format("{0}:{1}:{2}: {3}", vn, pm.part == null? "_part" : pm.part.Title(), pm.moduleName, msg), args); 
		}

		public static void LogF(string msg, params object[] args) { Log(Utils.Format(msg, args)); }
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

		public static float ClampAngle(float a) { a = a%360; return a < 0? 360+a : a; }
		public static double ClampAngle(double a) { a = a%360; return a < 0? 360+a : a; }

		public static float CenterAngle(float a) { return a > 180? a-360 : a; }
		public static double CenterAngle(double a) { return a > 180? a-360 : a; }

		public static float EWA(float old, float cur, float ratio = 0.7f)
		{ return (1-ratio)*old + ratio*cur; }

		public static Vector3 EWA(Vector3 old, Vector3 cur, float ratio = 0.7f)
		{ return (1-ratio)*old + ratio*cur; }

		public static double Haversine(double a) { return (1-Math.Cos(a))/2; }

		/// <summary>
		/// Returns the angle (in degrees) between a radial vector A and the projection 
		/// of a radial vector B on a plane defined by A and tangetA.
		/// The tangentA vector also defines the positive direction from A to B, so 
		/// the returned angle lies in the [-180, 180] interval.
		/// </summary>
		/// <param name="A">Radial vector A.</param>
		/// <param name="B">Radial vector B.</param>
		/// <param name="tangentA">Tangent vector to A.</param>
		public static double ProjectionAngle(Vector3d A, Vector3d B, Vector3d tangentA)
		{
			var Am = A.magnitude;
			var Ba = Vector3d.Dot(B, A)/Am;
			var Bt = Vector3d.Dot(B, Vector3d.Exclude(A, tangentA).normalized);
			return Math.Atan2(Bt, Ba)*Mathf.Rad2Deg;
		}
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

		public static bool ButtonSwitch(string name, bool current_value, string tooltip = "", params GUILayoutOption[] options)
		{
			return string.IsNullOrEmpty(tooltip)? 
				GUILayout.Button(name, current_value ? Styles.green_button : Styles.yellow_button, options) : 
				GUILayout.Button(new GUIContent(name, tooltip), current_value ? Styles.green_button : Styles.yellow_button, options);
		}

		public static bool ButtonSwitch(string name, ref bool current_value, string tooltip = "", params GUILayoutOption[] options)
		{
			var ret = ButtonSwitch(name, current_value, tooltip, options);
			if(ret) current_value = !current_value;
			return ret;
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
			if(body.pqsController == null) return 0;
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
					var alt = body.pqsController.GetSurfaceHeight(relSurfacePosition);
					if(body.ocean && alt < body.Radius) alt = body.Radius;
					error = Math.Abs(curRadius - alt);
					if(error < (body.pqsController.radiusMax - body.pqsController.radiusMin) / 100)
					{
						var surfacePoint = body.position + relSurfacePosition;
						return new Coordinates(body.GetLatitude(surfacePoint), body.GetLongitude(surfacePoint));
					}
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

		public bool UpdateValue(float cvalue)
		{
			if(float.TryParse(svalue, out Value)) return true;
			Value = cvalue;
			return false;
		}

		public bool Draw(float cvalue, string suffix = "", bool show_set_button = true)
		{
			if(string.IsNullOrEmpty(svalue) || !Value.Equals(cvalue))
			{
				Value = cvalue;
				svalue = Value.ToString("F1");
			}
			svalue = GUILayout.TextField(svalue, GUILayout.ExpandWidth(true), GUILayout.MinWidth(70));
			if(!string.IsNullOrEmpty(suffix)) GUILayout.Label(suffix, Styles.label, GUILayout.ExpandWidth(false));
			return 
				show_set_button && 
				GUILayout.Button("Set", Styles.normal_button, GUILayout.Width(50)) && 
				UpdateValue(cvalue);
		}
	}

	//adapted from MechJeb
	public class Coordinates
	{
		public double Lat, Lon;
		public Coordinates(double lat, double lon) 
		{ Lat = Utils.ClampAngle(lat); Lon = Utils.ClampAngle(lon); }
		public Coordinates(Vessel vsl) : this(vsl.latitude, vsl.longitude) {}

		public static string AngleToDMS(double angle)
		{
			var d = (int)Math.Floor(Math.Abs(angle));
			var m = (int)Math.Floor(60 * (Math.Abs(angle) - d));
			var s = (int)Math.Floor(3600 * (Math.Abs(angle) - d - m / 60.0));
			return String.Format("{0:0}°{1:00}'{2:00}\"", Math.Sign(angle)*d, m, s);
		}

		public override string ToString()
		{ return string.Format("Lat: {0} Lon: {1}", AngleToDMS(Lat), AngleToDMS(Lon)); }
	}

	#region Timers
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
	#endregion

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

	public class SingleAction
	{
		bool done;
		public Action action;

		public void Run() 
		{ if(!done) { action(); done = true; } }

		public void Run(Action act) 
		{ if(!done) { act(); done = true; } }

		public void Reset() { done = false; }

		public static implicit operator bool(SingleAction a) { return a.done; }
	}

	public abstract class Extremum<T> where T : IComparable
	{
		protected T v2, v1, v0;
		protected int i;

		public void Update(T cur) { v2 = v1; v1 = v0; v0 = cur; if(i < 3) i++; }
		public abstract bool True { get; }
		public void Reset() { v2 = v1 = v0 = default(T); i = 0; }
	}

	public class FloatMinimum: Extremum<float>
	{ public override bool True { get { return i > 2 && v2 >= v1 && v1 < v0; } } }

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

		public void Add(int i, int j, float v) { e[i, j] += v; }

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