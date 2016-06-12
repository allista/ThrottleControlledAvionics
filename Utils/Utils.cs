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
using System.Text.RegularExpressions;
using UnityEngine;

namespace ThrottleControlledAvionics
{
	public static partial class Utils
	{
		public const double TwoPI  = Math.PI*2;
		public const double HalfPI = Math.PI/2;
		public const float  Sin45  = 0.70710678f;
		public const float  G0 = 9.80665f; //m/s2


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
			return Utils.Format(
				"PeA:    {} m\n" +
				"ApA:    {} m\n" +
				"Body R: {} m\n" +
				"PeR:    {} m\n" +
				"ApR:    {} m\n" +
				"Ecc:    {}\n" +
				"Inc:    {} deg\n" +
				"LAN:    {} deg\n" +
				"MA:     {} rad\n" +
				"TA:     {} deg\n" +
				"AoP:    {} deg\n" +
				"Period: {} s\n" +
				"epoch:   {}\n" +
				"T@epoch: {} s\n" +
				"T:       {} s\n" +
				"Vel: {} m/s\n" +
				"Pos: {} m\n",
				o.PeA, o.ApA,
				o.referenceBody.Radius, o.PeR, o.ApR, 
				o.eccentricity, o.inclination, o.LAN, o.meanAnomaly, o.trueAnomaly, o.argumentOfPeriapsis,
				o.period, o.epoch, o.ObTAtEpoch, o.ObT,
				formatVector(o.vel), formatVector(o.pos));
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

		public static float Circle(float a, float min, float max)
		{ if(a > max) a = a%max+min; return a < min? max-min+a : a; }
		public static double Circle(double a, double min, double max)
		{ if(a > max) a = a%max+min; return a < min? max-min+a : a; }

		public static float ClampAngle(float a) { a = a%360; return a < 0? 360+a : a; }
		public static double ClampAngle(double a) { a = a%360; return a < 0? 360+a : a; }

		public static float CenterAngle(float a) { return a > 180? a-360 : a; }
		public static double CenterAngle(double a) { return a > 180? a-360 : a; }

		public static float AngleDelta(float a, float b)
		{
			var d = Utils.CenterAngle(b)-Utils.CenterAngle(a);
			return Mathf.Abs(d) > 180? -Mathf.Sign(d)*(360-Mathf.Abs(d)) : d;
		}

		public static double AngleDelta(double a, double b)
		{
			var d = Utils.CenterAngle(b)-Utils.CenterAngle(a);
			return Math.Abs(d) > 180? -Math.Sign(d)*(360-Math.Abs(d)) : d;
		}

		public static double ClampRad(double a) { a = a%TwoPI; return a < 0? TwoPI+a : a; }
		public static double CenterRad(double a) { return a > Math.PI? a-TwoPI : a; }
		public static double RadDelta(double a, double b)
		{
			var d = Utils.CenterRad(b)-Utils.CenterRad(a);
			return Math.Abs(d) > Math.PI? -Math.Sign(d)*(TwoPI-Math.Abs(d)) : d;
		}

		public static double Acot(double x) { return HalfPI - Math.Atan(x); }

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

		public static float EWA(float old, float cur, float ratio = 0.7f)
		{ return (1-ratio)*old + ratio*cur; }

		public static Vector3 EWA(Vector3 old, Vector3 cur, float ratio = 0.7f)
		{ return (1-ratio)*old + ratio*cur; }
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

		public static double TerrainAltitude(CelestialBody body, double Lat, double Lon)
		{
			if(body.pqsController == null) return 0;
			var alt = body.pqsController.GetSurfaceHeight(body.GetRelSurfaceNVector(Lat, Lon)) - body.pqsController.radius;
			return body.ocean && alt < 0? 0 : alt;
		}

		public static double TerrainAltitude(CelestialBody body, Vector3d wpos)
		{ return TerrainAltitude(body, body.GetLatitude(wpos), body.GetLongitude(wpos)); }

		public static Vector2 GetMousePosition(Rect window) 
		{
			var mouse_pos = Input.mousePosition;
			return new Vector2(mouse_pos.x-window.x, Screen.height-mouse_pos.y-window.y).clampToScreen();
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

		public double Alt(CelestialBody body) { return Utils.TerrainAltitude(body, Lat, Lon); }
		public string Biome(CelestialBody body) { return ScienceUtil.GetExperimentBiome(body, Lat, Lon); }

		//adopted from MechJeb
		static Coordinates Search(CelestialBody body, Ray mouseRay)
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

		public static Coordinates GetAtPointer(CelestialBody body)
		{
			var mouseRay = PlanetariumCamera.Camera.ScreenPointToRay(Input.mousePosition);
			mouseRay.origin = ScaledSpace.ScaledToLocalSpace(mouseRay.origin);
			return Search(body, mouseRay);
		}

		public static Coordinates GetAtPointerInFlight()
		{
			var body = FlightGlobals.currentMainBody;
			var mouseRay = FlightCamera.fetch.mainCamera.ScreenPointToRay(Input.mousePosition);
			RaycastHit raycast;
			return Physics.Raycast(mouseRay, out raycast, (float)body.Radius * 4f, 1 << 15)? 
				new Coordinates(body.GetLatitude(raycast.point), Utils.CenterAngle(body.GetLongitude(raycast.point))) : 
				Search(body, mouseRay);
		}

		public override string ToString()
		{ return string.Format("Lat: {0} Lon: {1}", AngleToDMS(Lat), AngleToDMS(Lon)); }

		public string FullDescription(CelestialBody body)
		{ return string.Format("{0}\nAlt: {1:F0}m {2}", this, Alt(body), Biome(body)); }

		public string FullDescription(Vessel vsl) { return FullDescription(vsl.mainBody);}
	}

	public class ListDict<K,V> : Dictionary<K, List<V>>
	{
		public void Add(K key, V value)
		{
			List<V> lst;
			if(TryGetValue(key, out lst))
				lst.Add(value);
			else this[key] = new List<V>{value};
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
