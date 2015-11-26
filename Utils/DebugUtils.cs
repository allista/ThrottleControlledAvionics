//  Author:
//       Allis Tauri <allista@gmail.com>
//
//  Copyright (c) 2015 Allis Tauri
//
// This work is licensed under the Creative Commons Attribution-ShareAlike 4.0 International License. 
// To view a copy of this license, visit http://creativecommons.org/licenses/by-sa/4.0/ 
// or send a letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.

#if DEBUG
using System;
using System.Linq;
using System.Collections.Generic;
using UnityEngine;

namespace ThrottleControlledAvionics
{
	static class DebugUtils
	{
		public static void CSV(params object[] args)
		{ 
			var row = "tag: ";
			for(int i = 0; i < args.Length-1; i++) 
			{ row += "{"+i+"}, "; }
			row += "{"+(args.Length-1)+"}\n";
			Utils.Log(row, args);
		}

		public static void logVectors(string tag, bool normalize = true, params Vector3[] vecs)
		{
			var s = tag+":\n";
			foreach(var v in vecs)
			{
				var vn = normalize? v.normalized : v;
				s += string.Format("({0}, {1}, {2}),\n", vn.x, vn.y, vn.z);
			}
			Utils.Log(s);
		}

		public static void logBounds(string name, Bounds b)
		{
			Utils.Log("Bounds:  {0}\n" +
			    "Center:  {1}\n" +
			    "Extents: {2}\n" +
			    "Min:     {3}\n" +
			    "Max:     {4}\n" +
			    "Volume:  {5}", 
			    name, b.center, b.extents, b.min, b.max,
			          b.size.x*b.size.y*b.size.z);
		}
	}

	//adapted from MechJeb
	public static class GLUtils
	{
		static Material _material;
		static Material material
		{
			get
			{
				if(_material == null) _material = new Material(Shader.Find("Particles/Additive"));
				return _material;
			}
		}

		static Camera GLBeginWorld(out float far)
		{
			var camera = MapView.MapIsEnabled? PlanetariumCamera.Camera : FlightCamera.fetch.mainCamera;
			far = camera.farClipPlane;
			camera.farClipPlane = far*100;
			GL.PushMatrix();
			material.SetPass(0);
			GL.LoadProjectionMatrix(camera.projectionMatrix);
			GL.modelview = camera.worldToCameraMatrix;
			return camera;
		}

		public static void GLTriangleMap(Vector3d[] worldVertices, Color c)
		{
			float far;
			var camera = GLBeginWorld(out far);
			GL.Begin(GL.TRIANGLES);
			GL.Color(c);
			GL.Vertex(worldVertices[0]);
			GL.Vertex(worldVertices[1]);
			GL.Vertex(worldVertices[2]);
			GL.End();
			GL.PopMatrix();
			camera.farClipPlane = far;
		}

		public static void GLTriangleMap(Vector3[] worldVertices, Color c)
		{
			float far;
			var camera = GLBeginWorld(out far);
			GL.Begin(GL.TRIANGLES);
			GL.Color(c);
			GL.Vertex(worldVertices[0]);
			GL.Vertex(worldVertices[1]);
			GL.Vertex(worldVertices[2]);
			GL.End();
			GL.PopMatrix();
			camera.farClipPlane = far;
		}

		public static void GLLine(Vector3 ori, Vector3 end, Color c)
		{
			float far;
			var camera = GLBeginWorld(out far);
			GL.Begin(GL.LINES);
			GL.Color(c);
			GL.Vertex(ori);
			GL.Vertex(end);
			GL.End();
			GL.PopMatrix();
			camera.farClipPlane = far;
		}

		public static void GLVec(Vector3 ori, Vector3 vec, Color c)
		{ GLLine(ori, ori+vec, c); }

//		edges[0] = new Vector3(min.x, min.y, min.z); //left-bottom-back
//		edges[1] = new Vector3(min.x, min.y, max.z); //left-bottom-front
//		edges[2] = new Vector3(min.x, max.y, min.z); //left-top-back
//		edges[3] = new Vector3(min.x, max.y, max.z); //left-top-front
//		edges[4] = new Vector3(max.x, min.y, min.z); //right-bottom-back
//		edges[5] = new Vector3(max.x, min.y, max.z); //right-bottom-front
//		edges[6] = new Vector3(max.x, max.y, min.z); //right-top-back
//		edges[7] = new Vector3(max.x, max.y, max.z); //right-top-front

		public static void GLBounds(Bounds b, Transform T, Color col)
		{
			var c = Utils.BoundCorners(b);
			for(int i = 0; i < 8; i++) c[i] = T.TransformPoint(c[i]);
			GLLine(c[0], c[1], col);
			GLLine(c[1], c[5], col);
			GLLine(c[5], c[4], col);
			GLLine(c[4], c[0], col);

			GLLine(c[2], c[3], col);
			GLLine(c[3], c[7], col);
			GLLine(c[7], c[6], col);
			GLLine(c[6], c[2], col);

			GLLine(c[2], c[0], col);
			GLLine(c[3], c[1], col);
			GLLine(c[7], c[5], col);
			GLLine(c[6], c[4], col);
		}
	}

	class NamedStopwatch
	{
		readonly System.Diagnostics.Stopwatch sw = new System.Diagnostics.Stopwatch();
		readonly string name;

		public NamedStopwatch(string name)
		{ this.name = name; }

		public double ElapsedSecs 
		{ get { return sw.ElapsedTicks/(double)System.Diagnostics.Stopwatch.Frequency; } }

		public void Start()
		{
			Utils.Log("{0}: start counting time", name);
			sw.Start();
		}

		public void Stamp()
		{
			Utils.Log("{0}: elapsed time: {1}us", name, 
			          sw.ElapsedTicks/(System.Diagnostics.Stopwatch.Frequency/(1000000L)));
		}

		public void Stop() { sw.Stop(); Stamp(); }

		public void Reset() { sw.Stop(); sw.Reset(); }
	}

	class Profiler
	{
		class Counter
		{
			public long Total { get; protected set; }
			public uint Count { get; protected set; }

			public void Add(long val) { Total += val; Count++; }
			public virtual void Add(Counter c) {}
			public virtual double Avg { get { return (double)Total/Count; } }
			public override string ToString() { return string.Format("avg: {0}us", Avg); }
		}

		class SumCounter : Counter
		{
			double avg;
			public override double Avg { get { return avg; } }

			public override void Add(Counter c)
			{
				Count = (uint)Mathf.Max(Count, c.Count);
				avg += c.Avg;
			}
		}

		readonly System.Diagnostics.Stopwatch sw = new System.Diagnostics.Stopwatch();
		readonly Dictionary<string, Counter> counters = new Dictionary<string, Counter>();

		long last = 0;

		public void Start() { sw.Stop(); sw.Reset(); sw.Start(); last = 0; }

		public void Log(string name)
		{
			var current = sw.ElapsedTicks/(System.Diagnostics.Stopwatch.Frequency/(1000000L));
			Counter v;
			if(counters.TryGetValue(name, out v)) 
				v.Add(current-last);
			else 
			{
				v = new Counter();
				v.Add(current-last);
				counters[name] = v;
			}
			last = current;
		}

		static void make_report(List<KeyValuePair<string,Counter>> clist)
		{
			var report = "\nName, NumCalls, Avg.Time (us)\n";
			foreach(var c in clist)
				report += string.Format("{0}, {1}, {2}\n", c.Key, c.Value.Count, c.Value.Avg);
			Utils.Log("Profiler Report:"+report);
		}

		public void PlainReport()
		{
			var clist = counters.ToList();
			clist.Sort((a, b) => b.Value.Avg.CompareTo(a.Value.Avg));
			make_report(clist);
		}

		public void TreeReport()
		{
			var cum_counters = new Dictionary<string, Counter>();
			foreach(var c in counters)
			{
				cum_counters[c.Key] = c.Value;
				var names = c.Key.Split(new []{'.'});
				var cname = "";
				for(int i = 0; i < names.Length-1; i++)
				{
					cname += "."+names[i];
					Counter v;
					if(cum_counters.TryGetValue(cname, out v)) v.Add(c.Value);
					else 
					{
						v = new SumCounter();
						v.Add(c.Value);
						cum_counters[cname] = v;
					}
				}
			}
			var clist = cum_counters.ToList();
			clist.Sort((a, b) => a.Key.CompareTo(b.Key));
			make_report(clist);
		}
	}

	class DebugCounter
	{
		int count = 0;
		string name = "";
		public DebugCounter(string name = "Debug", params object[] args) { this.name = string.Format(name, args); }
		public void Log(string msg="", params object[] args) 
		{ 
			if(msg == "") Utils.Log("{0}: {1}", name, count++); 
			else Utils.Log("{0}: {1} {2}", name, count++, string.Format(msg, args)); 
		}
		public void Reset() { count = 0; }
	}

	public class DebugModuleRCS : ModuleRCS
	{
		public override void OnStart(StartState state)
		{
			base.OnStart(state);
			this.Log("ThrusterTransforms:\n{0}",
			         thrusterTransforms.Aggregate("", (s, t) => s+t.name+": "+t.position+"\n"));
		}

		public new void FixedUpdate()
		{
			base.FixedUpdate();
			this.Log("Part: enabled {2}, shielded {0}, controllable {1}", 
			         part.ShieldedFromAirstream, part.isControllable, enabled);
			if(thrustForces.Length > 0)
			{
				this.Log("ThrustForces:\n{0}",
				         thrustForces.Aggregate("", (s, f) => s+f+", "));
				this.Log("FX.Power:\n{0}",
				         thrusterFX.Aggregate("", (s, f) => s+f.Power+", "+f.Active+"; "));
			}
		}
	}
}
#endif
