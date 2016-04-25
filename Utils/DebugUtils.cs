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
using System.IO;
using System.Linq;
using System.Collections.Generic;
using System.Diagnostics;
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

		public static void logOrbit(string name, Orbit o)
		{ Utils.Log("Orbit: {0}\n{1}", name, Utils.formatOrbit(o)); }

		public static string getStacktrace(int skip = 0) { return new StackTrace(skip+1, true).ToString(); }

		public static void LogF(string msg, params object[] args)
		{ Utils.Log("{0}\n{1}", Utils.Format(msg, args), getStacktrace(1)); }

		//does not work with monodevelop generated .mdb files =(
//		public static void LogException(Action action)
//		{
//			try { action(); }
//			catch(Exception ex)
//			{
//				// Get stack trace for the exception with source file information
//				var st = new StackTrace(ex, true);
//				// Get the top stack frame
//				var frame = st.GetFrame(st.FrameCount-1);
//				// Log exception coordinates and stacktrace
//				Utils.Log("\nException in {0} at line {1}, column {2}\n{3}", 
//				          frame.GetFileName(), frame.GetFileLineNumber(), frame.GetFileColumnNumber(), 
//				          st.ToString());
//			}
//		}
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

	[KSPAddon(KSPAddon.Startup.MainMenu, false)]
	public class LoadTestGame : MonoBehaviour
	{
		static readonly string config   = TCAScenario.PluginFolder("LoadTestGame.conf");
		static readonly string savesdir = KSPUtil.ApplicationRootPath+"saves";

		void Awake()
		{
			var game = "default";
			var save = "persistent";
			if(File.Exists(config))
			{
				var cfg = ConfigNode.Load(config);
				var val = cfg.GetValue("game");
				if(val != null) game = val;
				val = cfg.GetValue("save");
				if(val != null) save = val;
			}
			else Utils.LogF("LoadTestGame: Configuration file not found: {}", config);
			var savefile = savesdir+"/"+game+"/"+save+".sfs";
			if(!File.Exists(savefile)) 
			{
				Utils.LogF("No such file: {}", savefile);
				return;
			}
			HighLogic.CurrentGame = GamePersistence.LoadGame(save, game, false, false);
			if (HighLogic.CurrentGame != null)
			{
				GamePersistence.UpdateScenarioModules(HighLogic.CurrentGame);
				HighLogic.SaveFolder = game;
				HighLogic.CurrentGame.Start();
			}
		}
	}
}
#endif
