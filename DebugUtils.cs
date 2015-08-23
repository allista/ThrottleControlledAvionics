//   DebugUtils.cs
//
//  Author:
//       Allis Tauri <allista@gmail.com>
//
//  Copyright (c) 2015 Allis Tauri
//
// This work is licensed under the Creative Commons Attribution 4.0 International License. 
// To view a copy of this license, visit http://creativecommons.org/licenses/by/4.0/ 
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

		public static void logVectors(string tag, params Vector3[] vecs)
		{
			var s = tag+":\n";
			foreach(var v in vecs)
			{
				var vn = v.normalized;
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
}
#endif
