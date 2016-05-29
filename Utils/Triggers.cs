//  Author:
//       Allis Tauri <allista@gmail.com>
//
//  Copyright (c) 2016 Allis Tauri
//
// This work is licensed under the Creative Commons Attribution-ShareAlike 4.0 International License. 
// To view a copy of this license, visit http://creativecommons.org/licenses/by-sa/4.0/ 
// or send a letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.
//
using System;

namespace ThrottleControlledAvionics
{
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
			return string.Format("time: {0} < next time {1}: {2}; remaining: {3}", 
			                     time, next_time, time < next_time, next_time-time);
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

		public void Start() { next_time = DateTime.Now.AddSeconds(Period); }

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

		public bool RunIf(Action action, bool predicate)
		{
			if(predicate) { if(Check) { action(); Reset(); return true; } }
			else Reset(); 
			return false;
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
}

