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

namespace ThrottleControlledAvionics
{
	public abstract class MultiplexerBase : ConfigNodeObject
	{
		[Persistent] public string State;
		public bool Paused;
		public abstract void Off();
	}

	public class Multiplexer<T> : MultiplexerBase where T : struct
	{
		public T state { get; protected set; }

		readonly Dictionary<T, Action<bool>> callbacks = new Dictionary<T, Action<bool>>();
		public readonly List<MultiplexerBase> Conflicts = new List<MultiplexerBase>();

		public Multiplexer() 
		{ if(!typeof(T).IsEnum) throw new ArgumentException("Multiplexer<T> T must be an enumerated type"); }

		public void AddConflicts(params MultiplexerBase[] ms) { Conflicts.AddRange(ms); }

		public bool Any(params T[] keys)
		{
			for(int i = 0, l = keys.Length; i < l; i++)
			{ if(state.Equals(keys[i])) return true; }
			return false;
		}

		public bool this[T key] 
		{ 
			get { return key.Equals(state); } 
			set 
			{ 
				if(Paused) return;
				if(value) On(key);
				else if(key.Equals(state)) Off();
			}
		}

		public void On(T key) 
		{ 
			if(Paused) return;
//			Utils.Log("{0}.On: state {1}, key {2}", GetType(), state, key);//debug
			if(!key.Equals(state)) Off();
			state = key;
			Action<bool> callback;
			if(callbacks.TryGetValue(key, out callback))
			{ if(callback != null) callback(true); }
		}

		public override void Off() 
		{ 
			if(Paused || state.Equals(default(T))) return;
//			Utils.Log("{0}.Off: state {1}", GetType(), state);//debug
			var old_state = state; //prevents recursion
			state = default(T);
			Action<bool> callback;
			if(callbacks.TryGetValue(old_state, out callback))
			{ if(callback != null) callback(false); }
		}

		public void Toggle(T key) { this[key] = !this[key]; }
		public void OnIfNot(T key) { if(!state.Equals(key)) On(key); }
		public void OffIfOn(T key) { if(state.Equals(key)) Off(); }
		public void OffIfOn(params T[] keys) 
		{ 
			if(state.Equals(default(T))) return;
			for(int i = 0, keysLength = keys.Length; i < keysLength; i++)
			{ if(state.Equals(keys[i])) { Off(); return; } }
		}

		void ConflictsOff()
		{
			Paused = true;
			Conflicts.ForEach(m => m.Off());
			Paused = false;
		}

		public void XOn(T key) { ConflictsOff(); On(key); }
		public void XOnIfNot(T key) { if(!state.Equals(key)) { ConflictsOff(); On(key); } }
		public void XOff() { ConflictsOff(); Off(); }
		public void XOffIfOn(T key) { if(state.Equals(key)) { ConflictsOff(); Off(); } }
		public void XToggle(T key) { ConflictsOff(); Toggle(key); }

		public static implicit operator bool(Multiplexer<T> m) { return !m[default(T)]; }

		public void ClearCallbacks() { callbacks.Clear(); Paused = false; }

		public void AddCallback(T key, Action<bool> callback)
		{
			if(callbacks.ContainsKey(key))
				callbacks[key] += callback;
			else callbacks[key] = callback;
		}

		public void AddSingleCallback(Action<bool> callback)
		{
			foreach(T key in Enum.GetValues(typeof(T)))
				AddCallback(key, callback);
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
}

