//   Multiplexer.cs
//
//  Author:
//       Allis Tauri <allista@gmail.com>
//
//  Copyright (c) 2015 Allis Tauri
//
// This work is licensed under the Creative Commons Attribution 4.0 International License. 
// To view a copy of this license, visit http://creativecommons.org/licenses/by/4.0/ 
// or send a letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.

using System;
using System.Collections.Generic;

namespace ThrottleControlledAvionics
{
	public class Multiplexer<T> : ConfigNodeObject where T : struct
	{
		[Persistent] public string State;
		public T state { get; protected set; }
		public bool Paused;

		readonly Dictionary<T, Action<bool>> callbacks = new Dictionary<T, Action<bool>>();

		public Multiplexer() 
		{ if(!typeof(T).IsEnum) throw new ArgumentException("Multiplexer<T> T must be an enumerated type"); }

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
			if(!key.Equals(state)) Off();
			state = key;
			Action<bool> callback;
			if(callbacks.TryGetValue(key, out callback))
			{ if(callback != null) callback(true); }
		}
		public void Off() 
		{ 
			if(Paused || state.Equals(default(T))) return;
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

