//  Author:
//       Allis Tauri <allista@gmail.com>
//
//  Copyright (c) 2015 Allis Tauri
//
// This work is licensed under the Creative Commons Attribution-ShareAlike 4.0 International License. 
// To view a copy of this license, visit http://creativecommons.org/licenses/by-sa/4.0/ 
// or send a letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.

using System;
using System.Reflection;
using System.Collections.Generic;

namespace ThrottleControlledAvionics
{
	public abstract class Multiplexer : ConfigNodeObject
	{
		public enum Command { Off, On, Resume }
		public delegate void Callback(Command cmd);

		[Persistent] public string State;
		public bool Paused;

		public abstract void Off();
		public abstract void Resume();
		public abstract void ClearCallbacks();

		public readonly List<Multiplexer> Conflicts = new List<Multiplexer>();
		public void AddConflicts(params Multiplexer[] ms) { Conflicts.AddRange(ms); }
		protected void ConflictsOff()
		{
			Paused = true;
			Conflicts.ForEach(m => m.Off());
			Paused = false;
		}

		protected Callback GetCallback(object handler, string name)
		{
			var ht = handler.GetType();
			var callback_info = ht.GetMethod(name+"Callback", new [] {typeof(Command)});
			return callback_info != null? 
				Delegate.CreateDelegate(typeof(Callback), handler, callback_info) as Callback :
				null;
		}
	}

	public class Multiplexer<T> : Multiplexer where T : struct
	{
		public T state { get; protected set; }
		readonly Dictionary<T, Callback> callbacks = new Dictionary<T, Callback>();

		public Multiplexer() 
		{ if(!typeof(T).IsEnum) throw new ArgumentException("Multiplexer<T> T must be an enumerated type"); }

		#region Logic
		public static implicit operator bool(Multiplexer<T> m) { return !m[default(T)]; }

		public bool Any(params T[] keys)
		{
			for(int i = 0, l = keys.Length; i < l; i++)
			{ if(state.Equals(keys[i])) return true; }
			return false;
		}

		public bool Not(T key)
		{ return !state.Equals(key) && !state.Equals(default(T)); }

		public bool Not(params T[] keys)
		{
			if(state.Equals(default(T))) return false;
			for(int i = 0, l = keys.Length; i < l; i++)
			{ if(state.Equals(keys[i])) return false; }
			return true;
		}
		#endregion

		#region Control
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
//			DebugUtils.LogF("\n{}.On: {}->{}", GetType(), state, key);//debug
			if(!key.Equals(state)) Off();
			state = key;
			Callback callback;
			if(callbacks.TryGetValue(key, out callback))
			{ if(callback != null) callback(Command.On); }
		}

		public override void Off() 
		{ 
			if(Paused || state.Equals(default(T))) return;
//			DebugUtils.LogF("\n{}.Off: {}->None", GetType(), state);//debug
			var old_state = state; //prevents recursion
			state = default(T);
			Callback callback;
			if(callbacks.TryGetValue(old_state, out callback))
			{ if(callback != null) callback(Command.Off); }
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

		public void XOn(T key) { ConflictsOff(); On(key); }
		public void XOnIfNot(T key) { if(!state.Equals(key)) { ConflictsOff(); On(key); } }
		public void XOff() { ConflictsOff(); Off(); }
		public void XOffIfOn(T key) { if(state.Equals(key)) { ConflictsOff(); Off(); } }
		public void XToggle(T key) { ConflictsOff(); Toggle(key); }

		public override void Resume()
		{
			if(state.Equals(default(T))) return;
			Callback callback;
			if(callbacks.TryGetValue(state, out callback))
			{ if(callback != null) callback(Command.Resume); }
		}
		#endregion

		#region Callbacks
		public override void ClearCallbacks() { callbacks.Clear(); Paused = false; }

		public void AddCallback(Callback callback, T key)
		{
			if(callbacks.ContainsKey(key))
				callbacks[key] += callback;
			else callbacks[key] = callback;
		}

		public void AddCallback(Callback callback, params T[] keys) 
		{ foreach(T key in keys) AddCallback(callback, key); }

		public void AddSingleCallback(Callback callback)
		{
			foreach(T key in Enum.GetValues(typeof(T)))
				AddCallback(callback, key);
		}

		public void SetSingleCallback(Callback callback)
		{
			ClearCallbacks();
			foreach(T key in Enum.GetValues(typeof(T)))
				callbacks[key] = callback;
		}

		public void AddHandler(object handler, T key) 
		{ 
			var callback = GetCallback(handler, key.ToString());
			if(callback == null)
			{
				Utils.Log("{0}: no public method named {1}", handler.GetType().Name, key);
				return;
			}
			AddCallback(callback, key);
		}

		public void AddHandler(object handler, params T[] keys) 
		{ foreach(T key in keys) AddHandler(handler, key); }

		public void SetSingleHandler(object handler)
		{
			ClearCallbacks();
			foreach(T key in Enum.GetValues(typeof(T)))
				AddHandler(handler, key);
		}
		#endregion

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