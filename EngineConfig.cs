//   EngineConfig.cs
//
//  Author:
//       Allis Tauri <allista@gmail.com>
//
//  Copyright (c) 2015 Allis Tauri
//
// This work is licensed under the Creative Commons Attribution-ShareAlike 3.0 International License. 
// To view a copy of this license, visit http://creativecommons.org/licenses/by-sa/3.0/ 
// or send a letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.

using System;
using System.Collections.Generic;
using UnityEngine;

namespace ThrottleControlledAvionics
{
	public class EngineConfig : ConfigNodeObject
	{
		new public const string NODE_NAME = "ENGINECFG";

		[Persistent] string Name;
		[Persistent] bool On;
		[Persistent] float Limit;
		[Persistent] string role;
		public TCARole Role;
		public bool Changed;

		public EngineConfig() {}
		public EngineConfig(EngineWrapper e) { Update(e); }

		public EngineConfig(EngineConfig c)
		{
			Name  = c.Name;
			On    = c.On;
			Limit = c.Limit;
			Role  = c.Role;
		}

		public void Update(EngineWrapper e)
		{
			Name  = e.name;
			On    = e.isOperational;
			Limit = e.thrustLimit;
			Role  = e.Role;
		}

		public override void Load (ConfigNode node)
		{
			base.Load(node);
			try { Role = (TCARole)Enum.Parse(typeof(TCARole), role); }
			catch { Role = default(TCARole); }
		}

		public override void Save (ConfigNode node)
		{
			role = Enum.GetName(typeof(TCARole), Role);
			base.Save(node);
		}

		public void Apply(EngineWrapper e)
		{
			if(e == null || e.info == null) return;
			e.SetRole(Role);
			if(On && !e.engine.EngineIgnited) e.engine.Activate();
			else if(!On && e.engine.EngineIgnited) e.engine.Shutdown();
			if(Role == TCARole.MANUAL) e.forceThrustPercentage(Limit*100);
			Changed = false;
		}

		void RoleControl()
		{
			if(GUILayout.Button("<", Styles.normal_button, GUILayout.Width(15)))
			{ Role = TCAEngineInfo.PrevRole(Role); Changed = true; }
			GUILayout.Label(TCAEngineInfo.RoleNames[(int)Role], GUILayout.Width(120));
			if(GUILayout.Button(">", Styles.normal_button, GUILayout.Width(15)))
			{ Role = TCAEngineInfo.NextRole(Role); Changed = true; }
		}

		public bool Draw(string name = null, bool with_role = true)
		{
			GUILayout.BeginHorizontal();
			GUILayout.Label(string.IsNullOrEmpty(name)? Name : name, GUILayout.Width(180));
			if(GUILayout.Button(On? "On" : "Off", On? Styles.green_button : Styles.red_button, GUILayout.Width(30)))
			{ On = !On; Changed = true; }
			if(with_role) RoleControl();
			if(Role == TCARole.MANUAL)
			{
				var lim = Utils.FloatSlider("", Limit, 0f, 1f, "P1", tooltip: "Throttle");
				if(!lim.Equals(Limit)) { Limit = lim; Changed = true; }
			}
			GUILayout.EndHorizontal();
			return Changed;
		}
	}

	public abstract class EngineConfigDB<K> : ConfigNodeObject
	{
		public Dictionary<K, EngineConfig> DB = new Dictionary<K, EngineConfig>();

		protected abstract bool TryParseK(string k, out K K);

		public override void Save (ConfigNode node)
		{
			foreach(var k in DB.Keys) 
				DB[k].Save(node.AddNode(k.ToString()));
			base.Save(node);
		}

		public override void Load (ConfigNode node)
		{
			base.Load(node);
			DB.Clear();
			if(node == null) return;
			foreach(var n in node.GetNodes())
			{
				K k;
				if(TryParseK(n.name, out k))
					DB[k] = ConfigNodeObject.FromConfig<EngineConfig>(n);
			}
		}

		#region reduced dict interface
		public EngineConfig this[K k] { get { return DB[k]; } set { DB[k] = value; } }
		public bool TryGetValue(K k, out EngineConfig c) { return DB.TryGetValue(k, out c); }
		public void Add(K k, EngineConfig c) { DB.Add(k, c); }
		public bool Remove(K k) { return DB.Remove(k); }
		public void Clear() { DB.Clear(); }
		public bool ContainsKey(K k) { return DB.ContainsKey(k); }
		public bool ContainsValue(EngineConfig c) { return DB.ContainsValue(c); }
		public int  Count { get { return DB.Count; } }
		public Dictionary<K, EngineConfig>.KeyCollection Keys { get { return DB.Keys; } }
		public Dictionary<K, EngineConfig>.ValueCollection Values { get { return DB.Values; } }
		#endregion
	}

	public class EngineConfigIntDB : EngineConfigDB<int>
	{
		protected override bool TryParseK (string k, out int K)
		{ return int.TryParse(k, out K); }
	}

	public class EngineConfigUintDB : EngineConfigDB<uint>
	{
		protected override bool TryParseK (string k, out uint K)
		{ return uint.TryParse(k, out K); }
	}

	public class EnginesProfile : ConfigNodeObject
	{
		new public const string NODE_NAME = "ENGINESPROF";
		static readonly string[] OnPlanetStates = { "On Planets", "In Space", "Always" };

		[Persistent] public string Name;
		[Persistent] public bool Active;
		[Persistent] public bool Default;
		[Persistent] public int  OnPlanet = 2;
		[Persistent] public int  Stage = -1;

		[Persistent] public EngineConfigUintDB Single = new EngineConfigUintDB();
		[Persistent] public EngineConfigIntDB  Groups = new EngineConfigIntDB();

		public bool Changed, Edit;

		public EnginesProfile() {}
		public EnginesProfile(string name, IList<EngineWrapper> engines) 
		{ Name = name; Init(engines); }

		public EnginesProfile(EnginesProfile p)
		{
			Name = p.Name+" (Copy)";
			OnPlanet = p.OnPlanet;
			foreach(var c in p.Single.DB) 
				Single[c.Key] = new EngineConfig(c.Value);
			foreach(var c in p.Groups.DB) 
				Groups[c.Key] = new EngineConfig(c.Value);
		}

		public void Init(IList<EngineWrapper> engines)
		{
			Single.Clear();
			Groups.Clear();
			foreach(var e in engines)
			{
				if(e.Group > 0)
				{
					if(!Groups.ContainsKey(e.Group))
						Groups[e.Group] = new EngineConfig(e);
				}
				else if(!Single.ContainsKey(e.part.flightID))
					Single[e.part.flightID] = new EngineConfig(e);
			}
		}

		public void Update(IList<EngineWrapper> engines)
		{
			var groups = new EngineConfigIntDB();
			var single = new EngineConfigUintDB();
			for(int i = 0, enginesCount = engines.Count; i < enginesCount; i++)
			{
				var e = engines[i];
				var c = GetConfig(e);
				if(c == null)
				{
					if(e.Group > 0)
					{
						if(!groups.ContainsKey(e.Group))
							groups[e.Group] = new EngineConfig(e);
					}
					else if(!single.ContainsKey(e.part.flightID))
						single[e.part.flightID] = new EngineConfig(e);
				}
				else if(e.Group > 0) groups[e.Group] = c;
				else
				{
					c.Update(e);
					single[e.part.flightID] = c;
				}
			}
			Changed = Groups.Count != groups.Count || Single.Count != single.Count;
			Groups = groups; Single = single;
			if(Changed) Apply(engines);
		}

		public EngineConfig GetConfig(EngineWrapper e)
		{
			EngineConfig c;
			if(e.Group > 0) { if(!Groups.TryGetValue(e.Group, out c)) return null; }
			else if(!Single.TryGetValue(e.part.flightID, out c)) return null;
			return c;
		}

		public void Apply(IList<EngineWrapper> engines)
		{
			for(int i = 0, enginesCount = engines.Count; i < enginesCount; i++) 
			{
				var e = engines[i];
				var c = GetConfig(e);
				if(c != null) c.Apply(e);
			}
			Changed = false;
		}

		public bool Usable(bool on_planet)
		{
			if(Default) return true;
			if(on_planet) return OnPlanet == 0 || OnPlanet == 2;
			else return OnPlanet == 1 || OnPlanet == 2;
		}

		void StageControl()
		{
			if(GUILayout.Button("<", Styles.normal_button, GUILayout.Width(15)))
			{ if(Stage >= 0) Stage--; }
			GUILayout.Label(new GUIContent(Stage < 0? "Off" : Stage.ToString(), 
			                              "Automatically activate at stage"), 
			                GUILayout.Width(20));
			if(GUILayout.Button(">", Styles.normal_button, GUILayout.Width(15)))
				Stage++;
		}

		void OnPlanetControl()
		{
			if(GUILayout.Button(new GUIContent(OnPlanetStates[OnPlanet],
			                                  "Should only be active"), 
			                    Styles.normal_button, GUILayout.Width(80)))
				OnPlanet = (OnPlanet+1)%3;
		}

		void TitleControl()
		{
			if(Active) GUILayout.Toggle(Active, "", GUILayout.Width(20));
			else 
			{
				Active = GUILayout.Toggle(Active, new GUIContent("", "Activate"), GUILayout.Width(20));
				Changed |= Active;
			}
			if(Edit) Name = GUILayout.TextField(Name, GUILayout.ExpandWidth(true), GUILayout.MinWidth(50));
			else GUILayout.Label(Name, Styles.white, GUILayout.ExpandWidth(true), GUILayout.MinWidth(50));
		}

		public bool Draw()
		{
			GUILayout.BeginVertical();
			GUILayout.BeginHorizontal();
			//header controls
			TitleControl();
			if(!Default)
			{
				OnPlanetControl();
				StageControl();
			}
			//default switch
			if(Default) GUILayout.Toggle(Default, "Default", GUILayout.Width(60));
			else { Default = GUILayout.Toggle(Default, "Default", GUILayout.Width(60)); }
			//edit button
			if(GUILayout.Button(Edit? "Done" : "Edit", 
				Edit? Styles.green_button : Styles.normal_button, GUILayout.Width(50)))
				Edit = !Edit;
			//delete button
			var delete = !Default && 
				!GUILayout.Button(new GUIContent("X", "Delete profile"), Styles.red_button, GUILayout.Width(20));
			GUILayout.EndHorizontal();
			if(Edit)
			{
				foreach(var k in Groups.Keys)
					Changed |= Groups[k].Draw(string.Format("Group {0}", k));
				foreach(var k in Single.Keys)
					Changed |= Single[k].Draw();
			}
			GUILayout.EndVertical();
			return delete;
		}

		public void DrawManual()
		{
			GUILayout.BeginVertical();
			foreach(var k in Groups.Keys)
			{
				var c = Groups[k];
				if(c.Role != TCARole.MANUAL) continue;
				Changed |= c.Draw(string.Format("Group {0}", k), false);
			}
			foreach(var k in Single.Keys)
			{
				var c = Single[k];
				if(c.Role != TCARole.MANUAL) continue;
				Changed |= c.Draw(with_role:false);
			}
			GUILayout.EndVertical();
		}
	}

	public class EnginesProfileDB : ConfigNodeObject
	{
		new public const string NODE_NAME = "ENGPROFILES";

		public List<EnginesProfile> DB = new List<EnginesProfile>();

		public EnginesProfile Default { get; private set; }
		public EnginesProfile Active { get; private set; }
		public bool Empty { get { return DB.Count == 0; } }

		Vector2 enginesScroll, manualScroll;

		public override void Load (ConfigNode node)
		{
			base.Load(node);
			DB.Clear();
			foreach(var n in node.GetNodes())
			{
				var c = ConfigNodeObject.FromConfig<EnginesProfile>(n);
				if(c.Active) Active = c;
				if(c.Default) Default = c;
				DB.Add(c);
			}
			if(!Empty)
			{
				if(Default == null) 
				{ Default = DB[0]; Default.Default = true; }
				if(Active == null)
				{ Active = Default; Active.Active = true; }
			}
		}

		public override void Save (ConfigNode node)
		{
			foreach(var p in DB) 
				p.Save(node.AddNode(EnginesProfile.NODE_NAME));
			base.Save(node);
		}

		public void AddProfile(IList<EngineWrapper> engines, string name = "Default")
		{ 
			DB.Add(new EnginesProfile(name, engines));
			if(DB.Count == 1) 
			{
				DB[0].Active  = true;
				DB[0].Default = true;
				DB[0].Changed = true;
				Active = Default = DB[0];
			}
		}

		public void CopyActive()
		{ if(Active != null) DB.Add(new EnginesProfile(Active)); }

		void Activate(EnginesProfile p)
		{
			Active.Active  = false;
			Active = p;
			Active.Changed = true;
			Active.Active  = true;
		}

		void SetDefault(EnginesProfile p)
		{
			Default.Default = false; 
			Default = p;
			Default.Default = true;
		}

		public void ActivateOnStage(int stage, IList<EngineWrapper> engines)
		{
			foreach(var p in DB)
			{
				if(p.Stage < 0 || p.Stage != stage) continue;
				Activate(p);
				Active.Apply(engines);
				break;
			}
		}

		public void OnPlanetChanged(bool on_planet)
		{
			if(DB.Count < 2) return;
			bool found = false;
			foreach(var p in DB)
			{
				if(p == Active || p == Default || 
				   !p.Usable(on_planet)) continue;
				Activate(p);
				found = true;
				break;
			}
			if(!found) 
			{
				if(Active.Usable(on_planet)) return;
				Activate(Default);
			}
		}

		public void Draw(int height)
		{
			if(DB.Count == 0) return;
			GUILayout.BeginVertical(Styles.white);
			enginesScroll = GUILayout.BeginScrollView(enginesScroll, GUILayout.Height(height));
			GUILayout.BeginVertical();
			var num_profs = DB.Count ;
			var del = new List<EnginesProfile>(num_profs);
			for(int i = 0; i < num_profs; i++)
			{
				var p = DB[i];
				if(!p.Draw() && p != Default)
				{
					del.Add(p);
					if(p == Active) Activate(Default);
					continue;
				}
				if(p.Active && p != Active)	Activate(p);
				if(p.Default && p != Default) SetDefault(p);
			}
			if(del.Count > 0) foreach(var p in del) DB.Remove(p);
			GUILayout.EndVertical();
			GUILayout.EndScrollView();
			if(GUILayout.Button("Add Profile", Styles.green_button, GUILayout.ExpandWidth(true)))
				CopyActive();
			GUILayout.EndVertical();
		}

		public void DrawManual(int height)
		{
			if(Active == null) return;
			GUILayout.BeginVertical(Styles.white);
			manualScroll = GUILayout.BeginScrollView(manualScroll, GUILayout.Height(height));
			Active.DrawManual();
			GUILayout.EndScrollView();
			GUILayout.EndVertical();
		}
	}
}

