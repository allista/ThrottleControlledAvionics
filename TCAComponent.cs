//  Author:
//       Allis Tauri <allista@gmail.com>
//
//  Copyright (c) 2015 Allis Tauri
//
// This work is licensed under the Creative Commons Attribution-ShareAlike 4.0 International License. 
// To view a copy of this license, visit http://creativecommons.org/licenses/by-sa/4.0/ 
// or send a letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.

using System;
using System.Linq;
using System.Collections.Generic;
using System.Reflection;
using UnityEngine;
using AT_Utils;

namespace ThrottleControlledAvionics
{
	public interface ITCAComponent
	{
		VesselConfig CFG { get; }
		TCAState State { get; }
		void SetState(TCAState state);
		bool IsStateSet(TCAState state);
	}

	public abstract class TCAComponent : ConfigNodeObject, ITCAComponent
	{
		public readonly ModuleTCA TCA;
		public VesselWrapper VSL { get { return TCA.VSL; } }
		internal static Globals GLB { get { return Globals.Instance; } }
		public VesselConfig CFG { get { return TCA.VSL.CFG; } }
		public TCAState State { get { return VSL.State; } }
		public void SetState(TCAState state) { VSL.State |= state; }
		public bool IsStateSet(TCAState state) { return VSL.IsStateSet(state); }

		protected SquadControl SQD;

		protected TCAComponent(ModuleTCA tca) { TCA = tca; }

		public List<FieldInfo> get_all_module_fields(Type t, List<FieldInfo> list = null)
		{
			if(list == null) list = new List<FieldInfo>();
			list.AddRange(t.GetFields(BindingFlags.DeclaredOnly|BindingFlags.Instance|BindingFlags.NonPublic)
			              .Where(fi => fi.FieldType.IsSubclassOf(typeof(TCAModule))));
			if(t.BaseType != null) get_all_module_fields(t.BaseType, list);
			return list;
		}

		public void InitModuleFields()
		{
			var ModuleFields = get_all_module_fields(GetType());
			ModuleFields.ForEach(fi => fi.SetValue(this, TCA.GetModule(fi.FieldType)));
		}

		protected void apply(Action<ModuleTCA> action)
		{
			if(SQD == null) action(TCA);
			else SQD.Apply(action);
		}

		protected void apply_cfg(Action<VesselConfig> action)
		{
			if(SQD == null) action(CFG);
			else SQD.ApplyCFG(action);
		}

		protected void Message(float duration, string msg, params object[] args)
		{ if(VSL.IsActiveVessel) Utils.Message(duration, msg, args); }

		protected void Message(string msg, params object[] args) { Message(5, msg, args); }

		protected void ClearStatus() { if(VSL.IsActiveVessel) TCAGui.ClearStatus(); }

		protected void Status(double seconds, string msg, params object[] args)
		{ if(VSL.IsActiveVessel) TCAGui.Status(seconds, msg, args); }

		protected void Status(string msg, params object[] args) 
		{ Status(-1, msg, args); }

		protected void Status(double seconds, string color, string msg, params object[] args)
		{ if(VSL.IsActiveVessel) TCAGui.Status(seconds, color, msg, args); }

		protected void Status(string color, string msg, params object[] args) 
		{ Status(-1, color, msg, args); }

		#if DEBUG
		protected string LogTemplate(string msg)
		{ return string.Format("{0}.{1}: {2}", VSL.vessel.vesselName, GetType().Name, msg); }

		protected void Log(string msg, params object[] args) { Utils.Log(LogTemplate(msg), args); }
		protected void LogFST(string msg, params object[] args) { DebugUtils.Log(LogTemplate(msg), args); }

		protected void CSV(params object[] args)
		{
			var tag = string.Format("{0}.{1}", VSL.vessel.vesselName, GetType().Name);
			var args1 = new object[args.Length+1];
			args1[0]= tag; args.CopyTo(args1, 1);
			DebugUtils.CSV(args1);
		}
		#endif
	}

	public abstract class DrawableComponent : TCAComponent
	{
		protected DrawableComponent(ModuleTCA tca) : base(tca) {}
		public abstract void Draw();
	}

	public class TCAModule : DrawableComponent
	{
		public class ModuleConfig : ConfigNodeObject
		{
			public virtual void Init() {}
		}

		public bool ControlsActive { get; protected set; } = true;
		public bool IsActive { get; protected set; }
		public bool Working { get; protected set; }

		protected TCAModule(ModuleTCA tca) : base(tca) {}

		public virtual void Init() { InitModuleFields(); LoadFromConfig(); }
		public void OnFixedUpdate() { UpdateState(); Update(); }
		public virtual void Reset() {}
		public virtual void ClearFrameState() {}
		public virtual void OnEnable(bool enabled) {}
		public virtual void ProcessKeys() {}
		public override void Draw() {}

		protected virtual void UpdateState() { IsActive = VSL != null && CFG.Enabled; ControlsActive = true; }
		protected virtual void Update() {}
		protected virtual void reset() {}

		protected void SetTarget(WayPoint wp = null) { VSL.SetTarget(wp); }
		protected void SetTarget(Vessel vsl) { SetTarget(new WayPoint(vsl)); }

		protected WayPoint Target2WP()
		{
			if(VSL.Target == null) return null;
			return VSL.Target as WayPoint ?? 
				new WayPoint(VSL.Target);
		}

		public bool RegisterTo<S>(Func<VesselWrapper,bool> predicate = null) 
			where S : TCAService
		{
			var srv = TCA.GetModule<S>();
			return srv != null && srv.Register(this, predicate);
		}

		public bool NeedRadarWhenMooving()
		{ return RegisterTo<Radar>(vsl => vsl.HorizontalSpeed.MoovingFast); }

		public bool UnregisterFrom<S>() 
			where S : TCAService
		{
			var srv = TCA.GetModule<S>();
			return srv != null && srv.Unregister(this);
		}

		public void SaveToConfig()
		{
			var node = new ConfigNode(GetType().Name);
			Save(node);
			CFG.ModuleConfigs[node.name] = node;
		}

		public void LoadFromConfig()
		{
			ConfigNode node;
			var name = GetType().Name;
			if(CFG.ModuleConfigs.TryGetValue(name, out node)) Load(node);
			//deprecated: Old configuration conversion
			else if(CFG.LoadedConfig != null) Load(CFG.LoadedConfig);
		}
	}

	public abstract class AutopilotModule : TCAModule
	{
		protected AutopilotModule(ModuleTCA tca) : base(tca) {}

		public override void Init() 
		{ 
			base.Init(); 
			VSL.vessel.OnAutopilotUpdate -= UpdateCtrlState; 
			VSL.vessel.OnAutopilotUpdate += UpdateCtrlState; 
		
		}
		public override void Reset() { VSL.vessel.OnAutopilotUpdate -= UpdateCtrlState; }
		public void UpdateCtrlState(FlightCtrlState s) { UpdateState(); OnAutopilotUpdate(s); }
		protected abstract void OnAutopilotUpdate(FlightCtrlState s);
	}

	public abstract class TCAService : TCAModule
	{
		public struct Client
		{
			public readonly TCAModule Module;
			public Func<VesselWrapper,bool> Predicate;

			public Client(TCAModule module, Func<VesselWrapper,bool> predicate = null)
			{ Module = module; Predicate = predicate; }

			public static implicit operator bool(Client c)
			{ return c.Module != null && (c.Predicate == null || c.Predicate(c.Module.VSL)); }

			public override int GetHashCode() { return Module.GetHashCode(); }
		}

		readonly HashSet<Client> Clients = new HashSet<Client>();
		protected bool HasActiveClients { get { return Clients.Any(c => c); } }

		public bool Register(TCAModule module, Func<VesselWrapper,bool> predicate = null) 
		{ return Clients.Add(new Client(module, predicate)); }

		public bool Unregister(TCAModule module) 
		{ return Clients.Remove(new Client(module)); }

		protected TCAService(ModuleTCA tca) : base(tca) {}
	}
}
