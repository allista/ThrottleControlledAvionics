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

        public virtual bool Valid { get { return TCA != null && TCA.Valid; } }

		protected SquadControl SQD;

		protected TCAComponent(ModuleTCA tca) { TCA = tca; }

		public void InitModuleFields() { TCA.InitModuleFields(this); }

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

		protected string LogTemplate(string msg)
		{ return string.Format("{0}.{1}: {2}", VSL.vessel.vesselName, GetType().Name, msg); }

		protected void Log(string msg, params object[] args) { Utils.Log(LogTemplate(msg), args); }

		#if DEBUG
        protected void AddDebugMessage(string msg, params object[] args)
        { if(VSL.IsActiveVessel) TCAGui.AddDebugMessage(msg, args); }

		protected void LogFST(string msg, params object[] args) { DebugUtils.Log(LogTemplate(msg), args); }

		protected void CSV(params object[] args)
		{
			var tag = string.Format("{0}.{1}.csv", VSL.vessel.vesselName, GetType().Name).Replace(' ', '_');
			DebugUtils.CSV(tag, args);
		}
		#endif
	}

	public abstract class DrawableComponent : TCAComponent
	{
        protected TCAGui UI { get { return TCAGui.Instance; } }
		protected DrawableComponent(ModuleTCA tca) : base(tca) {}
		public abstract void Draw();
	}

	public class TCAModule : DrawableComponent
	{
		public class ModuleConfig : ConfigNodeObject
		{
            public class MinMax : ConfigNodeObject
            {
                [Persistent] public float Min;
                [Persistent] public float Max;
                public MinMax(float min, float max) { Min = min; Max = max; }
            }
			public virtual void Init() {}
		}

		public bool ControlsActive { get; protected set; } = true;
		public bool IsActive { get; protected set; }
		public bool Working { get; protected set; }

		protected TCAModule(ModuleTCA tca) : base(tca) {}

		public virtual void Init() { InitModuleFields(); LoadFromConfig(); }
		public void OnFixedUpdate() 
        { 
            if(CFG.Enabled)
            {
                UpdateState(); 
                Update(); 
            }
        }
		public virtual void Reset() {}
		public virtual void ClearFrameState() {}
		public virtual void OnEnable(bool enabled) {}
		public virtual void ProcessKeys() {}
		public override void Draw() {}

		protected virtual void UpdateState() 
        { 
            IsActive = VSL != null && CFG.Enabled; 
            ControlsActive = true; 
        }
		protected virtual void Update() {}
		protected virtual void reset() {}

		protected void SetTarget(WayPoint wp = null) { VSL.SetTarget(this, wp); }
		protected void SetTarget(Vessel vsl) { SetTarget(new WayPoint(vsl)); }
		protected void UseTarget() { VSL.TargetUsers.Add(this); }
		protected void StopUsingTarget() { VSL.TargetUsers.Remove(this); }

		public bool RegisterTo<S>(Func<VesselWrapper,bool> predicate = null) 
			where S : TCAService
		{
			var srv = TCA.GetModule<S>();
			return srv != null && srv.Register(this, predicate);
		}

		public bool NeedCPSWhenMooving()
		{ 
            var ret = RegisterTo<Radar>(vsl => vsl.HorizontalSpeed.MoovingFast); 
            return RegisterTo<CollisionPreventionSystem>() && ret;
        }

        public bool NeedCPS()
        {
            var ret = RegisterTo<Radar>(); 
            return RegisterTo<CollisionPreventionSystem>() && ret;
        }

        public void ReleaseCPS()
        {
            UnregisterFrom<Radar>();
            UnregisterFrom<CollisionPreventionSystem>();
        }

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
		}

		public void SaveGame(string description)
		{
			if(Globals.Instance.AutosaveBeforeLanding)
				Utils.SaveGame(VSL.vessel.vesselName.Replace(" ", "_")+"-"+description);
		}
	}

	public abstract class AutopilotModule : TCAModule
	{
		protected AutopilotModule(ModuleTCA tca) : base(tca) {}

        protected FlightCtrlState CS;

		public override void Init() 
		{ 
			base.Init(); 
			VSL.vessel.OnAutopilotUpdate -= UpdateCtrlState; 
			VSL.vessel.OnAutopilotUpdate += UpdateCtrlState; 
		
		}

		public override void Reset() { VSL.vessel.OnAutopilotUpdate -= UpdateCtrlState; }

		public void UpdateCtrlState(FlightCtrlState s) 
        { 
            if(CFG.Enabled)
            {
                CS = s;
                UpdateState(); 
                OnAutopilotUpdate(); 
            }
        }

		protected abstract void OnAutopilotUpdate();
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
