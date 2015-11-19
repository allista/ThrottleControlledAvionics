//   Autopilot.cs
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
using UnityEngine;

namespace ThrottleControlledAvionics
{
	public interface ITCAModule
	{
		VesselConfig CFG { get; }
		TCAState State { get; }
		void SetState(TCAState state);
		bool IsStateSet(TCAState state);
	}

	public class TCAModule : ITCAModule
	{
		public class ModuleConfig : ConfigNodeObject
		{
			public virtual void Init() {}
		}

		protected VesselWrapper VSL;

		public static TCAGlobals GLB { get { return TCAScenario.Globals; } }
		public VesselConfig CFG { get { return VSL.CFG; } }
		public TCAState State { get { return VSL.State; } }
		public bool IsActive { get; protected set; }
		public bool Working { get; protected set; }
		public void SetState(TCAState state) { VSL.State |= state; }
		public bool IsStateSet(TCAState state) { return VSL.IsStateSet(state); }

		public virtual void Init() {}
		public virtual void Enable(bool enable = true) {}
		protected virtual void UpdateState() {}
		protected virtual void Update() {}
		public void OnFixedUpdate() { UpdateState(); Update(); }
		public virtual void Reset() {}

		protected void BlockSAS(bool block = true) 
		{ 
			if(!block) return;
			if(!CFG.SASIsControlled)
				CFG.SASWasEnabled = VSL.ActionGroups[KSPActionGroup.SAS]; 
			CFG.SASIsControlled = true;
		}

		protected void SetTarget(WayPoint wp)
		{
			CFG.Target = wp;
			var t = wp == null? null : wp.GetTarget();
			if(VSL.IsActiveVessel && t != null)
				ScreenMessages.PostScreenMessage("Target: "+t.GetName(),
				                                 5, ScreenMessageStyle.UPPER_CENTER);
			VSL.vessel.targetObject = t;
		}

		#region SquadMode
		public void SquadAction(Action<VesselWrapper> action)
		{
			if(ThrottleControlledAvionics.VSL != this.VSL) return;
			ThrottleControlledAvionics.Apply(tca => action(tca.VSL));
		}
		#endregion

		#if DEBUG
		protected void Log(string msg, params object[] args)
		{ 
			var s = string.Format("{0}.{1}: {2}", VSL.vessel.vesselName, GetType().Name, msg);
			Utils.Log(s, args);
		}

		protected void CSV(params object[] args)
		{
			var tag = string.Format("{0}.{1}", VSL.vessel.vesselName, GetType().Name);
			var args1 = new object[args.Length+1];
			args1[0]= tag; args.CopyTo(args1, 1);
			DebugUtils.CSV(args1);
		}
		#endif
	}

	public abstract class AutopilotModule : TCAModule
	{
		public override void Init() { VSL.OnAutopilotUpdate -= UpdateCtrlState; VSL.OnAutopilotUpdate += UpdateCtrlState; }
		public override void Reset() { VSL.OnAutopilotUpdate -= UpdateCtrlState; }
		public void UpdateCtrlState(FlightCtrlState s) { UpdateState(); OnAutopilotUpdate(s); }
		protected abstract void OnAutopilotUpdate(FlightCtrlState s);

		protected void DisableSAS()
		{
			// Disable the new SAS so it won't interfere. But enable it while in timewarp for compatibility with PersistentRotation
			if (TimeWarp.WarpMode != TimeWarp.Modes.HIGH || TimeWarp.CurrentRateIndex == 0)
				VSL.ActionGroups.SetGroup(KSPActionGroup.SAS, false);
		}

		protected void SetRot(Vector3 rot, FlightCtrlState s)
		{
			s.pitch = Utils.Clamp(rot.x, -1, 1);
			s.roll  = Utils.Clamp(rot.y, -1, 1);
			s.yaw   = Utils.Clamp(rot.z, -1, 1);
//			Log("Set Rot: {0}:{1}, {2}:{3}, {4}:{5}", 
//			    s.pitch, s.pitchTrim, s.roll, s.rollTrim, s.yaw, s.yawTrim);//debug 
		}
	}
}