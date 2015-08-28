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
		public void SetState(TCAState state) { VSL.State |= state; }
		public bool IsStateSet(TCAState state) { return VSL.IsStateSet(state); }

		public virtual void Init() {}
		public virtual void Enable(bool enable = true) {}
		public virtual void UpdateState() {}
		public virtual void Reset() {}

		public void BlockSAS(bool block = true) 
		{ 
			if(!block) return;
			if(!CFG.SASIsControlled)
				CFG.SASWasEnabled = VSL.ActionGroups[KSPActionGroup.SAS]; 
			CFG.SASIsControlled = true;
		}

		#if DEBUG
		protected void Log(string msg, params object[] args)
		{ 
			var s = string.Format("{0}.{1}: {2}", VSL.vessel.vesselName, GetType().Name, msg);
			Utils.Log(s, args);
		}
		#endif
	}

	public abstract class AutopilotModule : TCAModule
	{
		public override void Init() { VSL.OnAutopilotUpdate -= Update; VSL.OnAutopilotUpdate += Update; }
		protected abstract void Update(FlightCtrlState s);
		public override void Reset() { VSL.OnAutopilotUpdate -= Update; }

		protected bool UserIntervening(FlightCtrlState s)
		{
			return !Mathfx.Approx(s.pitch, s.pitchTrim, 0.1f) ||
				!Mathfx.Approx(s.roll, s.rollTrim, 0.1f) ||
				!Mathfx.Approx(s.yaw, s.yawTrim, 0.1f);
		}

		protected void SetRot(Vector3 rot, FlightCtrlState s)
		{
			s.pitch = s.pitchTrim = rot.x;
			s.roll = s.rollTrim = rot.y;
			s.yaw = s.yawTrim = rot.z;
		}
	}
}

