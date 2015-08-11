//   ModuleTCA.cs
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
using System.Reflection;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace ThrottleControlledAvionics
{
	public class ModuleTCA : PartModule, ITCAModule
	{
		#if DEBUG
		internal static Profiler prof = new Profiler();
		#endif

		const string TCA_PART = "ThrottleControlledAvionics";

		public static TCAGlobals GLB { get { return TCAConfiguration.Globals; } }
		public VesselWrapper VSL { get; private set; }
		public VesselConfig CFG { get { return VSL.CFG; } }
		public TCAState State { get { return VSL.State; } set { VSL.State = value; } }
		public void SetState(TCAState state) { VSL.State |= state; }
		public bool IsStateSet(TCAState state) { return Available && VSL.IsStateSet(state); }

		#region Modules
		EngineOptimizer eng;
		VerticalSpeedControl vsc;
		HorizontalSpeedControl hsc;
		AltitudeControl alt;
		RCSOptimizer rcs;
		CruiseControl cc;
		PointNavigator pn;
		Radar rad;
		List<TCAModule> modules;
		FieldInfo[] mod_fields;
		#endregion

		#region Public Info
		public float TorqueError { get { return eng == null? 0f : eng.TorqueError; } }
		public bool  Available { get { return enabled && VSL != null; } }
		public bool  Controllable { get { return Available && vessel.IsControllable; } }
		#endregion

		#region Initialization
		#if DEBUG
		public void OnReloadGlobals() { VSL.Init(); invoke_in_modules("Init"); }
		#endif

		public override string GetInfo()
		{
			TCAConfiguration.LoadGlobals();
			return (!GLB.IntegrateIntoCareer || Utils.PartIsPurchased(TCA_PART))?
				"Software Installed" : "Not Available";
		}

		public override void OnAwake()
		{
			base.OnAwake();
			GameEvents.onVesselWasModified.Add(onVesselModify);
		}

		internal void OnDestroy() 
		{ 
			GameEvents.onVesselWasModified.Remove(onVesselModify);
			reset();
		}

		public override void OnStart(StartState state)
		{
			base.OnStart(state);
			if(state == StartState.Editor || state == StartState.None)
			{ enabled = isEnabled = false; return; }
			check_priority();
			check_career_part();
			init();
		}

		void onVesselModify(Vessel vsl)
		{ 
			if(vsl != vessel) return;
			reset();
			check_priority();
			init();
		}

		void check_priority()
		{
			if(vessel == null || vessel.parts == null) goto disable;
			var TCA_part = vessel.parts.FirstOrDefault(p => p.HasModule<ModuleTCA>());
			if(TCA_part != part) goto disable;
			var TCA = TCA_part.Modules.OfType<ModuleTCA>().FirstOrDefault();
			if(TCA == this) return;
			disable: enabled = isEnabled = false;
		}

		void check_career_part()
		{
			if(!enabled) return;
			enabled = isEnabled = !GLB.IntegrateIntoCareer || Utils.PartIsPurchased(TCA_PART);
		}

		void create_modules()
		{
			var mt = typeof(TCAModule);
			var vt = typeof(VesselWrapper);
			if(mod_fields == null)
				mod_fields = GetType()
					.GetFields(BindingFlags.DeclaredOnly|BindingFlags.NonPublic|BindingFlags.Instance)
					.Where(fi => fi.FieldType.IsSubclassOf(mt)).ToArray();
			modules = new List<TCAModule>(mod_fields.Length);
			foreach(var fi in mod_fields)
			{
				if(!fi.FieldType.IsSubclassOf(mt)) continue;
				var method = fi.FieldType.GetConstructor(new [] {vt});
				if(method != null)
				{
					fi.SetValue(this, method.Invoke(fi.GetValue(this), new [] {VSL}));
					modules.Add((TCAModule)fi.GetValue(this));
				}
				else Utils.Log("Failed to create {0}. No constructor found.", fi.FieldType);
			}
		}

		void delete_modules()
		{
			if(mod_fields == null) return;
			mod_fields.ForEach(mf => mf.SetValue(this, null));
		}

		void invoke_in_modules(string m)
		{
			var mt = typeof(TCAModule);
			var method = mt.GetMethod(m);
			if(method == null) Utils.Log("No {0} method in {1} found.", m, mt.Name);
			for(int i = 0; i<modules.Count; i++) method.Invoke(modules[i], null);
		}

		void init()
		{
			if(!enabled) return;
			VSL = new VesselWrapper(vessel);
			VSL.UpdateState();
			VSL.UpdateEngines();
			enabled = isEnabled = VSL.Engines.Count > 0 || VSL.RCS.Count > 0;
			if(!enabled) { VSL = null; return; }
			create_modules();
			VSL.Init(); 
			modules.ForEach(m => m.Init());
			vessel.OnAutopilotUpdate += block_throttle;
			hsc.ConnectAutopilot();
			cc.ConnectAutopilot();
			VSL.UpdateCommons();
			VSL.UpdateVerticalStats();
			if(CFG.GoToTarget) pn.GoToTarget(VSL.vessel.targetObject != null);
			else if(CFG.FollowPath) pn.FollowPath(CFG.Waypoints.Count > 0);
			else if(CFG.CruiseControl) UpdateNeededVeloctiy();
			ThrottleControlledAvionics.AttachTCA(this);
		}

		void reset()
		{
			if(VSL == null) return;
			VSL.OnAutopilotUpdate -= block_throttle;
			if(NeededVelocityUpdater != null) 
				StopCoroutine(NeededVelocityUpdater);
			modules.ForEach(m => m.Reset());
			delete_modules();
			VSL = null; 
		}

		IEnumerator<YieldInstruction> NeededVelocityUpdater;
		IEnumerator<YieldInstruction> update_needed_velocity()
		{
			while(VSL != null && CFG.CruiseControl && VSL.OnPlanet)
			{
				cc.UpdateNeededVelocity();
				yield return new WaitForSeconds(GLB.CC.Delay);
			}
		}
		void UpdateNeededVeloctiy()
		{
			NeededVelocityUpdater = update_needed_velocity();
			StartCoroutine(NeededVelocityUpdater);
		}
		#endregion

		#region Controls
		public void ToggleTCA()
		{
			CFG.Enabled = !CFG.Enabled;
			if(!CFG.Enabled) //reset engine limiters
			{
				VSL.Engines.ForEach(e => e.forceThrustPercentage(100));
				State = TCAState.Disabled;
			}
		}

		public void BlockThrottle(bool state)
		{
			if(state == CFG.BlockThrottle) return;
			CFG.BlockThrottle = state;
			if(CFG.BlockThrottle && CFG.VerticalCutoff >= GLB.VSC.MaxSpeed)
				CFG.VerticalCutoff = 0;
		}

		public void ToggleHvAutopilot() { hsc.Enable(!CFG.KillHorVel); }
		public void ToggleAltitudeAutopilot() { alt.Enable(!CFG.ControlAltitude); }
		public void AltitudeAboveTerrain(bool state) { alt.SetAltitudeAboveTerrain(state); }

		public void ToggleCruiseControl()
		{
			cc.Enable(!CFG.CruiseControl);
			if(CFG.CruiseControl) UpdateNeededVeloctiy();
		}

		public void ToggleGoToTarget() { pn.GoToTarget(!CFG.GoToTarget);}
		public void ToggleFollowPath() { pn.FollowPath(!CFG.FollowPath);}
		#endregion

		void block_throttle(FlightCtrlState s)
		{ if(CFG.Enabled && CFG.BlockThrottle) s.mainThrottle = 1f; }

		public override void OnUpdate()
		{
			if(IsStateSet(TCAState.HaveActiveEngines)) VSL.UpdateMoI();
		}

		public override void OnFixedUpdate()
		{
			//initialize systems
			VSL.UpdateState();
			if(!CFG.Enabled) return;
			State = TCAState.Enabled;
			if(!VSL.ElectricChargeAvailible) return;
			SetState(TCAState.HaveEC);
			if(!VSL.CheckEngines()) return;
			SetState(TCAState.HaveActiveEngines);
			//update state
			VSL.UpdateCommons();
			if(VSL.NumActive > 0)
			{
//				modules.ForEach(m => m.UpdateState());
				for(int i = 0; i < modules.Count; i++) modules[i].UpdateState();
				if(vsc.IsActive) VSL.UpdateVerticalStats();
				if(hsc.IsActive) VSL.UpdateHorizontalStats();
				//these follow specific order
				alt.Update();
				rad.Update();
				vsc.Update();
				pn.Update();
			}
			//handle engines
			VSL.InitEngines();
			if(VSL.NumActive > 0)
			{
				VSL.SortEngines();
				//:balance-only engines
				if(VSL.BalancedEngines.Count > 0)
				{
					VSL.UpdateTorque(VSL.ActiveManualEngines);
					eng.Optimize(VSL.BalancedEngines, Vector3.zero);
				}
				VSL.UpdateTorque(VSL.ActiveManualEngines, VSL.BalancedEngines);
				//:optimize limits for steering
				eng.PresetLimitsForTranslation();
				eng.Steer();
			}
			rcs.Steer();
			VSL.SetThrustLimiters();
		}
	}
}



