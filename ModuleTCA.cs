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
using System.Linq;
using UnityEngine;

namespace ThrottleControlledAvionics
{
	public class ModuleTCA : PartModule, ITCAModule
	{
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
//		CruiseControl cc;
		#endregion

		#region Public Info
		public float TorqueError { get { return eng == null? 0f : eng.TorqueError; } }
		public bool  Available { get { return enabled && VSL != null; } }
		public bool  Controllable { get { return Available && vessel.IsControllable; } }
		#endregion

		#region Initialization
		#if DEBUG
		public void OnReloadGlobals()
		{ VSL.Init(); eng.Init(); vsc.Init(); hsc.Init(); alt.Init(); rcs.Init(); }
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

		void init()
		{
			if(!enabled) return;
			VSL = new VesselWrapper(vessel);
			VSL.UpdateEngines();
			enabled = isEnabled = VSL.Engines.Count > 0 || VSL.RCS.Count > 0;
			if(!enabled) { VSL = null; return; }
			eng = new EngineOptimizer(VSL);
			vsc = new VerticalSpeedControl(VSL);
			hsc = new HorizontalSpeedControl(VSL);
			alt = new AltitudeControl(VSL);
			rcs = new RCSOptimizer(VSL);
			hsc.ConnectAutopilot();
			vessel.OnAutopilotUpdate += block_throttle;
			VSL.Init(); eng.Init(); vsc.Init(); hsc.Init(); alt.Init(); rcs.Init();
			ThrottleControlledAvionics.AttachTCA(this);
		}

		void reset()
		{
			if(VSL != null) 
			{
				VSL.OnAutopilotUpdate -= block_throttle;
				hsc.DisconnectAutopilot();
			}
			VSL = null; eng = null; vsc = null; hsc = null; alt = null; rcs = null;
		}
		#endregion

		#region Controls
		public void ActivateTCA(bool state)
		{
			if(state == CFG.Enabled) return;
			CFG.Enabled = state;
			if(!CFG.Enabled) //reset engine limiters
			{
				VSL.Engines.ForEach(e => e.forceThrustPercentage(100));
				State = TCAState.Disabled;
			}
		}
		public void ToggleTCA() { ActivateTCA(!CFG.Enabled); }

		public void BlockThrottle(bool state)
		{
			if(state == CFG.BlockThrottle) return;
			CFG.BlockThrottle = state;
			if(CFG.BlockThrottle && CFG.VerticalCutoff >= GLB.VSC.MaxSpeed)
				CFG.VerticalCutoff = 0;
		}

		public void KillHorizontalVelocity(bool state)
		{
			if(state == CFG.KillHorVel) return;
			CFG.KillHorVel = state;
			hsc.BlockSAS(CFG.KillHorVel);
			if(CFG.KillHorVel) CruiseControl(false);
		}
		public void ToggleHvAutopilot() { KillHorizontalVelocity(!CFG.KillHorVel); }

		public void MaintainAltitude(bool state)
		{
			if(state == CFG.ControlAltitude) return;
			CFG.ControlAltitude = state;
			if(CFG.ControlAltitude)
			{
				VSL.UpdateAltitude();
				CFG.DesiredAltitude = VSL.Altitude;
			}
		}
		public void ToggleAltitudeAutopilot() { MaintainAltitude(!CFG.ControlAltitude); }

		public void AltitudeAboveTerrain(bool state)
		{
			if(state == CFG.AltitudeAboveTerrain) return;
			CFG.AltitudeAboveTerrain = state;
			VSL.UpdateAltitude();
			if(CFG.AltitudeAboveTerrain)
				CFG.DesiredAltitude -= VSL.TerrainAltitude;
			else CFG.DesiredAltitude += VSL.TerrainAltitude;
		}
		public void ToggleAltitudeAboveTerrain() { AltitudeAboveTerrain(!CFG.AltitudeAboveTerrain); }

		public void CruiseControl(bool state)
		{
			if(state == CFG.CruiseControl) return;
			CFG.CruiseControl = state;
			hsc.BlockSAS(CFG.CruiseControl);
			if(CFG.CruiseControl)
			{
				hsc.NeededHorVelocity = VSL.HorizontalVelocity;
				KillHorizontalVelocity(false);
			}
			else hsc.NeededHorVelocity = Vector3d.zero;
		}
		public void ToggleCruiseControl() { CruiseControl(!CFG.CruiseControl);}
		#endregion

		void block_throttle(FlightCtrlState s)
		{ if(CFG.Enabled && CFG.BlockThrottle) s.mainThrottle = 1f; }

		public void FixedUpdate()
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
				eng.UpdateState();
				vsc.UpdateState();
				hsc.UpdateState();
				alt.UpdateState();
				if(vsc.IsActive || alt.IsActive || hsc.IsActive) 
					VSL.UpdateOnPlanetStats();
				alt.Update();
				vsc.Update();
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
				VSL.UpdateETorqueLimits();
				if(hsc.IsActive) 
				{
					VSL.UpdateWTorqueLimits();
					VSL.UpdateRTorqueLimits();
				}
				eng.PresetLimitsForTranslation();
				eng.Steer();
			}
			rcs.Steer();
			VSL.SetThrustLimiters();
		}
	}
}



