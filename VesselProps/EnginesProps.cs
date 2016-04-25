//  Author:
//       Allis Tauri <allista@gmail.com>
//
//  Copyright (c) 2016 Allis Tauri
//
// This work is licensed under the Creative Commons Attribution-ShareAlike 4.0 International License. 
// To view a copy of this license, visit http://creativecommons.org/licenses/by-sa/4.0/ 
// or send a letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.
//
using System;
using System.Linq;
using System.Collections.Generic;
using UnityEngine;

namespace ThrottleControlledAvionics
{
	public class EnginesProps : VesselProps
	{
		public EnginesProps(VesselWrapper vsl) : base(vsl) {}

		public List<EngineWrapper> All       = new List<EngineWrapper>();
		public List<EngineWrapper> Active    = new List<EngineWrapper>();
		public List<EngineWrapper> Balanced  = new List<EngineWrapper>();
		public List<EngineWrapper> Maneuver  = new List<EngineWrapper>();
		public List<EngineWrapper> Steering  = new List<EngineWrapper>();
		public List<EngineWrapper> Manual    = new List<EngineWrapper>();

		public List<RCSWrapper> RCS = new List<RCSWrapper>();
		public List<RCSWrapper> ActiveRCS = new List<RCSWrapper>();

		public int  NumActive { get; private set; }
		public int  NumActiveRCS { get; private set; }
		public bool NoActiveRCS { get; private set; }
		public bool ForceUpdateEngines = false;

		public Vector3  Thrust { get; private set; } //current total thrust
		public Vector3  MaxThrust { get; private set; }
		public float    MaxThrustM { get; private set; }
		public float    ThrustDecelerationTime { get; private set; }

		public float    TorqueResponseTime { get; private set; }
		public bool     SlowTorque { get; private set; }

		public float    MassFlow { get; private set; }
		public float    MaxMassFlow { get; private set; }

		public void Clear() { All.Clear(); RCS.Clear(); }
		public void Add(EngineWrapper e) { All.Add(e); }

		public bool Check()
		{
			//update engines' list if needed
			var num_engines = All.Count;
			if(!ForceUpdateEngines)
			{
				for(int i = 0; i < num_engines; i++)
				{ ForceUpdateEngines |= !All[i].Valid(VSL); if(ForceUpdateEngines) break; }
				if(!ForceUpdateEngines)
				{
					for(int i = 0; i < RCS.Count; i++)
					{ ForceUpdateEngines |= !RCS[i].Valid(VSL); if(ForceUpdateEngines) break; }
				}
			}
			if(ForceUpdateEngines) 
			{ 
				VSL.UpdateParts(); 
				num_engines = All.Count;
				ForceUpdateEngines = false;
			}
			//unflameout engines
			if(VSL.vessel.ctrlState.mainThrottle > 0)
			{
				for(int i = 0; i < num_engines; i++)
				{ 
					var e = All[i]; 
					if(e.engine.flameout &&
					   e.Role != TCARole.MANUAL) 
						e.forceThrustPercentage(10); 
				}
			}
			//sync with active profile
			if(CFG.ActiveProfile.Activated) CFG.ActiveProfile.OnActivated(VSL);
			if(VSL.TCA.ProfileSyncAllowed)
			{
				if(CFG.ActiveProfile.Changed) CFG.ActiveProfile.Apply(All);
				else CFG.ActiveProfile.Update(All);
			}
			//get active engines and RCS
			Active.Clear(); Active.Capacity = All.Count;
			for(int i = 0; i < num_engines; i++)
			{ var e = All[i]; if(e.isOperational) Active.Add(e); }
			ActiveRCS.Clear();
			if(vessel.ActionGroups[KSPActionGroup.RCS])
			{
				for(int i = 0; i < RCS.Count; i++)
				{ var t = RCS[i]; if(t.isOperational) ActiveRCS.Add(t); }
			}
			NumActive = Active.Count;
			NumActiveRCS = ActiveRCS.Count;
			NoActiveRCS = NumActiveRCS == 0 || 
				VSL.Controls.Steering.sqrMagnitude < GLB.InputDeadZone && 
				VSL.Controls.Translation.sqrMagnitude < GLB.InputDeadZone;
			return NumActive > 0 && vessel.ctrlState.mainThrottle > 0 || !NoActiveRCS;
		}

		public void Sort()
		{
			Steering.Clear(); Steering.Capacity = NumActive;
			Maneuver.Clear(); Maneuver.Capacity = NumActive;
			Balanced.Clear(); Balanced.Capacity = NumActive;
			Manual.Clear();   Manual.Capacity   = NumActive;
			for(int i = 0; i < NumActive; i++)
			{
				var e = Active[i];
				switch(e.Role)
				{
				case TCARole.MAIN:
					Steering.Add(e);
					break;
				case TCARole.MANEUVER:
					Steering.Add(e);
					Maneuver.Add(e);
					break;
				case TCARole.BALANCE:
					Balanced.Add(e);
					break;
				case TCARole.MANUAL:
					Manual.Add(e);
					break;
				}
			}
		}

		public override void Update()
		{
			//init engine wrappers, thrust and torque information
			Thrust = Vector3.zero;
			MaxThrust = Vector3.zero;
			MassFlow = 0f;
			MaxMassFlow = 0f;
			ThrustDecelerationTime = 0f;
			SlowTorque = false;
			TorqueResponseTime = 0f;
			var total_torque = 0f;
			for(int i = 0; i < NumActive; i++) 
			{
				var e = Active[i];
				e.InitState();
				e.thrustDirection = refT.InverseTransformDirection(e.wThrustDir);
				e.wThrustLever = e.wThrustPos-VSL.Physics.wCoM;
				e.specificTorque = refT.InverseTransformDirection(Vector3.Cross(e.wThrustLever, e.wThrustDir));
				e.torqueRatio = Mathf.Pow(Mathf.Clamp01(1-Mathf.Abs(Vector3.Dot(e.wThrustLever.normalized, e.wThrustDir))), 
				                          GLB.ENG.TorqueRatioFactor);
				//do not include maneuver engines' thrust into the total to break the feedback loop with HSC
				if(e.Role != TCARole.MANEUVER) Thrust += e.wThrustDir*e.finalThrust;
				if(e.isVSC)
				{
					MaxThrust += e.wThrustDir*e.nominalCurrentThrust(1);
					MaxMassFlow += e.engine.maxThrust/e.engine.realIsp/Utils.G0;
					if(e.useEngineResponseTime && e.finalThrust > 0)
					{
						var decelT = 1f/e.engineDecelerationSpeed;
						if(decelT > ThrustDecelerationTime) ThrustDecelerationTime = decelT;
					}
				}
				if(e.useEngineResponseTime && (e.Role == TCARole.MAIN || e.Role == TCARole.MANEUVER))
				{
					var torque = e.Torque(1).magnitude;
					total_torque += torque;
					TorqueResponseTime += torque*Mathf.Max(e.engineAccelerationSpeed, e.engineDecelerationSpeed);
				}
				MassFlow += e.engine.requestedMassFlow*e.engine.propellantReqMet/100;
			}
			if(MassFlow > MaxMassFlow) MaxMassFlow = MassFlow;
			if(TorqueResponseTime > 0) TorqueResponseTime = total_torque/TorqueResponseTime;
			SlowTorque = TorqueResponseTime > 0;
			MaxThrustM = MaxThrust.magnitude;
			//init RCS wrappers if needed
			if(!NoActiveRCS)
			{
				for(int i = 0; i < NumActiveRCS; i++)
				{
					var t = ActiveRCS[i];
					t.InitState();
					t.thrustDirection = refT.InverseTransformDirection(t.wThrustDir);
					t.wThrustLever = t.wThrustPos-VSL.Physics.wCoM;
					t.specificTorque = refT.InverseTransformDirection(Vector3.Cross(t.wThrustLever, t.wThrustDir));
					t.torqueRatio = Mathf.Pow(Mathf.Clamp01(1-Mathf.Abs(Vector3.Dot(t.wThrustLever.normalized, t.wThrustDir))), GLB.RCS.TorqueRatioFactor);
					t.currentTorque = t.Torque(1);
					t.currentTorque_m = t.currentTorque.magnitude;
				}
			}
		}

		public void Tune()
		{
			//calculate VSF correction
			if(VSL.IsStateSet(TCAState.VerticalSpeedControl))
			{
				//calculate min imbalance
				var min_imbalance = Vector3.zero;
				for(int i = 0; i < NumActive; i++) min_imbalance += Active[i].Torque(0);
				min_imbalance = VSL.Torque.EnginesLimits.Clamp(min_imbalance);
				//correct VerticalSpeedFactor if needed
				if(!min_imbalance.IsZero())
				{
					var anti_min_imbalance = Vector3.zero;
					for(int i = 0; i < NumActive; i++)
					{
						var e = Active[i];
						if(Vector3.Dot(e.specificTorque, min_imbalance) < 0)
							anti_min_imbalance += e.specificTorque * e.nominalCurrentThrust(1);
					}
					anti_min_imbalance = Vector3.Project(anti_min_imbalance, min_imbalance);
					VSL.OnPlanetParams.VSF = Mathf.Clamp(VSL.OnPlanetParams.VSF, Mathf.Clamp01(min_imbalance.magnitude/anti_min_imbalance.magnitude
					                                     *GLB.VSC.BalanceCorrection), 1f);
				}
				for(int i = 0; i < NumActive; i++)
				{
					var e = Active[i];
					if(e.isVSC)
					{
						e.VSF = e.VSF > 0 ? VSL.OnPlanetParams.VSF : VSL.OnPlanetParams.MinVSF;
						e.throttle = e.VSF * vessel.ctrlState.mainThrottle;
					}
					else 
					{
						e.throttle = vessel.ctrlState.mainThrottle;
						e.VSF = 1f;
					}
					e.currentTorque = e.Torque(e.throttle);
					e.currentTorque_m = e.currentTorque.magnitude;
				}
			}
			else
			{
				for(int i = 0; i < NumActive; i++)
				{
					var e = Active[i];
					e.VSF = 1f;
					e.throttle = vessel.ctrlState.mainThrottle;
					e.currentTorque = e.Torque(e.throttle);
					e.currentTorque_m = e.currentTorque.magnitude;
				}
			}
		}

		public void SetControls()
		{
			for(int i = 0; i < NumActive; i++)
			{
				var e = Active[i];
				if(e.gimbal != null) 
					e.gimbal.gimbalLimiter = VSL.Controls.GimbalLimit;
				if(!Equals(e.Role, TCARole.MANUAL))
					e.thrustLimit = Mathf.Clamp01(e.VSF * e.limit);
				else if(VSL.Controls.ManualTranslationSwitch.On)
					e.forceThrustPercentage(e.limit*100);
				else if(VSL.Controls.ManualTranslationSwitch.WasSet)
					e.forceThrustPercentage(0);
			}
			VSL.Controls.ManualTranslationSwitch.Checked();
			if(NoActiveRCS) return;
			for(int i = 0; i < NumActiveRCS; i++)
			{
				var t = ActiveRCS[i];
				t.thrustLimit = Mathf.Clamp01(t.limit);
			}
		}
	}
}

