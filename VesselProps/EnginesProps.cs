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
using AT_Utils;

namespace ThrottleControlledAvionics
{
	public class EnginesProps : VesselProps
	{
		public class EnginesStats
		{
			public Vector3 Thrust;
			public double  MassFlow;

			public Vector6 TorqueLimits = new Vector6();
			public Vector3 Torque;
			public Vector3 AngularA;
		}

		public EnginesProps(VesselWrapper vsl) : base(vsl) {}

		public List<EngineWrapper> All        = new List<EngineWrapper>();
		public List<EngineWrapper> Active     = new List<EngineWrapper>();
		public List<EngineWrapper> Balanced   = new List<EngineWrapper>();
		public List<EngineWrapper> UnBalanced = new List<EngineWrapper>();
		public List<EngineWrapper> Maneuver   = new List<EngineWrapper>();
		public List<EngineWrapper> Steering   = new List<EngineWrapper>();
		public List<EngineWrapper> Manual     = new List<EngineWrapper>();

		public List<RCSWrapper> RCS = new List<RCSWrapper>();
		public List<RCSWrapper> ActiveRCS = new List<RCSWrapper>();
		public Vector6 MaxThrustRCS = new Vector6();

		public int  NumActive { get; private set; }
		public int  NumActiveRCS { get; private set; }
		public bool NoActiveEngines { get; private set; }
		public bool NoActiveRCS { get; private set; }
		public bool HaveMainEngines { get; private set; }
		public bool ForceUpdateEngines = false;

		public bool HaveNextStageEngines { get; private set; }
		public int  NearestEnginedStage { get; private set; }

		public KSPActionGroup ActionGroups  { get; private set; } = KSPActionGroup.None;

		public Vector3  Thrust { get; private set; } //current total thrust
		public Vector3  MaxThrust { get; private set; }
		public Vector3  ManualThrust { get; private set; }
		public Vector6  ManualThrustLimits { get; private set; } = Vector6.zero;
		public float    MaxThrustM { get; private set; }
		public float    MaxAccel { get; private set; }
		public float    ThrustDecelerationTime { get; private set; }

		public float    TorqueResponseTime { get; private set; }
		public bool     SlowTorque { get; private set; }

		public float    MassFlow { get; private set; }
		public float    ManualMassFlow { get; private set; }
		public float    MaxMassFlow { get; private set; }
		public float    MaxVe { get; private set; } //Specific impulse in m/s, aka effective exhaust velocity

		public void Clear() { All.Clear(); RCS.Clear(); }
		public bool Add(ModuleEngines m) 
		{ 
			if(m == null) return false;
			All.Add(new EngineWrapper(m));
			return true;
		}
		public bool Add(ModuleRCS m)
		{ 
			if(m == null) return false;
			RCS.Add(new RCSWrapper(m));
			return true;
		}

		public Vector3 CurrentMaxThrustDir 
		{
			get
			{
				var thrust = MaxThrust;
				if(thrust.IsZero()) thrust =  NearestEnginedStageMaxThrust;
				if(thrust.IsZero()) thrust = -VSL.Controls.Transform.up;
				return thrust.normalized;
			}
		}

		public Vector3 CurrentThrustDir 
		{ get { return Thrust.IsZero()? CurrentMaxThrustDir : Thrust.normalized; } }

		public float ThrustAtAlt(float vel, float alt, out float mflow)
		{
			var flow = 0f;
			var thrust = 0f;
			Active.ForEach(e => 
			{ 
				float mFlow;
				thrust += e.ThrustAtAlt(vel, alt, out mFlow); 
				flow += mFlow; 
			});
			mflow = flow;
			return thrust;
		}

		public float TTB(float dV, float thrust, float mflow, float throttle)
		{
			if(thrust.Equals(0)) return float.MaxValue;
			if(dV.Equals(0)) return 0;
			return (CheatOptions.InfinitePropellant?
			        VSL.Physics.M/thrust*dV : FuelNeeded(dV, thrust/mflow)/mflow) / throttle;
		}

		public float TTB(float dV)
		{ return TTB(dV, MaxThrustM, MaxMassFlow, ThrottleControl.NextThrottle(dV, 1, VSL)); }

		public float FuelNeeded(float dV, float Ve) { return VSL.Physics.M*(1-Mathf.Exp(-dV/Ve)); }
		public float FuelNeeded(float dV) { return FuelNeeded(dV, MaxVe); }
		public float FuelNeededAtAlt(float dV, float alt) 
		{ 
			float mflow;
			float thrust = ThrustAtAlt(0, alt, out mflow);
			return FuelNeeded(dV, thrust/mflow); 
		}

		public float DeltaV(float Ve, float fuel_mass) { return Ve*Mathf.Log(VSL.Physics.M/(VSL.Physics.M-fuel_mass)); }
		public float DeltaV(float fuel_mass) { return DeltaV(MaxVe, fuel_mass); }

		public float MaxHoverTimeASL(float fuel_mass)
		{ 
			float mflow;
			float thrust = ThrustAtAlt(0, 0, out mflow);
			return (float)(thrust/(VSL.Physics.M-fuel_mass) / (VSL.Body.GeeASL*Utils.G0) * fuel_mass/mflow); 
		}

		public float MaxDeltaVAtAlt(float alt) 
		{ 
			float mflow;
			float thrust = ThrustAtAlt(0, alt, out mflow);
			return DeltaV(thrust/mflow, GetAvailableFuelMass()); 
		}

		public float MaxDeltaV { get { return DeltaV(MaxVe, GetAvailableFuelMass()); } }

		public float RelVeASL 
		{ 
			get 
			{ 
				float mflow;
				float thrust = ThrustAtAlt(0, 0, out mflow);
				return (float)(thrust - VSL.Body.GeeASL*Utils.G0*VSL.Physics.M)/mflow; 
			} 
		}

		public Vector3 NearestEnginedStageMaxThrust
		{ get { return GetNearestEnginedStageStats().Thrust; } }

		public EnginesStats GetNearestEnginedStageStats()
		{ return GetStageStats(NearestEnginedStage); }

		public EnginesStats GetCurrentStageStats()
		{ return GetStageStats(VSL.vessel.currentStage); }

		public EnginesStats GetStageStats(int stage)
		{ return GetEnginesStats(All.Where(e => e.part.inverseStage >= stage).ToList()); }

		public EnginesStats GetEnginesStats(IList<EngineWrapper> engines)
		{
			var stats = new EnginesStats();
			for(int i = 0, count = engines.Count; i < count; i++)
			{
				var e = engines[i];
				e.InitState();
				e.InitTorque(VSL, GLB.ENG.TorqueRatioFactor);
				var throttle = e.Role == TCARole.MANUAL ? e.thrustLimit : 1;
				if(throttle > 0)
				{
					var thrust = e.nominalCurrentThrust(throttle);
					if(e.Role != TCARole.MANEUVER)
					{
						stats.Thrust += e.wThrustDir * thrust;
						stats.MassFlow += e.MaxFuelFlow*throttle;
					}
					if(e.isSteering) stats.TorqueLimits.Add(e.specificTorque*thrust);
				}
			}
			stats.Torque = stats.TorqueLimits.Max;
			stats.AngularA = VSL.Torque.AngularAcceleration(stats.Torque);
			return stats;
		}

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
				else CFG.ActiveProfile.Update(All, true);
			}
			//get active engines and RCS
			NearestEnginedStage = -1;
			HaveNextStageEngines = false;
			var groups = KSPActionGroup.None;
			Active.Clear(); Active.Capacity = All.Count;
			for(int i = 0; i < num_engines; i++)
			{ 
				var e = All[i]; 
				if(e.isOperational) Active.Add(e);
				//check action groups
				for(int j = 0; j < e.engine.Actions.Count; j++)
					groups |= e.engine.Actions[j].actionGroup;
				//update staging information
				if(e.part.inverseStage > NearestEnginedStage && e.part.inverseStage <= vessel.currentStage)
					NearestEnginedStage = e.part.inverseStage;
				HaveNextStageEngines |= e.part.inverseStage >= 0 && e.part.inverseStage < vessel.currentStage;
			}
			ActionGroups = groups;
			ActiveRCS.Clear();
			if(vessel.ActionGroups[KSPActionGroup.RCS])
			{
				for(int i = 0; i < RCS.Count; i++)
				{ var t = RCS[i]; if(t.isOperational) ActiveRCS.Add(t); }
			}
			NumActive = Active.Count;
			NumActiveRCS = ActiveRCS.Count;
			NoActiveEngines = NumActive == 0;
			NoActiveRCS = NumActiveRCS == 0 || 
				VSL.Controls.Steering.sqrMagnitude < GLB.InputDeadZone && 
				VSL.Controls.Translation.sqrMagnitude < GLB.InputDeadZone;
			return !(NoActiveEngines && NoActiveRCS);
		}

		public void Sort()
		{
			bool have_mains = false;
			Steering.Clear(); Steering.Capacity = NumActive;
			Maneuver.Clear(); Maneuver.Capacity = NumActive;
			Balanced.Clear(); Balanced.Capacity = NumActive;
			UnBalanced.Clear(); UnBalanced.Capacity = NumActive;
			Manual.Clear();   Manual.Capacity   = NumActive;
			for(int i = 0; i < NumActive; i++)
			{
				var e = Active[i];
				switch(e.Role)
				{
				case TCARole.MAIN:
					have_mains = true;
					Steering.Add(e);
					break;
				case TCARole.MANEUVER:
					Steering.Add(e);
					Maneuver.Add(e);
					break;
				case TCARole.BALANCE:
					Balanced.Add(e);
					break;
				case TCARole.UNBALANCE:
					UnBalanced.Add(e);
					break;
				case TCARole.MANUAL:
					Manual.Add(e);
					break;
				}
			}
			HaveMainEngines = have_mains;
		}

		public override void Update()
		{
			//init engine wrappers, thrust and torque information
			Thrust = Vector3.zero;
			MaxThrust = Vector3.zero;
			ManualThrust = Vector3.zero;
			ManualThrustLimits = Vector6.zero;
			MassFlow = 0f;
			MaxMassFlow = 0f;
			ManualMassFlow = 0f;
			ThrustDecelerationTime = 0f;
			SlowTorque = false;
			TorqueResponseTime = 0f;
			var total_torque = 0f;
			for(int i = 0; i < NumActive; i++) 
			{
				var e = Active[i];
				e.InitState();
				e.InitTorque(VSL, GLB.ENG.TorqueRatioFactor);
				//do not include maneuver engines' thrust into the total to break the feedback loop with HSC
				if(e.Role != TCARole.MANEUVER) 
					Thrust += e.wThrustDir*e.finalThrust;
				if(e.Role == TCARole.MANUAL)
				{
					ManualThrust += e.wThrustDir*e.finalThrust;
					ManualMassFlow += e.RealFuelFlow;
					ManualThrustLimits.Add(e.thrustDirection*e.nominalCurrentThrust(1));
				}
				if(e.isVSC)
				{
					MaxThrust += e.wThrustDir*e.nominalCurrentThrust(1);
					MaxMassFlow += e.MaxFuelFlow;
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
				MassFlow += e.RealFuelFlow;
			}
			if(MassFlow > MaxMassFlow) MaxMassFlow = MassFlow;
			if(TorqueResponseTime > 0) TorqueResponseTime = total_torque/TorqueResponseTime;
			SlowTorque = TorqueResponseTime > 0;
			MaxThrustM = MaxThrust.magnitude;
			MaxVe = MaxThrustM/MaxMassFlow;
			MaxAccel = MaxThrustM/VSL.Physics.M;
			//init RCS wrappers and calculate MaxThrust taking torque imbalance into account
			MaxThrustRCS = new Vector6();
			var RCSThrusImbalance = new Vector3[6];
			for(int i = 0; i < NumActiveRCS; i++)
			{
				var t = ActiveRCS[i];
				t.InitState();
				for(int j = 0, tCount = t.rcs.thrusterTransforms.Count; j < tCount; j++)
				{
					var T = t.rcs.thrusterTransforms[j];
					var thrust = refT.InverseTransformDirection((t.rcs.useZaxis ? T.forward : T.up) * t.maxThrust);
					MaxThrustRCS.Add(thrust);
					var pos = refT.InverseTransformDirection(T.position - VSL.Physics.wCoM);
					var athrust = Vector3.zero;
					for(int k = 0; k < 3; k++)
					{
						athrust[k] = thrust[k];
						var p = pos; p[k] = 0;
						RCSThrusImbalance[thrust[k] > 0 ? k : k + 3] += Vector3.Cross(p, athrust);
						athrust[k] = 0;
					}
				}
				if(NoActiveRCS) continue;
				t.InitTorque(VSL, GLB.RCS.TorqueRatioFactor);
				t.currentTorque = t.Torque(1);
				t.currentTorque_m = t.currentTorque.magnitude;
			}
			if(!MaxThrustRCS.IsZero())
				MaxThrustRCS.Scale(new Vector6(
					1/Utils.ClampL(RCSThrusImbalance[0].sqrMagnitude, 1),
					1/Utils.ClampL(RCSThrusImbalance[1].sqrMagnitude, 1),
					1/Utils.ClampL(RCSThrusImbalance[2].sqrMagnitude, 1),
					1/Utils.ClampL(RCSThrusImbalance[3].sqrMagnitude, 1),
					1/Utils.ClampL(RCSThrusImbalance[4].sqrMagnitude, 1),
					1/Utils.ClampL(RCSThrusImbalance[5].sqrMagnitude, 1)));
		}

		public void Tune()
		{
			//calculate VSF correction
			if(CFG.VSCIsActive)
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
						if(e.VSF.Equals(1)) e.VSF = VSL.OnPlanetParams.VSF;
						e.throttle = e.VSF * vessel.ctrlState.mainThrottle;
					}
					else 
					{
						e.VSF = 1f;
						e.throttle = vessel.ctrlState.mainThrottle;
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

		public bool ActivateInactiveEngines() 
		{ 
			if(CFG.AutoStage && NoActiveEngines && HaveNextStageEngines)
			{ VSL.ActivateNextStage(); return true; }
			return false;
		}

		public bool ActivateNextStageOnFlameout()
		{
			if(!CFG.AutoStage || !HaveNextStageEngines) return false;
			var this_engines = All.Where(e => e.part.inverseStage >= vessel.currentStage).ToList();
			return stage_cooldown.RunIf(VSL.ActivateNextStage,
			                            this_engines.Count > 0 && 
			                            this_engines.Any(e => e.engine.flameout));
		}
		readonly Timer stage_cooldown = new Timer(0.5);

		public bool ActivateEngines()
		{ return VSL.Engines.ActivateNextStageOnFlameout() || VSL.Engines.ActivateInactiveEngines(); }

		void GetAvailableFuel(Part origin, int resourceID, ResourceFlowMode flowMode, List<PartResource> resources)
		{
			double total_amount = 0.0;
			double max_amount = 0.0;
			PartResource partResource;
			switch (flowMode)
			{
			case ResourceFlowMode.NO_FLOW:
				partResource = origin.Resources.Get(resourceID);
				if(partResource != null && partResource.flowState)
					resources.Add(partResource);
				break;
			case ResourceFlowMode.ALL_VESSEL:
			case ResourceFlowMode.STAGE_PRIORITY_FLOW:
			case ResourceFlowMode.ALL_VESSEL_BALANCE:
			case ResourceFlowMode.STAGE_PRIORITY_FLOW_BALANCE:
			case ResourceFlowMode.STAGE_STACK_FLOW:
			case ResourceFlowMode.STAGE_STACK_FLOW_BALANCE:
				if(flowMode == ResourceFlowMode.STAGE_STACK_FLOW_BALANCE ||
				   flowMode == ResourceFlowMode.STAGE_STACK_FLOW)
					origin.FindResource_StackPriority(origin, resources, resourceID, 0, Part.NewRequestID(), true, ref total_amount, ref max_amount, null, true);
				else
				{
					for (int j = vessel.parts.Count - 1; j >= 0; j--)
					{
						partResource = vessel.parts[j].Resources.Get(resourceID);
						if(partResource != null && partResource.flowState) resources.Add(partResource);
					}
				}
				break;
			case ResourceFlowMode.STACK_PRIORITY_SEARCH:
				origin.FindResource_StackPriority(origin, resources, resourceID, 1e-20, Part.NewRequestID(), false, ref total_amount, ref max_amount, null, PhysicsGlobals.Stack_PriUsesSurf);
				break;
			}
		}

		public float GetAvailableFuelMass()
		{
			double fuel_mass = 0;
			var available_resources = new List<PartResource>();
			var added_resources = new HashSet<int>();
			Active.ForEach(e => e.engine.GetConsumedResources()
			               .ForEach(r => GetAvailableFuel(e.part, r.id, r.resourceFlowMode, available_resources)));
			for(int i = 0, count = available_resources.Count; i < count; i++)
			{
				var r = available_resources[i];
				if(added_resources.Add(r.info.id))
					fuel_mass += r.amount * r.info.density;
			}
			return (float)fuel_mass;
		}
	}
}

