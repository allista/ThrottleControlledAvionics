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
		public class EnginesStats : VesselProps
		{
			public EnginesStats(VesselWrapper vsl) : base(vsl) 
			{ TorqueInfo = new TorqueInfo(vsl); }

			public Vector3 MaxThrust;
			public double  MaxMassFlow;

			public Vector6 TorqueLimits = new Vector6();
			public readonly TorqueInfo TorqueInfo;

			public override void Update()
			{ TorqueInfo.Update(TorqueLimits.Max); }
		}

		public class EnginesDB : List<EngineWrapper>
		{
			public List<EngineWrapper> Main       = new List<EngineWrapper>();
			public List<EngineWrapper> Balanced   = new List<EngineWrapper>();
			public List<EngineWrapper> UnBalanced = new List<EngineWrapper>();
			public List<EngineWrapper> Maneuver   = new List<EngineWrapper>();
			public List<EngineWrapper> Steering   = new List<EngineWrapper>();
			public List<EngineWrapper> Manual     = new List<EngineWrapper>();

			public void SortByRole()
			{
				var count = Count;
				Main.Clear(); Main.Capacity = count;
				Steering.Clear(); Steering.Capacity = count;
				Maneuver.Clear(); Maneuver.Capacity = count;
				Balanced.Clear(); Balanced.Capacity = count;
				UnBalanced.Clear(); UnBalanced.Capacity = count;
				Manual.Clear(); Manual.Capacity = count;
				for(int i = 0; i < count; i++)
				{
					var e = this[i];
					switch(e.Role)
					{
					case TCARole.MAIN:
						Main.Add(e);
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
			}
		}

		public EnginesProps(VesselWrapper vsl) : base(vsl) {}

		public List<EngineWrapper> All = new List<EngineWrapper>();
		public EnginesDB Active = new EnginesDB();

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
		public float    DecelerationTime { get; private set; }
		public float    AccelerationTime { get; private set; }
		public bool     Slow { get; private set; }

		public float    MassFlow { get; private set; }
		public float    ManualMassFlow { get; private set; }
		public float    MaxMassFlow { get; private set; }
		public float    MaxVe { get; private set; } //Specific impulse in m/s, aka effective exhaust velocity

		public override void Clear() { All.Clear(); RCS.Clear(); }
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

		public Vector3 refT_forward_axis
		{ get { return VSL.OnPlanetParams.NoseUp? VSL.refT.forward : VSL.refT.up; } }

		public Vector3 refT_thrust_axis
		{ get { return VSL.OnPlanetParams.NoseUp? VSL.refT.up : VSL.refT.forward; } }

		public Vector3 CurrentMaxThrustDir 
		{
			get
			{
				var thrust = MaxThrust;
				if(thrust.IsZero()) thrust =  NearestEnginedStageMaxThrust;
				if(thrust.IsZero()) thrust = -refT_thrust_axis;
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
		{ get { return GetNearestEnginedStageStats().MaxThrust; } }

		public EnginesStats GetNearestEnginedStageStats()
		{ return GetStageStats(NearestEnginedStage); }

		public EnginesStats GetCurrentStageStats()
		{ return GetStageStats(VSL.vessel.currentStage); }

		public EnginesStats GetStageStats(int stage)
		{ return GetEnginesStats(All.Where(e => e.part.inverseStage >= stage).ToList()); }

		public EnginesStats GetEnginesStats(IList<EngineWrapper> engines)
		{
			var stats = new EnginesStats(VSL);
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
						stats.MaxThrust += e.wThrustDir * thrust;
						stats.MaxMassFlow += e.MaxFuelFlow*throttle;
					}
					if(e.isSteering) stats.TorqueLimits.Add(e.specificTorque*thrust);
				}
			}
			stats.Update();
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
			Active.SortByRole(); 
			HaveMainEngines = Active.Main.Count > 0;
			VSL.Controls.TranslationAvailable = VSL.Engines.Active.Maneuver.Count > 0 || VSL.Engines.NumActiveRCS > 0;
		}

		void update_MaxThrust()
		{
			//first optimize engines for zero torque to have actual MaxThrust in case of unbalanced ship design
			if(VSL.Torque != null && VSL.TCA != null && VSL.TCA.ENG != null)
			{
				if(Active.Balanced.Count > 0)
				{
					VSL.Torque.UpdateImbalance(Active.Manual, Active.UnBalanced);
					VSL.TCA.ENG.OptimizeLimitsForTorque(Active.Balanced, Vector3.zero);
				}
				VSL.Torque.UpdateImbalance(Active.Manual, Active.UnBalanced, Active.Balanced);
				VSL.TCA.ENG.OptimizeLimitsForTorque(Active.Steering, Vector3.zero);
			}
			DecelerationTime = 0f;
			AccelerationTime = 0f;
			MaxThrust = Vector3.zero;
			MaxMassFlow = 0f;
			Slow = false;
			var have_steering = !VSL.Controls.Steering.IsZero();
			var total_thrust = 0f;
			for(int i = 0; i < NumActive; i++) 
			{
				var e = Active[i];
				if(e.isVSC)
				{
					var thrust = e.nominalCurrentThrust(e.limit);
					total_thrust += thrust;
					MaxThrust += e.wThrustDir*thrust;
					MaxMassFlow += e.MaxFuelFlow*e.limit;
					if(e.useEngineResponseTime && e.finalThrust > 0)
					{
						if(e.engineDecelerationSpeed > 0)
							DecelerationTime += thrust/e.engineDecelerationSpeed;
						if(e.engineAccelerationSpeed > 0)
							AccelerationTime += thrust/e.engineAccelerationSpeed;
					}
				}
				if(e.isSteering && have_steering) e.InitLimits();
			}
			if(DecelerationTime > 0) { DecelerationTime /= total_thrust; Slow = true; }
			if(AccelerationTime > 0) { AccelerationTime /= total_thrust; Slow = true; }
			if(MassFlow > MaxMassFlow) MaxMassFlow = MassFlow;
			MaxThrustM = MaxThrust.magnitude;
			MaxVe = MaxThrustM/MaxMassFlow;
			MaxAccel = MaxThrustM/VSL.Physics.M;
		}

		void update_RCS()
		{
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
				t.UpdateCurrentTorque(1);
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

		public override void Update()
		{
			//init engine wrappers, update thrust and torque information
			Thrust = Vector3.zero;
			ManualThrust = Vector3.zero;
			ManualThrustLimits = Vector6.zero;
			MassFlow = 0f;
			ManualMassFlow = 0f;
			for(int i = 0; i < NumActive; i++) 
			{
				var e = Active[i];
				e.InitState();
				e.InitTorque(VSL, GLB.ENG.TorqueRatioFactor);
				e.UpdateCurrentTorque(1);
				//do not include maneuver engines' thrust into the total to break the feedback loop with HSC
				if(e.Role != TCARole.MANEUVER) 
					Thrust += e.wThrustDir*e.finalThrust;
				if(e.Role == TCARole.MANUAL)
				{
					ManualThrust += e.wThrustDir*e.finalThrust;
					ManualMassFlow += e.RealFuelFlow;
					ManualThrustLimits.Add(e.thrustDirection*e.nominalCurrentThrust(1));
				}
				MassFlow += e.RealFuelFlow;
			}
			update_MaxThrust();
			update_RCS();
			//update engines' current torque
			var throttle = VSL.vessel.ctrlState.mainThrottle;
			var vsc_throttle = (VSL.OnPlanet && CFG.VSCIsActive)? throttle*VSL.OnPlanetParams.GeeVSF : throttle;
			for(int i = 0; i < NumActive; i++) 
			{
				var e = Active[i];
				e.UpdateCurrentTorque(e.isVSC? vsc_throttle : throttle);
			}
		}

		public void Tune()
		{
			//calculate VSF correction
			if(VSL.OnPlanet && CFG.VSCIsActive)
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
						e.UpdateCurrentTorque(e.VSF * vessel.ctrlState.mainThrottle);
					}
					else 
					{
						e.VSF = 1f;
						e.UpdateCurrentTorque(vessel.ctrlState.mainThrottle);
					}
				}
			}
			else
			{
				for(int i = 0; i < NumActive; i++)
				{
					var e = Active[i];
					e.VSF = 1f;
					e.UpdateCurrentTorque(vessel.ctrlState.mainThrottle);
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
				e.preset_limit = false;
			}
			VSL.Controls.ManualTranslationSwitch.Checked();
			if(NoActiveRCS) return;
			for(int i = 0; i < NumActiveRCS; i++)
			{
				var t = ActiveRCS[i];
				t.thrustLimit = Mathf.Clamp01(t.limit);
				t.preset_limit = false;
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
			var this_engines = All.Where(e => e.Role != TCARole.MANEUVER && e.part.inverseStage >= vessel.currentStage).ToList();
			return stage_cooldown.RunIf(VSL.ActivateNextStage,
			                            this_engines.Count == 0 ||
			                            this_engines.Any(e => e.engine.flameout));
		}
		readonly Timer stage_cooldown = new Timer(0.5);

		public bool ActivateEngines()
		{ return VSL.Engines.ActivateNextStageOnFlameout() || VSL.Engines.ActivateInactiveEngines(); }

		static void collect_fuels(EngineWrapper e, Dictionary<int, PartResourceDefinition> db)
		{
			e.engine.GetConsumedResources()
				.ForEach(r => { if(!db.ContainsKey(r.id)) db.Add(r.id, r); });
		}

		public float GetAvailableFuelMass()
		{
			double fuel_mass = 0;
			double amount, max_amount;
			var fuels = new Dictionary<int, PartResourceDefinition>();
			Active.ForEach(e => collect_fuels(e, fuels));
			foreach(var r in fuels)
			{
				vessel.GetConnectedResourceTotals(r.Key, out amount, out max_amount);
				fuel_mass += amount * r.Value.density;
			}
			return (float)fuel_mass;
		}
	}
}

