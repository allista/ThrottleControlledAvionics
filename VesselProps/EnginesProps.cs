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
		public bool HaveThrusters { get; private set; }
		public bool ForceUpdateParts = false;

		public bool HaveNextStageEngines { get; private set; }
		public int  NearestEnginedStage { get; private set; }

		public KSPActionGroup ActionGroups  { get; private set; } = KSPActionGroup.None;

		public Vector3  Thrust { get; private set; } //current total thrust
		public Vector3  DefThrust { get; private set; }
		public Vector3  MaxThrust { get; private set; }
		public Vector3  MaxDefThrust { get; private set; }
		public Vector3  DefManualThrust { get; private set; }
		public Vector6  ManualThrustLimits { get; private set; } = Vector6.zero;
        public Vector6  ManualThrustSpeed { get; private set; } = Vector6.zero;
		public float    MaxThrustM { get; private set; }
		public float    MaxAccel { get; private set; }
		public float    TMR { get; private set; }
        public float    AccelerationSpeed { get; private set; } //thrust-wighted engine acceleration speed
        public float    DecelerationSpeed { get; private set; } //thrust-wighted engine deceleration speed
		public float    AccelerationTime90 { get; private set; } //time it takes to accelerate to 90% of thrust
        public float    DecelerationTime10 { get; private set; } //time it takes to decelerate to 10% of thrust
        public bool     Slow { get; private set; } //if there are engines that have non-zero engine acceleration/deceleration speed

		public float    MassFlow { get; private set; }
		public float    ManualMassFlow { get; private set; }
		public float    MaxMassFlow { get; private set; }
		public float    MaxVe { get; private set; } //Specific impulse in m/s, aka effective exhaust velocity

		public override void Clear() { All.Clear(); RCS.Clear(); }

		public bool AddEngine(PartModule pm) 
		{ 
			var engine = pm as ModuleEngines;
			if(engine == null) return false;
			All.Add(new EngineWrapper(engine));
			return true;
		}

		public bool AddRCS(PartModule pm)
		{ 
			var rcs = pm as ModuleRCS;
			if(rcs == null) return false;
			RCS.Add(new RCSWrapper(rcs));
			return true;
		}

		public Vector3 refT_forward_axis
		{ get { return VSL.OnPlanetParams.NoseUp? VSL.refT.forward : VSL.refT.up; } }

		public Vector3 refT_thrust_axis
		{ get { return VSL.OnPlanetParams.NoseUp? VSL.refT.up : VSL.refT.forward; } }

		public Vector3 FallbackThrustDir(Vector3 fallback)
		{ return fallback.IsZero()? -refT_thrust_axis : fallback.normalized; }

		public Vector3 CurrentDefThrustDir 
		{ get { return MaxDefThrust.IsZero()? FallbackThrustDir(NearestEnginedStageMaxDefThrust) : MaxDefThrust.normalized; } }

		public Vector3 CurrentMaxThrustDir 
		{ get { return MaxThrust.IsZero()? FallbackThrustDir(NearestEnginedStageMaxThrust) : MaxThrust.normalized; } }

		public Vector3 CurrentThrustDir 
		{ get { return Thrust.IsZero()? CurrentMaxThrustDir : Thrust.normalized; } }

		public float ThrustAtAlt(float vel, float alt, out float mflow)
		{
			var flow = 0f;
			var thrust = 0f;
            for(int i = 0; i < NumActive; i++) 
            {
                var e = Active[i];
                if(e.isVSC)
                {
    				float mFlow;
    				thrust += e.ThrustAtAlt(vel, alt, out mFlow); 
    				flow += mFlow; 
                }
			}
			mflow = flow;
			return thrust;
		}

        public static float TTB(float dV, float thrust, float mass, float mflow, float throttle, out float end_mass)
        {
            if(CheatOptions.InfinitePropellant)
            {
                end_mass = mass;
                return mass/thrust*dV/throttle;
            }
            var fuel = FuelNeeded(dV, thrust/mflow, mass);
            end_mass = mass-fuel;
            return fuel/mflow/throttle;
        }

		public float TTB(float dV, float thrust, float mflow, float throttle)
		{
			if(thrust.Equals(0)) return float.MaxValue;
			if(dV.Equals(0)) return 0;
			return (CheatOptions.InfinitePropellant?
                    VSL.Physics.M/thrust*dV : FuelNeeded(dV, thrust/mflow, VSL.Physics.M)/mflow) / throttle;
		}

		public float TTB(float dV, float throttle = -1)
		{ 
			if(throttle < 0) throttle = ThrottleControl.NextThrottle(dV, 1, VSL);
			return TTB(dV, MaxThrustM, MaxMassFlow, throttle); 
		}

        public float TTB_Precise(float dV)
        {
            var ttb = 0f;
            var mass = VSL.Physics.M;
            var throttle = ThrottleControl.NextThrottle(dV, 1, mass, MaxThrustM, DecelerationTime10);
            while(dV > 0.1)
            {
                throttle = ThrottleControl.NextThrottle(dV, throttle, mass, MaxThrustM, DecelerationTime10);
                dV /= 2;
                ttb += TTB(dV, MaxThrustM, mass, MaxMassFlow, throttle, out mass);
            }
            throttle = ThrottleControl.NextThrottle(dV, throttle, mass, MaxThrustM, DecelerationTime10);
            return ttb + TTB(dV, MaxThrustM, mass, MaxMassFlow, throttle, out mass);
        }

		public float AntigravTTB(float vertical_V, float throttle = -1)
		{
			if(throttle < 0) throttle = ThrottleControl.NextThrottle(vertical_V, 1, VSL);
			return VSL.Engines.TTB(vertical_V, 
			                       Utils.ClampL(MaxThrustM - VSL.Physics.StG*VSL.Physics.M, 1e-5f), 
			                       MaxMassFlow, throttle);
		}

		public float OnPlanetTTB(Vector3 dV, Vector3 up, float alt = float.MinValue)
		{
            var Vm = dV.sqrMagnitude;
			var vV = Vector3.Dot(dV, up);
			var hV = Mathf.Sqrt(Vm-vV*vV);
            var V = new Vector2(hV, vV);
            var V0 = V;
			Vm = Mathf.Sqrt(Vm);
			var mflow = MaxMassFlow;
			var thrust = MaxThrustM;
			if(VSL.vessel.mainBody.atmosphere && !alt.Equals(float.MinValue))
				thrust = ThrustAtAlt(Vm, alt, out mflow);
			var dT = TTB(Vm, thrust, mflow, 1)/10;
            var mass = VSL.Physics.M;
            var min_mass = mass-AvailableFuelMass;
            var g = Vector2.up*VSL.Physics.StG;
            var Ve = thrust/mflow;
            var T = 0f;
//            Log("dT {}, Vm {}, thrust {}, mflow {}, Ve {}, m {}, V0 {}, G {}", 
//                dT, Vm, thrust, mflow, Ve, mass, V0, VSL.Physics.StG);//debug
            while(mass > min_mass && Vector2.Dot(V, V0) > 0)
            {
                var dt = Mathf.Max(dT*V.magnitude/Vm, 0.1f);
                var m1 = mass-mflow*dt;
                V += V.normalized*Ve*Mathf.Log(m1/mass)-g*dt;
                mass = m1;
                T += dt;
//                Log("T {}, dT {}, V {}, m {} > {}, V*V0 {}", T, dt, V, mass, min_mass, Vector2.Dot(V, V0));//debug
            }
            return T;
		}

        public static float FuelNeeded(float dV, float Ve, float mass) { return mass*(1-Mathf.Exp(-dV/Ve)); }
        public float FuelNeeded(float dV, float Ve) { return FuelNeeded(dV, Ve, VSL.Physics.M); }
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
            return DeltaV(thrust/mflow, AvailableFuelMass); 
		}

        public float MaxDeltaV { get { return DeltaV(MaxVe, AvailableFuelMass); } }

		public float RelVeASL 
		{ 
			get 
			{ 
				float mflow;
				float thrust = ThrustAtAlt(0, 0, out mflow);
				return (float)(thrust - VSL.Body.GeeASL*Utils.G0*VSL.Physics.M)/mflow; 
			} 
		}

		public Vector3 NearestEnginedStageMaxDefThrust
		{ get { return GetNearestEnginedStageStats().MaxDefThrust; } }

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
						stats.MaxDefThrust += e.defThrustDir * thrust;
						stats.MaxMassFlow += e.MaxFuelFlow * throttle;
					}
					if(e.isSteering) stats.TorqueLimits.Add(e.specificTorque*thrust);
				}
			}
			stats.Update();
			return stats;
		}

		#region Clusters
		public ClustersDB Clusters;
		Vector3 local_dir_cluster_request;
		Vector3 dV_cluster_request;

		ActionDamper clusters_check_cooldown = new ActionDamper();
		Timer cluster_switch_cooldown = new Timer();

		public void RequestNearestClusterActivation(Vector3 local_dir)
		{ local_dir_cluster_request = local_dir; }

		public void RequestFastestClusterActivation(Vector3 dV)
		{ 
			CFG.SmartEngines.OnIfNot(SmartEnginesMode.Fastest);
			dV_cluster_request = dV; 
		}

		public void RequestBestClusterActivation(Vector3 dV)
		{ 
			CFG.SmartEngines.OnIfNot(SmartEnginesMode.Best);
			dV_cluster_request = dV; 
		}

		public void RequestClusterActivationForManeuver(Vector3 dV)
		{ 
			switch(CFG.SmartEngines.state)
			{
			case SmartEnginesMode.Closest:
				local_dir_cluster_request = -VSL.LocalDir(dV);
				break;
			case SmartEnginesMode.Fastest:
			case SmartEnginesMode.Best:
				dV_cluster_request = dV; 
				break;
			}
		}

		void activate_cluster(Func<bool> activate)
		{
			if(cluster_switch_cooldown.Start() ||
			   cluster_switch_cooldown.TimePassed)
			{
				bool activated = false;
				clusters_check_cooldown.Run(() => activated = activate());
				if(activated) cluster_switch_cooldown.Restart();
			}
		}

		void activate_cluster_by_request()
		{
			if(!local_dir_cluster_request.IsZero())
				activate_cluster(() => Clusters.ActivateClosest(local_dir_cluster_request));
			else if(!dV_cluster_request.IsZero())
			{
				if(CFG.SmartEngines.state == SmartEnginesMode.Fastest)
					activate_cluster(() => Clusters.ActivateFastest(dV_cluster_request));
				else if(CFG.SmartEngines.state == SmartEnginesMode.Best)
					activate_cluster(() => Clusters.ActivateBestForManeuver(dV_cluster_request));
			}
			local_dir_cluster_request = Vector3.zero;
			dV_cluster_request = Vector3.zero;
		}
		#endregion

		public bool Check()
		{
			//update engines' list if needed
			var num_engines = All.Count;
			if(!ForceUpdateParts)
			{
				for(int i = 0; i < num_engines; i++)
				{ ForceUpdateParts |= !All[i].Valid(VSL); if(ForceUpdateParts) break; }
				if(!ForceUpdateParts)
				{
					for(int i = 0; i < RCS.Count; i++)
					{ ForceUpdateParts |= !RCS[i].Valid(VSL); if(ForceUpdateParts) break; }
				}
			}
			//update parts if needed
			if(ForceUpdateParts) 
			{ 
				VSL.UpdateParts();
				num_engines = All.Count;
				ForceUpdateParts = false;
			}
			//update engine clusters if needed
			else if(Clusters.Dirty)
				Clusters.Update();
			//activate appropriate cluster if requested
            if(CFG.UseSmartEngines && Clusters.Multi && 
               (TimeWarp.WarpMode == TimeWarp.Modes.LOW ||
                TimeWarp.CurrentRateIndex == 0))
			{
//				Log("Clusters: inactive {}, mixed {}", Clusters.Inactive, Clusters.Mixed);//debug
				if(Clusters.Inactive) Clusters.ActivateClosest(Vector3.back);
				else if(Clusters.Mixed) Clusters.Reactivate();
				else activate_cluster_by_request();
			}
			//unflameout engines
            if(VSL.PreUpdateControls.mainThrottle > 0)
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
//			Log("All engines: {}", VSL.Engines.All);//debug
			//update active engines
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
			//update active RCS
			ActiveRCS.Clear();
			if(vessel.ActionGroups[KSPActionGroup.RCS])
			{
				for(int i = 0; i < RCS.Count; i++)
				{ var t = RCS[i]; if(t.isOperational) ActiveRCS.Add(t); }
			}
			//cache counts and flags
			NumActive = Active.Count;
			NumActiveRCS = ActiveRCS.Count;
			NoActiveEngines = NumActive == 0;
			NoActiveRCS = NumActiveRCS == 0 || 
				VSL.Controls.Steering.sqrMagnitude < GLB.InputDeadZone && 
				VSL.Controls.Translation.sqrMagnitude < GLB.InputDeadZone;
			//switch single coaxial engine to UnBalanced mode
			if(NumActive == 1)
			{
				var e = Active[0];
			  	if(e.Role != TCARole.UNBALANCE &&
			   	   e.Role != TCARole.MANUAL &&
                   e.defTorqueRatio < GLB.ENG.UnBalancedThreshold)
				{
					Utils.Message("{0} was switched to UnBalanced mode.", e.name);
					e.SetRole(TCARole.UNBALANCE);
					if(VSL.TCA.ProfileSyncAllowed)
						CFG.ActiveProfile.Update(All);
				}
			}
			return !(NoActiveEngines && NoActiveRCS);
		}

		public void Sort() 
		{ 
			Active.SortByRole(); 
			HaveMainEngines = Active.Main.Count > 0;
			HaveThrusters = HaveMainEngines || Active.Balanced.Count > 0 || Active.UnBalanced.Count > 0;
			VSL.Controls.TranslationAvailable = VSL.Engines.Active.Maneuver.Count > 0 || VSL.Engines.NumActiveRCS > 0;
		}

		void update_MaxThrust()
		{
			//first optimize engines for zero torque to have actual MaxThrust in case of unbalanced ship design
			if(VSL.Torque != null && VSL.TCA != null && VSL.TCA.ENG != null)
			{
				if(Active.Balanced.Count > 0)
				{
					VSL.Torque.UpdateImbalance(true, Active.Manual, Active.UnBalanced);
                    VSL.TCA.ENG.OptimizeLimitsForTorque(Active.Balanced, Vector3.zero, true);
				}
				VSL.Torque.UpdateImbalance(true, Active.Manual, Active.UnBalanced, Active.Balanced);
				VSL.TCA.ENG.OptimizeLimitsForTorque(Active.Steering, Vector3.zero, true);
			}
            AccelerationSpeed = 0f;
            DecelerationSpeed = 0;
            AccelerationTime90 = 0f;
            DecelerationTime10 = 0f;
			MaxDefThrust = Vector3.zero;
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
					MaxDefThrust += e.defThrustDir*thrust;
					MaxThrust += e.wThrustDir*thrust;
					MaxMassFlow += e.MaxFuelFlow*e.limit;
					if(e.useEngineResponseTime && e.finalThrust > 0)
					{
						if(e.engineDecelerationSpeed > 0)
							DecelerationSpeed += thrust*e.engineDecelerationSpeed;
						if(e.engineAccelerationSpeed > 0)
							AccelerationSpeed += thrust*e.engineAccelerationSpeed;
					}
				}
				if(e.isSteering && have_steering) e.InitLimits();
			}
            if(AccelerationSpeed > 0)
            { 
                AccelerationSpeed /= total_thrust; 
                AccelerationTime90 = Utils.LerpTime(AccelerationSpeed, 0.9f);
                Slow = true;
            }
            if(DecelerationSpeed > 0) 
            { 
                DecelerationSpeed /= total_thrust; 
                DecelerationTime10 = Utils.LerpTime(DecelerationSpeed, 0.9f);
                Slow = true;
            }
			if(MassFlow > MaxMassFlow) MaxMassFlow = MassFlow;
			MaxThrustM = MaxThrust.magnitude;
			MaxVe = MaxThrustM/MaxMassFlow;
			MaxAccel = MaxThrustM/VSL.Physics.M;
			TMR = MaxAccel/Utils.G0;
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
				t.ApplyPreset();
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
			DefThrust = Vector3.zero;
			DefManualThrust = Vector3.zero;
			ManualThrustLimits = Vector6.zero;
            ManualThrustSpeed = Vector6.zero;
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
				{
					Thrust += e.wThrustDir*e.finalThrust;
					DefThrust += e.defThrustDir*e.finalThrust;
				}
				if(e.Role == TCARole.MANUAL)
				{
					DefManualThrust += e.defThrustDir*e.finalThrust;
					ManualMassFlow += e.RealFuelFlow;
                    var man_thrust = e.defThrustDirL*e.nominalFullThrust;
                    ManualThrustLimits.Add(man_thrust);
                    if(e.useEngineResponseTime)
                        ManualThrustSpeed.Add(man_thrust*Mathf.Max(e.engineAccelerationSpeed, e.engineDecelerationSpeed));
				}
				MassFlow += e.RealFuelFlow;
			}
            ManualThrustSpeed.Scale(ManualThrustLimits.Inverse());
			update_MaxThrust();
			update_RCS();
			//update engines' current torque
            var throttle = VSL.PreUpdateControls.mainThrottle;
			var vsc_throttle = (VSL.OnPlanet && CFG.VSCIsActive)? throttle*VSL.OnPlanetParams.GeeVSF : throttle;
			for(int i = 0; i < NumActive; i++) 
			{
				var e = Active[i];
				e.UpdateCurrentTorque(e.isVSC? vsc_throttle : throttle);
				e.ApplyPreset();
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
                            anti_min_imbalance += e.specificTorque * e.nominalFullThrust;
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
                        e.UpdateCurrentTorque(e.VSF * VSL.PostUpdateControls.mainThrottle);
					}
					else 
					{
						e.VSF = 1f;
                        e.UpdateCurrentTorque(VSL.PostUpdateControls.mainThrottle);
					}
				}
			}
			else
			{
				for(int i = 0; i < NumActive; i++)
				{
					var e = Active[i];
					e.VSF = 1f;
                    e.UpdateCurrentTorque(VSL.PostUpdateControls.mainThrottle);
				}
			}
		}

		public void SetControls()
		{
			for(int i = 0; i < NumActive; i++)
			{
				var e = Active[i];
				if(e.gimbal != null && e.Role != TCARole.MANEUVER)
					e.gimbal.gimbalLimiter = VSL.Controls.GimbalLimit;
				if(!Equals(e.Role, TCARole.MANUAL))
					e.thrustLimit = Mathf.Clamp01(e.VSF * e.limit);
				else if(VSL.Controls.ManualTranslationSwitch.On)
					e.forceThrustPercentage(e.limit*100);
				else if(VSL.Controls.ManualTranslationSwitch.WasSet)
					e.forceThrustPercentage(0);
				e.preset_limit = -1;
			}
			VSL.Controls.ManualTranslationSwitch.Checked();
			if(NoActiveRCS) return;
			for(int i = 0; i < NumActiveRCS; i++)
			{
				var t = ActiveRCS[i];
				t.thrustLimit = Mathf.Clamp01(t.limit);
				t.preset_limit = -1;
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
//            Log("NoActiveEngines {}, HaveThrusters {}, HaveNextStage {}", NoActiveEngines, HaveThrusters, HaveNextStageEngines);//debug
			if(!CFG.AutoStage || !HaveNextStageEngines) return false;
			var this_engines = All.Where(e => e.Role != TCARole.MANEUVER && e.part.inverseStage >= vessel.currentStage).ToList();
//            Log("this stage engines: {}", this_engines);//debug
            if(this_engines.Count == 0 || this_engines.Any(e => e.engine.flameout))
            { VSL.ActivateNextStage(); return true; }
            return false;
		}
		public bool ActivateEngines()
		{ return VSL.Engines.ActivateNextStageOnFlameout() || VSL.Engines.ActivateInactiveEngines(); }

        public void ActivateEnginesAndRun(Callback action)
        {
            if(NoActiveEngines)
            {
                if(CFG.AutoStage)
                {
                    if(HaveNextStageEngines)
                    {
                        VSL.ActivateNextStageImmidiate();
                        VSL.TCA.StartCoroutine(CallbackUtil.WaitUntil(() => !NoActiveEngines, action));
                    }
                    else Utils.Message("No engines left to activate");
                }
                else Utils.Message("Automatic staging is disabled");
            }
            else action();
        }

		static void collect_fuels(EngineWrapper e, Dictionary<int, PartResourceDefinition> db)
		{
			e.engine.GetConsumedResources()
				.ForEach(r => { if(!db.ContainsKey(r.id)) db.Add(r.id, r); });
		}

        float _AvailableFuelMass = -1;
		public float AvailableFuelMass
		{
            get
            {
                if(_AvailableFuelMass < 0)
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
                    _AvailableFuelMass = (float)fuel_mass;
                }
                return _AvailableFuelMass;
            }
		}

        public override void ClearFrameState()
        {
            base.ClearFrameState();
            _AvailableFuelMass = -1;
        }
	}

    public class EnginesStats : VesselProps
    {
        public EnginesStats(VesselWrapper vsl) : base(vsl) 
        { TorqueInfo = new TorqueInfo(vsl); }

        public Vector3 MaxThrust;
        public Vector3 MaxDefThrust;
        public double  MaxMassFlow;
        public double  MaxFuelMass;
        public double  MaxDeltaV;

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

    public class EngineCluster : VesselProps
    {
        public const float MaxDistance = 0.70710678f; //45 deg

        public bool Enabled { get; private set; }
        public List<EngineWrapper> Engines { get; private set; }

        public Vector3 Dir { get; private set; } = Vector3.zero;
        public Vector3 MaxThrust { get; private set; } = Vector3.zero;
        public double  MaxMassFlow { get; private set; } = 0;

        public float DistanceTo(Vector3 local_dir)
        { return 1-Vector3.Dot(local_dir, Dir); }

        public float DistanceTo(EngineWrapper e)
        { return DistanceTo(VSL.LocalDir(e.defThrustDir)); }

        public EngineCluster(VesselWrapper vsl) : base(vsl) { Engines = new List<EngineWrapper>(); }
        public EngineCluster(VesselWrapper vsl, EngineWrapper e) : this(vsl) { Add(e); }
        public EngineCluster(VesselWrapper vsl, params EngineWrapper[] engines) : this(vsl) { Add(engines); }

        public void Add(EngineWrapper e)
        {
            Engines.Add(e);
            MaxThrust = MaxThrust + VSL.LocalDir(e.defThrustDir) * e.engine.maxThrust;
            MaxMassFlow = MaxMassFlow + e.engine.maxFuelFlow;
            Dir = MaxThrust.normalized;
        }

        public void Add(params EngineWrapper[] engines)
        {
            for(int i = 0, len = engines.Length; i < len; i++)
            {
                var e = engines[i];
                Engines.Add(e);
                MaxThrust = MaxThrust + VSL.LocalDir(e.defThrustDir) * e.engine.maxThrust;
                MaxMassFlow = MaxMassFlow + e.engine.maxFuelFlow;
            }
            Dir = MaxThrust.normalized;
        }

        static void enable_engine(EngineWrapper e)
        { if(!e.engine.EngineIgnited) e.engine.Activate(); }

        static void disable_engine(EngineWrapper e)
        { if(e.engine.EngineIgnited) e.engine.Shutdown(); }

        public void Enable(bool enable = true)
        {
            if(enable) 
            {
                Engines.ForEach(enable_engine);
                Enabled = true;
            }
            else
            {
                Engines.ForEach(disable_engine);
                Enabled = false;
            }
        }

        public override string ToString()
        {
            return string.Format("EngineCluster: Enabled: {0}, Engines: {1}, MaxMassFlow: {2}\nMaxThrust: {3}", 
                                 Enabled, Engines.Count, MaxMassFlow, MaxThrust);
        }
    }

    public class ClustersDB : VesselProps
    {
        List<EngineCluster> clusters = new List<EngineCluster>();
        Transform savedRefT;

        public EngineCluster Active { get; private set; }
        public int Count { get { return clusters.Count; } }
        public bool Inactive { get { return Active == null; } }
        public bool Multi { get { return clusters.Count > 1; } }
        public bool Dirty { get { return refT != savedRefT; } }
        public bool Mixed
        {
            get
            {
                for(int j = 0, clustersCount = clusters.Count; j < clustersCount; j++)
                {
                    var c = clusters[j];
                    if(c == Active)
                    {
                        if(c.Engines.Any(e => !e.engine.EngineIgnited))
                            return true;
                    }
                    else
                    {
                        if(c.Engines.Any(e => e.engine.EngineIgnited))
                            return true;
                    }
                }
                return false;
            }
        }

        public ClustersDB(VesselWrapper vsl) : base(vsl) {}

        public override void Update()
        {
            var active_dir = Vector3.zero;
            //deactivate engines if in SmartEngines mode
            if(CFG.UseSmartEngines) 
            {
                //save active direction, if any
                if(Active != null) 
                    active_dir = VSL.WorldDir(Active.Dir);
                Deactivate();
            }
            clusters.Clear();
            Active = null;
            //repartition enines into clusters
            savedRefT = refT;
            VSL.Engines.All.Sort((a, b) => b.engine.maxThrust.CompareTo(a.engine.maxThrust));
            for(int i = 0, enginesCount = VSL.Engines.All.Count; i < enginesCount; i++)
            {
                var e = VSL.Engines.All[i];
                //do not include maneuver or manual engines into clusters
                if(e.Role == TCARole.MANEUVER || e.Role == TCARole.MANUAL)
                    continue;
                //do not include engines with stack-attached children; these are probably blocked by decouplers
                if(e.part.children.Any(ch => ch.srfAttachNode == null || ch.srfAttachNode.attachedPart != e.part))
                    continue;
                e.UpdateThrustInfo();
                float min_dist;
                var closest = find_closest(VSL.LocalDir(e.defThrustDir), out min_dist);
                if(min_dist < EngineCluster.MaxDistance) closest.Add(e);
                else clusters.Add(new EngineCluster(VSL, e));
            }
            //activate the cluster that is nearest to the previous active direction
            if(CFG.UseSmartEngines && !active_dir.IsZero())
                activate(Closest(VSL.LocalDir(active_dir)));
        }

        EngineCluster find_closest(Vector3 local_dir, out float min_dist)
        {
            min_dist = float.MaxValue;
            EngineCluster closest = null;
            for(int j = 0, clustersCount = clusters.Count; j < clustersCount; j++)
            {
                var c = clusters[j];
                var d = c.DistanceTo(local_dir);
                if(d < min_dist)
                {
                    min_dist = d;
                    closest = c;
                }
            }
            return closest;
        }

        public EngineCluster Closest(Vector3 local_dir)
        {
            float min_dist;
            return find_closest(local_dir, out min_dist);
        }

        public EngineCluster Fastest(Vector3 dV)
        {
            var dVm = dV.magnitude;
            var loc_dir = -VSL.LocalDir(dV);
            var min_time = float.MaxValue;
            EngineCluster fastest = null;
            for(int j = 0, clustersCount = clusters.Count; j < clustersCount; j++)
            {
                var c = clusters[j];
                var time = VSL.Torque.NoEngines.RotationTime2Phase(Utils.Angle2(loc_dir, c.Dir), 1);
                if(VSL.Info.Countdown > 0 && time > VSL.Info.Countdown) continue;
                time += VSL.Engines.TTB(dVm, c.MaxThrust.magnitude, (float)c.MaxMassFlow, 1);
                if(time < min_time)
                {
                    min_time = time;
                    fastest = c;
                }
            }
            return fastest ?? Closest(loc_dir);
        }

        public EngineCluster BestForManeuver(Vector3 dV)
        {
            var dVm = dV.magnitude;
            var loc_dV = -VSL.LocalDir(dV);
            var min_score = float.MaxValue;
            EngineCluster best = null;
            for(int j = 0, clustersCount = clusters.Count; j < clustersCount; j++)
            {
                var c = clusters[j];
                var score = VSL.Torque.NoEngines.RotationTime2Phase(Utils.Angle2(loc_dV, c.Dir), 1);
                var thrust = c.MaxThrust.magnitude;
                var ttb = VSL.Engines.TTB(dVm, thrust, (float)c.MaxMassFlow, 1);
                if(VSL.Info.Countdown > 0 && score+ttb/2 > VSL.Info.Countdown) continue;
                if(ttb < GLB.MAN.ClosestCluster) ttb = 0;
                score += ttb;
                var fuel = VSL.Engines.FuelNeeded(dVm, (float)(thrust/c.MaxMassFlow));
                if(fuel/VSL.Physics.M > GLB.MAN.EfficientCluster) score += fuel*GLB.MAN.EfficiencyWeight;
                if(score < min_score)
                {
                    min_score = score;
                    best = c;
                }
            }
            return best ?? Closest(loc_dV);
        }

        bool activate(EngineCluster cluster)
        {
            if(cluster == null || cluster == Active) return false;
            Deactivate();
            cluster.Enable();
            CFG.ActiveProfile.Update(VSL.Engines.All, true);
            Active = cluster;
            return true;
        }

        public bool ActivateClosest(Vector3 local_dir)
        { return activate(Closest(local_dir)); }

        public bool ActivateFastest(Vector3 dV)
        { return activate(Fastest(dV)); }

        public bool ActivateBestForManeuver(Vector3 dV)
        { return activate(BestForManeuver(dV)); }

        public void Deactivate()
        { 
            clusters.ForEach(c => c.Enable(false));
            Active = null;
        }

        public void Reactivate()
        {
            if(Active == null) return;
            var c = Active;
            Active = null;
            activate(c);
        }
    }
}

