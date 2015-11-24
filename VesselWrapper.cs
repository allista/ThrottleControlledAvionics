//   VesselWrapper.cs
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
using System.Collections.Generic;
using UnityEngine;

namespace ThrottleControlledAvionics
{
	public class VesselWrapper
	{
		const float Gee = 9.82f;

		const string ElectricChargeName = "ElectricCharge";
		static PartResourceDefinition _electric_charge;
		public static PartResourceDefinition ElectricCharge
		{ 
			get
			{ 
				if(_electric_charge == null)
					_electric_charge = PartResourceLibrary.Instance.GetDefinition(ElectricChargeName);
				return _electric_charge;
			} 
		}

		public ModuleTCA TCA { get; private set; }
		public Vessel vessel { get; private set; }
		public Transform refT { get; private set; } //transform of the controller-part
		public VesselConfig CFG { get; private set; }
		public TCAGlobals GLB { get { return TCAScenario.Globals; } }

		public Vector3 LocalDir(Vector3 worldV) { return refT.InverseTransformDirection(worldV); }
		public Vector3 WorldDir(Vector3 localV) { return refT.TransformDirection(localV); }

		public List<EngineWrapper> Engines          = new List<EngineWrapper>();
		public List<EngineWrapper> ActiveEngines    = new List<EngineWrapper>();
		public List<EngineWrapper> BalancedEngines  = new List<EngineWrapper>();
		public List<EngineWrapper> ManeuverEngines  = new List<EngineWrapper>();
		public List<EngineWrapper> SteeringEngines  = new List<EngineWrapper>();
		public List<EngineWrapper> ManualEngines    = new List<EngineWrapper>();
		public List<ModuleReactionWheel> RWheels    = new List<ModuleReactionWheel>();
		public List<RCSWrapper> RCS = new List<RCSWrapper>();
		public List<RCSWrapper> ActiveRCS = new List<RCSWrapper>();

		public bool NoActiveRCS { get; private set; }
		public int  NumActiveEngines { get; private set; }
		public int  NumActiveRCS { get; private set; }
		public bool CanUpdateEngines = true;
		public bool ForceUpdateParts;

		//physics
		public Vector6 E_TorqueLimits { get; private set; } = Vector6.zero; //torque limits of engines
		public Vector6 W_TorqueLimits { get; private set; } = Vector6.zero; //torque limits of reaction wheels
		public Vector6 R_TorqueLimits { get; private set; } = Vector6.zero; //torque limits of rcs
		public Vector6 ManualThrustLimits { get; private set; } = Vector6.zero;
		public Bounds  EnginesExhaust { get; private set; }

		public float DistToExhaust(Vector3 world_point)
		{ return Mathf.Sqrt(EnginesExhaust.SqrDistance(refT.InverseTransformPoint(world_point))); }

		public Bounds  B { get; private set; } //bounds
		public Vector3 C { get; private set; } //center
		public float   H { get; private set; } //height
		public float   R { get; private set; } //radius
		public float   M { get; private set; } //mass
		public float   StG { get; private set; } //gee at position
		public float   G { get; private set; } //gee - centrifugal acceleration
		public double  UT { get; private set; } //Planetarium.GetUniversalTime
		public float  DTWR { get; private set; }
		public float  MaxDTWR { get; private set; }
		public float  MaxTWR { get; private set; }
		public float  AccelSpeed { get; private set; }
		public float  DecelSpeed { get; private set; }
		public float  ThrustDecelerationTime { get; private set; }
		public bool   SlowThrust { get; private set; }
		public bool   SlowTorque { get; private set; }
		public float  TorqueResponseTime { get; private set; }
		public float  VSF; //vertical speed factor
		public float  MinVSF;
		public float  MinVSFtwr;

		public Vector3d   Up { get; private set; }  //up unit vector in world space
		public Vector3d   UpL { get; private set; }  //up unit vector in world space
		public Vector3    Fwd { get; private set; }  //fwd unit vector of the Control module in world space
		public Vector3    FwdL { get; private set; }  //fwd unit vector of the Control module in world space
		public Vector3    HFwd { get; private set; }  //fwd unit vector of the Control module in world space
		public bool       NoseUp { get; private set; }  //if the forward is refT.forward or refT.up
		public Vector3    wCoM { get; private set; } //center of mass in world space
		public Vector3    MoI { get; private set; } = Vector3.one; //main diagonal of inertia tensor
		public Matrix3x3f InertiaTensor { get; private set; }
		public Vector3    MaxAngularA { get; private set; } //current maximum angular acceleration
		public Vector3    wMaxAngularA { get; private set; } //current maximum angular acceleration
		public float      MaxAngularA_m { get; private set; }
		public float      MaxAAMod { get; private set; }
		LowPassFilterF    MaxAAFilter = new LowPassFilterF();
		public Vector3    MaxPitchRollAA { get; private set; }
		public float      MaxPitchRollAA_m { get; private set; }

		public Vector3  Thrust { get; private set; } //current total thrust
		public Vector3  MaxThrust { get; private set; }
		public float    MaxThrustM { get; private set; }
		public Vector3  ManualThrust { get; private set; }
		public Vector3  Torque { get; private set; } //current torque applied to the vessel by engines
		public Vector3  MaxTorque { get; private set; }
		public float    MaxMassFlow { get; private set; }
		public float    MassFlow { get; private set; }
		public float    AbsVerticalSpeed { get; private set; }
		public float    RelVerticalSpeed { get; private set; }
		public float    VerticalSpeed { get; private set; }
		public float    VerticalSpeedDisp { get; private set; }
		public float    VerticalAccel { get; private set; }
		public float    Altitude { get; private set; }
		public float    AbsAltitude { get; private set; }
		public float    RelAltitude { get; private set; }
		public float    PrevRelAltitude { get; private set; }
		public float    AltitudeAhead;
		public float    TimeAhead;
		public float    TerrainAltitude { get; private set; }
		public Vector3d HorizontalVelocity { get; private set; }
		public float    HorizontalSpeed { get; private set; }
		public Vector3d PredictedSrfVelocity(float time) { return vessel.srf_velocity+vessel.acceleration*time; }
		public Vector3d PredictedHorVelocity(float time) { return Vector3d.Exclude(Up, vessel.srf_velocity+vessel.acceleration*time); }
		public bool     AltitudeAboveGround { get; private set; }
		public Vector3d NeededHorVelocity;
		public Vector3d ForwardDirection;
		public List<Vector3d> CourseCorrections = new List<Vector3d>();
		public Vector3d CourseCorrection;
		public Vector3  Destination;

		public TCAState State;
		public void SetState(TCAState state) { State |= state; }
		public bool IsStateSet(TCAState state) { return (State & state) == state; }
		public bool TranslationAvailable { get; private set; }
		public bool ElectricChargeAvailible
		{
			get
			{
				var ec = vessel.GetActiveResource(ElectricCharge);
				return ec != null && ec.amount > 0;
			}
		}

		public CelestialBody mainBody { get { return vessel.mainBody; } }
		public Orbit orbit { get { return vessel.orbit; } }
		public bool OnPlanet { get; private set; }
		public bool InOrbit { get; private set; }
		public bool isEVA { get { return vessel.isEVA; } }
		public bool IsActiveVessel { get; private set; }
		public bool LandedOrSplashed { get { return vessel.LandedOrSplashed; } }
		public ActionGroupList ActionGroups { get { return vessel.ActionGroups; } }
		public ITargetable Target { get { return vessel.targetObject; } set { vessel.targetObject = value; } }
		public bool HasTarget { get { return vessel.targetObject != null && !(vessel.targetObject is CelestialBody); } }
		public bool HasManeuverNode { get { return vessel.patchedConicSolver != null && vessel.patchedConicSolver.maneuverNodes.Count > 0; } }
		public Vessel.Situations Situation { get { return vessel.situation; } }
		//controls
		public FlightCtrlState ctrlState { get { return vessel.ctrlState; } }
		public FlightInputCallback OnAutopilotUpdate 
		{ get { return vessel.OnAutopilotUpdate; } set { vessel.OnAutopilotUpdate = value; } }
		//steering and translation
		public Vector3 Steering { get; private set; }
		public Vector3 Translation { get; private set; }
		public Vector3 ManualTranslation;
		public Switch ManualTranslationSwitch = new Switch();
		//maneuvering
		public double Countdown;
		public float TTB;

		public VesselWrapper(ModuleTCA tca) 
		{ TCA = tca; vessel = tca.vessel; CFG = tca.CFG; }

		public void Log(string msg, params object[] args) { vessel.Log(msg, args); }

		public void Init() 
		{
			CanUpdateEngines = true;
			AltitudeAhead = float.MinValue;
			OnPlanet = _OnPlanet();
			InOrbit = _InOrbit();
			MaxAAFilter.Tau = GLB.MaxAAFilter;
		}

		bool _OnPlanet() 
		{ 
			return (vessel.situation != Vessel.Situations.DOCKED   &&
			        vessel.situation != Vessel.Situations.ORBITING &&
			        vessel.situation != Vessel.Situations.ESCAPING); 
		}

		bool _InOrbit()
		{
			return vessel.situation == Vessel.Situations.ORBITING ||
				vessel.situation == Vessel.Situations.SUB_ORBITAL;
		}

		public bool AutopilotDisabled { get; private set; }
		public void UpdateAutopilotInfo(FlightCtrlState s)
		{
			AutopilotDisabled = 
				!Mathfx.Approx(s.pitch, s.pitchTrim, 0.1f) ||
				!Mathfx.Approx(s.roll, s.rollTrim, 0.1f) ||
				!Mathfx.Approx(s.yaw, s.yawTrim, 0.1f);
		}

		public void SetNeededHorVelocity(Vector3d hV)
		{
			CFG.SavedUp = Up;
			CFG.NeededHorVelocity = hV;
			NeededHorVelocity = hV;
		}

		public void SetUnpackDistance(float distance)
		{
			var sit = vessel.vesselRanges.GetSituationRanges(vessel.situation);
			sit.pack = distance*1.5f;
			sit.unpack = distance;
		}

		#region Engines
		public void UpdateParts()
		{
			EngineWrapper.ThrustPI.setMaster(CFG.Engines);
			Engines.Clear(); RCS.Clear(); RWheels.Clear();
			for(int i = 0, vesselPartsCount = vessel.Parts.Count; i < vesselPartsCount; i++)
			{
				Part p = vessel.Parts[i];
				for(int j = 0, pModulesCount = p.Modules.Count; j < pModulesCount; j++)
				{
					//engines
					var module = p.Modules[j];
					var engine = module as ModuleEngines;
					if(engine != null)
					{
						Engines.Add(new EngineWrapper(engine));
						continue;
					}
					//reaction wheels
					var rwheel = module as ModuleReactionWheel;
					if(rwheel != null)
					{
						RWheels.Add(rwheel);
						continue;
					}
					//rcs
					var rcs = module as ModuleRCS;
					if(rcs != null)
					{
						RCS.Add(new RCSWrapper(rcs));
						continue;
					}
				}
			}
			if(CFG.EnginesProfiles.Empty) CFG.EnginesProfiles.AddProfile(Engines);
			else if(CFG.Enabled && CanUpdateEngines) CFG.ActiveProfile.Update(Engines);
		}

		public bool CheckEngines()
		{
			//update engines' list if needed
			var num_engines = Engines.Count;
			if(!ForceUpdateParts)
			{
				for(int i = 0; i < num_engines; i++)
				{ ForceUpdateParts |= !Engines[i].Valid(this); if(ForceUpdateParts) break; }
				if(!ForceUpdateParts)
				{
					for(int i = 0; i < RCS.Count; i++)
					{ ForceUpdateParts |= !RCS[i].Valid(this); if(ForceUpdateParts) break; }
				}
			}
			if(ForceUpdateParts) 
			{ 
				UpdateParts(); 
				num_engines = Engines.Count;
				ForceUpdateParts = false;
			}
			//unflameout engines
			for(int i = 0; i < num_engines; i++)
			{ var e = Engines[i]; if(e.engine.flameout) e.forceThrustPercentage(1); }
			//sync with active profile
			if(CFG.ActiveProfile.Activated) CFG.ActiveProfile.OnActivated(this);
			if(CanUpdateEngines)
			{
				if(CFG.ActiveProfile.Changed) CFG.ActiveProfile.Apply(Engines);
				else CFG.ActiveProfile.Update(Engines);
			}
			//get active engines
			ActiveEngines.Clear(); ActiveEngines.Capacity = Engines.Count;
			for(int i = 0; i < num_engines; i++)
			{ var e = Engines[i]; if(e.isOperational) ActiveEngines.Add(e); }
			ActiveRCS = vessel.ActionGroups[KSPActionGroup.RCS]? 
				RCS.Where(t => t.isOperational).ToList() : new List<RCSWrapper>();
			NumActiveEngines = ActiveEngines.Count;
			NumActiveRCS = ActiveRCS.Count;
			NoActiveRCS = NumActiveRCS == 0 || 
				Steering.sqrMagnitude < GLB.InputDeadZone && 
				Translation.sqrMagnitude < GLB.InputDeadZone;
			return NumActiveEngines > 0 && vessel.ctrlState.mainThrottle > 0 || !NoActiveRCS;
		}

		public void SortEngines()
		{
			SteeringEngines.Clear(); SteeringEngines.Capacity = NumActiveEngines;
			ManeuverEngines.Clear(); ManeuverEngines.Capacity = NumActiveEngines;
			BalancedEngines.Clear(); BalancedEngines.Capacity = NumActiveEngines;
			ManualEngines.Clear();   ManualEngines.Capacity   = NumActiveEngines;
			for(int i = 0; i < NumActiveEngines; i++)
			{
				var e = ActiveEngines[i];
				switch(e.Role)
				{
				case TCARole.MAIN:
					SteeringEngines.Add(e);
					break;
				case TCARole.MANEUVER:
					SteeringEngines.Add(e);
					ManeuverEngines.Add(e);
					break;
				case TCARole.BALANCE:
					BalancedEngines.Add(e);
					break;
				case TCARole.MANUAL:
					ManualEngines.Add(e);
					break;
				}
			}
		}

		public void TuneEngines()
		{
			//calculate VSF correction
			if(IsStateSet(TCAState.VerticalSpeedControl))
			{
				//calculate min imbalance
				var min_imbalance = Vector3.zero;
				for(int i = 0; i < NumActiveEngines; i++) min_imbalance += ActiveEngines[i].Torque(0);
				min_imbalance = E_TorqueLimits.Clamp(min_imbalance);
				//correct VerticalSpeedFactor if needed
				if(!min_imbalance.IsZero())
				{
					var anti_min_imbalance = Vector3.zero;
					for(int i = 0; i < NumActiveEngines; i++)
					{
						var e = ActiveEngines[i];
						if(Vector3.Dot(e.specificTorque, min_imbalance) < 0)
							anti_min_imbalance += e.specificTorque * e.nominalCurrentThrust(1);
					}
					anti_min_imbalance = Vector3.Project(anti_min_imbalance, min_imbalance);
					VSF = Mathf.Clamp(VSF, Mathf.Clamp01(min_imbalance.magnitude/anti_min_imbalance.magnitude
					                                     *GLB.VSC.BalanceCorrection), 1f);
				}
				for(int i = 0; i < NumActiveEngines; i++)
				{
					var e = ActiveEngines[i];
					if(e.isVSC)
					{
						e.VSF = e.VSF > 0 ? VSF : MinVSF;
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
				for(int i = 0; i < NumActiveEngines; i++)
				{
					var e = ActiveEngines[i];
					e.VSF = 1f;
					e.throttle = vessel.ctrlState.mainThrottle;
					e.currentTorque = e.Torque(e.throttle);
					e.currentTorque_m = e.currentTorque.magnitude;
				}
			}
		}

		public void SetEnginesControls()
		{
			for(int i = 0; i < NumActiveEngines; i++)
			{
				var e = ActiveEngines[i];
				if(e.gimbal != null) 
					e.gimbal.gimbalLimiter = TCA.ATC.GimbalLimit;
				if(!Equals(e.Role, TCARole.MANUAL))
					e.thrustLimit = Mathf.Clamp01(e.VSF * e.limit);
				else if(ManualTranslationSwitch.On)
					e.forceThrustPercentage(e.limit*100);
				else if(ManualTranslationSwitch.WasSet)
					e.engine.thrustPercentage = 0;
			}
			ManualTranslationSwitch.Checked();
			if(NoActiveRCS) return;
			for(int i = 0; i < NumActiveRCS; i++)
			{
				var t = ActiveRCS[i];
				t.thrustLimit = Mathf.Clamp01(t.limit);
			}
		}
		#endregion

		#region Updates
		public void UpdateState()
		{
			//update onPlanet state
			var on_planet = _OnPlanet();
			var in_orbit = _InOrbit();
			if(on_planet != OnPlanet) 
			{
				CFG.EnginesProfiles.OnPlanetChanged(on_planet);
				if(!on_planet) 
				{ 
					if(CFG.BlockThrottle) 
						TCA.THR.Throttle = 0f;
					CFG.DisableVSC();
					CFG.Nav.Off(); 
					CFG.HF.Off();
				}
			}
			OnPlanet = on_planet;
			InOrbit = in_orbit;
			IsActiveVessel = vessel != null && vessel == FlightGlobals.ActiveVessel;
			if(!CFG.HF && !CFG.AT) UnblockSAS();
		}

		public void UpdatePhysicsParams()
		{
			UT   = Planetarium.GetUniversalTime();
			wCoM = vessel.CurrentCoM;
			refT = vessel.ReferenceTransform;
			Up   = (wCoM - vessel.mainBody.position).normalized;
			UpL  = refT.InverseTransformDirection(Up);
			M    = vessel.GetTotalMass();
			StG  = (float)(vessel.mainBody.gMagnitudeAtCenter/(vessel.mainBody.position - wCoM).sqrMagnitude);
			G    = Utils.ClampL(StG-(float)vessel.CentrifugalAcc.magnitude, 1e-5f);
			UpdateAltitudeInfo();
		}

		public void UpdateAltitudeInfo()
		{ 
			PrevRelAltitude = RelAltitude;
			AbsAltitude = (float)vessel.altitude;
			TerrainAltitude = (float)((vessel.mainBody.ocean && vessel.terrainAltitude < 0)? 0 : vessel.terrainAltitude);
			RelAltitude = (float)(vessel.altitude) - TerrainAltitude;
			Altitude = CFG.AltitudeAboveTerrain? RelAltitude : AbsAltitude;
			AltitudeAboveGround = 
				CFG.AltitudeAboveTerrain && CFG.DesiredAltitude >= 0 ||
				!CFG.AltitudeAboveTerrain && CFG.DesiredAltitude >= TerrainAltitude; 
		}

		public void UpdateCommons()
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
			for(int i = 0; i < NumActiveEngines; i++) 
			{
				var e = ActiveEngines[i];
				e.InitState();
				e.thrustDirection = refT.InverseTransformDirection(e.wThrustDir);
				e.wThrustLever = e.wThrustPos-wCoM;
				e.specificTorque = refT.InverseTransformDirection(Vector3.Cross(e.wThrustLever, e.wThrustDir));
				e.torqueRatio = Mathf.Pow(Mathf.Clamp01(1-Mathf.Abs(Vector3.Dot(e.wThrustLever.normalized, e.wThrustDir))), 
				                          GLB.ENG.TorqueRatioFactor);
				//do not include maneuver engines' thrust into the total to break the feedback loop with HSC
				if(e.Role != TCARole.MANEUVER) Thrust += e.wThrustDir*e.finalThrust;
				if(e.isVSC)
				{
					MaxThrust += e.wThrustDir*e.nominalCurrentThrust(1);
					MaxMassFlow += e.engine.maxThrust/e.engine.realIsp;
					if(e.useEngineResponseTime && e.finalThrust > 0)
					{
						var decelT = 1f/e.engineDecelerationSpeed;
						if(decelT > ThrustDecelerationTime) ThrustDecelerationTime = decelT;
					}
				}
				if(e.useEngineResponseTime && (e.Role == TCARole.MAIN || e.Role == TCARole.MANEUVER))
				{
					total_torque += e.currentTorque_m;
					TorqueResponseTime = e.currentTorque_m*Mathf.Max(e.engineAccelerationSpeed, e.engineDecelerationSpeed);
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
					t.wThrustLever = t.wThrustPos-wCoM;
					t.specificTorque = refT.InverseTransformDirection(Vector3.Cross(t.wThrustLever, t.wThrustDir));
					t.torqueRatio = Mathf.Pow(Mathf.Clamp01(1-Mathf.Abs(Vector3.Dot(t.wThrustLever.normalized, t.wThrustDir))), GLB.RCS.TorqueRatioFactor);
					t.currentTorque = t.Torque(1);
					t.currentTorque_m = t.currentTorque.magnitude;
				}
			}
			//update torque limits and MaxAA
			UpdateETorqueLimits();
			UpdateRTorqueLimits();
			UpdateWTorqueLimits();
			UpdateMaxAngularA();
			MaxPitchRollAA   = Vector3.ProjectOnPlane(MaxAngularA, refT.InverseTransformDirection(Thrust));
			MaxPitchRollAA_m = MaxPitchRollAA.magnitude;
			//update steering and translation
			Steering = new Vector3(vessel.ctrlState.pitch, vessel.ctrlState.roll, vessel.ctrlState.yaw);
			Translation = new Vector3(vessel.ctrlState.X, vessel.ctrlState.Z, vessel.ctrlState.Y);
			if(!Steering.IsZero()) //tune steering if MaxAA has changed drastically
				Steering = Steering*Utils.ClampH(MaxAAMod, 1)/Steering.CubeNorm().magnitude;
			if(!Translation.IsZero()) Translation = Translation/Translation.CubeNorm().magnitude;
			TranslationAvailable = ManeuverEngines.Count > 0 || RCS.Count > 0 && ActionGroups[KSPActionGroup.RCS];
			//reset things
			CourseCorrections.Clear();
			Destination = Vector3.zero;
			Countdown = -1;
			TTB = -1;
		}

		void UpdateETorqueLimits()
		{
			E_TorqueLimits = Vector6.zero;
			for(int i = 0, count = SteeringEngines.Count; i < count; i++)
				E_TorqueLimits.Add(SteeringEngines[i].currentTorque);
		}

		void UpdateWTorqueLimits()
		{
			W_TorqueLimits = Vector6.zero;
			for(int i = 0, count = RWheels.Count; i < count; i++)
			{
				var w = RWheels[i];
				if(!w.operational) continue;
				var torque = new Vector3(w.PitchTorque, w.RollTorque, w.YawTorque);
				W_TorqueLimits.Add(torque);
				W_TorqueLimits.Add(-torque);
			}
		}

		void UpdateRTorqueLimits()
		{
			R_TorqueLimits = Vector6.zero;
			for(int i = 0; i < NumActiveRCS; i++)
			{
				var r = ActiveRCS[i];
				for(int j = 0, tcount = r.rcs.thrusterTransforms.Count; j < tcount; j++)
				{
					var t = r.rcs.thrusterTransforms[j];
					if(t == null) continue;
					R_TorqueLimits.Add(refT.InverseTransformDirection(Vector3.Cross(t.position-wCoM, t.up)*r.rcs.thrusterPower));
				}
			}
		}

		public void UpdateTorque(params IList<EngineWrapper>[] engines)
		{
			Torque = Vector3.zero;
			for(int i = 0; i < engines.Length; i++)
			{
				for(int j = 0; j < engines[i].Count; j++)
				{
					var e = engines[i][j];
					Torque += e.Torque(e.throttle * e.limit);
				}
			}
			Torque = E_TorqueLimits.Clamp(Torque);
		}

		public void UpdateOnPlanetStats()
		{
			if(!OnPlanet) return;
			AccelSpeed = 0f; DecelSpeed = 0f; SlowThrust = false;
			//calculate altitude, vertical and horizontal speed and acceleration
			//unlike the vessel.verticalSpeed, this method is unaffected by ship's rotation (from MechJeb)
			AbsVerticalSpeed  = (float)Vector3d.Dot(vessel.srf_velocity, Up);
			VerticalAccel     = (AbsVerticalSpeed-VerticalSpeed)/TimeWarp.fixedDeltaTime;
			VerticalSpeed     = AbsVerticalSpeed;
			VerticalSpeedDisp = AbsVerticalSpeed;
			//use relative vertical speed instead of absolute if following terrain
			if(CFG.AltitudeAboveTerrain)
			{
				RelVerticalSpeed  = (RelAltitude - PrevRelAltitude)/TimeWarp.fixedDeltaTime;
				VerticalSpeedDisp = RelVerticalSpeed;
			}
			HorizontalVelocity = Vector3d.Exclude(Up, vessel.srf_velocity);
			HorizontalSpeed = (float)HorizontalVelocity.magnitude;
			//calculate total downward thrust and slow engines' corrections
			ManualThrust = Vector3.zero;
			ManualThrustLimits = Vector6.zero;
			var down_thrust = 0f;
			var slow_thrust = 0f;
			var fast_thrust = 0f;
			for(int i = 0; i < NumActiveEngines; i++)
			{
				var e = ActiveEngines[i];
				e.VSF = 1f;
				if(e.thrustInfo == null) continue;
				if(e.isVSC)
				{
					var dcomponent = -Vector3.Dot(e.wThrustDir, Up);
					if(dcomponent <= 0) e.VSF = 0;
					else 
					{
						var dthrust = e.nominalCurrentThrust(e.best_limit)*dcomponent;
						if(e.useEngineResponseTime && dthrust > 0) 
						{
							slow_thrust += dthrust;
							AccelSpeed += e.engineAccelerationSpeed*dthrust;
							DecelSpeed += e.engineDecelerationSpeed*dthrust;
						}
						else fast_thrust = dthrust;
						down_thrust += dthrust;
					}
				}
				if(e.Role == TCARole.MANUAL) 
				{
					ManualThrustLimits.Add(e.thrustDirection*e.nominalCurrentThrust(1));
					ManualThrust += e.wThrustDir*e.finalThrust;
				}
			}
			MaxTWR  = MaxThrustM/M/G;
			MaxDTWR = Utils.EWA(MaxDTWR, down_thrust/M/G, 0.1f);
			DTWR = Vector3.Dot(Thrust, Up) < 0? Vector3.Project(Thrust, Up).magnitude/M/G : 0f;
			if(refT != null)
			{
				Fwd = Vector3.Cross(refT.right, -MaxThrust).normalized;
				FwdL = refT.InverseTransformDirection(Fwd);
				NoseUp = Vector3.Dot(Fwd, refT.forward) >= 0.9;
				HFwd = Vector3.ProjectOnPlane(Fwd, Up).normalized;
			}
			MinVSFtwr = 1/Utils.ClampL(MaxTWR, 1);
			var mVSFtor = (MaxPitchRollAA_m > 0)? Utils.ClampH(GLB.VSC.MinVSFf/MaxPitchRollAA_m, GLB.VSC.MaxVSFtwr*MinVSFtwr) : 0;
			MinVSF = Mathf.Lerp(0, mVSFtor, Mathf.Pow(Steering.sqrMagnitude, 0.25f));
			var controllable_thrust = slow_thrust+fast_thrust;
			if(controllable_thrust.Equals(0)) return;
			//correct setpoint for current TWR and slow engines
			if(AccelSpeed > 0) AccelSpeed = controllable_thrust/AccelSpeed*GLB.VSC.ASf;
			if(DecelSpeed > 0) DecelSpeed = controllable_thrust/DecelSpeed*GLB.VSC.DSf;
			SlowThrust = AccelSpeed > 0 || DecelSpeed > 0;
		}

		public void UpdateBounds()
		{
			var b = new Bounds();
			bool inited = false;
			var parts = vessel.parts;
			for(int i = 0, partsCount = parts.Count; i < partsCount; i++)
			{
				Part p = parts[i];
				if(p == null) continue;
				var meshes = p.FindModelComponents<MeshFilter>();
				for(int mi = 0, meshesLength = meshes.Length; mi < meshesLength; mi++)
				{
					//skip meshes without renderer
					var m = meshes[mi];
					if(m.renderer == null || !m.renderer.enabled) continue;
					var bounds = Utils.BoundCorners(m.sharedMesh.bounds);
					for(int j = 0; j < 8; j++)
					{
						var c = refT.InverseTransformPoint(m.transform.TransformPoint(bounds[j]));
						if(inited) b.Encapsulate(c);
						else
						{
							b = new Bounds(c, Vector3.zero);
							inited = true;
						}
					}
				}
			}
			B = b;
			C = refT.TransformPoint(B.center);
			H = Mathf.Abs(Vector3.Dot(refT.TransformDirection(B.extents), Up))+Vector3.Dot(C-vessel.CurrentCoM, Up);
			R = B.extents.magnitude;
		}

		public void UpdateExhaustInfo()
		{
			var b = new Bounds();
			var inited = false;
			foreach(var e in Engines)
			{
				if(!e.Valid(this) || !e.engine.exhaustDamage) continue;
				for(int k = 0, tCount = e.engine.thrustTransforms.Count; k < tCount; k++)
				{
					var t = e.engine.thrustTransforms[k];
					if(t == null) continue;
					var term = refT.InverseTransformPoint(t.position + t.forward * e.engine.exhaustDamageMaxRange);
					if(inited) b.Encapsulate(term);
					else { b = new Bounds(term, Vector3.zero); inited = true; }
				}
			}
			b.Encapsulate(B);
			EnginesExhaust = b;
		}

		public void UnblockSAS(bool set_flag = true)
		{
			if(CFG.SASIsControlled) 
				ActionGroups.SetGroup(KSPActionGroup.SAS, CFG.SASWasEnabled);
			if(set_flag) CFG.SASIsControlled = false;
		}

		public Vector3 AngularAcceleration(Vector3 torque)
		{
			return new Vector3
				(MoI.x.Equals(0)? float.MaxValue : torque.x/MoI.x,
				 MoI.y.Equals(0)? float.MaxValue : torque.y/MoI.y,
				 MoI.z.Equals(0)? float.MaxValue : torque.z/MoI.z);
		}

		void UpdateMaxAngularA()
		{
			MaxTorque = E_TorqueLimits.Max+R_TorqueLimits.Max+W_TorqueLimits.Max;
			MaxAngularA = AngularAcceleration(MaxTorque);
			wMaxAngularA = refT.TransformDirection(MaxAngularA);
			MaxAngularA_m = MaxAngularA.magnitude;
			if(MaxAngularA_m > 0)
			{
				MaxAAMod = MaxAAFilter.Update(MaxAngularA_m)/MaxAngularA_m;
				MaxAAMod *= MaxAAMod*MaxAAMod;
			}
			else MaxAAMod = 1;
		}

		#region From MechJeb2
		// KSP's calculation of the vessel's moment of inertia is broken.
		// This function is somewhat expensive :(
		// Maybe it can be optimized more.
		static readonly Vector3[] unitVectors = { new Vector3(1, 0, 0), new Vector3(0, 1, 0), new Vector3(0, 0, 1) };
		public void UpdateMoI()
		{
			if(vessel == null || vessel.rigidbody == null) return;
			var CoM = vessel.CurrentCoM;
			InertiaTensor = new Matrix3x3f();
			var vesselTransform = vessel.GetTransform();
			var inverseVesselRotation = Quaternion.Inverse(vesselTransform.rotation);
			for(int pi = 0, vesselpartsCount = vessel.parts.Count; pi < vesselpartsCount; pi++)
			{
				Part p = vessel.parts[pi];
				var rb = p.Rigidbody;
				if(rb == null) continue;
				//Compute the contributions to the vessel inertia tensor due to the part inertia tensor
				Vector3 principalMoments = rb.inertiaTensor;
				Quaternion principalAxesRot = inverseVesselRotation * p.transform.rotation * rb.inertiaTensorRotation;
				Quaternion invPrincipalAxesRot = Quaternion.Inverse(principalAxesRot);
				for(int j = 0; j < 3; j++)
				{
					Vector3 partInertiaTensorTimesjHat = principalAxesRot * Vector3.Scale(principalMoments, invPrincipalAxesRot * unitVectors[j]);
					for(int i = 0; i < 3; i++)
						InertiaTensor.Add(i, j, Vector3.Dot(unitVectors[i], partInertiaTensorTimesjHat));
				}
				//Compute the contributions to the vessel inertia tensor due to the part mass and position
				float partMass = rb.mass;
				Vector3 partPosition = vesselTransform.InverseTransformDirection(rb.worldCenterOfMass - CoM);
				for(int i = 0; i < 3; i++)
				{
					InertiaTensor.Add(i, i, partMass * partPosition.sqrMagnitude);
					for(int j = 0; j < 3; j++)
						InertiaTensor.Add(i, j, -partMass * partPosition[i] * partPosition[j]);
				}
			}
			MoI = new Vector3(InertiaTensor[0, 0], InertiaTensor[1, 1], InertiaTensor[2, 2]);
			MoI = refT.InverseTransformDirection(vessel.transform.TransformDirection(MoI));
		}
		#endregion
		#endregion
	}

	/// <summary>
	/// Binary flags of TCA state.
	/// They should to be checked in this particular order, as they are set sequentially:
	/// If a previous flag is not set, the next ones are not either.
	/// </summary>
	[Flags] public enum TCAState 
	{ 
		//basic state
		Disabled 			   = 0,
		Enabled 			   = 1 << 0,
		HaveEC 				   = 1 << 1, 
		HaveActiveEngines 	   = 1 << 2,
		Unoptimized			   = 1 << 3,
		//vertical flight
		VerticalSpeedControl   = 1 << 4,
		AltitudeControl        = 1 << 5,
		LoosingAltitude 	   = 1 << 6,
		//cruise radar
		ObstacleAhead	 	   = 1 << 7,
		GroundCollision	 	   = 1 << 8,
		Ascending		 	   = 1 << 9,
		//autopilot
		Scanning               = 1 << 10,
		Searching              = 1 << 11,
		CheckingSite           = 1 << 12,
		Landing                = 1 << 13,
		VTOLAssist             = 1 << 14,
		StabilizeFlight        = 1 << 15,
		//composite
		Nominal				   = Enabled | HaveEC | HaveActiveEngines,
		NoActiveEngines        = Enabled | HaveEC,
		NoEC                   = Enabled,
	}
}

