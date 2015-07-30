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

using System.Linq;
using System.Collections.Generic;
using UnityEngine;

namespace ThrottleControlledAvionics
{
	public class VesselWrapper
	{
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

		public Vessel vessel { get; private set; }
		public Transform refT { get; private set; } //transform of the controller-part
		public VesselConfig CFG { get; private set; }

		public List<EngineWrapper> Engines         = new List<EngineWrapper>();
		public List<EngineWrapper> ActiveEngines   = new List<EngineWrapper>();
		public List<EngineWrapper> BalancedEngines = new List<EngineWrapper>();
		public List<EngineWrapper> ManeuverEngines = new List<EngineWrapper>();
		public List<EngineWrapper> SteeringEngines = new List<EngineWrapper>();
		public List<EngineWrapper> ManualEngines   = new List<EngineWrapper>();
		public List<ModuleReactionWheel> RWheels   = new List<ModuleReactionWheel>();

		public int  NumActive { get; private set; }
		public bool NormalizeLimits = true; //if engines' limits should be normalized

		//physics
		public Vector6 E_TorqueLimits { get; private set; } //torque limits of engines
		public Vector6 R_TorqueLimits { get; private set; } //torque limits of reaction wheels

		public float M { get; private set; }
		public float MaxTWR { get; private set; }
		public float AccelSpeed { get; private set; }
		public float DecelSpeed { get; private set; }
		public float VSF; //vertical speed factor

		public Vector3d   up { get; private set; }  //up unit vector in world space
		public Vector3    CoM { get { return vessel.CoM + vessel.rb_velocity*TimeWarp.fixedDeltaTime; } } //current center of mass of unpacked vessel
		public Vector3    wCoM { get; private set; } //center of mass in world space
		public Vector3    MoI { get; private set; } = Vector3.one; //main diagonal of inertia tensor
		public Matrix3x3f InertiaTensor { get; private set; }
		public Vector3    MaxAngularA { get; private set; } //current maximum angular acceleration

		public Vector3 Torque { get; private set; } //current torque applied to the vessel by the engines
		public float   VerticalSpeed { get; private set; }
		public float   VerticalAccel { get; private set; }
		public float   Altitude { get; private set; }
		public float   TerrainAltitude { get; private set; }

		public TCAState State;
		public bool IsStateSet(TCAState state) { return (State & state) == state; }
		public bool ElectricChargeAvailible
		{
			get
			{
				var ec = vessel.GetActiveResource(ElectricCharge);
				return ec != null && ec.amount > 0;
			}
		}

		public bool OnPlanet { get; private set; }
		public bool isEVA { get { return vessel.isEVA; } }
		public bool LandedOrSplashed { get { return vessel.LandedOrSplashed; } }
		public ActionGroupList ActionGroups { get { return vessel.ActionGroups; } }
		public FlightCtrlState ctrlState { get { return vessel.ctrlState; } }
		public FlightInputCallback OnAutopilotUpdate 
		{ get { return vessel.OnAutopilotUpdate; } set { vessel.OnAutopilotUpdate = value; } }

		public bool TCA_Available;
		public bool IsControllable { get { return vessel.IsControllable; } }

		public VesselWrapper(Vessel vsl) { vessel = vsl; }

		public void Init() {}

		#region Engines
		public void UpdateEngines()
		{
			if(vessel == null) return;
			CFG = TCAConfiguration.GetConfig(vessel);
			EngineWrapper.ThrustPI.setMaster(CFG.Engines);
			Engines.Clear();
			foreach(Part p in vessel.Parts)
				foreach(var module in p.Modules)
				{	
					var engine = module as ModuleEngines;
					if(engine != null) Engines.Add(new EngineWrapper(engine));
					var rwheel = module as ModuleReactionWheel;
					if(rwheel != null) RWheels.Add(rwheel);
				}
		}

		public bool CheckEngines()
		{
			if(Engines.Any(e => !e.Valid)) UpdateEngines();
			ActiveEngines = Engines.Where(e => e.isOperational).ToList();
			NumActive = ActiveEngines.Count;
			return NumActive > 0;
		}

		public void SortEngines()
		{
			SteeringEngines.Clear();
			ManeuverEngines.Clear();
			BalancedEngines.Clear();
			ManualEngines.Clear();
			for(int i = 0; i < NumActive; i++)
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

		public void InitEngines()
		{
			NormalizeLimits = true;
			//calculate specific torques and min imbalance
			var min_imbalance = Vector3.zero;
			for(int i = 0; i < NumActive; i++)
			{
				var e = ActiveEngines[i];
				e.InitState();
				e.thrustDirection = refT.InverseTransformDirection(e.wThrustDir);
				var lever = e.wThrustPos-wCoM;
				e.specificTorque = refT.InverseTransformDirection(Vector3.Cross(lever, e.wThrustDir));
				e.torqueRatio = Mathf.Pow(Mathf.Clamp01(1-Mathf.Abs(Vector3.Dot(lever.normalized, e.wThrustDir))), TCAConfiguration.Globals.TRQ.TorqueRatioFactor);
				min_imbalance += e.Torque(0);
			}
			//calculate engine's torue, torque limits and set VSF
			if(IsStateSet(TCAState.VerticalSpeedControl))
			{
				//correct VerticalSpeedFactor if needed
				if(!min_imbalance.IsZero())
				{
					var anti_min_imbalance = Vector3.zero;
					for(int i = 0; i < NumActive; i++)
					{
						var e = ActiveEngines[i];
						if(Vector3.Dot(e.specificTorque, min_imbalance) < 0)
							anti_min_imbalance += e.specificTorque * e.nominalCurrentThrust(1);
					}
					anti_min_imbalance = Vector3.Project(anti_min_imbalance, min_imbalance);
					VSF = Mathf.Clamp(VSF, Mathf.Clamp01(min_imbalance.magnitude/anti_min_imbalance.magnitude
					                                     *TCAConfiguration.Globals.VSC.BalanceCorrection), 1f);
				}
				for(int i = 0; i < NumActive; i++)
				{
					var e = ActiveEngines[i];
					if(e.isVSC)
					{
						if(e.VSF > 0) e.VSF = VSF;
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
					var e = ActiveEngines[i];
					e.VSF = 1f;
					e.throttle = vessel.ctrlState.mainThrottle;
					e.currentTorque = e.Torque(e.throttle);
					e.currentTorque_m = e.currentTorque.magnitude;
				}
			}
		}

		public void SetThrustLimiters()
		{
			for(int i = 0; i < NumActive; i++)
			{
				var e = ActiveEngines[i];
				if(e.Role == TCARole.MANUAL) continue;
				e.thrustPercentage = Mathf.Clamp(100 * e.VSF * e.limit, 0f, 100f);
			}
		}
		#endregion

		#region Updates
		public void UpdateAltitude()
		{ 
			if(CFG.AltitudeAboveTerrain)
			{
				TerrainAltitude = (float)((vessel.mainBody.ocean && vessel.terrainAltitude < 0)? 0 : vessel.terrainAltitude);
				Altitude = (float)(vessel.altitude) - TerrainAltitude;
			}
			else Altitude = (float)vessel.altitude; 
		}

		public void UpdateState()
		{
			OnPlanet = (vessel.situation != Vessel.Situations.DOCKED   &&
			            vessel.situation != Vessel.Situations.ORBITING &&
			            vessel.situation != Vessel.Situations.ESCAPING);
		}

		public void UpdateCommons()
		{
			wCoM = vessel.CoM + vessel.rb_velocity*TimeWarp.fixedDeltaTime;
			refT = vessel.ReferenceTransform;
			up   = (wCoM - vessel.mainBody.position).normalized;
		}

		public void UpdateETorqueLimits()
		{
			E_TorqueLimits = new Vector6();
			for(int i = 0; i < SteeringEngines.Count; i++)
				E_TorqueLimits.Add(SteeringEngines[i].currentTorque);
		}

		public void UpdateRTorqueLimits()
		{
			R_TorqueLimits = new Vector6();
			for(int i = 0; i < RWheels.Count; i++)
			{
				var w = RWheels[i];
				if(!w.operational) continue;
				R_TorqueLimits.Add(refT.InverseTransformDirection(new Vector3(w.PitchTorque, w.RollTorque, w.YawTorque)));
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
			NormalizeLimits = Torque.IsSmallerThan(TCAConfiguration.Globals.TRQ.OptimizationTorqueCutoff);
		}

		public void UpdateOnPlanetStats()
		{
			AccelSpeed = 0f; DecelSpeed = 0f; MaxTWR = 0f;
			if(!OnPlanet) return;
			//calculate altitude, vertical speed and acceleration
			var upV = 0f;
			if(CFG.ControlAltitude)
			{
				//update vessel altitude
				var old_alt = Altitude;
				UpdateAltitude();
				//use relative vertical speed instead of absolute
				if(CFG.AltitudeAboveTerrain)
					upV = 0.3f*VerticalSpeed+0.7f*(Altitude-old_alt)/TimeWarp.fixedDeltaTime;
			}
			//unlike the vessel.verticalSpeed, this method is unaffected by ship's rotation 
			else upV = (float)Vector3d.Dot(vessel.srf_velocity, up); //from MechJeb
			VerticalAccel = 0.3f*VerticalAccel + 0.7f*(upV-VerticalSpeed)/TimeWarp.fixedDeltaTime; //high-freq filter
			VerticalSpeed = upV;
			//calculate total downward thrust and slow engines' corrections
			var down_thrust = 0f;
			var slow_thrust = 0f;
			var fast_thrust = 0f;
			for(int i = 0; i < NumActive; i++)
			{
				var e = ActiveEngines[i];
				e.VSF = 1f;
				if(e.thrustInfo == null) continue;
				if(e.isVSC)
				{
					var dcomponent = -Vector3.Dot(e.wThrustDir, up);
					if(dcomponent <= 0) e.VSF = 0f;
					else 
					{
						var dthrust = e.nominalCurrentThrust(e.best_limit)*e.thrustMod*dcomponent;
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
			}
			M = vessel.GetTotalMass();
			MaxTWR = down_thrust/9.81f/M;
			var controllable_thrust = slow_thrust+fast_thrust;
			if(controllable_thrust.Equals(0)) return;
			//correct setpoint for current TWR and slow engines
			if(AccelSpeed > 0) AccelSpeed = controllable_thrust/AccelSpeed*TCAConfiguration.Globals.VSC.ASf;
			if(DecelSpeed > 0) DecelSpeed = controllable_thrust/DecelSpeed*TCAConfiguration.Globals.VSC.DSf;
		}

		public void UpdateRotationalStats()
		{
			update_MoI();
			update_MaxAngularA();
		}

		void update_MaxAngularA()
		{
			var max_torque = E_TorqueLimits.Max;
			var new_angularA = new Vector3
				(
					!MoI.x.Equals(0)? max_torque.x/MoI.x : float.MaxValue,
					!MoI.y.Equals(0)? max_torque.y/MoI.y : float.MaxValue,
					!MoI.z.Equals(0)? max_torque.z/MoI.z : float.MaxValue
				);
			MaxAngularA = 0.3f*MaxAngularA + 0.7f*new_angularA;
		}

		#region From MechJeb2
		// KSP's calculation of the vessel's moment of inertia is broken.
		// This function is somewhat expensive :(
		// Maybe it can be optimized more.
		static readonly Vector3[] unitVectors = { new Vector3(1, 0, 0), new Vector3(0, 1, 0), new Vector3(0, 0, 1) };
		void update_MoI()
		{
			if(vessel == null || vessel.rigidbody == null) return;
			InertiaTensor = new Matrix3x3f();
			Transform vesselTransform = vessel.GetTransform();
			Quaternion inverseVesselRotation = Quaternion.Inverse(vesselTransform.rotation);
			foreach(Part p in vessel.parts)
			{
				var rb = p.Rigidbody;
				if (rb == null) continue;
				//Compute the contributions to the vessel inertia tensor due to the part inertia tensor
				Vector3 principalMoments = rb.inertiaTensor;
				Quaternion principalAxesRot = inverseVesselRotation * p.transform.rotation * rb.inertiaTensorRotation;
				Quaternion invPrincipalAxesRot = Quaternion.Inverse(principalAxesRot);
				for (int j = 0; j < 3; j++)
				{
					Vector3 partInertiaTensorTimesjHat = principalAxesRot * Vector3.Scale(principalMoments, invPrincipalAxesRot * unitVectors[j]);
					for (int i = 0; i < 3; i++)
						InertiaTensor[i, j] += Vector3.Dot(unitVectors[i], partInertiaTensorTimesjHat);
				}
				//Compute the contributions to the vessel inertia tensor due to the part mass and position
				float partMass = p.TotalMass();
				Vector3 partPosition = vesselTransform.InverseTransformDirection(rb.worldCenterOfMass - wCoM);
				for(int i = 0; i < 3; i++)
				{
					InertiaTensor[i, i] += partMass * partPosition.sqrMagnitude;
					for (int j = 0; j < 3; j++)
						InertiaTensor[i, j] += -partMass * partPosition[i] * partPosition[j];
				}
			}
			MoI = new Vector3(InertiaTensor[0, 0], InertiaTensor[1, 1], InertiaTensor[2, 2]);
			MoI = refT.InverseTransformDirection(vessel.transform.TransformDirection(MoI));
		}
		#endregion
		#endregion
	}
}

