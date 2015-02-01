/* Name: Throttle Controlled Avionics, Fork by Allis Tauri
 *
 * Authors: Quinten Feys & Willem van Vliet & Allis Tauri
 * License: BY: Attribution-ShareAlike 3.0 Unported (CC BY-SA 3.0): 
 * http://creativecommons.org/licenses/by-sa/3.0/
 * 
 */

using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace ThrottleControlledAvionics
{
	[KSPAddon(KSPAddon.Startup.Flight, false)]
	public class ThrottleControlledAvionics : MonoBehaviour
	{
		const string TCA_PART = "ThrottleControlledAvionics";

		#region State
		TCAGui GUI;
		Vessel vessel;
		public VesselConfig CFG { get; private set; }
		public readonly List<EngineWrapper> Engines = new List<EngineWrapper>();
		public float TorqueError { get; private set; }
		public float TorqueAngle { get; private set; }
		public TCAState State { get; private set; }
		//career and technical availability
		public bool Available { get; private set; }
		public bool Controllable { get { return Available && vessel.IsControllable; } }
		//physics
		Vector3d up;
		Vector3 wCoM;    //center of mass in world space
		Transform refT;  //transform of the controller-part
		Vector3 steering; //previous steering vector
		Vector6 torque;
		float   throttle;
		Matrix3x3f inertiaTensor;
		Vector3 MoI = Vector3.one;
		Vector3 angularA = Vector3.zero;
		PIv_Controller angularA_filter = new PIv_Controller();
		PIv_Controller hV_steering = new PIv_Controller();
		public float VerticalSpeedFactor { get; private set; } = 1f;
		public float VerticalSpeed { get; private set; }
		public bool IsStateSet(TCAState s) { return (State & s) == s; }
		#endregion

		#region Engine Logic
		public void Awake()
		{
			TCAConfiguration.Load();
			angularA_filter.setPI(TCAConfiguration.Globals.AngularA);
			GameEvents.onVesselChange.Add(onVesselChange);
			GameEvents.onVesselWasModified.Add(onVesselModify);
			GameEvents.onGameStateSave.Add(onSave);
		}

		internal void OnDestroy() 
		{ 
			TCAConfiguration.Save();
			if(GUI != null) GUI.OnDestroy();
			GameEvents.onVesselChange.Remove(onVesselChange);
			GameEvents.onVesselWasModified.Remove(onVesselModify);
			GameEvents.onGameStateSave.Remove(onSave);
		}

		void onVesselChange(Vessel vsl)
		{ 
			if(vsl == null || vsl.Parts == null) return;
			save(); reset();
			vessel = vsl;
			init();
		}

		void onVesselModify(Vessel vsl)
		{ if(vessel == vsl) init(); }

		void save() { TCAConfiguration.Save(); if(GUI != null) GUI.SaveConfig(); }
		void onSave(ConfigNode node) { save(); }

		void reset()
		{
			if(vessel != null) 
			{
				vessel.OnAutopilotUpdate -= block_throttle;
				vessel.OnAutopilotUpdate -= compensate_horizontal_speed;
			}
			vessel = null; 
			CFG = null;
			Engines.Clear();
			angularA_filter.Reset();
			Available = false;
		}

		void init()
		{
			Available = false;
			vessel.OnAutopilotUpdate += block_throttle;
			vessel.OnAutopilotUpdate += compensate_horizontal_speed;
			if(!vessel.isEVA && 
			   (!TCAConfiguration.Globals.IntegrateIntoCareer ||
			    Utils.PartIsPurchased(TCA_PART)))
			{
				updateEnginesList();
				if(Engines.Count > 0)
				{
					if(GUI == null) GUI = new TCAGui(this);
					Available = true;
					return;
				}
			} 
			if(GUI != null) { GUI.OnDestroy(); GUI = null; }
		}

		void block_throttle(FlightCtrlState s)
		{ if(Available && CFG.Enabled && CFG.BlockThrottle) s.mainThrottle = 1f; }

		void updateEnginesList()
		{
			if(vessel == null) return;
			CFG = TCAConfiguration.GetConfig(vessel);
			EngineWrapper.ThrustPI.setMaster(CFG.Engines);
			Engines.Clear();
			foreach(Part p in vessel.Parts)
				foreach(var module in p.Modules)
				{	
					EngineWrapper engine = null;
					if(module is ModuleEngines)
						engine = new EngineWrapper(module as ModuleEngines);
					else if(module is ModuleEnginesFX)
						engine = new EngineWrapper(module as ModuleEnginesFX);
					if(engine != null && !engine.throttleLocked) Engines.Add(engine);
				}
		}

		public void ActivateTCA(bool state)
		{
			if(state == CFG.Enabled) return;
			CFG.Enabled = state;
			if(!CFG.Enabled) //reset engine limiters
			{
				Engines.ForEach(e => e.forceThrustPercentage(100));
				State = TCAState.Disabled;
			}
		}
		public void ToggleTCA() { ActivateTCA(!CFG.Enabled); }

		public void BlockThrottle(bool state)
		{
			if(state == CFG.BlockThrottle) return;
			CFG.BlockThrottle = state;
			if(CFG.BlockThrottle && !CFG.VerticalSpeedControl)
				CFG.VerticalCutoff = 0;
		}

		public void OnGUI() 
		{ 
			if(!Available) return;
			Styles.Init();
			if(Controllable) GUI.DrawGUI(); 
			GUI.UpdateToolbarIcon();
		}

		public void Update()
		{ 
			if(!Controllable) return;
			GUI.OnUpdate();
			if(CFG.Enabled && CFG.BlockThrottle)
			{
				if(GameSettings.THROTTLE_UP.GetKey())
					CFG.VerticalCutoff = Mathf.Lerp(CFG.VerticalCutoff, 
					                                TCAConfiguration.Globals.MaxCutoff, 
					                                CFG.VSControlSensitivity);
				else if(GameSettings.THROTTLE_DOWN.GetKey())
					CFG.VerticalCutoff = Mathf.Lerp(CFG.VerticalCutoff, 
					                                -TCAConfiguration.Globals.MaxCutoff, 
					                                CFG.VSControlSensitivity);
				else if(GameSettings.THROTTLE_FULL.GetKeyDown())
					CFG.VerticalCutoff = TCAConfiguration.Globals.MaxCutoff;
				else if(GameSettings.THROTTLE_CUTOFF.GetKeyDown())
					CFG.VerticalCutoff = -TCAConfiguration.Globals.MaxCutoff;
			}
		}

		public void FixedUpdate()
		{
			if(!Available || !CFG.Enabled) return;
			State = TCAState.Enabled;
			//check for throttle and Electrich Charge
			if(vessel.ctrlState.mainThrottle <= 0) return;
			State |= TCAState.Throttled;
			if(!vessel.ElectricChargeAvailible()) return;
			State |= TCAState.HaveEC;
			//update engines if needed
			if(Engines.Any(e => !e.Valid)) updateEnginesList();
			var active_engines = Engines.Where(e => e.isEnabled).ToList();
			if(active_engines.Count == 0) return;
			State |= TCAState.HaveActiveEngines;
			//calculate steering
			wCoM     = vessel.findWorldCenterOfMass();
			refT     = vessel.GetReferenceTransformPart().transform; //should be in a callback?
			up       = (wCoM - vessel.mainBody.position).normalized;
			steering = new Vector3(vessel.ctrlState.pitch, vessel.ctrlState.roll, vessel.ctrlState.yaw);
			if(!steering.IsZero())
			{
				steering = steering/steering.CubeNorm().magnitude;
				if(!CFG.AutoTune) steering *= CFG.SteeringGain;
				steering.Scale(CFG.SteeringModifier);
			}
			//tune engines' limits
			VerticalSpeedFactor = getVerticalSpeedFactor();
			optimizeLimitsIteratively(active_engines);
			//set thrust limiters of engines taking vertical speed limit into account
			active_engines.ForEach(e => e.thrustPercentage = Mathf.Clamp(100 * VerticalSpeedFactor * e.limit, 0f, 100f));
		}

		bool optimizeLimits(List<EngineWrapper> engines, Vector3 target, float target_m, float eps)
		{
			var compensation = Vector3.zero;
			for(int i = 0; i < engines.Count; i++)
			{
				var e = engines[i];
				var d = Vector3.Dot(e.currentTorque, target);
				e.limit_tmp = d < 0? -d/target_m/e.currentTorque.magnitude : 0f;
				if(e.limit_tmp > 0)
					compensation += e.specificTorque * e.nominalCurrentThrust(throttle * e.limit_tmp * e.limit);
			}
			var compensation_m = compensation.magnitude;
			if(compensation_m < eps) return false;
			var limits_norm = Mathf.Clamp01(target_m/compensation_m);
			engines.ForEach(e => e.limit *= (1-Mathf.Clamp01(e.limit_tmp*limits_norm)));
			return true;
		}

		void optimizeLimitsIteratively(List<EngineWrapper> engines)
		{
			//calculate specific torques and min-max imbalance
			var min_imbalance = Vector3.zero;
			var max_imbalance = Vector3.zero;
			for(int i = 0; i < engines.Count; i++)
			{
				var e = engines[i];
				var info = e.thrustInfo;
				e.limit = 1f;
				e.thrustDirection = refT.InverseTransformDirection(info.dir);
				e.specificTorque  = refT.InverseTransformDirection(Vector3.Cross(info.pos-wCoM, info.dir));
				min_imbalance += e.specificTorque * e.nominalCurrentThrust(0);
				max_imbalance += e.specificTorque * e.nominalCurrentThrust(1);
			}
			//correct VerticalSpeedFactor if needed
			if(!min_imbalance.IsZero() && !max_imbalance.IsZero())
				VerticalSpeedFactor = Mathf.Clamp(VerticalSpeedFactor, Mathf.Clamp01(min_imbalance.magnitude/max_imbalance.magnitude), 1f);
			//calculate initial imbalance and needed torque
			torque = new Vector6();
			throttle = vessel.ctrlState.mainThrottle * VerticalSpeedFactor;
			var torque_imbalance = Vector3.zero;
			var needed_torque = Vector3.zero;
			for(int i = 0; i < engines.Count; i++)
			{
				var e = engines[i];
				e.currentTorque   = e.specificTorque * e.nominalCurrentThrust(throttle);
				torque_imbalance += e.currentTorque;
				torque.Add(e.currentTorque);
				if(Vector3.Dot(e.currentTorque, steering) > 0)
					needed_torque += e.currentTorque;
			}
			needed_torque = torque.Clamp(Vector3.Project(needed_torque, steering) * steering.magnitude);
			//tune steering gains
			if(CFG.AutoTune) tuneSteering();
			//optimize engines' limits
			TorqueAngle = 0f;
			TorqueError = 0f;
			float last_error = -1f;
			float last_angle = -1f;
			bool  optimized  = true;
			for(int i = 0; i < TCAConfiguration.Globals.MaxIterations; i++)
			{
				TorqueAngle = needed_torque.IsZero()? 0f : Vector3.Angle(torque_imbalance, needed_torque);
				if(!optimized || 
				   TorqueAngle > 0 && last_angle > 0 && 
				   TorqueAngle - last_angle > TCAConfiguration.Globals.OptimizationPrecision) 
				{
					if(TorqueAngle < TCAConfiguration.Globals.OptimizationAngleCutoff ||
					   needed_torque.IsZero()) break;
					needed_torque = Vector3.zero;
					engines.ForEach(e => e.limit = 1);
					i = 0;
				}
				var target  = needed_torque-torque_imbalance;
				TorqueError = target.magnitude;
				if(TorqueError < TCAConfiguration.Globals.OptimizationPrecision || last_error > 0 && 
				   Mathf.Abs(last_error-TorqueError) < TCAConfiguration.Globals.OptimizationPrecision) 
					break;
				optimized = optimizeLimits(engines, target, target.magnitude, TCAConfiguration.Globals.OptimizationPrecision);
				torque_imbalance = engines.Aggregate(Vector3.zero, 
				                                     (v, e) => v + 
				                                     e.specificTorque * e.nominalCurrentThrust(throttle * e.limit));
				last_error = TorqueError;
				last_angle = TorqueAngle;
			}
			//debug
//			Utils.Log("Engines:\n"+engines.Aggregate("", (s, e) => s + "vec"+e.specificTorque+",\n"));
//			Utils.Log(
//				"Steering: {0}\n" +
//				"Needed Torque: {1}\n" +
//				"Torque Error: {2}kNm, {3} deg\n" +
//				"Torque Clamp:\n   +{4}\n   -{5}\n" +
//				"Limits: [{6}]", 
//				steering,
//				needed_torque,
//				TorqueError, TorqueAngle,
//				torque.positive, 
//				torque.negative,
//				engines.Aggregate("", (s, e) => s+e.limit+" ").Trim()
//			);
		}

		float getVerticalSpeedFactor()
		{
			if(vessel.situation == Vessel.Situations.DOCKED   ||
			   vessel.situation == Vessel.Situations.ORBITING ||
			   vessel.situation == Vessel.Situations.ESCAPING ||
			   CFG.VerticalCutoff >= TCAConfiguration.Globals.MaxCutoff) return 1f;
			State |= TCAState.VerticalSpeedControl;
			//unlike the vessel.verticalSpeed, this method is unaffected by ship's rotation 
			var upV = (float)Vector3d.Dot(vessel.srf_velocity, up); //from MechJeb
			var upA = (upV-VerticalSpeed)/TimeWarp.fixedDeltaTime;
			var err = CFG.VerticalCutoff-upV;
			VerticalSpeed = upV;
			var K = upV < CFG.VerticalCutoff?
				Mathf.Clamp01(err/TCAConfiguration.Globals.K0/Mathf.Pow(Utils.ClampL(upA/TCAConfiguration.Globals.K1+1, TCAConfiguration.Globals.L1), 2f)) :
				Mathf.Clamp01(err*upA/Mathf.Pow(Utils.ClampL(-err*TCAConfiguration.Globals.K2, TCAConfiguration.Globals.L2), 2f));
			if(upA < 0 && upV < CFG.VerticalCutoff && K >= 1)
				State |= TCAState.LoosingAltitude;
			return K;
		}

		void compensate_horizontal_speed(FlightCtrlState s)
		{
			if(!Available || !CFG.Enabled || refT == null) return;
			var hV = Vector3d.Exclude(up, vessel.srf_velocity);
			var hVm = hV.magnitude;
			if(hVm < 0.001) return;
			hV_steering.setPI(0.9f, 0.2f);

			var thrust = refT.TransformDirection(Engines.Aggregate(Vector3.zero, (v, e) => v + e.thrustDirection*e.finalThrust));
			if(thrust.IsZero()) return;

			var needed_thrust = (hV - up*hVm*Utils.ClampL(10/(float)hVm, 1)).normalized*thrust.magnitude;
			var err = Vector3.Angle(thrust, needed_thrust)/180;

			var needed_rot = Vector3.Cross(needed_thrust, thrust).normalized*err;
			var rot = vessel.angularVelocity;
			hV_steering.Update(refT.InverseTransformDirection(needed_rot - rot/180)*10);
			var act = hV_steering.Action;
			s.pitch = Mathf.Clamp(act.x, -1, 1);
			s.roll = Mathf.Clamp(act.y, -1, 1);
			s.yaw = Mathf.Clamp(act.z, -1, 1);

			Utils.Log(//debug
					  "Error: {0}\n" +
					  "Action {1}\n" +
					  "Thrust: {2}\n" +
					  "Needed Thrust: {3}\n" +
					  "hV: {4}\n" +
					  "steering: {5}\n" +
			          "rot: {6}\n" +
			          "needed rot: {7}", 
			          err, hV_steering.Action,
			          thrust, needed_thrust,
			          hV,
			          new Vector3(s.pitch, s.roll, s.yaw),
			          rot, needed_rot
			);
		}

		void tuneSteering()
		{
			//calculate maximum angular acceleration for each axis
			updateMoI();
			var max_torque = torque.Max;
			var new_angularA = new Vector3
				(
					MoI.x != 0? max_torque.x/MoI.x : float.MaxValue,
					MoI.y != 0? max_torque.y/MoI.y : float.MaxValue,
					MoI.z != 0? max_torque.z/MoI.z : float.MaxValue
				);
			new_angularA = refT.transform.InverseTransformDirection(vessel.transform.TransformDirection(new_angularA));//test
			angularA_filter.Update(new_angularA - angularA);
			angularA += angularA_filter.Action;
			//tune steering modifiers
			CFG.SteeringModifier.x = Mathf.Clamp(TCAConfiguration.Globals.SteeringCurve.Evaluate(angularA.x)/100f, 0, 1);
			CFG.SteeringModifier.y = Mathf.Clamp(TCAConfiguration.Globals.SteeringCurve.Evaluate(angularA.y)/100f, 0, 1);
			CFG.SteeringModifier.z = Mathf.Clamp(TCAConfiguration.Globals.SteeringCurve.Evaluate(angularA.z)/100f, 0, 1);
			//tune PI coefficients
			CFG.Engines.P = TCAConfiguration.Globals.EnginesCurve.Evaluate(angularA.magnitude);
			CFG.Engines.I = CFG.Engines.P/2f;
			//debug
//			Utils.Log("max_torque: {0}\n" + 
//                      "MoI {1}\n" +
//                      "angularA {2}", 
//                      max_torque, MoI, angularA);
		}
		#endregion

		#region From MechJeb2
		// KSP's calculation of the vessel's moment of inertia is broken.
		// This function is somewhat expensive :(
		// Maybe it can be optimized more.
		static readonly Vector3[] unitVectors = { new Vector3(1, 0, 0), new Vector3(0, 1, 0), new Vector3(0, 0, 1) };
		void updateMoI()
		{
			if(vessel == null || vessel.rigidbody == null) return;
			inertiaTensor = new Matrix3x3f();
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
						inertiaTensor[i, j] += Vector3.Dot(unitVectors[i], partInertiaTensorTimesjHat);
				}
				//Compute the contributions to the vessel inertia tensor due to the part mass and position
				float partMass = p.TotalMass();
				Vector3 partPosition = vesselTransform.InverseTransformDirection(rb.worldCenterOfMass - wCoM);
				for(int i = 0; i < 3; i++)
				{
					inertiaTensor[i, i] += partMass * partPosition.sqrMagnitude;
					for (int j = 0; j < 3; j++)
						inertiaTensor[i, j] += -partMass * partPosition[i] * partPosition[j];
				}
			}
			MoI = new Vector3(inertiaTensor[0, 0], inertiaTensor[1, 1], inertiaTensor[2, 2]);
		}
		#endregion
	}

	/// <summary>
	/// Binary flags of TCA state.
	/// They should to be checked in this particular order, as they are set sequentially:
	/// If a previous flag is not set, the next ones are not either.
	/// </summary>
	[Flags] public enum TCAState 
	{ 
		Disabled 			   = 0,
		Enabled 			   = 1 << 0,
		Throttled 			   = 1 << 1,
		HaveEC 				   = 1 << 2, 
		HaveActiveEngines 	   = 1 << 3,
		VerticalSpeedControl   = 1 << 4,
		LoosingAltitude 	   = 1 << 5,
		Nominal				   = Enabled | Throttled | HaveEC | HaveActiveEngines,
		NoActiveEngines        = Enabled | Throttled | HaveEC,
		NoEC                   = Enabled | Throttled,
	}
}
