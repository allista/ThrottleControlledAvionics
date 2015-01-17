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
		TCAGui GUI;

		#region State
		Vessel vessel;
		public VesselConfig CFG { get; private set; }
		public readonly List<EngineWrapper> Engines = new List<EngineWrapper>();
		public bool HaveEC { get; private set; } = true;
		public bool Throttled { get { return vessel != null && vessel.ctrlState.mainThrottle > 0; } }
		public float TorqueError { get; private set; }
		//physics
		Vector3 wCoM;    //center of mass in world space
		Transform refT;  //transform of the controller-part
		Vector3 steering; //previous steering vector
		Vector6 torque;
		Matrix3x3f inertiaTensor;
		Vector3 MoI = Vector3.one;
		public float VerticalSpeedFactor { get; private set; } = 1f;
		public float VerticalSpeed { get; private set; }
		#endregion

		#region Engine Logic
		public void Awake()
		{
			TCAConfiguration.Load();
			GUI = new TCAGui(this);
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
			TCAConfiguration.Save();
			vessel = vsl;
			updateEnginesList();
		}

		void onVesselModify(Vessel vsl)
		{ if(vessel == vsl) updateEnginesList(); }

		void onSave(ConfigNode node) { TCAConfiguration.Save(); }

		public void ActivateTCA(bool state)
		{
			if(CFG == null || state == CFG.Enabled) return;
			CFG.Enabled = state;
			if(!CFG.Enabled) //reset engine limiters
				Engines.ForEach(e => e.forceThrustPercentage(100));
		}

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

		public void OnGUI() 
		{ 
			Styles.Init();
			GUI.DrawGUI(); 
			GUI.UpdateToolbarIcon();
		}

		public void Update()
		{ 
			if(Input.GetKeyDown(TCAConfiguration.Globals.TCA_Key)) 
				ActivateTCA(!CFG.Enabled); 
		}

		public void FixedUpdate()
		{
			if(!CFG.Enabled || vessel == null) return;
			//check for throttle and Electrich Charge
			if(vessel.ctrlState.mainThrottle <= 0) return;
			HaveEC = vessel.ElectricChargeAvailible();
			if(!HaveEC) return;
			//update engines if needed
			if(Engines.Any(e => !e.Valid)) updateEnginesList();
			var active_engines = Engines.Where(e => e.isEnabled).ToList();
			if(active_engines.Count == 0) return;
			//calculate steering
			wCoM         = vessel.findWorldCenterOfMass();
			refT         = vessel.GetReferenceTransformPart().transform; //should be in a callback?
			steering = new Vector3(vessel.ctrlState.pitch, vessel.ctrlState.roll, vessel.ctrlState.yaw);
			if(!steering.IsZero())
			{
				steering = steering/steering.CubeNorm().magnitude;
				if(!CFG.AutoTune) steering *= CFG.SteeringGain;
				steering.Scale(CFG.SteeringModifier);
			}
			//tune engines' limits
			optimizeLimitsIteratively(active_engines, 
			                          TCAConfiguration.Globals.OptimizationPrecision, 
			                          TCAConfiguration.Globals.MaxIterations);
			//set thrust limiters of engines taking vertical speed limit into account
			VerticalSpeedFactor = getVerticalSpeedFactor();
			active_engines.ForEach(e => e.thrustPercentage = Mathf.Clamp(100 * VerticalSpeedFactor * e.limit, 0f, 100f));
		}

		static bool optimizeLimits(List<EngineWrapper> engines, Vector3 target, float target_m, float eps)
		{
			var compensation = Vector3.zero;
			for(int i = 0; i < engines.Count; i++)
			{
				var e = engines[i];
				var d = Vector3.Dot(e.currentTorque, target);
				e.limit_tmp = d < 0? -d/target_m/e.currentTorque.magnitude : 0f;
				compensation += e.limit_tmp > 0? e.limit_tmp * e.currentTorque * e.limit : Vector3.zero;
			}
			var compensation_m = compensation.magnitude;
			if(compensation_m < eps) return false;
			var limits_norm = target_m/compensation_m;
			engines.ForEach(e => e.limit *= (1-Mathf.Clamp01(e.limit_tmp*limits_norm)));
			return true;
		}

		void optimizeLimitsIteratively(List<EngineWrapper> engines, float eps, int max_iter)
		{
			//calculate initial imbalance and needed torque
			torque = new Vector6();
			var torque_imbalance = Vector3.zero;
			var needed_torque = Vector3.zero;
			for(int i = 0; i < engines.Count; i++)
			{
				var e = engines[i];
				var info = e.thrustInfo;
				e.limit = 1f;
				e.thrustDirection = refT.InverseTransformDirection(info.dir);
				e.specificTorque  = refT.InverseTransformDirection(Vector3.Cross(info.pos-wCoM, info.dir));
				e.currentThrust   = e.maxThrust * vessel.ctrlState.mainThrottle;
				e.currentTorque   = e.specificTorque * e.currentThrust;
				torque_imbalance += e.currentTorque;
				torque.Add(e.currentTorque);
				if(Vector3.Dot(e.currentTorque, steering) > 0)
					needed_torque += e.currentTorque;
			}
			needed_torque = torque.Clamp(Vector3.Project(needed_torque, steering) * steering.magnitude);
			//tune steering gains
			if(CFG.AutoTune) tuneSteering();
			//optimize engines' limits
			TorqueError = 0f;
			float last_error = -1f;
			for(int i = 0; i < max_iter; i++)
			{
				TorqueError = (torque_imbalance-needed_torque).magnitude;
				if(TorqueError < eps || last_error > 0f && Mathf.Abs(last_error-TorqueError) < eps) break;
				var target = needed_torque - torque_imbalance;
				if(!optimizeLimits(engines, target, target.magnitude, eps)) break;
				torque_imbalance = engines.Aggregate(Vector3.zero, (v, e) => v + e.currentTorque * e.limit);
				last_error = TorqueError;
			}
		}

		public bool LimitingSpeed 
		{ 
			get
			{
				return
					vessel.situation != Vessel.Situations.DOCKED &&
					vessel.situation != Vessel.Situations.ORBITING &&
					vessel.situation != Vessel.Situations.ESCAPING &&
					CFG.VerticalCutoff < TCAConfiguration.Globals.MaxCutoff;
			}
		}

		float getVerticalSpeedFactor()
		{
			if(!LimitingSpeed) return 1f;
			//unlike the vessel.verticalSpeed, this method is unaffected by ship's rotation 
			var up  = (wCoM - vessel.mainBody.position).normalized;
			var upV = (float)Vector3d.Dot(vessel.srf_velocity, up); //from MechJeb
			var upA = (upV-VerticalSpeed)/TimeWarp.fixedDeltaTime;
			var err = CFG.VerticalCutoff-upV;
			VerticalSpeed = upV;
			return upV < CFG.VerticalCutoff?
				Mathf.Clamp01(err/TCAConfiguration.Globals.K0/Mathf.Pow(Utils.ClampL(upA/TCAConfiguration.Globals.K1+1, TCAConfiguration.Globals.L1), 2f)) :
				Mathf.Clamp01(err*upA/Mathf.Pow(Utils.ClampL(-err*TCAConfiguration.Globals.K2, TCAConfiguration.Globals.L2), 2f));
		}

		void tuneSteering()
		{
			//calculate maximum angular acceleration for each axis
			updateMoI();
			var max_torque = torque.Max * VerticalSpeedFactor;
			var angularA = new Vector3
				(
					MoI.x != 0? max_torque.x/MoI.x : float.MaxValue,
					MoI.y != 0? max_torque.y/MoI.y : float.MaxValue,
					MoI.z != 0? max_torque.z/MoI.z : float.MaxValue
				);
			//tune steering modifiers
			CFG.SteeringModifier.x = Mathf.Clamp(TCAConfiguration.Globals.SteeringCurve.Evaluate(angularA.x)/100f, 0, 100);
			CFG.SteeringModifier.y = Mathf.Clamp(TCAConfiguration.Globals.SteeringCurve.Evaluate(angularA.y)/100f, 0, 100);
			CFG.SteeringModifier.z = Mathf.Clamp(TCAConfiguration.Globals.SteeringCurve.Evaluate(angularA.z)/100f, 0, 100);
			//tune PI coefficients
			CFG.Engines.P = TCAConfiguration.Globals.EnginesCurve.Evaluate(angularA.magnitude);
			CFG.Engines.I = CFG.Engines.P/2f;
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
}
