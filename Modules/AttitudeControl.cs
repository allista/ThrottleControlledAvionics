//   AttitudeController.cs
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
using UnityEngine;

namespace ThrottleControlledAvionics
{
	public class AttitudeControl : AutopilotModule
	{
		public class Config : ModuleConfig
		{
			new public const string NODE_NAME = "ATC";

			[Persistent] public float P = 0.9f, I = 20f;
			[Persistent] public float MinD  = 0.02f, MaxD  = 0.07f;
			[Persistent] public float MinTf = 0.1f,  MaxTf = 1f;
			[Persistent] public float InertiaFactor = 10f, AngularMomentumFactor = 0.002f;
			[Persistent] public float MoIFactor = 0.01f;
			[Persistent] public float AngleThreshold = 25f;
			[Persistent] public float OmegaThreshold = 0.3f; //deg/s
			public float Omega2Threshold;

			public override void Init()
			{
				base.Init();
				Omega2Threshold = Mathf.Pow(OmegaThreshold*Mathf.Deg2Rad, 2);
			}
		}
		static Config ATC { get { return TCAScenario.Globals.ATC; } }

		Vector3  angularVelocity { get { return VSL.vessel.angularVelocity; } }
		Transform  vesselTransform { get { return VSL.vessel.transform; } }
		Orbit orbit { get { return VSL.vessel.orbit; } }

		public float AngleError { get; private set; }
		readonly PIDv_Controller2 pid = new PIDv_Controller2();
		Transform refT;
		Quaternion attitude_error, locked_attitude;
		bool attitude_locked;
		Vector3 thrust, lthrust, needed_lthrust, steering;
		float omega2, p_omega2, pp_omega2;

		public AttitudeControl(VesselWrapper vsl) { VSL = vsl; }

		public override void Init() 
		{ 
			base.Init(); 
			pid.P = ATC.P;
			CFG.AT.AddSingleCallback(Enable);
		}

		protected override void UpdateState() { IsActive = CFG.AT; }

		public override void Enable(bool enable = true)
		{
			reset();
			if(enable)
			{
				VSL.UpdateOnPlanetStats();
				if(!CFG.AT[Attitude.Custom])
				{
					CFG.HF.Off();
					CFG.Nav.Off();
					CFG.AP.Off();
				}
			}
			BlockSAS(enable);
		}

		void reset()
		{
			pid.Reset();
			refT = null;
			AngleError = 0;
			omega2 = 0;
			p_omega2 = 0;
			pp_omega2 = 0;
			attitude_locked = false;
//			Log("Reset");
		}

		void CalculateSteering()
		{
			Vector3 v;
			omega2 = VSL.vessel.angularVelocity.sqrMagnitude;
			attitude_error = Quaternion.identity;
			needed_lthrust = Vector3.zero;
			switch(CFG.AT.state)
			{
			case Attitude.Custom:
				attitude_error = VSL.CustomRotation;
				break;
			case Attitude.KillRot:
				if(refT != VSL.refT || 
				   !attitude_locked && p_omega2 <= pp_omega2 && p_omega2 < omega2)
				{
					refT = VSL.refT;
					locked_attitude = refT.rotation;
					attitude_locked = omega2 < ATC.Omega2Threshold;
//					Log("pp {0}, p {1}, o2 {2}, attitude_locked {3}",
//					    pp_omega2, p_omega2, omega2, attitude_locked);//debug
				}
				if(refT != null)
					attitude_error = Quaternion.Inverse(refT.rotation.Inverse()*locked_attitude);
//				if(attitude_locked) Log("attitude_error: {0}", attitude_error.eulerAngles);//debug
				break;
			case Attitude.Prograde:
				v = VSL.vessel.situation == Vessel.Situations.ORBITING ||
					VSL.vessel.situation == Vessel.Situations.SUB_ORBITAL? 
					-VSL.vessel.obt_velocity : -VSL.vessel.srf_velocity;
				needed_lthrust = VSL.refT.InverseTransformDirection(v);
				break;
			case Attitude.Retrograde:
				v = VSL.vessel.situation == Vessel.Situations.ORBITING ||
					VSL.vessel.situation == Vessel.Situations.SUB_ORBITAL? 
					VSL.vessel.obt_velocity : VSL.vessel.srf_velocity;
				needed_lthrust = VSL.refT.InverseTransformDirection(v);
				break;
			case Attitude.Normal:
				needed_lthrust = -VSL.refT.InverseTransformDirection(orbit.h.xzy);
				break;
			case Attitude.AntiNormal:
				needed_lthrust = VSL.refT.InverseTransformDirection(orbit.h.xzy);
				break;
			case Attitude.Radial:
				needed_lthrust = VSL.refT.InverseTransformDirection(Vector3.Cross(VSL.vessel.obt_velocity, orbit.h.xzy));
				break;
			case Attitude.AntiRadial:
				needed_lthrust = -VSL.refT.InverseTransformDirection(Vector3.Cross(VSL.vessel.obt_velocity, orbit.h.xzy));
				break;
			case Attitude.ManeuverNode:
				var solver = VSL.vessel.patchedConicSolver;
				if(solver == null || solver.maneuverNodes.Count == 0)
				{ CFG.AT.On(Attitude.KillRot); break; }
				needed_lthrust = VSL.refT.InverseTransformDirection(-solver.maneuverNodes[0].GetBurnVector(orbit));
				break;
			}
			if(!needed_lthrust.IsZero())
			{
				thrust = VSL.Thrust.IsZero()? VSL.MaxThrust : VSL.Thrust;
				lthrust = VSL.refT.InverseTransformDirection(thrust).normalized;
				needed_lthrust.Normalize();
				if(Vector3.Angle(needed_lthrust, lthrust) > ATC.AngleThreshold)
				{
					//rotational axis
					var axis = Vector3.Cross(needed_lthrust, lthrust).Exclude(lthrust.MaxI());
					if(axis.sqrMagnitude < 0.01f) 
						axis = VSL.MaxAngularA.Exclude(lthrust.MaxI()).MaxComponent();
					//main rotation component
					var axis1 = axis.MaxComponent();
					var lthrust_cmp1 = Vector3.ProjectOnPlane(lthrust, axis1);
					var needed_lthrust_cmp1 = Vector3.ProjectOnPlane(needed_lthrust, axis1);
					var angle1 = Vector3.Angle(needed_lthrust_cmp1, lthrust_cmp1);
					//second rotation component
					var axis2 = (axis - axis1).MaxComponent();
					var angle2 = Vector3.Angle(needed_lthrust, needed_lthrust_cmp1);
					//steering
					steering = (axis1.normalized * angle1 + axis2.normalized * angle2)*Mathf.Deg2Rad;
//					Log("\naxis {0}\naxis1 {1}\naxis2 {2}\nangle1 {3}, angle2 {4}, error {5}",
//					    axis, axis1, axis2, angle1, angle2, steering);//debug
				}
				else attitude_error = Quaternion.FromToRotation(needed_lthrust, lthrust);   
			}
			if(attitude_error != Quaternion.identity)
				steering = new Vector3(Utils.CenterAngle(attitude_error.eulerAngles.x),
				                       Utils.CenterAngle(attitude_error.eulerAngles.y),
				                       Utils.CenterAngle(attitude_error.eulerAngles.z))*Mathf.Deg2Rad;
			VSL.ResetCustomRotation();
			pp_omega2 = p_omega2;
			p_omega2 = omega2;
		}

		protected override void OnAutopilotUpdate(FlightCtrlState s)
		{
			//need to check all the prerequisites, because the callback is called asynchroniously
			if(!(CFG.Enabled && CFG.AT && VSL.refT != null && orbit != null)) return;
			//disable SAS
			VSL.ActionGroups.SetGroup(KSPActionGroup.SAS, false);
			//allow user to intervene
			if(UserIntervening(s)) { reset(); return; }
			//calculate needed steering
			CalculateSteering();
			AngleError = steering.magnitude*Mathf.Rad2Deg;
			//tune PID parameters and steering_error
			var angularM = Vector3.Scale(angularVelocity, VSL.MoI);
			var inertia  = Vector3.Scale(angularM.Sign(),
			                             Vector3.Scale(Vector3.Scale(angularM, angularM),
			                                           Vector3.Scale(VSL.MaxTorque, VSL.MoI).Inverse()))
				.ClampComponents(-Mathf.PI, Mathf.PI);
			var Tf = Mathf.Clamp(1/VSL.MaxAngularA_m, ATC.MinTf, ATC.MaxTf)/ATC.MinTf;
			steering += inertia / Mathf.Lerp(ATC.InertiaFactor, 1, VSL.MoI.magnitude*ATC.MoIFactor);
			Vector3.Scale(steering, VSL.MaxAngularA.normalized);
			pid.P = ATC.P*Tf;
			pid.I = ATC.I*Tf;
			pid.D = Mathf.Lerp(ATC.MinD, ATC.MaxD, angularM.magnitude/Utils.ClampL(steering.magnitude, 1e-45f)*ATC.AngularMomentumFactor)*Tf;
			//update PID controller and set steering
			pid.Update(steering, angularVelocity);
			SetRot(pid.Action, s);

//			Log("\ncontrol {0}\nsteering {1}\naction {2}\n==========================================\n", 
//			    control, steering*Mathf.Rad2Deg, pid.Action);//debug
			#if DEBUG
			ThrottleControlledAvionics.DebugMessage = string.Format("pid: {0}\nerror {1}", pid, steering);
			#endif
		}
	}
}
