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
			[Persistent] public float KillRotThreshold = 0.02f;
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
		float last_omega2;

		public AttitudeControl(VesselWrapper vsl) { VSL = vsl; }

		public override void Init() 
		{ 
			base.Init(); 
			pid.P = ATC.P;
			CFG.AT.AddSingleCallback(Enable);
		}

		public override void UpdateState() { IsActive = CFG.AT; }

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
			AngleError = 0;
			last_omega2 = 0;
			attitude_locked = false;
		}

		void CalculatedRotation()
		{
			Vector3 v;
			var thrust = VSL.Thrust.IsZero()? VSL.MaxThrust : VSL.Thrust;
			var lthrust = VSL.refT.InverseTransformDirection(thrust);
			var omega2 = VSL.vessel.angularVelocity.sqrMagnitude;
			attitude_error = Quaternion.identity;
			switch(CFG.AT.state)
			{
			case Attitude.Custom:
				attitude_error = VSL.CustomRotation;
				break;
			case Attitude.KillRot:
				if(!attitude_locked || omega2 > last_omega2 || refT != VSL.refT)
				{
					refT = VSL.refT;
					locked_attitude = refT.rotation;
					attitude_locked = true;
				}
				if(attitude_locked)
					attitude_error = refT.rotation.Inverse()*locked_attitude;
				break;
			case Attitude.Prograde:
				v = VSL.vessel.situation == Vessel.Situations.ORBITING ||
					VSL.vessel.situation == Vessel.Situations.SUB_ORBITAL? 
					-VSL.vessel.obt_velocity : -VSL.vessel.srf_velocity;
				attitude_error = Quaternion.FromToRotation(VSL.refT.InverseTransformDirection(v), lthrust);
				break;
			case Attitude.Retrograde:
				v = VSL.vessel.situation == Vessel.Situations.ORBITING ||
					VSL.vessel.situation == Vessel.Situations.SUB_ORBITAL? 
					VSL.vessel.obt_velocity : VSL.vessel.srf_velocity;
				attitude_error = Quaternion.FromToRotation(VSL.refT.InverseTransformDirection(v), lthrust);
				break;
			case Attitude.Normal:
				attitude_error = Quaternion.FromToRotation(-VSL.refT.InverseTransformDirection(orbit.h.xzy), lthrust);
				break;
			case Attitude.AntiNormal:
				attitude_error = Quaternion.FromToRotation(VSL.refT.InverseTransformDirection(orbit.h.xzy), lthrust);
				break;
			case Attitude.Radial:
				attitude_error = Quaternion.FromToRotation(VSL.refT.InverseTransformDirection(Vector3.Cross(VSL.vessel.obt_velocity, orbit.h.xzy)), lthrust);
				break;
			case Attitude.AntiRadial:
				attitude_error = Quaternion.FromToRotation(-VSL.refT.InverseTransformDirection(Vector3.Cross(VSL.vessel.obt_velocity, orbit.h.xzy)), lthrust);
				break;
			case Attitude.ManeuverNode:
				var solver = VSL.vessel.patchedConicSolver;
				if(solver == null || solver.maneuverNodes.Count == 0)
				{ CFG.AT.On(Attitude.KillRot); break; }
				attitude_error = 
					Quaternion.FromToRotation(VSL.refT.InverseTransformDirection(-solver.maneuverNodes[0].GetBurnVector(orbit)), lthrust);
				break;
			}
			last_omega2 = omega2;
			VSL.ResetCustomRotation();
		}

		protected override void Update(FlightCtrlState s)
		{
			//need to check all the prerequisites, because the callback is called asynchroniously
			if(!(CFG.Enabled && CFG.AT && VSL.refT != null && orbit != null)) return;
			//disable SAS
			VSL.ActionGroups.SetGroup(KSPActionGroup.SAS, false);
			//allow user to intervene
			if(UserIntervening(s)) { reset(); return; }
			//calculate needed rotation
			CalculatedRotation();
			//calculate corresponding rotation
			var steering = new Vector3(Utils.CenterAngle(attitude_error.eulerAngles.x),
			                           Utils.CenterAngle(attitude_error.eulerAngles.y),
			                           Utils.CenterAngle(attitude_error.eulerAngles.z))*Mathf.Deg2Rad;
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

			ThrottleControlledAvionics.DebugMessage = string.Format("pid: {0}\nerror {1}", pid, steering);//debug
		}
	}
}
