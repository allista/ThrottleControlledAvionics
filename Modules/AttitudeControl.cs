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

			[Persistent] public float P = 0.9f, If = 20f;
			[Persistent] public float MinD  = 0.02f, MaxD  = 0.07f;
			[Persistent] public float MinTf = 0.1f,  MaxTf = 1f;
			[Persistent] public float InertiaFactor = 10f, AngularMomentumFactor = 0.002f;
			[Persistent] public float MoIFactor = 0.01f;
			[Persistent] public float KillRotThreshold = 0.02f;
			public float TfSpan;

			public override void Init() 
			{ 
				base.Init();
				TfSpan = MaxTf-MinTf;
			}
		}
		static Config ATC { get { return TCAScenario.Globals.ATC; } }

		Vector3  angularVelocity { get { return VSL.vessel.angularVelocity; } }
		readonly PIDv_Controller2 pid = new PIDv_Controller2();
		Vector3 locked_bearing;
		Quaternion attitude_error;
		bool bearing_locked, rotating, was_rotating;
		public float AngleError { get; private set; }

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
			pid.Reset();
			AngleError = 0;
			bearing_locked = false;
			rotating = true;
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

		void CalculatedRotation()
		{
			Vector3 v;
			var thrust = VSL.Thrust.IsZero()? VSL.MaxThrust : VSL.Thrust;
			var lthrust = VSL.refT.InverseTransformDirection(thrust);
			attitude_error = Quaternion.identity;
			switch(CFG.AT.state)
			{
			case Attitude.Custom:
				attitude_error = VSL.CustomRotation;
				break;
			case Attitude.KillRot:
				if(bearing_locked)
					attitude_error = Quaternion.FromToRotation(VSL.refT.InverseTransformDirection(locked_bearing), lthrust);
				if(!bearing_locked || was_rotating && !rotating)
				{
					locked_bearing = thrust;
					bearing_locked = true;
				}
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
				attitude_error = Quaternion.FromToRotation(VSL.refT.InverseTransformDirection(-VSL.vessel.orbit.GetOrbitNormal().xzy), lthrust);
				break;
			case Attitude.AntiNormal:
				attitude_error = Quaternion.FromToRotation(VSL.refT.InverseTransformDirection(VSL.vessel.orbit.GetOrbitNormal().xzy), lthrust);
				break;
			case Attitude.Zenith:
				attitude_error = Quaternion.FromToRotation(-VSL.UpL, lthrust);
				break;
			case Attitude.Nadir:
				attitude_error = Quaternion.FromToRotation(VSL.UpL, lthrust);
				break;
			case Attitude.ManeuverNode:
				var solver = VSL.vessel.patchedConicSolver;
				if(solver == null || solver.maneuverNodes.Count == 0)
				{ CFG.AT.On(Attitude.KillRot); break; }
				attitude_error = 
					Quaternion.FromToRotation(VSL.refT.InverseTransformDirection(-solver.maneuverNodes[0].GetBurnVector(VSL.vessel.orbit)), lthrust);
				break;
			}
			VSL.ResetCustomRotation();
		}

		protected override void Update(FlightCtrlState s)
		{
			//need to check all the prerequisites, because the callback is called asynchroniously
			if(!(CFG.Enabled && CFG.AT && VSL.refT != null && VSL.vessel.orbit != null)) return;
			//disable SAS
			VSL.ActionGroups.SetGroup(KSPActionGroup.SAS, false);
			//allow user to intervene
			if(UserIntervening(s)) 
			{ pid.Reset(); bearing_locked = false; rotating = true; return; }
			//calculate needed rotation
			was_rotating = rotating;
			if(VSL.vessel.angularVelocity.sqrMagnitude > ATC.KillRotThreshold * 10) rotating = true;
			else rotating &= VSL.vessel.angularVelocity.sqrMagnitude >= ATC.KillRotThreshold;
			CalculatedRotation();
			//calculate corresponding rotation
			var steering_error = new Vector3(Utils.CenterAngle(attitude_error.eulerAngles.x),
			                                 Utils.CenterAngle(attitude_error.eulerAngles.y),
			                                 Utils.CenterAngle(attitude_error.eulerAngles.z))*Mathf.Deg2Rad;
			AngleError = steering_error.magnitude*Mathf.Rad2Deg;
			//tune PID parameters and steering_error
			var angularM = Vector3.Scale(angularVelocity, VSL.MoI);
			var inertia  = Vector3.Scale(angularM.Sign(),
			                             Vector3.Scale(Vector3.Scale(angularM, angularM),
			                                           Vector3.Scale(VSL.MaxTorque, VSL.MoI).Inverse()))
				.ClampComponents(-Mathf.PI, Mathf.PI);
			var Tf = Mathf.Clamp(1/VSL.MaxAngularA_m, ATC.MinTf, ATC.MaxTf);
			steering_error += inertia / Mathf.Lerp(ATC.InertiaFactor, 1, 
			                                       VSL.MoI.magnitude*ATC.MoIFactor);
			Vector3.Scale(steering_error, VSL.MaxAngularA.normalized);
			pid.D = Mathf.Lerp(ATC.MinD, ATC.MaxD, angularM.magnitude/Utils.ClampL(steering_error.sqrMagnitude, 1e-45f)*ATC.AngularMomentumFactor);
			pid.I = pid.P / (ATC.If * Tf/ATC.MinTf);
			//update PID controller and set steering
			pid.Update(steering_error, angularVelocity);
			ThrottleControlledAvionics.DebugMessage = string.Format("pid: {0}\nerror {1}", pid, steering_error);//debug
			SetRot(pid.Action, s);
		}
	}
}
