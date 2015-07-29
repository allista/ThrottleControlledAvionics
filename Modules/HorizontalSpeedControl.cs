//   HorizontalSpeedControl.cs
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
using UnityEngine;

namespace ThrottleControlledAvionics
{
	public class HorizontalSpeedControl : TCAModule
	{
		public class Config : ModuleConfig
		{
			new public const string NODE_NAME = "HSC";

			[Persistent] public float P = 0.9f, If = 20f;
			[Persistent] public float MinD = 0.02f, MaxD = 0.07f;
			[Persistent] public float MinTf = 0.1f, MaxTf = 1f;
			[Persistent] public float TWRf = 5;
			[Persistent] public float upF  = 3;
			[Persistent] public float InertiaFactor = 10f, AngularMomentumFactor = 0.002f;
			[Persistent] public float AccelerationFactor = 1f, MinHvThreshold = 10f;
			[Persistent] public float MoIFactor = 0.01f;
			public float TfSpan;
			public override void Init() { TfSpan = MaxTf-MinTf; }
		}
		static Config HSC { get { return TCAConfiguration.Globals.HSC; } }

		public double   srfSpeed { get { return vessel.vessel.srfSpeed; } }
		public Vector3d srf_velocity { get { return vessel.vessel.srf_velocity; } }
		public Vector3d acceleration { get { return vessel.vessel.acceleration; } }
		public Vector3  angularVelocity { get { return vessel.vessel.angularVelocity; } }
		readonly PIDv_Controller pid = new PIDv_Controller();

		public HorizontalSpeedControl(VesselWrapper vsl) { vessel = vsl; }
		public override void Init() { pid.P = HSC.P; }
		public override void UpdateState() { IsActive = CFG.KillHorVel && vessel.OnPlanet; }

		public void ConnectAutopilot() { vessel.OnAutopilotUpdate += Update; }
		public void DisconnectAutopilot() { vessel.OnAutopilotUpdate -= Update; }

		void Update(FlightCtrlState s)
		{
			//need to check all the prerequisites
			if(!(vessel.TCA_Available && CFG.Enabled && CFG.KillHorVel && vessel.refT != null && vessel.OnPlanet)) return;
			//allow user to intervene
			if(!Mathfx.Approx(s.pitch, s.pitchTrim, 0.1f) ||
			   !Mathfx.Approx(s.roll, s.rollTrim, 0.1f) ||
			   !Mathfx.Approx(s.yaw, s.yawTrim, 0.1f)) return;
			//if the vessel is not moving, nothing to do
			if(vessel.LandedOrSplashed || srfSpeed < 0.01) return;
			//calculate total current thrust
			var thrust = vessel.ActiveEngines.Aggregate(Vector3.zero, (v, e) => v + e.thrustDirection*e.finalThrust);
			if(thrust.IsZero()) return;
			//disable SAS
			vessel.ActionGroups.SetGroup(KSPActionGroup.SAS, false);
			//calculate horizontal velocity
			var hV  = Vector3d.Exclude(vessel.up, srf_velocity);
			var hVm = hV.magnitude;
			//calculate needed thrust direction
			var MaxHv = Math.Max(acceleration.magnitude*HSC.AccelerationFactor, HSC.MinHvThreshold);
			var max_torque = vessel.E_TorqueLimits.Max+vessel.R_TorqueLimits.Max;
			Vector3 needed_thrust_dir;
			if(hVm > 1e-7)
			{
				//correction for low TWR and torque
				var upl  = vessel.refT.InverseTransformDirection(vessel.up);
				var hVl  = vessel.refT.InverseTransformDirection(hV);
				var TWR = Vector3.Dot(thrust, upl) < 0? Vector3.Project(thrust, upl).magnitude/9.81f/vessel.M : 0f;
				var twrF = Utils.ClampH(TWR/HSC.TWRf, 1);
				var torF = Utils.ClampH(Vector3.Scale(Vector3.ProjectOnPlane(max_torque, hVl), vessel.MoI.Inverse()).sqrMagnitude, 1);
				var upF  = Vector3.Dot(thrust, hVl) < 0? 1 : Mathf.Pow(Utils.ClampL(twrF*torF, 1e-9f), HSC.upF);
				needed_thrust_dir = hVl.normalized - upl*Utils.ClampL((float)(MaxHv/hVm), 1)/upF;
//				Utils.Log("needed thrust direction: {0}\n" +
//				          "TWR factor: {1}\n" +
//				          "torque factor: {2}\n" +
//				          "up factor: {3}\n" +
//				          "TWR: {4}\n" +
//				          "torque limits {5}\n" +
//				          "MoI {6}\n", 
//				          needed_thrust_dir,
//				          twrF,
//				          torF,
//				          upF, 
//				          vessel.TWR, 
//				          max_torque, 
//				          vessel.MoI
//				         );//debug
			}
			else needed_thrust_dir = vessel.refT.InverseTransformDirection(-vessel.up);
			//calculate corresponding rotation
			var attitude_error = Quaternion.FromToRotation(needed_thrust_dir, thrust);
			var steering_error = new Vector3(Utils.CenterAngle(attitude_error.eulerAngles.x),
			                                 Utils.CenterAngle(attitude_error.eulerAngles.y),
			                                 Utils.CenterAngle(attitude_error.eulerAngles.z))/180*Mathf.PI;
			//tune PID parameters and steering_error
			var angularM = Vector3.Scale(angularVelocity, vessel.MoI);
			var inertia  = Vector3.Scale(angularM.Sign(),
			                             Vector3.Scale(Vector3.Scale(angularM, angularM),
			                                           Vector3.Scale(max_torque, vessel.MoI).Inverse()))
				.ClampComponents(-Mathf.PI, Mathf.PI);
			var Tf = Mathf.Clamp(1/vessel.MaxAngularA.magnitude, HSC.MinTf, HSC.MaxTf);
			steering_error += inertia / Mathf.Lerp(HSC.InertiaFactor, 1, 
			                                       vessel.MoI.magnitude*HSC.MoIFactor);
			Vector3.Scale(steering_error, vessel.MaxAngularA.normalized);
			pid.D = Mathf.Lerp(HSC.MinD, HSC.MaxD, angularM.magnitude*HSC.AngularMomentumFactor);
			pid.I = pid.P / (HSC.If * Tf/HSC.MinTf);
			//update PID controller and set steering
			pid.Update(steering_error, angularVelocity);
			s.pitch = pid.Action.x;
			s.roll  = pid.Action.y;
			s.yaw   = pid.Action.z;
			#if DEBUG
//			Utils.Log(//debug
//			          "hV: {0}\n" +
//			          "Thrust: {1}\n" +
//			          "Needed Thrust Dir: {2}\n" +
//			          "H-T Angle: {3}\n" +
//			          "Steering error: {4}\n" +
//			          "Down comp: {5}\n" +
//			          "omega: {6}\n" +
//			          "Tf: {7}\n" +
//			          "PID: {8}\n" +
//			          "angularA: {9}\n" +
//			          "inertia: {10}\n" +
//			          "angularM: {11}\n" +
//			          "inertiaF: {12}\n" +
//			          "MaxHv: {13}\n" +
//			          "MoI: {14}",
//			          vessel.refT.InverseTransformDirection(hV),
//			          thrust, needed_thrust_dir,
//                    Vector3.Angle(vessel.refT.InverseTransformDirection(hV), needed_thrust_dir),
//			          steering_error,
//			          Utils.ClampL(10/(float)hVm, 1),
//			          angularVelocity, Tf,
//			          new Vector3(pid.P, pid.I, pid.D),
//                    vessel.AngularA, inertia, angularM,
//			          Mathf.Lerp(HSC.InertiaFactor, 1, 
//                               vessel.MoI.magnitude*HSC.MoIFactor),
//			          MaxHv,
//                    vessel.MoI
//			);
			#endif
		}
	}
}

