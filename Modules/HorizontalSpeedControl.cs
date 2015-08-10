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
	public class HorizontalSpeedControl : AutopilotModule
	{
		public class Config : ModuleConfig
		{
			new public const string NODE_NAME = "HSC";

			[Persistent] public float TranslationUpperThreshold = 5f;
			[Persistent] public float TranslationLowerThreshold = 0.2f;
			[Persistent] public float RotationLowerThreshold    = 0.01f;

			[Persistent] public float P = 0.9f, If = 20f;
			[Persistent] public float MinD  = 0.02f, MaxD  = 0.07f;
			[Persistent] public float MinTf = 0.1f,  MaxTf = 1f;
			[Persistent] public float TWRf  = 5;
			[Persistent] public float TorF  = 3;
			[Persistent] public float InertiaFactor = 10f, AngularMomentumFactor = 0.002f;
			[Persistent] public float AccelerationFactor = 1f, MinHvThreshold = 10f;
			[Persistent] public float MoIFactor = 0.01f;
			public float TfSpan;
			public override void Init() { TfSpan = MaxTf-MinTf; }
		}
		static Config HSC { get { return TCAConfiguration.Globals.HSC; } }

		public double   srfSpeed { get { return VSL.vessel.srfSpeed; } }
		public Vector3d acceleration { get { return VSL.vessel.acceleration; } }
		public Vector3  angularVelocity { get { return VSL.vessel.angularVelocity; } }
		readonly PIDv_Controller pid = new PIDv_Controller();

		public HorizontalSpeedControl(VesselWrapper vsl) { VSL = vsl; }
		public override void Init() { pid.P = HSC.P; }
		public override void UpdateState() { IsActive = (CFG.CruiseControl || CFG.KillHorVel) && VSL.OnPlanet; }

		public override void Enable(bool enable = true)
		{
//			if(enable == CFG.KillHorVel) return;
			CFG.KillHorVel = enable;
			pid.Reset();
			if(CFG.KillHorVel) 
			{
				CFG.CruiseControl = false;
				CFG.GoToTarget = false;
				CFG.FollowPath = false;
				VSL.UpdateHorizontalStats();
				CFG.Starboard = Vector3.zero;
				CFG.NeededHorVelocity = Vector3d.zero;
			}
			BlockSAS(CFG.KillHorVel);
		}

		protected override void Update(FlightCtrlState s)
		{
			//need to check all the prerequisites, because the callback is called asynchroniously
			if(!(CFG.Enabled && 
			     (CFG.CruiseControl || CFG.KillHorVel) && 
			     VSL.refT != null && VSL.OnPlanet)) return;
			//allow user to intervene
			if(UserIntervening(s)) { pid.Reset(); return; }
			//if the vessel is not moving, nothing to do
			if(VSL.LandedOrSplashed || srfSpeed < 0.01 || VSL.Thrust.IsZero()) return;
			//disable SAS
			VSL.ActionGroups.SetGroup(KSPActionGroup.SAS, false);
			//calculate horizontal velocity
			var thrust = VSL.refT.InverseTransformDirection(VSL.Thrust);
			var hV  = VSL.HorizontalVelocity-CFG.NeededHorVelocity;
			var hVl = VSL.refT.InverseTransformDirection(hV);
			var hVm = hV.magnitude;
			//calculate needed thrust direction
			var MaxHv = Math.Max(acceleration.magnitude*HSC.AccelerationFactor, HSC.MinHvThreshold);
			Vector3 needed_thrust_dir;
			if(hVm > HSC.RotationLowerThreshold)
			{
				//correction for low TWR and torque
				var upl  = VSL.refT.InverseTransformDirection(VSL.Up);
				var TWR  = Vector3.Dot(thrust, upl) < 0? Vector3.Project(thrust, upl).magnitude/TCAConfiguration.G/VSL.M : 0f;
				var twrF = Utils.ClampH(TWR/HSC.TWRf, 1);
				var torF = Utils.ClampH(Vector3.Scale(Vector3.ProjectOnPlane(VSL.MaxTorque, hVl), VSL.MoI.Inverse()).magnitude*HSC.TorF, 1);
				var upF  = Vector3.Dot(thrust, hVl) < 0? 1 : Utils.ClampL(twrF*torF, 1e-9f);
				if(IsStateSet(TCAState.LoosingAltitude)) upF /= 10;
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
//				          TWR, 
//				          VSL.MaxTorque, 
//				          VSL.MoI
//				         );//debug
			}
			else needed_thrust_dir = VSL.refT.InverseTransformDirection(-VSL.Up);
			if(hVm > HSC.TranslationLowerThreshold)
			{
				//also try to use translation control to slow down
				var hVl_dir = Utils.ClampH((float)(hVm/HSC.TranslationUpperThreshold), 1)*hVl.CubeNorm();	
				s.X = hVl_dir.x; s.Z = hVl_dir.y; s.Y = hVl_dir.z;
			}
			//calculate corresponding rotation
			var attitude_error = Quaternion.FromToRotation(needed_thrust_dir, thrust);
			var steering_error = new Vector3(Utils.CenterAngle(attitude_error.eulerAngles.x),
			                                 Utils.CenterAngle(attitude_error.eulerAngles.y),
			                                 Utils.CenterAngle(attitude_error.eulerAngles.z))/180*Mathf.PI;
			//tune PID parameters and steering_error
			var angularM = Vector3.Scale(angularVelocity, VSL.MoI);
			var inertia  = Vector3.Scale(angularM.Sign(),
			                             Vector3.Scale(Vector3.Scale(angularM, angularM),
			                                           Vector3.Scale(VSL.MaxTorque, VSL.MoI).Inverse()))
				.ClampComponents(-Mathf.PI, Mathf.PI);
			var Tf = Mathf.Clamp(1/VSL.MaxAngularA_m, HSC.MinTf, HSC.MaxTf);
			steering_error += inertia / Mathf.Lerp(HSC.InertiaFactor, 1, 
			                                       VSL.MoI.magnitude*HSC.MoIFactor);
			Vector3.Scale(steering_error, VSL.MaxAngularA.normalized);
			pid.D = Mathf.Lerp(HSC.MinD, HSC.MaxD, angularM.magnitude*HSC.AngularMomentumFactor);
			pid.I = pid.P / (HSC.If * Tf/HSC.MinTf);
			//update PID controller and set steering
			pid.Update(steering_error, angularVelocity);
			SetRot(pid.Action, s);
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
//			          "pitch, roll, yaw: {14}," +
//			          "X, Y, Z {15}",
//			          VSL.refT.InverseTransformDirection(hV),
//			          thrust, needed_thrust_dir,
//                      Vector3.Angle(VSL.refT.InverseTransformDirection(hV), needed_thrust_dir),
//			          steering_error,
//			          Utils.ClampL(10/(float)hVm, 1),
//			          angularVelocity, Tf,
//			          pid,
//                      VSL.MaxAngularA, inertia, angularM,
//			          Mathf.Lerp(HSC.InertiaFactor, 1, 
//                      VSL.MoI.magnitude*HSC.MoIFactor),
//			          MaxHv,
//			          pid.Action, new Vector3(s.X, s.Y, s.Z)
//			);
			#endif
		}
	}
}

