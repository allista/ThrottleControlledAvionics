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
			[Persistent] public PID_Controller ManualTranslationPID = new PID_Controller(0.5f, 0, 0.5f, -1, 1);

			[Persistent] public float P = 0.9f, If = 20f;
			[Persistent] public float MinD  = 0.02f, MaxD  = 0.07f;
			[Persistent] public float MinTf = 0.1f,  MaxTf = 1f;
			[Persistent] public float TWRf  = 5;
			[Persistent] public float TorF  = 3;
			[Persistent] public float HVCurve = 2;
			[Persistent] public float InertiaFactor = 10f, AngularMomentumFactor = 0.002f;
			[Persistent] public float AccelerationFactor = 1f, MinHvThreshold = 10f;
			[Persistent] public float MoIFactor = 0.01f;
			public float TfSpan;

			public override void Init() 
			{ 
				base.Init();
				TfSpan = MaxTf-MinTf; 
			}
		}
		static Config HSC { get { return TCAScenario.Globals.HSC; } }

		double   srfSpeed { get { return VSL.vessel.srfSpeed; } }
		Vector3d acceleration { get { return VSL.vessel.acceleration; } }
		Vector3  angularVelocity { get { return VSL.vessel.angularVelocity; } }
		readonly PIDv_Controller2 pid = new PIDv_Controller2();
		readonly PIDf_Controller translation_pid = new PIDf_Controller();
		float manual_translation = 0.1f;

		public HorizontalSpeedControl(VesselWrapper vsl) { VSL = vsl; }

		public override void Init() 
		{ 
			base.Init(); 
			pid.P = HSC.P;
			translation_pid.setPID(HSC.ManualTranslationPID);
			manual_translation = 0.1f;
			CFG.HF.AddCallback(HFlight.Stop, Enable);
			CFG.HF.AddCallback(HFlight.Level, Enable);
			CFG.HF.AddCallback(HFlight.Move, Move);
		}

		public override void UpdateState() { IsActive = CFG.HF && VSL.OnPlanet; }

		public override void Enable(bool enable = true)
		{
			pid.Reset();
			translation_pid.Reset();
			manual_translation = 0.1f;
			if(enable) 
			{
				CFG.Nav.Off();
				VSL.UpdateOnPlanetStats();
				CFG.Starboard = Vector3.zero;
				CFG.NeededHorVelocity = Vector3d.zero;
			}
			else EnableManualTranslation(false); 
			BlockSAS(enable);
		}

		public void Move(bool enable = true)
		{
			pid.Reset();
			translation_pid.Reset();
			manual_translation = 0.1f;
			if(enable) VSL.UpdateOnPlanetStats();
			else EnableManualTranslation(false);
			BlockSAS(enable);
		}

		void EnableManualTranslation(bool enable = true)
		{
			VSL.ManualTranslationEnabled = enable;
			if(VSL.ManualTranslationEnabled) return;
			var Changed = false;
			for(int i = 0, count = VSL.ManualEngines.Count; i < count; i++)
			{
				var e = VSL.ManualEngines[i];
				if(e.engine.thrustPercentage > 0)
				{
					Changed = true;
					e.limit = e.best_limit = 0;
					e.forceThrustPercentage(0);
				}
			}
			if(Changed) CFG.ActiveProfile.Update(VSL.ActiveEngines);
		}

		protected override void Update(FlightCtrlState s)
		{
			//need to check all the prerequisites, because the callback is called asynchroniously
			if(!(CFG.Enabled && CFG.HF && VSL.refT != null && VSL.OnPlanet)) return;
			//disable SAS
			VSL.ActionGroups.SetGroup(KSPActionGroup.SAS, false);
			//allow user to intervene
			if(UserIntervening(s)) { pid.Reset(); return; }
			var thrust = VSL.refT.InverseTransformDirection(VSL.Thrust);
			var upl    = VSL.refT.InverseTransformDirection(VSL.Up);
			var needed_thrust_dir = -upl;
			if(!CFG.HF[HFlight.Level])
			{
				//if the vessel is not moving, nothing to do
				if(VSL.LandedOrSplashed || VSL.Thrust.IsZero()) return;
				//calculate horizontal velocity
				var nV  = CFG.NeededHorVelocity*CFG.NHVf+CFG.CourseCorrection;
				var hV  = VSL.HorizontalVelocity-nV;
				var hVl = VSL.refT.InverseTransformDirection(hV);
				var hVm = hV.magnitude;
				//calculate needed thrust direction
				if(hVm > HSC.RotationLowerThreshold)
				{
					//correction for low TWR and torque
					var twrF  = Utils.ClampH(VSL.DTWR/HSC.TWRf, 1);
					var torF  = Utils.ClampH(Vector3.ProjectOnPlane(VSL.MaxPitchRollAA, hVl).magnitude
					                         *(float)Utils.ClampL(hVm, 1)
					                         *HSC.TorF, 1);
					var upF   = Vector3.Dot(thrust, hVl) < 0? 1 : Utils.ClampL(twrF*torF, 1e-9f);
					var MaxHv = Math.Max(acceleration.magnitude*HSC.AccelerationFactor, HSC.MinHvThreshold);
					needed_thrust_dir = hVl.normalized - upl*Utils.ClampL((float)Math.Pow(MaxHv/hVm, HSC.HVCurve), 1)/upF;
	//				Utils.Log("needed thrust direction: {0}\n" +
	//				          "TWR factor: {1}\n" +
	//				          "torque factor: {2}\n" +
	//				          "up factor: {3}\n" +
	//				          "torque limits {4}\n" +
	//				          "MoI {5}\n", 
	//				          needed_thrust_dir,
	//				          twrF,
	//				          torF,
	//				          upF, 
	//				          VSL.MaxTorque, 
	//				          VSL.MoI
	//				         );//debug
				}
				if(hVm > HSC.TranslationLowerThreshold)
				{
					//also try to use translation control
					var hVl_dir = hVl.CubeNorm();
					if(nV.magnitude < HSC.TranslationUpperThreshold)
					{
						EnableManualTranslation(false);
						var trans = Utils.ClampH((float)hVm/HSC.TranslationUpperThreshold, 1)*hVl_dir;
						s.X = trans.x; s.Z = trans.y; s.Y = trans.z;
					}
					else 
					{
						var inclination  = Vector3.Dot(VSL.MaxThrust.normalized, VSL.Up);
						var thrust_pitch = Vector3.Dot(VSL.ManualThrust.normalized, VSL.Up);
						translation_pid.Update((1+inclination)*thrust_pitch);
						manual_translation = Utils.Clamp(manual_translation+translation_pid.Action, 0, 1);
						VSL.ManualTranslation = manual_translation*hVl_dir;
//						Log("incl {0}; pitch {1}; error {2}; action {3}; translation {4}", 
//						    inclination, thrust_pitch, (1+inclination)*thrust_pitch, translation_pid.Action, manual_translation);//debug
						EnableManualTranslation();
					}
				}
				else EnableManualTranslation(false);
			}
			else EnableManualTranslation(false);
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
//			          "pitch, roll, yaw: {13}," +
//			          "X, Y, Z {14}",
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
//			          pid.Action, new Vector3(s.X, s.Y, s.Z)
//			);
		}
	}
}

