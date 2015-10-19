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
using UnityEngine;

namespace ThrottleControlledAvionics
{
	public class HorizontalSpeedControl : AutopilotModule
	{
		public class Config : ModuleConfig
		{
			new public const string NODE_NAME = "HSC";

			[Persistent] public float TranslationUpperThreshold  = 5f;
			[Persistent] public float TranslationLowerThreshold  = 0.2f;
			[Persistent] public float RotationLowerThreshold     = 0.01f;
			[Persistent] public float TranslationMaxAngle        = 45f;
			[Persistent] public float ManualTranslationIMinSpeed = 20f;
			[Persistent] public float ManualTranslationCutoff    = 0.001f;
			[Persistent] public PID_Controller ManualTranslationPID = new PID_Controller(0.5f, 0, 0.5f, 0, 1);
			public float TranslationMaxCos;

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
				TranslationMaxCos = Mathf.Cos(TranslationMaxAngle*Mathf.Deg2Rad);
			}
		}
		static Config HSC { get { return TCAScenario.Globals.HSC; } }

		double   srfSpeed { get { return VSL.vessel.srfSpeed; } }
		Vector3d acceleration { get { return VSL.vessel.acceleration; } }
		Vector3  angularVelocity { get { return VSL.vessel.angularVelocity; } }
		readonly PIDv_Controller2 pid = new PIDv_Controller2();
		readonly PIDf_Controller translation_pid = new PIDf_Controller();

		public HorizontalSpeedControl(VesselWrapper vsl) { VSL = vsl; }

		public override void Init() 
		{ 
			base.Init(); 
			pid.P = HSC.P;
			translation_pid.setPID(HSC.ManualTranslationPID);
			CFG.HF.AddCallback(HFlight.Stop, Enable);
			CFG.HF.AddCallback(HFlight.Level, Enable);
			CFG.HF.AddCallback(HFlight.Move, Move);
			#if DEBUG
			RenderingManager.AddToPostDrawQueue(1, RadarBeam);
			#endif
		}

		public override void UpdateState() 
		{ 
			IsActive = CFG.HF && VSL.OnPlanet; 
			if(!IsActive && VSL.ManualTranslationSwitch.On)
				EnableManualTranslation(false);
		}

		public override void Enable(bool enable = true)
		{
			pid.Reset();
			translation_pid.Reset();
			if(enable) 
			{
				CFG.Nav.Off();
				VSL.UpdateOnPlanetStats();
				CFG.Starboard = Vector3.zero;
				VSL.NeededHorVelocity = Vector3d.zero;
			}
			else EnableManualTranslation(false); 
			BlockSAS(enable);
		}

		public void Move(bool enable = true)
		{
			pid.Reset();
			translation_pid.Reset();
			if(enable) VSL.UpdateOnPlanetStats();
			else EnableManualTranslation(false);
			BlockSAS(enable);
		}

		void EnableManualTranslation(bool enable = true)
		{
			VSL.ManualTranslationSwitch.Set(enable);
			if(VSL.ManualTranslationSwitch.On) return;
			var Changed = false;
			for(int i = 0, count = VSL.ManualEngines.Count; i < count; i++)
			{
				var e = VSL.ManualEngines[i];
				if(!e.engine.thrustPercentage.Equals(0))
				{
					Changed = true;
					e.limit = e.best_limit = 0;
					e.engine.thrustPercentage = 0;
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
				VSL.CourseCorrection = Vector3d.zero;
				for(int i = 0, count = VSL.CourseCorrections.Count; i < count; i++)
					VSL.CourseCorrection += VSL.CourseCorrections[i];
				var nV  = VSL.NeededHorVelocity*VSL.NHVf+VSL.CourseCorrection;
				var hV  = VSL.HorizontalVelocity-nV;
				var rV  = hV; //velocity that is needed to be handled by attitude control of the total thrust
				var fV  = hV; //forward-backward velocity with respect to the manual thrust vector
				var hVm = hV.magnitude;
				var with_manual_thrust = VSL.ManualEngines.Count > 0;
				if(with_manual_thrust && 
				   VSL.ManualThrust.sqrMagnitude > HSC.TranslationLowerThreshold &&
				   hVm > HSC.TranslationLowerThreshold && 
				   Vector3.Dot(VSL.ManualThrust.normalized, hV.normalized) > 0)
				{
					thrust -= VSL.refT.InverseTransformDirection(VSL.ManualThrust);
					rV = Vector3.ProjectOnPlane(hV, VSL.ManualThrust);
					fV = hV-rV;
				}
				var rVm = rV.magnitude;
				//calculate needed thrust direction
				if(rVm > HSC.RotationLowerThreshold)
				{
					var rVl   = VSL.refT.InverseTransformDirection(rV);
					//correction for low TWR and torque
					var twrF  = Utils.ClampH(VSL.DTWR/HSC.TWRf, 1);
					var torF  = Utils.ClampH(Mathf.Abs(Vector3.Dot(Vector3.Cross(hV, VSL.MaxThrust).normalized, VSL.wMaxAngularA))
					                         *(float)Utils.ClampL(rVm, 1)
					                         *HSC.TorF, 1);
					var upF   = Vector3.Dot(thrust, rVl) < 0? 1 : Utils.ClampL(twrF*torF, 1e-9f);
					var MaxHv = Math.Max(acceleration.magnitude*HSC.AccelerationFactor, HSC.MinHvThreshold);
					needed_thrust_dir = rVl.normalized - upl*Utils.ClampL((float)Math.Pow(MaxHv/rVm, HSC.HVCurve), 1)/upF;
				}
				if(hVm > HSC.TranslationLowerThreshold)
				{
					//also try to use translation
					var nVm = nV.magnitude;
					var hVl_dir = VSL.refT.InverseTransformDirection(hV).CubeNorm();
					var cVl_lat = VSL.refT.InverseTransformDirection(Vector3.ProjectOnPlane(VSL.CourseCorrection, nV));
					var cVl_lat_m = cVl_lat.magnitude;
					var nVn = nVm > 0? nV/nVm : Vector3d.zero;
					var HVn = VSL.HorizontalVelocity.normalized;
					//normal translation controls (maneuver engines and RCS)
					if(nVm < HSC.TranslationUpperThreshold || 
					   Mathf.Abs((float)Vector3d.Dot(HVn, nVn)) < HSC.TranslationMaxCos)
					{
						var trans = Utils.ClampH((float)hVm/HSC.TranslationUpperThreshold, 1)*hVl_dir;
						s.X = trans.x; s.Z = trans.y; s.Y = trans.z;
					}
					else if(cVl_lat_m > HSC.TranslationLowerThreshold)
					{
						var trans = -Utils.ClampH((float)cVl_lat_m/HSC.TranslationUpperThreshold, 1)*cVl_lat.CubeNorm();
						s.X = trans.x; s.Z = trans.y; s.Y = trans.z;
					}
					//manual engine control
					if(with_manual_thrust && 
					   (nVm >= HSC.TranslationUpperThreshold ||
					    VSL.CourseCorrection.magnitude >= HSC.TranslationUpperThreshold ||
					    hVm >= HSC.TranslationUpperThreshold &&
					    (nVm < HSC.TranslationLowerThreshold || 
					     Vector3d.Dot(HVn, nVn) < -HSC.TranslationMaxCos)))
					{
						translation_pid.I = (VSL.HorizontalSpeed > HSC.ManualTranslationIMinSpeed && 
						                     VSL.vessel.mainBody.atmosphere)? 
							HSC.ManualTranslationPID.I*VSL.HorizontalSpeed : 0;
						translation_pid.Update((float)fV.magnitude);
						VSL.ManualTranslation = translation_pid.Action*hVl_dir;
//						Log("\nnV {0}\nfV {1}\nhV {2}\nmanual translation{3}\nerror {4}; throttle {5:P4}", 
//						    nV, hV, fV, VSL.ManualTranslation, fV.magnitude, translation_pid.Action);//debug
						EnableManualTranslation(translation_pid.Action > HSC.ManualTranslationCutoff);
					}
					else EnableManualTranslation(false);
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
		}

		#if DEBUG
		public void RadarBeam()
		{
			if(VSL == null || VSL.vessel == null) return;
			if(!VSL.NeededHorVelocity.IsZero())
				GLUtils.GLVec(VSL.wCoM,  VSL.NeededHorVelocity, Color.red);
			if(!VSL.CourseCorrection.IsZero())
				GLUtils.GLVec(VSL.wCoM, VSL.CourseCorrection, Color.blue);
		}

		public override void Reset()
		{
			base.Reset();
			RenderingManager.RemoveFromPostDrawQueue(1, RadarBeam);
		}
		#endif
	}
}

