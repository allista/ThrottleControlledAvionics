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
			[Persistent] public float RotationUpperThreshold     = 30f;
			[Persistent] public float TranslationMaxAngle        = 80f;
			[Persistent] public float RotationMaxAngle           = 15f;
			[Persistent] public float ManualTranslationIMinSpeed = 20f;
			[Persistent] public PID_Controller ManualTranslationPID = new PID_Controller(0.5f, 0, 0.5f, 0, 1);
			public float TranslationMaxCos;
			public float RotationMaxCos;

			[Persistent] public float TWRf  = 5;
			[Persistent] public float HVCurve = 2;
			[Persistent] public float AccelerationFactor = 1f, MinHvThreshold = 10f;

			public override void Init() 
			{ 
				base.Init();
				TranslationMaxCos = Mathf.Cos(TranslationMaxAngle*Mathf.Deg2Rad);
				RotationMaxCos = Mathf.Cos(RotationMaxAngle*Mathf.Deg2Rad);
			}
		}
		static Config HSC { get { return TCAScenario.Globals.HSC; } }

		double   srfSpeed { get { return VSL.vessel.srfSpeed; } }
		Vector3d acceleration { get { return VSL.vessel.acceleration; } }
		Vector3  angularVelocity { get { return VSL.vessel.angularVelocity; } }
		readonly PIDf_Controller translation_pid = new PIDf_Controller();
		Vector3d needed_thrust_dir;

		public HorizontalSpeedControl(VesselWrapper vsl) { VSL = vsl; }

		public override void Init() 
		{ 
			base.Init(); 
			translation_pid.setPID(HSC.ManualTranslationPID);
			CFG.HF.AddCallback(HFlight.Stop, Enable);
			CFG.HF.AddCallback(HFlight.Level, Enable);
			CFG.HF.AddCallback(HFlight.Move, Move);
			#if DEBUG
			RenderingManager.AddToPostDrawQueue(1, RadarBeam);
			#endif
		}

		protected override void UpdateState() 
		{ 
			IsActive = VSL.OnPlanet && CFG.HF; 
			if(IsActive) return;
			if(VSL.ManualTranslationSwitch.On)
				EnableManualTranslation(false);
			CFG.AT.OffIfOn(Attitude.Custom);
		}

		public override void Enable(bool enable = true)
		{
			translation_pid.Reset();
			if(enable) 
			{
				CFG.AT.OnIfNot(Attitude.Custom);
				VSL.UpdateOnPlanetStats();
				VSL.SetNeededHorVelocity(Vector3d.zero);
			}
			else 
			{
				CFG.AT.OffIfOn(Attitude.Custom);
				EnableManualTranslation(false); 
			}
			BlockSAS(enable);
		}

		public void Move(bool enable = true)
		{
			translation_pid.Reset();
			if(enable) 
			{
				CFG.AT.OnIfNot(Attitude.Custom);
				VSL.UpdateOnPlanetStats();
			}
			else 
			{
				CFG.AT.OffIfOn(Attitude.Custom);
				EnableManualTranslation(false); 
			}
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
			if(Changed && VSL.CanUpdateEngines) CFG.ActiveProfile.Update(VSL.ActiveEngines);
		}

		protected override void OnAutopilotUpdate(FlightCtrlState s)
		{
			//need to check all the prerequisites, because the callback is called asynchroniously
			if(!(CFG.Enabled && CFG.HF && VSL.refT != null && VSL.OnPlanet)) return;
			if(VSL.AutopilotDisabled) return;
			CFG.AT.OnIfNot(Attitude.Custom);
			//set forward direction
			VSL.ForwardDirection = VSL.NeededHorVelocity;
			//calculate prerequisites
			var thrust = VSL.refT.InverseTransformDirection(VSL.Thrust);
			needed_thrust_dir = -VSL.UpL;
			if(!CFG.HF[HFlight.Level])
			{
				//if the vessel is not moving, nothing to do
				if(VSL.LandedOrSplashed || VSL.Thrust.IsZero()) return;
				//calculate horizontal velocity
				VSL.CourseCorrection = Vector3d.zero;
				for(int i = 0, count = VSL.CourseCorrections.Count; i < count; i++)
					VSL.CourseCorrection += VSL.CourseCorrections[i];
				var nV  = VSL.NeededHorVelocity+VSL.CourseCorrection;
				var hV  = VSL.HorizontalVelocity-nV;
				var rV  = hV; //velocity that is needed to be handled by attitude control of the total thrust
				var fV  = hV; //forward-backward velocity with respect to the manual thrust vector
				var hVm = hV.magnitude;
				var with_manual_thrust = VSL.ManualEngines.Count > 0;
				if(with_manual_thrust && 
				   VSL.ManualThrust.sqrMagnitude/VSL.M > HSC.TranslationLowerThreshold &&
				   hVm > HSC.TranslationLowerThreshold && 
				   Vector3.Dot(VSL.ManualThrust, hV) > 0)
				{
					thrust -= VSL.refT.InverseTransformDirection(VSL.ManualThrust);
					rV = Vector3.ProjectOnPlane(hV, VSL.ManualThrust);
					fV = hV-rV;
				}
				var rVm = rV.magnitude;
				var fVm = fV.magnitude;
				//calculate needed thrust direction
				if(rVm > HSC.RotationLowerThreshold)
				{
					var rVl   = VSL.refT.InverseTransformDirection(rV);
					//correction for low TWR
					var twr   = VSL.SlowEngines? VSL.DTWR : VSL.MaxTWR*0.70710678f; //MaxTWR at 45deg
					var upF   = Utils.Clamp(twr/HSC.TWRf, 1e-9, 1);
					var MaxHv = Utils.ClampL(Vector3d.Project(acceleration, rV).magnitude*HSC.AccelerationFactor, HSC.MinHvThreshold);
					needed_thrust_dir = rVl.normalized - VSL.UpL*Utils.ClampL(Math.Pow(MaxHv/rVm, HSC.HVCurve), 1)/upF*Utils.ClampL(fVm/rVm, 1);
//					Log("upF {0}, MaxHv {1}, downF {2}\n" +
//					    "rV {3}\nhV {4}\nMT {5}\nneeded_thrust {6}\ntwr {7}", upF, MaxHv, Utils.ClampL(Math.Pow(MaxHv/rVm, HSC.HVCurve), 1)/upF*Utils.ClampL(fVm/rVm, 1), 
//					    rVl, VSL.refT.InverseTransformDirection(hV), VSL.refT.InverseTransformDirection(VSL.ManualThrust), needed_thrust_dir, twr);//debug
				}
				if(hVm > HSC.TranslationLowerThreshold)
				{
					//try to use translation
					var nVm = nV.magnitude;
					var hVl = VSL.refT.InverseTransformDirection(hV);
					var hVl_dir = hVl.CubeNorm();
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
						var trans = -Utils.ClampH(cVl_lat_m/HSC.TranslationUpperThreshold, 1)*cVl_lat.CubeNorm();
						s.X = trans.x; s.Z = trans.y; s.Y = trans.z;
					}
					//manual engine control
					if(with_manual_thrust && 
					   (nVm >= HSC.TranslationUpperThreshold ||
					    hVm >= HSC.TranslationUpperThreshold ||
					    VSL.CourseCorrection.magnitude >= HSC.TranslationUpperThreshold))
					{
						//turn the nose if nesessary
						var pure_hV = VSL.HorizontalVelocity-VSL.NeededHorVelocity;
						var NVm = VSL.NeededHorVelocity.magnitude;
						if(pure_hV.magnitude >= HSC.RotationUpperThreshold &&
						   (NVm < HSC.TranslationLowerThreshold || 
						    Vector3.Dot(HVn, VSL.NeededHorVelocity/NVm) < HSC.RotationMaxCos))
						{
							var max_MT = VSL.ManualThrustLimits.MaxInPlane(VSL.UpL);
							if(!max_MT.IsZero())
							{
								var rot = Quaternion.AngleAxis(Vector3.Angle(max_MT, Vector3.ProjectOnPlane(VSL.FwdL, VSL.UpL)),
								                               VSL.Up * Mathf.Sign(Vector3.Dot(max_MT, Vector3.right)));
									VSL.ForwardDirection = rot*pure_hV;
							}
						}
						translation_pid.I = (VSL.HorizontalSpeed > HSC.ManualTranslationIMinSpeed && 
						                     VSL.vessel.mainBody.atmosphere)? 
							HSC.ManualTranslationPID.I*VSL.HorizontalSpeed : 0;
						translation_pid.Update((float)fVm);
						VSL.ManualTranslation = translation_pid.Action*hVl_dir;
						EnableManualTranslation(translation_pid.Action > 0);
					}
					else EnableManualTranslation(false);
				}
				else EnableManualTranslation(false);
			}
			else 
			{
				EnableManualTranslation(false);
				if(thrust.IsZero()) 
					thrust = VSL.refT.InverseTransformDirection(VSL.MaxThrust);
			}
			VSL.AddCustomRotation(needed_thrust_dir, thrust);
		}

		#if DEBUG
		public void RadarBeam()
		{
			if(VSL == null || VSL.vessel == null || VSL.refT == null || !CFG.HF) return;
			if(!VSL.NeededHorVelocity.IsZero())
				GLUtils.GLVec(VSL.wCoM,  VSL.NeededHorVelocity, Color.red);
			if(!VSL.HorizontalVelocity.IsZero())
				GLUtils.GLVec(VSL.wCoM+VSL.Up,  VSL.HorizontalVelocity, Color.magenta);
			if(!VSL.ForwardDirection.IsZero())
				GLUtils.GLVec(VSL.wCoM+VSL.Up*2,  VSL.ForwardDirection, Color.green);
			if(!VSL.CourseCorrection.IsZero())
				GLUtils.GLVec(VSL.wCoM+VSL.Up*3, VSL.CourseCorrection, Color.blue);
		}

		public override void Reset()
		{
			base.Reset();
			RenderingManager.RemoveFromPostDrawQueue(1, RadarBeam);
		}
		#endif
	}
}

