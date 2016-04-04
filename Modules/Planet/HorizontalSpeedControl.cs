//  Author:
//       Allis Tauri <allista@gmail.com>
//
//  Copyright (c) 2015 Allis Tauri
//
// This work is licensed under the Creative Commons Attribution-ShareAlike 4.0 International License. 
// To view a copy of this license, visit http://creativecommons.org/licenses/by-sa/4.0/ 
// or send a letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.

using System;
using System.Collections.Generic;
using UnityEngine;

namespace ThrottleControlledAvionics
{
	[CareerPart]
	[RequireModules(typeof(AttitudeControl),
	                typeof(BearingControl),
	                typeof(SASBlocker))]
	[OptionalModules(typeof(TranslationControl))]
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

			[Persistent] public float TWRf  = 3;
			[Persistent] public float VSf   = 3;
			[Persistent] public float HVCurve = 2;
			[Persistent] public float SlowTorqueF = 2;
			[Persistent] public float AccelerationFactor = 1f, MinHvThreshold = 10f;
			[Persistent] public float LowPassF = 0.1f;

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
		readonly LowPassFilterVd filter = new LowPassFilterVd();
		Vector3d needed_thrust_dir;

		public List<Vector3d> CourseCorrections = new List<Vector3d>();
		Vector3d              CourseCorrection;

		//modules
		BearingControl BRC;
		AttitudeControl ATC;
		TranslationControl TRA;

		public HorizontalSpeedControl(ModuleTCA tca) : base(tca) {}

		public override void Init() 
		{ 
			base.Init(); 
			filter.Tau = HSC.LowPassF;
			translation_pid.setPID(HSC.ManualTranslationPID);
			CFG.HF.AddSingleCallback(ControlCallback);
			#if DEBUG
			RenderingManager.AddToPostDrawQueue(1, RadarBeam);
			#endif
		}

		#if DEBUG
		public void RadarBeam()
		{
			if(VSL == null || VSL.vessel == null || VSL.refT == null || !CFG.HF) return;
			if(!VSL.HorizontalSpeed.NeededVector.IsZero())
				GLUtils.GLVec(VSL.Physics.wCoM, VSL.HorizontalSpeed.NeededVector, Color.red);
//			if(!VSL.HorizontalSpeed.Vector.IsZero())
//				GLUtils.GLVec(VSL.Physics.wCoM+VSL.Physics.Up,  VSL.HorizontalSpeed.Vector, Color.magenta);
//			if(!TCA.CC.ForwardDirection.IsZero())
//				GLUtils.GLVec(VSL.Physics.wCoM+VSL.Physics.Up*2,  TCA.CC.ForwardDirection, Color.green);
			if(!CourseCorrection.IsZero())
				GLUtils.GLVec(VSL.Physics.wCoM+VSL.Physics.Up*3, CourseCorrection, Color.blue);
		}

		public override void Reset()
		{
			base.Reset();
			RenderingManager.RemoveFromPostDrawQueue(1, RadarBeam);
		}
		#endif

		public override void ClearFrameState() { CourseCorrections.Clear(); }

		protected override void UpdateState() 
		{ 
			IsActive = CFG.Enabled && VSL.OnPlanet && CFG.HF && VSL.refT != null; 
			if(IsActive) return;
			if(VSL.Controls.ManualTranslationSwitch.On)
				EnableManualTranslation(false);
		}

		public void ControlCallback(Multiplexer.Command cmd)
		{
			translation_pid.Reset();
			switch(cmd)
			{
			case Multiplexer.Command.Resume:
				RegisterTo<SASBlocker>();
				RegisterTo<Radar>(vsl => vsl.HorizontalSpeed.MoovingFast);
				break;

			case Multiplexer.Command.On:
				VSL.UpdateOnPlanetStats();
				if(CFG.HF[HFlight.Stop])
				{
					VSL.HorizontalSpeed.SetNeeded(Vector3d.zero);
					CFG.Nav.Off(); //any kind of navigation conflicts with the Stop program; obviously.
				}
				CFG.AT.OnIfNot(Attitude.Custom);
				goto case Multiplexer.Command.Resume;

			case Multiplexer.Command.Off:
				UnregisterFrom<SASBlocker>();
				UnregisterFrom<Radar>();
				CFG.AT.OffIfOn(Attitude.Custom);
				EnableManualTranslation(false); 
				break;
			}
		}

		void EnableManualTranslation(bool enable = true)
		{
			VSL.Controls.ManualTranslationSwitch.Set(enable);
			if(VSL.Controls.ManualTranslationSwitch.On) return;
			var Changed = false;
			for(int i = 0, count = VSL.Engines.Manual.Count; i < count; i++)
			{
				var e = VSL.Engines.Manual[i];
				if(!e.engine.thrustPercentage.Equals(0))
				{
					Changed = true;
					e.limit = e.best_limit = 0;
					e.forceThrustPercentage(0);
				}
			}
			if(Changed && TCA.ProfileSyncAllowed) 
				CFG.ActiveProfile.Update(VSL.Engines.Active);
		}

		protected override void OnAutopilotUpdate(FlightCtrlState s)
		{
			if(!IsActive) return;
			if(VSL.AutopilotDisabled) { filter.Reset(); return; }
			CFG.AT.OnIfNot(Attitude.Custom);
			//set forward direction
			BRC.ForwardDirection = VSL.HorizontalSpeed.NeededVector;
			//calculate prerequisites
			var thrust = VSL.Engines.Thrust;
			needed_thrust_dir = -VSL.Physics.Up;
			if(!CFG.HF[HFlight.Level])
			{
				//if the vessel is not moving, nothing to do
				if(VSL.LandedOrSplashed || VSL.Engines.Thrust.IsZero()) return;
				//calculate horizontal velocity
				CourseCorrection = Vector3d.zero;
				for(int i = 0, count = CourseCorrections.Count; i < count; i++)
					CourseCorrection += CourseCorrections[i];
				var nV  = VSL.HorizontalSpeed.NeededVector+CourseCorrection;
				var hV  = VSL.HorizontalSpeed.Vector-nV;
				var hVl = VSL.LocalDir(hV);
				var nVm = nV.magnitude;
				var hVm = hV.magnitude;
				var HVn = VSL.HorizontalSpeed.normalized;
				//if the manual translation can and should be used
				var rV  = hV; //velocity that is needed to be handled by attitude control of the total thrust
				var fV  = hV; //forward-backward velocity with respect to the manual thrust vector
				var with_manual_thrust = VSL.Engines.Manual.Count > 0 && (nVm >= HSC.TranslationUpperThreshold ||
				                                                          hVm >= HSC.TranslationUpperThreshold ||
				                                                          CourseCorrection.magnitude >= HSC.TranslationUpperThreshold);
				var manual_thrust = Vector3.ProjectOnPlane(VSL.OnPlanetParams.ManualThrust, VSL.Physics.Up);//test
				var zero_manual_thrust = manual_thrust.IsZero();
				if(with_manual_thrust &&
				   !zero_manual_thrust &&
				   Vector3.Dot(manual_thrust, hV) > 0)
				{
					thrust -= manual_thrust;
					rV = Vector3.ProjectOnPlane(hV, manual_thrust);
					fV = hV-rV;
				}
				var rVm = rV.magnitude;
				var fVm = fV.magnitude;
				//calculate needed thrust direction
				if(!(with_manual_thrust && zero_manual_thrust &&
				     VSL.HorizontalSpeed.Absolute <= HSC.TranslationLowerThreshold) &&
				   Utils.ClampL(rVm/fVm, 0) > HSC.RotationLowerThreshold)//test
				{
					//correction for low TWR
					var vsf   = CFG.VSCIsActive && VSL.VerticalSpeed.Absolute < 0? 
						Utils.Clamp(1-(Utils.ClampH(CFG.VerticalCutoff, 0)-VSL.VerticalSpeed.Absolute)/HSC.VSf, 1e-9, 1) : 1;
					var twr   = VSL.OnPlanetParams.SlowThrust? VSL.OnPlanetParams.DTWR : VSL.OnPlanetParams.MaxTWR*0.70710678f; //MaxTWR at 45deg
					var MaxHv = Utils.ClampL(Vector3d.Project(acceleration, rV).magnitude*HSC.AccelerationFactor, HSC.MinHvThreshold);
					var upF   = 
						Utils.ClampL(Math.Pow(MaxHv/rVm, HSC.HVCurve), 1)/
						Utils.Clamp(twr/HSC.TWRf, 1e-9, 1)/vsf*
						Utils.ClampL(fVm/rVm, 1);
					needed_thrust_dir = rV.normalized - VSL.Physics.Up*upF;
				}
				if(hVm > HSC.TranslationLowerThreshold)
				{
					//try to use translation
					var nVn = nVm > 0? nV/nVm : Vector3d.zero;
					var cV_lat = Vector3.ProjectOnPlane(CourseCorrection, nV);
					//normal translation controls (maneuver engines and RCS)
					if(TRA != null)
					{
						if(nVm < HSC.TranslationUpperThreshold || 
						   Mathf.Abs((float)Vector3d.Dot(HVn, nVn)) < HSC.TranslationMaxCos)
							TRA.AddDeltaV(hVl);
						else if(cV_lat.magnitude > HSC.TranslationLowerThreshold)
							TRA.AddDeltaV(-VSL.LocalDir(cV_lat));
					}
				}
				//manual engine control
				if(with_manual_thrust)
				{
					//turn the nose if nesessary
					var pure_hV = VSL.HorizontalSpeed.Vector-VSL.HorizontalSpeed.NeededVector;
					var NVm = VSL.HorizontalSpeed.NeededVector.magnitude;
					var transF = 1f;
					if(pure_hV.magnitude >= HSC.RotationUpperThreshold &&
					   (NVm < HSC.TranslationLowerThreshold || 
					    Vector3.Dot(HVn, VSL.HorizontalSpeed.NeededVector/NVm) < HSC.RotationMaxCos))
					{
						var max_MT = VSL.OnPlanetParams.ManualThrustLimits.MaxInPlane(VSL.Physics.UpL);
						if(!max_MT.IsZero())
						{
							var rot = Quaternion.AngleAxis(Vector3.Angle(max_MT, Vector3.ProjectOnPlane(VSL.OnPlanetParams.FwdL, VSL.Physics.UpL)),
							                               VSL.Physics.Up * Mathf.Sign(Vector3.Dot(max_MT, Vector3.right)));
							BRC.ForwardDirection = rot*pure_hV;
							transF = Utils.ClampL(Vector3.Dot(VSL.OnPlanetParams.Fwd, BRC.ForwardDirection.normalized), 0);
							transF *= transF*transF;
						}
					}
					translation_pid.I = (VSL.HorizontalSpeed > HSC.ManualTranslationIMinSpeed && 
					                     VSL.vessel.mainBody.atmosphere)? 
						HSC.ManualTranslationPID.I*VSL.HorizontalSpeed : 0;
					translation_pid.Update((float)fVm);
					VSL.Controls.ManualTranslation = translation_pid.Action*hVl.CubeNorm()*transF;
					EnableManualTranslation(translation_pid.Action > 0);
				}
				else EnableManualTranslation(false);
			}
			else 
			{
				EnableManualTranslation(false);
				if(thrust.IsZero()) thrust = VSL.Engines.MaxThrust;
			}
			//tune filter
			filter.Tau = VSL.Engines.SlowTorque ? 
				HSC.LowPassF / (1 + VSL.Engines.TorqueResponseTime * HSC.SlowTorqueF) : 
				HSC.LowPassF;
			ATC.AddCustomRotationW(filter.Update(needed_thrust_dir), thrust);

			#if DEBUG
//			CSV(VSL.Physics.UT, 
//			    filter.Value.x, filter.Value.y, filter.Value.z,
//			    thrust.x, thrust.y, thrust.z);//debug
			#endif
		}
	}
}

