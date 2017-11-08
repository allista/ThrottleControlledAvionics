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
using AT_Utils;

namespace ThrottleControlledAvionics
{
	public abstract class ThrustDirectionControl : AutopilotModule
	{
		public class Config : ModuleConfig
		{
			[Persistent] public float TWRf  = 3;
			[Persistent] public float VSf   = 3;
		}
		static Config TDC { get { return Globals.Instance.TDC; } }

		protected ThrustDirectionControl(ModuleTCA tca) : base(tca) {}
	}

	[CareerPart]
	[RequireModules(typeof(AttitudeControl),
	                typeof(BearingControl),
	                typeof(SASBlocker))]
	[OptionalModules(typeof(TranslationControl))]
	public class HorizontalSpeedControl : ThrustDirectionControl
	{
		public new class Config : ModuleConfig
		{
            public class ManualThrustConfig : ConfigNodeObject
            {
                [Persistent] public PIDf_Controller PID = new PIDf_Controller(0.5f, 0, 0.5f, 0, 1);
                [Persistent] public float ThrustF = 11.47f;
                [Persistent] public float I_MinSpeed = 20f;
                [Persistent] public float D_Max = 2;
                [Persistent] public float Turn_MinLateralDeltaV = 10f;
                [Persistent] public float Turn_MinDeltaV = 30f;
            }

			[Persistent] public float TranslationMaxDeltaV = 5f;
			[Persistent] public float TranslationMinDeltaV = 0.2f;
            [Persistent] public float TranslationMaxAngle  = 80f;

			[Persistent] public float RotationMinDeltaV = 0.01f;

			[Persistent] public float HVCurve = 2;
			[Persistent] public float MinHVCurve = 0.5f;
			[Persistent] public float SlowTorqueF = 2;
			[Persistent] public float AccelerationFactor = 1f, MinHvThreshold = 10f;
			[Persistent] public float LowPassF = 0.1f;

			[Persistent] public float MaxCorrectionWeight = 1f;

            [Persistent] public ManualThrustConfig ManualThrust = new ManualThrustConfig();

            [Persistent] public PIDf_Controller3 NeededThrustPID = new PIDf_Controller3(1, 0, 0, -1, 1, 1);
            [Persistent] public float TurnTime_Curve = 1.1f;

            public float TranslationMaxCos;

			public override void Init() 
			{ 
				base.Init();
				TranslationMaxCos = Mathf.Cos(TranslationMaxAngle*Mathf.Deg2Rad);
			}
		}
		static Config HSC { get { return Globals.Instance.HSC; } }

		public HorizontalSpeedControl(ModuleTCA tca) : base(tca) {}

		//modules
        #pragma warning disable 169
		BearingControl BRC;
		AttitudeControl ATC;
		TranslationControl TRA;
        #pragma warning restore 169

		readonly PIDf_Controller translation_pid = new PIDf_Controller();
        readonly PIDf_Controller3 needed_thrust_pid = new PIDf_Controller3();
        readonly LowPassFilterVd output_filter = new LowPassFilterVd();
		Vector3d needed_thrust_dir;

		readonly List<Vector3d> CourseCorrections = new List<Vector3d>();
		Vector3d CourseCorrection;
        Vector3 manual_thrust;

		public override void Init() 
		{ 
			base.Init(); 
			output_filter.Tau = HSC.LowPassF;
			translation_pid.setPID(HSC.ManualThrust.PID);
            needed_thrust_pid.setPID(HSC.NeededThrustPID);
			CFG.HF.AddSingleCallback(ControlCallback);
		}

		public void AddRawCorrection(Vector3d cor) 
		{ CourseCorrections.Add(cor); }

		public void AddWeightedCorrection(Vector3d cor) 
		{ 
			var cm = cor.magnitude;
			if(cm > 1e-10) cor *= Math.Sqrt(1/cm);
			if(VSL.Physics.G > 1e-10) cor *= Utils.ClampH(Utils.G0/VSL.Physics.G, HSC.MaxCorrectionWeight);
			CourseCorrections.Add(cor);
		}

		#if DEBUG

		public void DrawDebugLines()
		{
			if(VSL == null || VSL.vessel == null || VSL.refT == null || !CFG.HF) return;
			Utils.GLVec(VSL.refT.position, VSL.HorizontalSpeed.NeededVector, Color.yellow);
            Utils.GLVec(VSL.refT.position, VSL.WorldDir(manual_thrust), Color.magenta);
			Utils.GLVec(VSL.refT.position+VSL.Physics.Up*VSL.Geometry.H, VSL.HorizontalSpeed.Vector, Color.red);
			Utils.GLVec(VSL.refT.position+VSL.Physics.Up*VSL.Geometry.H*1.1, CourseCorrection, Color.green);
		}
		#endif

		public override void ClearFrameState() { CourseCorrections.Clear(); }

        public override void Disable()
        {
            CFG.HF.Off();
            if(VSL.Controls.ManualTranslationSwitch.On)
                EnableManualThrust(false);
        }

		protected override void UpdateState() 
		{ 
			base.UpdateState();
			IsActive &= VSL.OnPlanet && CFG.HF && VSL.refT != null; 
		}

		public void ControlCallback(Multiplexer.Command cmd)
		{
			translation_pid.Reset();
            needed_thrust_pid.Reset();
			switch(cmd)
			{
			case Multiplexer.Command.Resume:
				RegisterTo<SASBlocker>();
				NeedCPSWhenMooving();
				break;

			case Multiplexer.Command.On:
				VSL.UpdateOnPlanetStats();
				if(CFG.HF[HFlight.Stop])
				{
					VSL.HorizontalSpeed.SetNeeded(Vector3d.zero);
					CFG.Nav.Off(); //any kind of navigation conflicts with the Stop program; obviously.
				}
				else if(CFG.HF[HFlight.NoseOnCourse])
					CFG.BR.OnIfNot(BearingMode.Auto);
				CFG.AT.OnIfNot(Attitude.Custom);
				goto case Multiplexer.Command.Resume;

			case Multiplexer.Command.Off:
				UnregisterFrom<SASBlocker>();
				ReleaseCPS();
				CFG.AT.OffIfOn(Attitude.Custom);
				CFG.BR.OffIfOn(BearingMode.Auto);
				EnableManualThrust(false); 
				break;
			}
		}

		void EnableManualThrust(bool enable = true)
		{
			VSL.Controls.ManualTranslationSwitch.Set(enable);
            if(!CFG.Enabled || VSL.Controls.ManualTranslationSwitch.On) return;
			var Changed = false;
			for(int i = 0, count = VSL.Engines.Active.Manual.Count; i < count; i++)
			{
				var e = VSL.Engines.Active.Manual[i];
				if(!e.engine.thrustPercentage.Equals(0))
				{
					e.limit = 0;
					e.forceThrustPercentage(0);
					Changed = true;
				}
			}
			if(Changed && TCA.ProfileSyncAllowed) 
				CFG.ActiveProfile.Update(VSL.Engines.Active);
		}

		protected override void OnAutopilotUpdate()
		{
			if(VSL.AutopilotDisabled) { output_filter.Reset(); return; }
			CFG.AT.OnIfNot(Attitude.Custom);
			//calculate prerequisites
            var thrust = VSL.Engines.DefThrust;
			needed_thrust_dir = -VSL.Physics.Up;
			if(CFG.HF[HFlight.Level])
            {
                thrust = VSL.Engines.CurrentDefThrustDir;
                VSL.Controls.ManualTranslationSwitch.Set(false);
            }
            else 
			{
				//set forward direction
				if(CFG.HF[HFlight.NoseOnCourse] && !VSL.HorizontalSpeed.NeededVector.IsZero())
					BRC.ForwardDirection = VSL.HorizontalSpeed.NeededVector;
                
				//calculate horizontal velocity
				CourseCorrection = Vector3d.zero;
				for(int i = 0, count = CourseCorrections.Count; i < count; i++)
					CourseCorrection += CourseCorrections[i];
                var needed_vector  = VSL.HorizontalSpeed.NeededVector+CourseCorrection;
                var error_vector  = VSL.HorizontalSpeed.Vector-needed_vector;
                var error_vector_local = VSL.LocalDir(error_vector);
                var needed_abs = needed_vector.magnitude;
                var error_abs = error_vector.magnitude;
                var horizontal_speed_dir = VSL.HorizontalSpeed.normalized;
                var rotation_vector  = error_vector; //velocity that is needed to be handled by attitude control of the total thrust
                var translaion_vector  = Vector3d.zero; //forward-backward velocity with respect to the manual thrust vector
                var rotation_abs = error_abs;
                var translation_abs = 0.0;

                //decide if manual thrust can and should be used
                var with_manual_thrust = VSL.Engines.Active.Manual.Count > 0 && 
                    (needed_abs >= HSC.TranslationMaxDeltaV ||
                     error_abs >= HSC.TranslationMaxDeltaV ||
                     CourseCorrection.magnitude >= HSC.TranslationMaxDeltaV);
                manual_thrust = Vector3.zero;
                if(with_manual_thrust)
                {
                    var forward_dir = VSL.HorizontalSpeed.NeededVector.IsZero()? 
                        VSL.OnPlanetParams.Fwd : (Vector3)VSL.HorizontalSpeed.NeededVector;
                    //first, see if we need to turn the nose so that the maximum manual thrust points the right way
                    var translation_factor = 1f;
                    var pure_error_vector = VSL.HorizontalSpeed.Vector-VSL.HorizontalSpeed.NeededVector;
                    var pure_needed_abs = VSL.HorizontalSpeed.NeededVector.magnitude;
                    if(pure_error_vector.magnitude >= HSC.ManualThrust.Turn_MinDeltaV &&
                       (pure_needed_abs < HSC.TranslationMinDeltaV || 
                        Vector3.ProjectOnPlane(VSL.HorizontalSpeed.NeededVector, horizontal_speed_dir)
                        .magnitude > HSC.ManualThrust.Turn_MinLateralDeltaV))
                    {
                        manual_thrust = VSL.Engines.ManualThrustLimits.MaxInPlane(VSL.Physics.UpL);
                        if(!manual_thrust.IsZero())
                        {
                            var fwdH = Vector3.ProjectOnPlane(VSL.OnPlanetParams.FwdL, VSL.Physics.UpL);
                            var angle = Utils.Angle2(manual_thrust, fwdH);
                            var rot = Quaternion.AngleAxis(angle, VSL.Physics.Up * Mathf.Sign(Vector3.Dot(manual_thrust, Vector3.right)));
                            BRC.DirectionOverride = rot*pure_error_vector;
                            translation_factor = Utils.ClampL((Vector3.Dot(VSL.OnPlanetParams.Fwd, BRC.DirectionOverride.normalized)-0.5f), 0)*2;
                            forward_dir = BRC.DirectionOverride;
                        }
                    }
                    //simply use manual thrust currently available in the forward direction
                    else if(Vector3.Dot(forward_dir, error_vector) < 0)  
                    {
                        manual_thrust = VSL.Engines.ManualThrustLimits.Slice(VSL.LocalDir(-forward_dir));
                        translation_factor = Utils.ClampL((Vector3.Dot(VSL.WorldDir(manual_thrust.normalized), -forward_dir.normalized)-0.5f), 0)*2;
                    }
                    with_manual_thrust = !manual_thrust.IsZero();
    				if(with_manual_thrust)
    				{
                        thrust = VSL.Engines.CurrentDefThrustDir;
                        rotation_vector = Vector3.ProjectOnPlane(error_vector, forward_dir);
                        translaion_vector = error_vector-rotation_vector;
                        rotation_abs = rotation_vector.magnitude;
                        translation_abs = Utils.ClampL(translaion_vector.magnitude, 1e-5);
                        translation_factor *= Utils.Clamp(1+Vector3.Dot(thrust.normalized, pure_error_vector.normalized)*HSC.ManualThrust.ThrustF, 0, 1);
                        translation_factor *= translation_factor*translation_factor*translation_factor;
                        translation_pid.I = (VSL.HorizontalSpeed > HSC.ManualThrust.I_MinSpeed && 
                                             VSL.vessel.mainBody.atmosphere)? 
                            HSC.ManualThrust.PID.I*VSL.HorizontalSpeed : 0;
                        var D = VSL.Engines.ManualThrustSpeed.Project(error_vector_local.normalized).magnitude;
                        if(D > 0) 
                            D = Mathf.Min(HSC.ManualThrust.PID.D/D, HSC.ManualThrust.D_Max);
                        translation_pid.D = D;
                        translation_pid.Update((float)translation_abs);
                        VSL.Controls.ManualTranslation = translation_pid.Action*error_vector_local.CubeNorm()*translation_factor;
                        EnableManualThrust(translation_pid.Action > 0);
                    }
                }
                if(!with_manual_thrust)
                    EnableManualThrust(false);
                
				//use attitude control to point total thrust to modify horizontal velocity
                if(rotation_abs > HSC.RotationMinDeltaV && 
                   Utils.ClampL(rotation_abs/translation_abs, 0) > HSC.RotationMinDeltaV &&
                   (!with_manual_thrust || 
                    VSL.HorizontalSpeed.Absolute > HSC.TranslationMinDeltaV))
                {
                    var GeeF  = Mathf.Sqrt(VSL.Physics.G/Utils.G0);
                    var MaxHv = Utils.ClampL(Vector3d.Project(VSL.vessel.acceleration, rotation_vector).magnitude * HSC.AccelerationFactor, HSC.MinHvThreshold);
                    var upF   = Utils.ClampL(Math.Pow(MaxHv/rotation_abs, Utils.ClampL(HSC.HVCurve*GeeF, HSC.MinHVCurve)), GeeF) * 
                        Utils.ClampL(translation_abs/rotation_abs, 1) / VSL.OnPlanetParams.TWRf;
                    needed_thrust_dir = rotation_vector.normalized - VSL.Physics.Up*upF;
                }

				//try to use translation controls (maneuver engines and RCS)
				if(error_abs > HSC.TranslationMinDeltaV && TRA != null && CFG.CorrectWithTranslation)
				{
					var nVn = needed_abs > 0? needed_vector/needed_abs : Vector3d.zero;
					var cV_lat = Vector3.ProjectOnPlane(CourseCorrection, needed_vector);
					if(needed_abs < HSC.TranslationMaxDeltaV || 
					   Mathf.Abs((float)Vector3d.Dot(horizontal_speed_dir, nVn)) < HSC.TranslationMaxCos)
						TRA.AddDeltaV(error_vector_local);
					else if(cV_lat.magnitude > HSC.TranslationMinDeltaV)
						TRA.AddDeltaV(-VSL.LocalDir(cV_lat));
				}

                //sanity check
				if(thrust.IsZero()) 
                    thrust = VSL.Engines.CurrentDefThrustDir;
                //no need to avoid static obstacles if we're stopped
				if(CFG.HF[HFlight.Stop]) 
                    VSL.Altitude.DontCorrectIfSlow();
			}
			needed_thrust_dir.Normalize();
			//tune filter
            output_filter.Tau = VSL.Torque.Slow ? 
				HSC.LowPassF / (1 + VSL.Torque.EnginesResponseTimeM * HSC.SlowTorqueF) : 
				HSC.LowPassF;
			ATC.SetCustomRotationW(thrust, output_filter.Update(needed_thrust_dir).normalized);

			#if DEBUG
//			LogF("\nthrust {}\nneeded {}\nfilterred {}\nAttitudeError {}", 
//			     thrust, needed_thrust_dir, filter.Value.normalized, VSL.Controls.AttitudeError);//debug
//			CSV(VSL.Physics.UT, 
//			    filter.Value.x, filter.Value.y, filter.Value.z,
//			    thrust.x, thrust.y, thrust.z);//debug
			#endif
		}
	}
}

