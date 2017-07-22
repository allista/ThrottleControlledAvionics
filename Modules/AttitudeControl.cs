//  Author:
//       Allis Tauri <allista@gmail.com>
//
//  Copyright (c) 2015 Allis Tauri
//
// This work is licensed under the Creative Commons Attribution-ShareAlike 4.0 International License. 
// To view a copy of this license, visit http://creativecommons.org/licenses/by-sa/4.0/ 
// or send a letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.

using System;
using UnityEngine;
using AT_Utils;

namespace ThrottleControlledAvionics
{
	public abstract class AttitudeControlBase : ThrustDirectionControl
	{
		public new class Config : ModuleConfig
		{
			[Persistent] public PIDv_Controller2 PID = new PIDv_Controller2(
				Vector3.one*10f, Vector3.one*0.02f, Vector3.one*0.5f, -Vector3.one, Vector3.one
			);

            [Persistent] public PIDf_Controller2 AT_PID = new PIDf_Controller2(
                1, 0, 1, -10*Mathf.PI, 10*Mathf.PI
            );
            [Persistent] public PIDv_Controller AV_PID = new PIDv_Controller(
                Vector3.one*10f, Vector3.zero, Vector3.one*0.5f, -Vector3.one, Vector3.one
            );

            //new 2xPID
            public class PIDCascadeConfig : ConfigNodeObject
            {
                [Persistent] public float atP_Scale = 0.7f;
                [Persistent] public float atP_Min = 0.7f;
                [Persistent] public float atP_Max = 0.7f;
                [Persistent] public float atP_ErrThreshold = 0.7f;
                [Persistent] public float atP_ErrScale = 4;

                [Persistent] public int   atD_Curve = 3;
                [Persistent] public float atD_Scale = 35f;
                [Persistent] public float atD_Min = 2f;

                [Persistent] public float atI_Scale = 0.01f;
                [Persistent] public float atI_AV_Scale = 10;
                [Persistent] public float atI_ErrThreshold = 0.9f;
                [Persistent] public float atI_ErrCurve = 2;

                [Persistent] public float avP_Scale = 0.4f;
                [Persistent] public float avP_Min   = 0.4f;
                [Persistent] public float avI_Scale = 0.4f;

                [Persistent] public float AxisCorrection = 2f;
            }

            public class CascadeConfigFast : PIDCascadeConfig
            {
                [Persistent] public float atP_ErrCurve = 0.2f;
            }

            public class CascadeConfigSlow : PIDCascadeConfig
            {
                [Persistent] public float SlowTorqueF = 0.05f;
                [Persistent] public float MaxSlowF = 3f;
            }

            [Persistent] public CascadeConfigFast FastConfig = new CascadeConfigFast();
            [Persistent] public CascadeConfigSlow SlowConfig = new CascadeConfigSlow();

            //old PID
			[Persistent] public float MinAAf  = 0.1f, MaxAAf  = 1f;
			[Persistent] public float MaxAA   = 0.9f;
            [Persistent] public float MaxPD   = 0.9f;

			[Persistent] public float InertiaFactor = 10f, AngularMf = 0.002f;
			[Persistent] public float MoIFactor              = 0.01f;
			[Persistent] public float MinEf = 0.001f, MaxEf  = 5f;
			[Persistent] public float AALowPassF             = 1f;

            //attitude error and other stats
			[Persistent] public float AngleThreshold         = 60f;
			[Persistent] public float MaxAttitudeError       = 10f;  //deg
			[Persistent] public float AttitudeErrorThreshold = 3f;   //deg
			[Persistent] public float MaxTimeToAlignment     = 15f;  //s
			[Persistent] public float DragResistanceF        = 10f;
		}
		protected static Config ATCB { get { return Globals.Instance.ATCB; } }

		public struct Rotation 
		{ 
			public Vector3 current, needed; 
			public Rotation(Vector3 current, Vector3 needed)
			{ this.current = current; this.needed = needed; }
			public static Rotation Local(Vector3 current, Vector3 needed, VesselWrapper VSL)
			{ return new Rotation(VSL.LocalDir(current), VSL.LocalDir(needed)); }

			public override string ToString()
			{ return Utils.Format("[Rotation]: current {}, needed {}", current, needed); }
		}

		protected AttitudeControlBase(ModuleTCA tca) : base(tca) {}

		protected Vector3 steering;
		protected Vector3 angle_error;
        protected Vector3 rotation_axis;
		protected readonly PIDv_Controller2 steering_pid = new PIDv_Controller2();
        protected readonly PIDf_Controller2 at_pid = new PIDf_Controller2();
        protected readonly PIDv_Controller av_pid = new PIDv_Controller();
		protected readonly LowPassFilterV AAf_filter = new LowPassFilterV();
		protected readonly Timer AuthorityTimer = new Timer();
		protected readonly DifferentialF ErrorDif = new DifferentialF();

        protected Vector3 Torque
        { get { return VSL.Engines.Slow? VSL.Torque.MaxPossible.Torque*VSL.vessel.ctrlState.mainThrottle : VSL.Torque.MaxCurrent.Torque; } }

		protected Vector3 AA 
		{ 
            get 
            { 
                return VSL.Engines.Slow? 
                    VSL.Torque.MaxPossible.AA*Mathf.Min(VSL.vessel.ctrlState.mainThrottle, VSL.OnPlanetParams.GeeVSF) : 
                    VSL.Torque.MaxCurrent.AA; 
            } 
        }

		protected Vector3 CurrentAAf
		{ get { return AA.Inverse().ClampComponents(ATCB.MinAAf, ATCB.MaxAAf); } }

		public override void Init() 
		{ 
			base.Init();
			steering_pid.setPID(ATCB.PID);
            at_pid.setPID(ATCB.AT_PID);
            av_pid.setPID(ATCB.AV_PID);
			reset();
		}

		protected override void reset()
		{
			base.reset();
			steering_pid.Reset();
            at_pid.Reset();
            av_pid.Reset();
			AAf_filter.Reset();
			AAf_filter.Set(CurrentAAf);
			AAf_filter.Tau = 0;
			VSL.Controls.HaveControlAuthority = true;
			VSL.Controls.SetAttitudeError(180);
            rotation_axis = Vector3.zero;
		}

		protected static Vector3 rotation2steering(Quaternion rotation)
		{
			var euler = rotation.eulerAngles;
			return new Vector3(Utils.CenterAngle(euler.x)*Mathf.Deg2Rad,
			                   Utils.CenterAngle(euler.y)*Mathf.Deg2Rad,
			                   Utils.CenterAngle(euler.z)*Mathf.Deg2Rad);
		}

		protected Vector3 H(Vector3 wDir) { return Vector3.ProjectOnPlane(wDir, VSL.Physics.Up).normalized; }

		protected Quaternion world2local_rotation(Quaternion world_rotation)
		{ return VSL.refT.rotation.Inverse() * world_rotation * VSL.refT.rotation; }

		protected void update_angular_error(Quaternion direct_rotation)
		{
			angle_error = direct_rotation.eulerAngles;
			angle_error = new Vector3(
				Mathf.Abs(Utils.CenterAngle(angle_error.x)/180),
				Mathf.Abs(Utils.CenterAngle(angle_error.y)/180),
				Mathf.Abs(Utils.CenterAngle(angle_error.z)/180));	
		}

        protected Vector3 MaxComponentV(Vector3 v, float threshold)
        {
            threshold += 1;
            int maxI = 0;
            float maxC = Math.Abs(v.x);
            for(int i = 1; i < 3; i++)
            {
                var c = Math.Abs(v[i]);
                if(maxC <= 0 || c/maxC > threshold)
                {
                    maxC = c;
                    maxI = i;
                }
            }
            var ret = new Vector3();
            ret[maxI] = maxC;
            return ret;
        }

		protected void compute_steering(Vector3 current, Vector3 needed)
		{
			var cur_inv = current.IsInvalid() || current.IsZero();
			var ned_inv = needed.IsInvalid() || needed.IsZero();
			if(cur_inv || ned_inv)
			{
				Log("compute_steering: Invalid argumetns:\ncurrent {}\nneeded {}\ncurrent thrust {}", 
				    current, needed, VSL.Engines.CurrentDefThrustDir);
				steering = Vector3.zero;
				return;
			}
            needed.Normalize();
            current.Normalize();
            var current_maxI = current.MaxI();
			var direct_rotation = Quaternion.FromToRotation(needed, current);
			update_angular_error(direct_rotation);
			VSL.Controls.SetAttitudeError(Vector3.Angle(needed, current));
            rotation_axis = VSL.Controls.AttitudeError < 175f? 
                Vector3.Cross(current, needed) : 
                -MaxComponentV(VSL.Torque.MaxCurrent.AA.Exclude(current_maxI), 0.01f);
            rotation_axis.Normalize();
			//deprecated: calculate steering
			if(VSL.Controls.AttitudeError > ATCB.AngleThreshold)
			{
				//rotational axis
                var axis = -rotation_axis.Exclude(current_maxI);
				//main rotation component
				var axis1 = axis.MaxComponentV();
				var current_cmp1 = Vector3.ProjectOnPlane(current, axis1);
				var needed_cmp1 = Vector3.ProjectOnPlane(needed, axis1);
				var angle1 = Vector3.Angle(needed_cmp1, current_cmp1);
				//second rotation component
				var axis2 = (axis - axis1).MaxComponentV();
				var angle2 = Vector3.Angle(needed, needed_cmp1);
				//steering
				steering = (axis1.normalized * angle1 + 
				            axis2.normalized * angle2) * Mathf.Deg2Rad;
			}
			else steering = rotation2steering(direct_rotation);
			//FIXME: sometimes generates NaN
//			needed (2.309423E+09, -5.479368E+11, -2.858228E+11); |v| = 6.180087E+11
//			current (-0.0680542, -28.58647, -718.0868); |v| = 718.6556
//			angle 60.17245 > threshold
//			steering [pitch NaN, roll NaN, yaw NaN]
		}

		protected void compute_steering(Quaternion rotation)
		{ compute_steering(Vector3.up, rotation*Vector3.up); }

		protected void compute_steering(Rotation rotation)
		{ compute_steering(rotation.current, rotation.needed); }

		protected virtual void correct_steering() {}

        protected void tune_steering2()
        {
            VSL.Controls.GimbalLimit = VSL.vessel.ctrlState.mainThrottle.Equals(0)? 0 : VSL.OnPlanetParams.TWRf*100;
            var AV = CFG.BR? 
                Vector3.ProjectOnPlane(VSL.vessel.angularVelocity, VSL.LocalDir(VSL.Engines.refT_thrust_axis)) : 
                VSL.vessel.angularVelocity;
            var AVm = Vector3.Dot(AV, rotation_axis);
            var err = VSL.Controls.AttitudeError/180;
            var iErrf = 1-err;
            var maxAA = AA;
            var maxAAf = Mathf.Abs(Vector3.Dot(maxAA, rotation_axis.AbsComponents()));
            var imaxAA = maxAA.Inverse(0);
            var AM = Vector3.Scale(AV, VSL.Physics.MoI);
            if(VSL.Engines.Slow)
                tune_steering_slow(err, iErrf, AV, AVm, maxAAf, imaxAA, AM);
            else 
                tune_steering_fast(err, iErrf, AV, AVm, maxAAf, imaxAA, AM);
            correct_steering();
            TCAGui.DebugMessage += Utils.Format("\nMoI: {}\nAA: {}\nsteering: {}", VSL.Physics.MoI, maxAA, steering);//debug
        }

        void tune_pids_set_steering(Config.PIDCascadeConfig cfg,
                                    float err, float atI_iErrf, 
                                    float atP, float atD, 
                                    Vector3 avP, Vector3 avD,
                                    Vector3 AV, float AVm, float maxAAf)
        {
            if(atI_iErrf <= 0 || AVm < 0)
            {
                at_pid.I = 0;
                at_pid.IntegralError = 0;
            }
            else 
            {
                atI_iErrf = Mathf.Pow(atI_iErrf, cfg.atI_ErrCurve);
                at_pid.I = cfg.atI_Scale * maxAAf * atI_iErrf / (1+Utils.ClampL(AVm, 0)*cfg.atI_AV_Scale*atI_iErrf);
            }
            at_pid.P = atP;
            at_pid.D = atD;
            av_pid.P = avP;
            av_pid.I = cfg.avI_Scale * avP;
            av_pid.D = avD;
            TCAGui.DebugMessage = Utils.Format("atPID: {}\navPID: {}", at_pid, av_pid);//debug
            at_pid.Update(err*Mathf.PI, -AVm);
            var avErr = AV-rotation_axis*at_pid.Action + 
                Vector3.ProjectOnPlane(AV, rotation_axis).ScaleChain(avP)*cfg.AxisCorrection;
            av_pid.Update(avErr);
            var maxPRY = Mathf.Abs(av_pid.Action.MaxComponentF());
            steering = maxPRY > 1? av_pid.Action/maxPRY : av_pid.Action;
        }

        void tune_steering_fast(float err, float iErrf, 
                                Vector3 AV, float AVm, 
                                float maxAAf, Vector3 imaxAA, 
                                Vector3 AM)
        {
            var avP = (ATCB.FastConfig.avP_Scale*imaxAA).ClampComponentsL(ATCB.FastConfig.avP_Min);
            var atP = Utils.Clamp(ATCB.FastConfig.atP_Scale * imaxAA.magnitude *
                                  (1 + Utils.ClampL(iErrf-ATCB.FastConfig.atP_ErrThreshold, 0) * ATCB.FastConfig.atP_ErrScale),
                                  ATCB.FastConfig.atP_Min, Utils.ClampL(ATCB.FastConfig.atP_Max*maxAAf, ATCB.FastConfig.atP_Min));
            var atI_iErrf = Utils.ClampL(iErrf-ATCB.FastConfig.atI_ErrThreshold, 0);
            var atD = Utils.ClampL(atP - Mathf.Pow(maxAAf-1, ATCB.FastConfig.atD_Curve)*ATCB.FastConfig.atD_Scale, ATCB.FastConfig.atD_Min);
            atD *= Utils.ClampH(iErrf+
                                (1-Utils.ClampH(maxAAf, 1))+
                                Mathf.Abs(Vector3.Dot(AM, rotation_axis)), 1);
            atD /= atP;
            atP = Utils.ClampH(200*maxAAf*Utils.ClampL(iErrf, 0.1f), 1+Mathf.Pow(atI_iErrf, ATCB.FastConfig.atP_ErrCurve));
            tune_pids_set_steering(ATCB.FastConfig, err, atI_iErrf, atP, atD, avP, Vector3.zero, AV, AVm, maxAAf);
        }

        void tune_steering_slow(float err, float iErrf, 
                                Vector3 AV, float AVm, 
                                float maxAAf, Vector3 imaxAA, 
                                Vector3 AM)
        {
            var slow = (Vector3.one+Vector3.Scale(VSL.Torque.EnginesResponseTime, 
                                                  VSL.Torque.Engines.SpecificTorque)*ATCB.SlowConfig.SlowTorqueF)
                .ClampComponentsH(ATCB.SlowConfig.MaxSlowF);
            //compute coefficients of angular velocity PID
//            var avP = (ATCB.SlowConfig.avP_Scale*imaxAA).ClampComponentsL(ATCB.SlowConfig.avP_Min);
            var avD = Vector3.Scale(slow,slow);
            var avP = Vector3.one;
            //compute coefficients of attitude PID
            var atP = Utils.Clamp(ATCB.SlowConfig.atP_Scale * imaxAA.magnitude *
                                  (1 + Utils.ClampL(iErrf-ATCB.SlowConfig.atP_ErrThreshold, 0) * ATCB.SlowConfig.atP_ErrScale),
                                  ATCB.SlowConfig.atP_Min, ATCB.SlowConfig.atP_Max);
            var atI_iErrf = Utils.ClampL(iErrf-ATCB.SlowConfig.atI_ErrThreshold, 0);
            var atD = Utils.ClampL(atP - Mathf.Pow(maxAAf-1, ATCB.SlowConfig.atD_Curve)*ATCB.SlowConfig.atD_Scale, ATCB.SlowConfig.atD_Min);
            atD *= Utils.ClampH(iErrf+
                                (1-Utils.ClampH(maxAAf, 1))+
                                Mathf.Abs(Vector3.Dot(AM, rotation_axis)), 1);
            //tune PIDS and compute steering
            tune_pids_set_steering(ATCB.SlowConfig, err, atI_iErrf, atP, atD, avP, avD, AV, AVm, maxAAf);
            steering.Scale(slow.Inverse(0));
        }

		protected void tune_steering()
		{
            VSL.Controls.GimbalLimit = VSL.vessel.ctrlState.mainThrottle.Equals(0)? 0 : VSL.OnPlanetParams.TWRf*100;
			//calculate attitude error
			var Ef = Utils.Clamp(VSL.Controls.AttitudeError/180, ATCB.MinEf, 1);
//			var ini_steering = steering;//debug
			//tune lowpass filter
			AAf_filter.Tau = (1-Mathf.Sqrt(Ef))*ATCB.AALowPassF;
			//tune PID parameters
			var angularV = VSL.vessel.angularVelocity;
			var angularM = Vector3.Scale(angularV, VSL.Physics.MoI);
			var slow = VSL.Engines.Slow? 
				(Vector3.one+Vector3.Scale(VSL.Torque.EnginesResponseTime, 
                                           VSL.Torque.Engines.SpecificTorque)*ATCB.SlowConfig.SlowTorqueF)
                .ClampComponentsH(ATCB.SlowConfig.MaxSlowF) : Vector3.one;
			var slowi = slow.Inverse();
            var iErr = (Vector3.one-angle_error);
            var AAf = AAf_filter.Update(CurrentAAf);
            var PIf = AAf.ScaleChain(iErr.ClampComponentsL(1/ATCB.MaxEf)*ATCB.MaxEf, slowi);
			var AA_clamped = AA.ClampComponentsH(ATCB.MaxAA);
			steering_pid.P = Vector3.Scale(ATCB.PID.P, PIf);
			steering_pid.I = Vector3.Scale(ATCB.PID.I, PIf);
			steering_pid.D = ATCB.PID.D.ScaleChain((iErr
			                                        + (Vector3.one-AA_clamped/ATCB.MaxAA)
			                                        + angularM.AbsComponents()*ATCB.AngularMf
                                                   ).ClampComponentsH(1),
			                                       AAf, slow,slow).ClampComponentsL(0);
			//add inertia to handle constantly changing needed direction
			var inertia = angularM.Sign()
                .ScaleChain(iErr,
                            angularM, angularM, 
                            Vector3.Scale(VSL.Torque.MaxCurrent.Torque, VSL.Physics.MoI).Inverse(0))
				.ClampComponents(-Mathf.PI, Mathf.PI)
				/Mathf.Lerp(ATCB.InertiaFactor, 1, VSL.Physics.MoI.magnitude*ATCB.MoIFactor);
			steering += inertia;
			//update PID
			steering_pid.Update(steering, angularV);
			steering = Vector3.Scale(steering_pid.Action, slowi);
			//postprocessing by derived classes
			correct_steering();
            //debug: CSV Attitude Control
//            CSV(VSL.Altitude.Absolute, 
//                ini_steering*Mathf.Rad2Deg, 
//                steering, 
//                angularV, 
//                angularM,
//                inertia, 
//                Vector3.Scale(steering_pid.Action, slowi), 
//                steering_pid.P, steering_pid.I, steering_pid.D, 
//                AA, PIf, AAf, slow);
		}

		protected void set_authority_flag()
		{
			ErrorDif.Update(VSL.Controls.AttitudeError);
			if(ErrorDif.MaxOrder < 1) return;
			var max_alignment_time = VSL.Info.Countdown > 0? VSL.Info.Countdown : ATCB.MaxTimeToAlignment;
			var omega = Mathf.Abs(ErrorDif[1]/TimeWarp.fixedDeltaTime);
			var turn_time = VSL.Controls.MinAlignmentTime-omega/VSL.Torque.MaxCurrent.AA_rad/Mathf.Rad2Deg;
			if(VSL.Controls.HaveControlAuthority && 
			   VSL.Controls.AttitudeError > ATCB.MaxAttitudeError && 
			   (ErrorDif[1] >= 0 || turn_time > max_alignment_time))
				VSL.Controls.HaveControlAuthority = !AuthorityTimer.TimePassed;
			else if(!VSL.Controls.HaveControlAuthority && 
			        (VSL.Controls.AttitudeError < ATCB.AttitudeErrorThreshold || 
			         VSL.Controls.AttitudeError < ATCB.MaxAttitudeError*2 && ErrorDif[1] < 0 && 
			         turn_time < max_alignment_time))
				VSL.Controls.HaveControlAuthority = AuthorityTimer.TimePassed;
			else AuthorityTimer.Reset();
		}

        #if DEBUG
        public static bool UseOldPID;
        #endif
	}

	[CareerPart]
	[RequireModules(typeof(SASBlocker))]
	[OptionalModules(typeof(TimeWarpControl))]
	public class AttitudeControl : AttitudeControlBase
	{
		public new class Config : ModuleConfig
		{
			[Persistent] public float KillRotThreshold = 1e-5f;
		}
		static Config ATC { get { return Globals.Instance.ATC; } }

		readonly MinimumF momentum_min = new MinimumF();
		Transform refT;
		Quaternion locked_attitude;
		bool attitude_locked;

		BearingControl BRC;
		Vector3 lthrust, needed_lthrust;

		public AttitudeControl(ModuleTCA tca) : base(tca) {}

		public override void Init() 
		{ 
			base.Init();
			CFG.AT.SetSingleCallback(Enable);
		}

		protected override void UpdateState() 
		{ 
			base.UpdateState();
			IsActive &= CFG.AT; 
		}

		public void Enable(Multiplexer.Command cmd)
		{
			reset();
			switch(cmd)
			{
			case Multiplexer.Command.Resume:
				RegisterTo<SASBlocker>();
				break;

			case Multiplexer.Command.On:
				VSL.UpdateOnPlanetStats();
				goto case Multiplexer.Command.Resume;

			case Multiplexer.Command.Off:
				UnregisterFrom<SASBlocker>();
				break;
			}
		}

		public Rotation CustomRotation { get; private set; }

		public void SetCustomRotation(Vector3 current, Vector3 needed)
		{ CustomRotation = new Rotation(current, needed); }

		public void SetCustomRotationW(Vector3 current, Vector3 needed)
		{ CustomRotation = Rotation.Local(current, needed, VSL); }

		public void SetThrustDirW(Vector3 needed)
		{ CustomRotation = Rotation.Local(VSL.Engines.CurrentDefThrustDir, needed, VSL); }

		public void ResetCustomRotation() { CustomRotation = default(Rotation); }

		protected override void reset()
		{
			base.reset();
			refT = null;
			momentum_min.Reset();
			attitude_locked = false;
			needed_lthrust = Vector3.zero;
			lthrust = Vector3.zero;
		}

		public void UpdateCues()
		{
			switch(CFG.AT.state)
			{
			case Attitude.Normal:
				needed_lthrust = -VSL.LocalDir(VSL.orbit.h.xzy);
				break;
			case Attitude.AntiNormal:
				needed_lthrust = VSL.LocalDir(VSL.orbit.h.xzy);
				break;
			case Attitude.Radial:
				needed_lthrust = VSL.LocalDir(Vector3d.Cross(VSL.vessel.obt_velocity.normalized, VSL.orbit.h.xzy.normalized));
				break;
			case Attitude.AntiRadial:
				needed_lthrust = -VSL.LocalDir(Vector3d.Cross(VSL.vessel.obt_velocity.normalized, VSL.orbit.h.xzy.normalized));
				break;
			case Attitude.Target:
			case Attitude.AntiTarget:
			case Attitude.TargetCorrected:
				if(!VSL.HasTarget) 
				{ 
					Message("No target");
					CFG.AT.On(Attitude.KillRotation);
					break;
				}
				var dpos = VSL.vessel.transform.position-VSL.Target.GetTransform().position;
				if(CFG.AT.state == Attitude.TargetCorrected)
				{
					var dvel = VSL.vessel.GetObtVelocity()-VSL.Target.GetObtVelocity();
					needed_lthrust = VSL.LocalDir((dpos.normalized+Vector3.ProjectOnPlane(dvel, dpos).ClampMagnitudeH(1)).normalized);
				}
				else
				{
					needed_lthrust = VSL.LocalDir(dpos.normalized);
					if(CFG.AT.state == Attitude.AntiTarget) needed_lthrust *= -1;
				}
				break;
			}
		}

		void compute_steering()
		{
			Vector3 v;
			momentum_min.Update(VSL.vessel.angularMomentum.sqrMagnitude);
			lthrust = VSL.LocalDir(VSL.Engines.CurrentDefThrustDir);
			steering = Vector3.zero;
			switch(CFG.AT.state)
			{
			case Attitude.Custom:
				if(CustomRotation.Equals(default(Rotation)))
					goto case Attitude.KillRotation;
				lthrust = CustomRotation.current;
				needed_lthrust = CustomRotation.needed;
				break;
			case Attitude.HoldAttitude:
				if(refT != VSL.refT || !attitude_locked)
				{
					refT = VSL.refT;
					locked_attitude = refT.rotation;
					attitude_locked = true;
				}
				if(refT != null)
				{
					lthrust = Vector3.up;
					needed_lthrust = refT.rotation.Inverse()*locked_attitude*lthrust;
				}
				break;
			case Attitude.KillRotation:
				if(refT != VSL.refT || momentum_min.True)
				{
					refT = VSL.refT;
					locked_attitude = refT.rotation;
				}
				if(refT != null)
				{
					lthrust = Vector3.up;
					needed_lthrust = refT.rotation.Inverse()*locked_attitude*lthrust;
				}
				break;
			case Attitude.Prograde:
			case Attitude.Retrograde:
				v = VSL.InOrbit? VSL.vessel.obt_velocity : VSL.vessel.srf_velocity;
				if(v.magnitude < GLB.THR.MinDeltaV) { CFG.AT.On(Attitude.KillRotation); break; }
				if(CFG.AT.state == Attitude.Prograde) v *= -1;
				needed_lthrust = VSL.LocalDir(v.normalized);
				VSL.Engines.RequestNearestClusterActivation(needed_lthrust);
				break;
			case Attitude.RelVel:
			case Attitude.AntiRelVel:
				if(!VSL.HasTarget) 
				{ 
					Message("No target");
					CFG.AT.On(Attitude.KillRotation);
					break;
				}
				v = VSL.InOrbit? 
					VSL.Target.GetObtVelocity()-VSL.vessel.obt_velocity : 
					VSL.Target.GetSrfVelocity()-VSL.vessel.srf_velocity;
				if(v.magnitude < GLB.THR.MinDeltaV) { CFG.AT.On(Attitude.KillRotation); break; }
				if(CFG.AT.state == Attitude.AntiRelVel) v *= -1;
				needed_lthrust = VSL.LocalDir(v.normalized);
				VSL.Engines.RequestClusterActivationForManeuver(v);
				break;
			case Attitude.ManeuverNode:
				var solver = VSL.vessel.patchedConicSolver;
				if(solver == null || solver.maneuverNodes.Count == 0)
				{ 
					Message("No maneuver node");
					CFG.AT.On(Attitude.KillRotation); 
					break; 
				}
				v = -solver.maneuverNodes[0].GetBurnVector(VSL.orbit);
				needed_lthrust = VSL.LocalDir(v.normalized);
				VSL.Engines.RequestClusterActivationForManeuver(v);
				break;
			case Attitude.Normal:
			case Attitude.AntiNormal:
			case Attitude.Radial:
			case Attitude.AntiRadial:
			case Attitude.Target:
			case Attitude.AntiTarget:
				VSL.Engines.RequestNearestClusterActivation(needed_lthrust);
				break;
			}
            #if DEBUG
            if(FollowMouse)
                needed_lthrust = VSL.LocalDir(FlightCamera.fetch.mainCamera.ScreenPointToRay(Input.mousePosition).direction);
            #endif
			compute_steering(lthrust.normalized, needed_lthrust.normalized);
			ResetCustomRotation();
		}

		protected override void correct_steering()
		{
			if(BRC != null && BRC.IsActive)
				steering = Vector3.ProjectOnPlane(steering, lthrust);
		}


		protected override void OnAutopilotUpdate(FlightCtrlState s)
		{
			//need to check all the prerequisites, because the callback is called asynchroniously
			if(!(CFG.Enabled && CFG.AT && VSL.refT != null && VSL.orbit != null)) return;
			if(VSL.AutopilotDisabled) { reset(); return; }
			compute_steering();
            if(UseOldPID) tune_steering();
            else tune_steering2();
			set_authority_flag();
			VSL.Controls.AddSteering(steering);
		}

		#if DEBUG
        public bool FollowMouse;

//        ThrottleControl THR;
//
//        OscillationDetector3D OD = new OscillationDetector3D(3, 30, 100, 200, 1);
//        Timer test_timer = new Timer(1);
//        static readonly Vector3[] axes = {Vector3.up, Vector3.right};
//        Vector3 needed_thrust;
//        int direction = 1, axis = 0;
//        float throttle = 0.1f, dThrottle = 0.1f;
//        float curAAf = 0.1f, dAAf = 0.05f;
//        enum TestStage {START, TESTING, KILL_ROT, NEXT_AAf, NEXT_THROTTLE, NEXT_AXIS, DONE};
//        TestStage stage = TestStage.START;
//
//        void tune_steering_test(Vector3 AAf)
//        {
//            VSL.Controls.GimbalLimit = 0;//VSL.OnPlanetParams.TWRf*100;
//            //tune PID parameters
//            var angularV = VSL.vessel.angularVelocity;
//            var angularM = Vector3.Scale(angularV, VSL.Physics.MoI);
//            var slow = VSL.Engines.Slow? 
//                (Vector3.one+Vector3.Scale(VSL.Torque.EnginesResponseTime, 
//                                           VSL.Torque.Engines.SpecificTorque)*ATCB.SlowTorqueF)
//                .ClampComponentsH(ATCB.MaxSlowF) : Vector3.one;
//            var slowi = slow.Inverse();
//            var iErr = (Vector3.one-angle_error);
//            var PIf = AAf.ScaleChain(iErr.ClampComponentsL(1/ATCB.MaxEf)*ATCB.MaxEf, slowi);
////            var AA_clamped = AA.ClampComponentsH(ATCB.MaxAA);
//            steering_pid.P = Vector3.Scale(ATCB.PID.P, PIf);
//            steering_pid.I = Vector3.Scale(ATCB.PID.I, PIf);
//            steering_pid.D = ATCB.PID.D.ScaleChain((iErr
////                                                    + (Vector3.one-AA_clamped/ATCB.MaxAA)
//                                                    + angularM.AbsComponents()*ATCB.AngularMf
//                                                   ).ClampComponentsH(1),
//
//                                                   AAf, slow,slow).ClampComponentsL(0);
//            //add inertia to handle constantly changing needed direction
//            var inertia = angularM.Sign()
//                .ScaleChain(iErr, 
//                            angularM, angularM, 
//                            Vector3.Scale(VSL.Torque.MaxCurrent.Torque, VSL.Physics.MoI).Inverse(0))
//                .ClampComponents(-Mathf.PI, Mathf.PI)
//                /Mathf.Lerp(ATCB.InertiaFactor, 1, VSL.Physics.MoI.magnitude*ATCB.MoIFactor);
//            steering += inertia;
//            //update PID
//            steering_pid.Update(steering, angularV);
//            steering = Vector3.Scale(steering_pid.Action, slowi);
//            //postprocessing by derived classes
//            correct_steering();
//        }
//
//
//        protected override void OnAutopilotUpdate(FlightCtrlState s)
//        {
//            if(!(CFG.Enabled && stage != TestStage.DONE && VSL.refT != null && VSL.orbit != null)) return;
//            if(VSL.AutopilotDisabled) { reset(); return; }
//            lthrust = VSL.LocalDir(VSL.Engines.CurrentDefThrustDir).normalized;
//            if(VSL.IsActiveVessel)
//                TCAGui.DebugMessage = Utils.Format("stage: {}, axis: {}\ntimer: {}\ncur AAf {}, throttle {}\n", 
//                                                   stage, axis, test_timer, curAAf, throttle);
//            switch(stage)
//            {
//            case TestStage.START:
//                CheatOptions.InfinitePropellant = true;
//                CheatOptions.InfiniteElectricity = true;
//                CheatOptions.IgnoreMaxTemperature = true;
//                VSL.vessel.ActionGroups.SetGroup(KSPActionGroup.RCS, true);
//                Debug.ClearDeveloperConsole();
//                needed_thrust = VSL.WorldDir(Quaternion.AngleAxis(120*direction, axes[axis])*lthrust);
//                OD.Reset();
//                test_timer.Reset();
//                direction = -direction;
//                stage = TestStage.TESTING;
//                break;
//            case TestStage.KILL_ROT:
//                THR.Throttle = 0;
//                CFG.AT.Off();
//                VSL.vessel.ActionGroups.SetGroup(KSPActionGroup.SAS, true);
//                if(VSL.vessel.angularVelocity.sqrMagnitude > 1e-4) break;
//                VSL.vessel.ActionGroups.SetGroup(KSPActionGroup.SAS, false);
//                stage = TestStage.START;
//                break;
//            case TestStage.TESTING:
//                CFG.AT.OnIfNot(Attitude.Custom);
//                THR.Throttle = throttle;
//                needed_lthrust = VSL.LocalDir(needed_thrust);
//                compute_steering(lthrust, needed_lthrust);
//                tune_steering_test(Vector3.one*curAAf);
//                VSL.Controls.AddSteering(steering);
//                //detect oscillations
//                OD.Update(steering, TimeWarp.fixedDeltaTime);
//                if(VSL.IsActiveVessel)
//                    TCAGui.DebugMessage += 
//                        string.Format("pid: {0}\nsteering: {1}%\ngimbal limit: {2}\nOD: {3}",
//                                      steering_pid, steering_pid.Action*100, VSL.Controls.GimbalLimit, OD.Value);
//                
//                if(OD.Value.x > 0.1 ||
//                   OD.Value.y > 0.1 ||
//                   OD.Value.z > 0.1)
//                    stage = TestStage.NEXT_THROTTLE;
//                else if(VSL.Controls.AttitudeError < 1)
//                    stage = TestStage.NEXT_AAf;
//                break;
//            case TestStage.NEXT_AAf:
//                curAAf += dAAf;
//                stage = curAAf > 5+dAAf/2 ? TestStage.NEXT_THROTTLE : TestStage.KILL_ROT;
//                break;
//            case TestStage.NEXT_THROTTLE:
//                CSV(axes[axis] == Vector3.up? 0 : 1, throttle, AA, OD.Value, steering_pid.D, curAAf-dAAf);
//                curAAf = 0.1f;
//                throttle += dThrottle;
//                if(throttle > 1 && throttle < 1+dThrottle/2)
//                    throttle = 1;
//                stage = throttle > 1 ? TestStage.NEXT_AXIS : TestStage.KILL_ROT;
//                break;
//            case TestStage.NEXT_AXIS:
//                throttle = 0.1f;
//                axis += 1;
//                stage = axis < axes.Length ? TestStage.KILL_ROT : TestStage.DONE;
//                break;
//            }
//        }


		public void DrawDebugLines()
		{
			if(!CFG.AT || VSL == null || VSL.vessel == null || VSL.refT == null) return;
//			Utils.GLVec(VSL.refT.position, VSL.OnPlanetParams.Heading.normalized*2500, Color.white);
			Utils.GLVec(VSL.refT.position, VSL.WorldDir(lthrust.normalized)*20, Color.yellow);
			Utils.GLVec(VSL.refT.position, VSL.WorldDir(needed_lthrust.normalized)*20, Color.red);
            Utils.GLVec(VSL.refT.position, VSL.WorldDir(VSL.vessel.angularVelocity*20), Color.cyan);
            Utils.GLVec(VSL.refT.position, VSL.WorldDir(rotation_axis*at_pid.Action*20), Color.green);
//			Utils.GLVec(VSL.refT.position, VSL.WorldDir(steering*20), Color.cyan);
//			Utils.GLVec(VSL.refT.position, VSL.WorldDir(steering_pid.Action*20), Color.magenta);

//			Utils.GLVec(VSL.refT.position, VSL.refT.right*2, Color.red);
//			Utils.GLVec(VSL.refT.position, VSL.refT.forward*2, Color.blue);
//			Utils.GLVec(VSL.refT.position, VSL.refT.up*2, Color.green);

//			if(VSL.Target != null)
//				Utils.GLDrawPoint(VSL.Target.GetTransform().position, Color.red, 5);
//
//			VSL.Engines.All.ForEach(e => 
//			{
//				Utils.GLVec(e.wThrustPos, e.wThrustDir*2, Color.red);
//				Utils.GLVec(e.wThrustPos, e.defThrustDir*2, Color.yellow);
//			});
		}
		#endif
	}
}
