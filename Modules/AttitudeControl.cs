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
			[Persistent] public PID_Controller PID = new PID_Controller(10f, 0.02f, 0.5f, -1, 1);

			[Persistent] public float MinAAf = 0.1f, MaxAAf  = 1f;
			[Persistent] public float MaxAA   = 0.9f;

			[Persistent] public float InertiaFactor = 10f, AngularMf = 0.002f;
			[Persistent] public float MoIFactor              = 0.01f;
			[Persistent] public float MinEf = 0.001f, MaxEf  = 5f;
			[Persistent] public float SlowTorqueF            = 2;
			[Persistent] public float AALowPassF             = 1f;
			[Persistent] public float MinSteeringThreshold   = 0.1f;

			[Persistent] public float AngleThreshold         = 60f;
			[Persistent] public float MaxAttitudeError       = 10f;  //deg
			[Persistent] public float AttitudeErrorThreshold = 3f;   //deg
			[Persistent] public float MaxTimeToAlignment     = 15f;  //s
			[Persistent] public float DragResistanceF        = 10f;


			[Persistent] public float OD_low                 = 2f;   //Hz
			[Persistent] public float OD_high                = 20f;  //Hz
			[Persistent] public int   OD_bins                = 50;
			[Persistent] public int   OD_window              = 250;  //samples
			[Persistent] public float OD_smoothing           = 0.1f; //s
			[Persistent] public float OD_Threshold           = 0.5f;
		}
		static Config ATCB { get { return Globals.Instance.ATCB; } }

		public struct Rotation 
		{ 
			public Vector3 current, needed; 
			public Rotation(Vector3 current, Vector3 needed)
			{ this.current = current; this.needed = needed; }
			public static Rotation Local(Vector3 current, Vector3 needed, VesselWrapper VSL)
			{ return new Rotation(VSL.LocalDir(current), VSL.LocalDir(needed)); }
		}

		protected AttitudeControlBase(ModuleTCA tca) : base(tca) {}

		protected Vector3 steering;
		protected OscillationDetectorD x_OD, y_OD, z_OD;
		protected readonly PIDv_Controller2 steering_pid = new PIDv_Controller2();
		protected readonly LowPassFilterF AAf_filter = new LowPassFilterF();
		protected readonly Timer AuthorityTimer = new Timer();
		protected Vector3 angularV;
		protected Vector3 angularM;
		protected float AAf, Ef, PIf;
		protected readonly DifferentialF ErrorDif = new DifferentialF();

		protected Vector3 fwd_axis
		{ get { return VSL.OnPlanetParams.NoseUp? VSL.Controls.Transform.forward : VSL.Controls.Transform.up; } }

		protected Vector3 up_axis
		{
			get 
			{
				var axis = VSL.OnPlanetParams.NoseUp? VSL.Controls.Transform.up : VSL.Controls.Transform.forward;
				axis *= -Mathf.Sign(Vector3.Dot(axis, VSL.Physics.Up));
				return axis;
			}
		}

		protected static OscillationDetectorD new_OD()
		{ 
			return new OscillationDetectorD(ATCB.OD_low, ATCB.OD_high, 
			                                ATCB.OD_bins, ATCB.OD_window, 
			                                ATCB.OD_smoothing, ATCB.OD_Threshold); 
		}

		public override void Init() 
		{ 
			base.Init();
			steering_pid.setPID(ATCB.PID);
			x_OD = new_OD();
			y_OD = new_OD();
			z_OD = new_OD();
			reset();
		}

		protected override void reset()
		{
			base.reset();
			steering_pid.Reset();
			AAf_filter.Reset();
			AAf_filter.Tau = 0;
			VSL.Controls.HaveControlAuthority = true;
			VSL.Controls.SetAttitudeError(180);
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

		protected void compute_steering(Vector3 current, Vector3 needed)
		{
			#if DEBUG
			if(current.IsZero() || needed.IsZero())
				LogFST("compute steering:\ncurrent {}\nneeded {}\ncurrent thrust {}", current, needed, VSL.Engines.CurrentThrustDir);
			#endif
			VSL.Controls.SetAttitudeError(Vector3.Angle(needed, current));
			if(VSL.Controls.AttitudeError > ATCB.AngleThreshold)
			{
				//rotational axis
				var current_maxI = current.MaxI();
				var axis = Vector3.Cross(needed, current).Exclude(current_maxI);
				if(axis.sqrMagnitude < 0.01f) 
					axis = VSL.Torque.MaxCurrent.AA.Exclude(current_maxI).MaxComponentV();
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
//				LogF("\ncurrent_maxI: {}\n" +
//				     "axis:  {}\n" +
//				     "axis1: {}\n" +
//				     "axis2: {}\n" +
//				     "current_cmp1: {}\n" +
//				     "needed_cmp1:  {}\n" +
//				     "angle1: {}\n" +
//				     "angle2: {}\n",
//				     current_maxI, axis, axis1, axis2, current_cmp1, needed_cmp1, angle1, angle2);//debug
			}
			else steering = rotation2steering(Quaternion.FromToRotation(needed, current));
//			LogF("\nneeded {}\ncurrent {}\nangle {}\nsteering {}",
//			     needed, current, Vector3.Angle(needed, current), DebugUtils.FormatSteering(steering));//debug

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

		protected void tune_steering()
		{
			VSL.Controls.GimbalLimit = 0;
			//calculate attitude error and Aligned state
			Ef = Utils.Clamp(VSL.Controls.AttitudeError/180, ATCB.MinEf, 1);
			//tune lowpass filter
			AAf_filter.Tau = (1-Mathf.Sqrt(Ef))*ATCB.AALowPassF;
			//tune PID parameters
			angularV = VSL.vessel.angularVelocity;
			angularM = Vector3.Scale(angularV, VSL.Physics.MoI);
			var minI = steering.MinI();
			var AA = VSL.Torque.MaxCurrent.AA.Exclude(minI);
			var AAm = Utils.ClampL(AA.MaxComponentF(), 1e-5f);
			var slow = VSL.Engines.SlowTorque? 1+VSL.Engines.TorqueResponseTime*ATCB.SlowTorqueF : 1;
			AAf = AAf_filter.Update(Mathf.Clamp(1/AAm, ATCB.MinAAf, ATCB.MaxAAf));
			AAm = Utils.ClampH(AAm, ATCB.MaxAA);
			PIf = AAf*Utils.ClampL(1-Ef, 1/ATCB.MaxEf)*ATCB.MaxEf/slow;
			steering_pid.P = ATCB.PID.P*PIf;
			steering_pid.I = ATCB.PID.I*PIf;
			steering_pid.D = ATCB.PID.D*Utils.ClampH(Utils.ClampH(2-Ef-AAm/ATCB.MaxAA, 1)+angularM.magnitude*ATCB.AngularMf, 1)*AAf*AAf*slow*slow;
//			Log("\nsteering: {}", steering);//debug

			//tune steering
			var norm = AA.Inverse(0).CubeNorm();
			norm[minI] = Utils.ClampH(Mathf.Abs(angularV[minI]*Mathf.Rad2Deg), 1);
			steering = Vector3.Scale(steering, norm);
//			Log("minI {}\nnorm: {}\nsteering*norm: {}", minI, norm, steering);//debug

			//add inertia to handle constantly changing needed direction
			var inertia = Vector3.Scale(angularM.Sign(),
			                            Vector3.Scale(Vector3.Scale(angularM, angularM),
			                                          Vector3.Scale(VSL.Torque.MaxCurrent.Torque, VSL.Physics.MoI).Inverse(0)))
				.ClampComponents(-Mathf.PI, Mathf.PI)/
				Mathf.Lerp(ATCB.InertiaFactor, 1, VSL.Physics.MoI.magnitude*ATCB.MoIFactor);
			steering = steering + inertia;
//			Log("\ninertia: {}\nsteering+inertia: {}", inertia, steering);//debug

			//update PID
			steering_pid.Update(steering, angularV);
			steering = steering_pid.Action;
			steering[minI] *= norm[minI];
//			Log("\npid.Act: {}", steering);//debug

			correct_steering();

			x_OD.Update(steering.x, TimeWarp.fixedDeltaTime);
			y_OD.Update(steering.y, TimeWarp.fixedDeltaTime);
			z_OD.Update(steering.z, TimeWarp.fixedDeltaTime);

//			CSV(steering.x, steering.y, steering.z, x_OD.Value, y_OD.Value, z_OD.Value);//debug

			steering.x *= 1-(float)x_OD.Value;
			steering.y *= 1-(float)y_OD.Value;
			steering.z *= 1-(float)z_OD.Value;

//			Log("\nx_OD:\n{}" +
//			    "\ny_OD:\n{}" +
//			    "\nz_OD:\n{}", 
//			    x_OD, y_OD, z_OD);
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
	}

	[CareerPart]
	[RequireModules(typeof(SASBlocker))]
	[OptionalModules(typeof(TimeWarpControl))]
	public class AttitudeControl : AttitudeControlBase
	{
		public new class Config : ModuleConfig
		{
			[Persistent] public float RollFilter = 1f;
		}
		static Config ATC { get { return Globals.Instance.ATC; } }

		readonly MinimumF omega_min = new MinimumF();
		Transform refT;
		Quaternion locked_attitude;
		bool attitude_locked;

		BearingControl BRC;
		Vector3 lthrust, needed_lthrust;
		readonly LowPassFilterV roll_filter = new LowPassFilterV();

		public AttitudeControl(ModuleTCA tca) : base(tca) {}

		public override void Init() 
		{ 
			base.Init();
			CFG.AT.SetSingleCallback(Enable);
			roll_filter.Tau = ATC.RollFilter;
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
				VSL.Controls.StopWarp();
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
		{ CustomRotation = Rotation.Local(VSL.Engines.CurrentThrustDir, needed, VSL); }

		public void ResetCustomRotation() { CustomRotation = default(Rotation); }

		protected override void reset()
		{
			base.reset();
			refT = null;
			omega_min.Reset();
			attitude_locked = false;
		}

		void compute_steering()
		{
			Vector3 v;
			omega_min.Update(VSL.vessel.angularVelocity.sqrMagnitude);
			lthrust = VSL.LocalDir(VSL.Engines.CurrentThrustDir);
			needed_lthrust = Vector3.zero;
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
				if(refT != VSL.refT || omega_min.True)
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
				break;
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
				if(VSL.Target == null) { CFG.AT.On(Attitude.KillRotation); break; }
				needed_lthrust = VSL.LocalDir((VSL.Physics.wCoM-VSL.Target.GetTransform().position).normalized);
				break;
			case Attitude.AntiTarget:
				if(VSL.Target == null) { CFG.AT.On(Attitude.KillRotation); break; }
				needed_lthrust = VSL.LocalDir((VSL.Target.GetTransform().position-VSL.Physics.wCoM).normalized);
				break;
			case Attitude.RelVel:
			case Attitude.AntiRelVel:
				if(!VSL.HasTarget) { CFG.AT.On(Attitude.KillRotation); break; }
				v = VSL.InOrbit? 
					VSL.Target.GetObtVelocity()-VSL.vessel.obt_velocity : 
					VSL.Target.GetSrfVelocity()-VSL.vessel.srf_velocity;
				if(v.magnitude < GLB.THR.MinDeltaV) { CFG.AT.On(Attitude.KillRotation); break; }
				if(CFG.AT.state == Attitude.AntiRelVel) v *= -1;
				needed_lthrust = VSL.LocalDir(v.normalized);
				break;
			case Attitude.ManeuverNode:
				var solver = VSL.vessel.patchedConicSolver;
				if(solver == null || solver.maneuverNodes.Count == 0)
				{ CFG.AT.On(Attitude.KillRotation); break; }
				needed_lthrust = VSL.LocalDir(-solver.maneuverNodes[0].GetBurnVector(VSL.orbit).normalized);
				break;
			}
			if(!(lthrust.IsZero() || needed_lthrust.IsZero())) 
				compute_steering(lthrust.normalized, needed_lthrust.normalized);
			ResetCustomRotation();
		}

		protected override void correct_steering()
		{
			if(BRC != null && BRC.IsActive)
				steering = Vector3.ProjectOnPlane(steering, lthrust);
			else steering -= Vector3.Project(steering, lthrust)*VSL.Controls.InvAlignmentFactor;
		}

		protected override void OnAutopilotUpdate(FlightCtrlState s)
		{
			//need to check all the prerequisites, because the callback is called asynchroniously
			if(!(CFG.Enabled && CFG.AT && VSL.refT != null && VSL.orbit != null)) return;
			if(VSL.AutopilotDisabled) { reset(); return; }
			compute_steering();
			tune_steering();
			set_authority_flag();
			VSL.Controls.AddSteering(steering);

			#if DEBUG
//			var error = Quaternion.FromToRotation(needed_lthrust, lthrust).eulerAngles;
//			CSV(AttitudeError, 
//			    error.x, error.y, error.z,
//			    steering.x, steering.y, steering.z, 
//			    steering_pid.Action.x, steering_pid.Action.y, steering_pid.Action.z,
//			    angularV.x, angularV.y, angularV.z 
//			   );//debug
//			if(VSL.IsActiveVessel)
//				TCAGui.DebugMessage = 
//					string.Format("pid: {0}\nsteering: {1}%\ngimbal limit: {2}",
//					              steering_pid, steering_pid.Action*100, VSL.Controls.GimbalLimit);
			#endif
		}

		#if DEBUG
		public void DrawDebugLines()
		{
			if(VSL == null || VSL.vessel == null || VSL.refT == null) return;
//			Utils.GLVec(VSL.refT.position, VSL.OnPlanetParams.Heading.normalized*2500, Color.white);
			Utils.GLVec(VSL.refT.position, VSL.Engines.CurrentThrustDir*20, Color.cyan);
			Utils.GLVec(VSL.refT.position, VSL.WorldDir(needed_lthrust.normalized)*20, Color.green);
//			Utils.GLVec(VSL.refT.position, VSL.WorldDir(VSL.vessel.angularVelocity*20), Color.green);
//			Utils.GLVec(VSL.refT.position, VSL.WorldDir(steering*20), Color.cyan);
//			Utils.GLVec(VSL.refT.position, VSL.WorldDir(steering_pid.Action*20), Color.magenta);

//			Utils.GLVec(VSL.refT.position, VSL.refT.right*2, Color.red);
//			Utils.GLVec(VSL.refT.position, VSL.refT.forward*2, Color.blue);
//			Utils.GLVec(VSL.refT.position, VSL.refT.up*2, Color.green);
		}
		#endif

		public override void Draw()
		{
			#if DEBUG
			DrawDebugLines();
			#endif
			GUILayout.BeginHorizontal();
			GUILayout.Label(new GUIContent("T-SAS", "Thrust attitude control"), 
			                CFG.AT && !VSL.AutopilotDisabled? Styles.cyan : Styles.white, GUILayout.ExpandWidth(false));
			if(Utils.ButtonSwitch("Kill", CFG.AT[Attitude.KillRotation], "Kill rotation", GUILayout.ExpandWidth(false)))
				CFG.AT.XToggle(Attitude.KillRotation);
			if(Utils.ButtonSwitch("Hold", CFG.AT[Attitude.HoldAttitude], "Hold current attitude", GUILayout.ExpandWidth(false)))
				CFG.AT.XToggle(Attitude.HoldAttitude);
			if(Utils.ButtonSwitch("Maneuver", CFG.AT[Attitude.ManeuverNode], "Maneuver node", GUILayout.ExpandWidth(false)))
				CFG.AT.XToggle(Attitude.ManeuverNode);
			if(Utils.ButtonSwitch("PG", CFG.AT[Attitude.Prograde], "Prograde", GUILayout.ExpandWidth(false)))
				CFG.AT.XToggle(Attitude.Prograde);
			if(Utils.ButtonSwitch("RG", CFG.AT[Attitude.Retrograde], "Retrograde", GUILayout.ExpandWidth(false)))
				CFG.AT.XToggle(Attitude.Retrograde);
			if(Utils.ButtonSwitch("R+", CFG.AT[Attitude.Radial], "Radial", GUILayout.ExpandWidth(false)))
				CFG.AT.XToggle(Attitude.Radial);
			if(Utils.ButtonSwitch("R-", CFG.AT[Attitude.AntiRadial], "AntiRadial", GUILayout.ExpandWidth(false)))
				CFG.AT.XToggle(Attitude.AntiRadial);
			if(Utils.ButtonSwitch("N+", CFG.AT[Attitude.Normal], "Normal", GUILayout.ExpandWidth(false)))
				CFG.AT.XToggle(Attitude.Normal);
			if(Utils.ButtonSwitch("N-", CFG.AT[Attitude.AntiNormal], "AntiNormal", GUILayout.ExpandWidth(false)))
				CFG.AT.XToggle(Attitude.AntiNormal);
			if(Utils.ButtonSwitch("T+", CFG.AT[Attitude.Target], "Target", GUILayout.ExpandWidth(false)))
				CFG.AT.XToggle(Attitude.Target);
			if(Utils.ButtonSwitch("T-", CFG.AT[Attitude.AntiTarget], "AntiTarget", GUILayout.ExpandWidth(false)))
				CFG.AT.XToggle(Attitude.AntiTarget);
			if(Utils.ButtonSwitch("rV+", CFG.AT[Attitude.RelVel], "Relative Velocity", GUILayout.ExpandWidth(false)))
				CFG.AT.XToggle(Attitude.RelVel);
			if(Utils.ButtonSwitch("rV-", CFG.AT[Attitude.AntiRelVel], "Against Relative Velocity", GUILayout.ExpandWidth(false)))
				CFG.AT.XToggle(Attitude.AntiRelVel);
			if(GUILayout.Button("Auto", CFG.AT[Attitude.Custom]? Styles.enabled_button : Styles.grey, GUILayout.ExpandWidth(false)))
				CFG.AT.OffIfOn(Attitude.Custom);
			var err = "OFF";
			if(VSL.AutopilotDisabled) err = "USER";
			else if(CFG.AT) err = string.Format("Err: {0:F1}°", VSL.Controls.AttitudeError);
			GUILayout.Label(err, VSL.Controls.Aligned? Styles.green : Styles.white, GUILayout.ExpandWidth(true));
			GUILayout.EndHorizontal();
		}
	}
}
