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

namespace ThrottleControlledAvionics
{
	public abstract class AttitudeControlBase : ThrustDirectionControl
	{
		public new class Config : ModuleConfig
		{
			[Persistent] public PID_Controller PID = new PID_Controller(10f, 0.02f, 0.5f, -1, 1);

			[Persistent] public float MinAAf = 0.1f, MaxAAf  = 1f;
			[Persistent] public float InertiaFactor = 10f, AngularMf = 0.002f;
			[Persistent] public float MoIFactor              = 0.01f;
			[Persistent] public float MinEf = 0.001f, MaxEf  = 5f;
			[Persistent] public float SlowTorqueF            = 2;
			[Persistent] public float AALowPassF             = 1f;

			[Persistent] public float AngleThreshold         = 60f;
			[Persistent] public float MaxAttitudeError       = 10f;  //deg
			[Persistent] public float AttitudeErrorThreshold = 3f;   //deg
		}
		static Config ATCB { get { return TCAScenario.Globals.ATCB; } }

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
		protected readonly PIDv_Controller2 steering_pid = new PIDv_Controller2();
		protected readonly LowPassFilterF AAf_filter = new LowPassFilterF();
		protected Vector3 angularV;
		protected Vector3 angularM;
		protected float AAf, Ef, PIf;

		public float AttitudeError { get; private set; }
		public bool  Aligned { get; private set; }
		public float AttitudeFactor { get { return Utils.ClampL(1-AttitudeError/ATCB.MaxAttitudeError, 0); } }

		public override void Init() 
		{ 
			base.Init();
			steering_pid.setPID(ATCB.PID);
			reset();
		}

		protected override void reset()
		{
			steering_pid.Reset();
			AAf_filter.Reset();
			AAf_filter.Tau = 0;
			VSL.Controls.GimbalLimit = 100;
			AttitudeError = 180;
			Aligned = false;
		}

		protected static Vector3 rotation2steering(Quaternion rotation)
		{
			var euler = rotation.eulerAngles;
			return new Vector3(Utils.CenterAngle(euler.x)*Mathf.Deg2Rad,
			                   Utils.CenterAngle(euler.y)*Mathf.Deg2Rad,
			                   Utils.CenterAngle(euler.z)*Mathf.Deg2Rad);
		}

		protected Vector3 fwd_axis
		{ get { return VSL.OnPlanetParams.NoseUp? VSL.vessel.transform.forward : VSL.vessel.transform.up; } }

		protected Vector3 up_axis
		{
			get 
			{
				var axis = VSL.OnPlanetParams.NoseUp? VSL.vessel.transform.up : VSL.vessel.transform.forward;
				axis *= -Mathf.Sign(Vector3.Dot(axis, VSL.Physics.Up));
				return axis;
			}
		}

		protected Quaternion world2local_rotation(Quaternion world_rotation)
		{ return VSL.refT.rotation.Inverse() * world_rotation * VSL.refT.rotation; }

		protected void compute_steering(Vector3 current, Vector3 needed)
		{
			if(Vector3.Angle(needed, current) > ATCB.AngleThreshold)
			{
				//rotational axis
				var current_maxI = current.MaxI();
				var axis = Vector3.Cross(needed, current).Exclude(current_maxI);
				if(axis.sqrMagnitude < 0.01f) 
					axis = VSL.Torque.MaxAngularA.Exclude(current_maxI).MaxComponent();
				//main rotation component
				var axis1 = axis.MaxComponent();
				var current_cmp1 = Vector3.ProjectOnPlane(current, axis1);
				var needed_cmp1 = Vector3.ProjectOnPlane(needed, axis1);
				var angle1 = Vector3.Angle(needed_cmp1, current_cmp1);
				//second rotation component
				var axis2 = (axis - axis1).MaxComponent();
				var angle2 = Vector3.Angle(needed, needed_cmp1);
				//steering
				steering = (axis1.normalized * angle1 + 
				            axis2.normalized * angle2) * Mathf.Deg2Rad;
			}
			else steering = rotation2steering(Quaternion.FromToRotation(needed, current));
//			LogF("\nneeded {}\ncurrent {}\nangle {}, steering {}",
//			     needed, current, Vector3.Angle(needed, current), steering);//debug
		}

		protected void compute_steering(Quaternion rotation)
		{ compute_steering(Vector3.up, rotation*Vector3.up); }

		protected void compute_steering(Rotation rotation)
		{ compute_steering(rotation.current, rotation.needed); }

		protected void Steer(FlightCtrlState s, bool set_grater = false)
		{
			//calculate attitude error and Aligned state
			var steering_m = steering.magnitude;
			AttitudeError = steering_m*Mathf.Rad2Deg;
			Aligned &= AttitudeError < ATCB.MaxAttitudeError;
			Aligned |= AttitudeError < ATCB.AttitudeErrorThreshold;
			Ef = Utils.Clamp(steering_m/Mathf.PI, ATCB.MinEf, 1);
			//tune lowpass filter
			AAf_filter.Tau = (1-Mathf.Sqrt(Ef))*ATCB.AALowPassF;
			//tune PID parameters
			angularV = VSL.vessel.angularVelocity;
			angularM = Vector3.Scale(angularV, VSL.Physics.MoI);
			AAf = AAf_filter.Update(Mathf.Clamp(1/VSL.Torque.MaxAngularA_m, ATCB.MinAAf, ATCB.MaxAAf));
			var slow = VSL.Engines.SlowTorque? 1+VSL.Engines.TorqueResponseTime*ATCB.SlowTorqueF : 1;
			PIf = AAf*Utils.ClampL(1-Ef, 0.5f)*ATCB.MaxEf/slow;
			steering_pid.P = ATCB.PID.P*PIf;
			steering_pid.I = ATCB.PID.I*PIf;
			steering_pid.D = ATCB.PID.D*Utils.ClampH(Utils.ClampL(1-Ef*2, 0)+angularM.magnitude*ATCB.AngularMf, 1)*AAf*AAf*slow*slow;
			//set gimbal limit
			VSL.Controls.GimbalLimit = VSL.vessel.ctrlState.mainThrottle > 0? Ef*100 : 0;
			//tune steering
			var control = new Vector3(steering.x.Equals(0)? 0 : 1,
			                          steering.y.Equals(0)? 0 : 1,
			                          steering.z.Equals(0)? 0 : 1);
			steering.Scale(Vector3.Scale(VSL.Torque.MaxAngularA.Exclude(steering.MinI()), control).Inverse(0).CubeNorm());
			//add inertia
			steering += Vector3.Scale(angularM.Sign(),
			                          Vector3.Scale(Vector3.Scale(angularM, angularM),
			                                          Vector3.Scale(VSL.Torque.MaxTorque, VSL.Physics.MoI).Inverse(0)))
				.ClampComponents(-Mathf.PI, Mathf.PI)/
				Mathf.Lerp(ATCB.InertiaFactor, 1, VSL.Physics.MoI.magnitude*ATCB.MoIFactor);
			//update PID and set steering
			steering_pid.Update(steering, angularV);
			if(set_grater) SetGraterRot(steering_pid.Action, s);
			else SetRot(steering_pid.Action, s);
		}
	}

	[CareerPart]
	[RequireModules(typeof(SASBlocker))]
	[OptionalModules(typeof(TimeWarpControl))]
	public class AttitudeControl : AttitudeControlBase
	{
		readonly MinimumF omega_min = new MinimumF();
		Transform refT;
		Quaternion locked_attitude;
		bool attitude_locked;
		Vector3 thrust, lthrust, needed_lthrust;//debug

		Vector3 current_thrust 
		{
			get
			{
				var thrust = VSL.Engines.Thrust;
				if(thrust.IsZero()) thrust = VSL.Engines.MaxThrust;
				return thrust;
			}
		}

		TimeWarpControl TWR;

		public AttitudeControl(ModuleTCA tca) : base(tca) {}

		public override void Init() 
		{ 
			base.Init();
			CFG.AT.SetSingleCallback(Enable);
		}

		protected override void UpdateState() { IsActive = CFG.AT; }

		public void Enable(Multiplexer.Command cmd)
		{
			reset();
			switch(cmd)
			{
			case Multiplexer.Command.Resume:
				if(TWR != null) TWR.StopWarp();
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
		{ CustomRotation = Rotation.Local(current_thrust, needed, VSL); }

		public void ResetCustomRotation() { CustomRotation = default(Rotation); }

		protected override void reset()
		{
			base.reset();
			refT = null;
			omega_min.Reset();
			attitude_locked = false;
		}

		void CalculateSteering()
		{
			Vector3 v;
			omega_min.Update(VSL.vessel.angularVelocity.sqrMagnitude);
			thrust = current_thrust;
			lthrust = VSL.LocalDir(thrust).normalized;
			needed_lthrust = Vector3.zero;
			steering = Vector3.zero;
			switch(CFG.AT.state)
			{
			case Attitude.Custom:
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
				compute_steering(lthrust, needed_lthrust);
			ResetCustomRotation();
		}

		protected override void OnAutopilotUpdate(FlightCtrlState s)
		{
			//need to check all the prerequisites, because the callback is called asynchroniously
			if(!(CFG.Enabled && CFG.AT && VSL.refT != null && VSL.orbit != null)) return;
			VSL.Controls.GimbalLimit = 100;
			if(VSL.AutopilotDisabled) { reset(); return; }
			CalculateSteering();
			Steer(s);

			#if DEBUG
//			var error = Quaternion.FromToRotation(needed_lthrust, lthrust).eulerAngles;
//			CSV(AttitudeError, 
//			    error.x, error.y, error.z,
//			    steering.x, steering.y, steering.z, 
//			    steering_pid.Action.x, steering_pid.Action.y, steering_pid.Action.z,
//			    angularV.x, angularV.y, angularV.z 
//			   );//debug
			if(VSL.IsActiveVessel)
				TCAGui.DebugMessage = 
					string.Format("pid: {0}\nsteering: {1}%\ngimbal limit: {2}",
					              steering_pid, steering_pid.Action*100, VSL.Controls.GimbalLimit);
			#endif
		}

		#if DEBUG
		public void RadarBeam()
		{
			if(VSL == null || VSL.vessel == null || VSL.refT == null) return;
//			if(!thrust.IsZero())
//				GLUtils.GLVec(VSL.Physics.wCoM, thrust.normalized*20, Color.red);
//			if(!needed_lthrust.IsZero())
//				GLUtils.GLVec(VSL.Physics.wCoM, VSL.WorldDir(needed_lthrust.normalized)*20, Color.yellow);
			GLUtils.GLVec(VSL.Physics.wCoM, VSL.WorldDir(VSL.vessel.angularVelocity*20), Color.green);
//			GLUtils.GLVec(VSL.refT.position, VSL.WorldDir(steering.normalized*20), Color.cyan);
			GLUtils.GLVec(VSL.refT.position, VSL.WorldDir(steering_pid.Action.normalized*20), Color.magenta);

			GLUtils.GLVec(VSL.refT.position, VSL.refT.right*2, Color.red);
			GLUtils.GLVec(VSL.refT.position, VSL.refT.forward*2, Color.blue);
			GLUtils.GLVec(VSL.refT.position, VSL.refT.up*2, Color.green);
		}
		#endif

		public override void Draw()
		{
			#if DEBUG
//			RadarBeam();
			#endif
			GUILayout.BeginHorizontal();
			GUILayout.Label(new GUIContent("T-SAS", "Thrust attitude control"), 
			            CFG.AT? Styles.cyan : Styles.white, GUILayout.ExpandWidth(false));
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
			GUILayout.Label(CFG.AT? string.Format("Err: {0:F1}°", AttitudeError) : "", 
			            Aligned? Styles.green : Styles.white, GUILayout.ExpandWidth(true));
			GUILayout.EndHorizontal();
		}
	}
}
