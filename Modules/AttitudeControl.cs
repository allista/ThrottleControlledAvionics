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
	[CareerPart]
	[RequireModules(typeof(SASBlocker))]
	[OptionalModules(typeof(TimeWarpControl))]
	public class AttitudeControl : AutopilotModule
	{
		public class Config : ModuleConfig
		{
			new public const string NODE_NAME = "ATC";

			[Persistent] public PID_Controller PID = new PID_Controller(10f, 0.02f, 0.5f, -1, 1);

			[Persistent] public float MinAAf = 0.1f, MaxAAf  = 1f;
			[Persistent] public float InertiaFactor = 10f, AngularMf = 0.002f;
			[Persistent] public float MoIFactor              = 0.01f;
			[Persistent] public float AngleThreshold         = 25f;
			[Persistent] public float MinEf = 0.001f, MaxEf  = 5f;
			[Persistent] public float SlowTorqueF            = 2;
			[Persistent] public float AALowPassF             = 1f;

			[Persistent] public float MaxAttitudeError       = 10f;  //deg
			[Persistent] public float AttitudeErrorThreshold = 3f;   //deg
		}
		static Config ATC { get { return TCAScenario.Globals.ATC; } }

		Transform  vesselTransform { get { return VSL.vessel.transform; } }
		readonly PIDv_Controller2 pid = new PIDv_Controller2();
		readonly FloatMinimum omega_min = new FloatMinimum();
		readonly LowPassFilterF AAf_filter = new LowPassFilterF();
		Transform refT;
		Quaternion attitude_error, locked_attitude;
		bool attitude_locked;
		Vector3 thrust, lthrust, needed_lthrust, steering;
		Vector3 angularV { get { return VSL.vessel.angularVelocity; } }
		Vector3 angularM;
		float AAf, Ef, PIf;

		public float AttitudeError { get; private set; }
		public bool Aligned { get; private set; }
		public float AttitudeFactor { get { return Utils.ClampL(1-AttitudeError/ATC.MaxAttitudeError, 0); } }

		TimeWarpControl TWR;

		public AttitudeControl(ModuleTCA tca) : base(tca) {}

		public override void Init() 
		{ 
			base.Init();
			pid.setPID(ATC.PID);
			reset();
			CFG.AT.SetSingleCallback(Enable);
			#if DEBUG
//			RenderingManager.AddToPostDrawQueue(1, RadarBeam);
			#endif
		}

		#if DEBUG
		public void RadarBeam()
		{
			if(VSL == null || VSL.vessel == null || VSL.refT == null || !CFG.AT) return;
			if(!thrust.IsZero())
				GLUtils.GLVec(VSL.Physics.wCoM, thrust.normalized*20, Color.red);
			if(!needed_lthrust.IsZero())
				GLUtils.GLVec(VSL.Physics.wCoM, VSL.WorldDir(needed_lthrust.normalized)*20, Color.yellow);
		}

		public override void Reset()
		{
			base.Reset();
//			RenderingManager.RemoveFromPostDrawQueue(1, RadarBeam);
		}
		#endif

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

		public Quaternion CustomRotation { get; private set; }

		public void AddCustomRotation(Vector3 from, Vector3 to)
		{ CustomRotation = Quaternion.FromToRotation(from, to) * CustomRotation; }

		public void AddCustomRotationW(Vector3 from, Vector3 to)
		{ CustomRotation = VSL.refT.rotation.Inverse() * Quaternion.FromToRotation(from, to) * VSL.refT.rotation * CustomRotation; }

		public void ResetCustomRotation() { CustomRotation = Quaternion.identity; }

		void reset()
		{
			pid.Reset();
			AAf_filter.Reset();
			AAf_filter.Tau = 0;
			refT = null;
			VSL.Controls.GimbalLimit = 100;
			AttitudeError = 180;
			Aligned = false;
			omega_min.Reset();
			attitude_locked = false;
		}

		void CalculateSteering()
		{
			Vector3 v;
			omega_min.Update(VSL.vessel.angularVelocity.sqrMagnitude);
			attitude_error = Quaternion.identity;
			needed_lthrust = Vector3.zero;
			steering = Vector3.zero;
			switch(CFG.AT.state)
			{
			case Attitude.Custom:
				attitude_error = CustomRotation;
				break;
			case Attitude.HoldAttitude:
				if(refT != VSL.refT || !attitude_locked)
				{
					refT = VSL.refT;
					locked_attitude = refT.rotation;
					attitude_locked = true;
				}
				if(refT != null)
					attitude_error = Quaternion.Inverse(refT.rotation.Inverse()*locked_attitude);
				break;
			case Attitude.KillRotation:
				if(refT != VSL.refT || omega_min.True)
				{
					refT = VSL.refT;
					locked_attitude = refT.rotation;
				}
				if(refT != null)
					attitude_error = Quaternion.Inverse(refT.rotation.Inverse()*locked_attitude);
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
			if(!needed_lthrust.IsZero())
			{
				thrust = VSL.Engines.Thrust.IsZero()? VSL.Engines.MaxThrust : VSL.Engines.Thrust;
				lthrust = VSL.LocalDir(thrust).normalized;
				if(Vector3.Angle(needed_lthrust, lthrust) > ATC.AngleThreshold)
				{
					//rotational axis
					var lthrust_maxI = lthrust.MaxI();
					var axis = Vector3.Cross(needed_lthrust, lthrust).Exclude(lthrust_maxI);
					if(axis.sqrMagnitude < 0.01f) 
						axis = VSL.Torque.MaxAngularA.Exclude(lthrust_maxI).MaxComponent();
					//main rotation component
					var axis1 = axis.MaxComponent();
					var lthrust_cmp1 = Vector3.ProjectOnPlane(lthrust, axis1);
					var needed_lthrust_cmp1 = Vector3.ProjectOnPlane(needed_lthrust, axis1);
					var angle1 = Vector3.Angle(needed_lthrust_cmp1, lthrust_cmp1);
					//second rotation component
					var axis2 = (axis - axis1).MaxComponent();
					var angle2 = Vector3.Angle(needed_lthrust, needed_lthrust_cmp1);
					//steering
					steering = (axis1.normalized * angle1 + 
					            axis2.normalized * angle2) * Mathf.Deg2Rad;
				}
				else attitude_error = Quaternion.FromToRotation(needed_lthrust, lthrust);   
			}
			if(attitude_error != Quaternion.identity)
			{
				var euler = attitude_error.eulerAngles;
				steering = new Vector3(Utils.CenterAngle(euler.x),
				                       Utils.CenterAngle(euler.y),
				                       Utils.CenterAngle(euler.z))*Mathf.Deg2Rad;
					
				#if DEBUG
				thrust = VSL.Engines.Thrust.IsZero()? VSL.Engines.MaxThrust : VSL.Engines.Thrust;
				lthrust = VSL.LocalDir(thrust).normalized;
				needed_lthrust = attitude_error.Inverse()*lthrust;
				#endif
			}
			ResetCustomRotation();
		}

		protected override void OnAutopilotUpdate(FlightCtrlState s)
		{
			//need to check all the prerequisites, because the callback is called asynchroniously
			if(!(CFG.Enabled && CFG.AT && VSL.refT != null && VSL.orbit != null)) return;
			VSL.Controls.GimbalLimit = 100;
			if(VSL.AutopilotDisabled) { reset(); return; }
			//calculate needed steering
			CalculateSteering();
			//calculate attitude error and Aligned state
			var steering_m = steering.magnitude;
			AttitudeError = steering_m*Mathf.Rad2Deg;
			Aligned &= AttitudeError < ATC.MaxAttitudeError;
			Aligned |= AttitudeError < ATC.AttitudeErrorThreshold;
			Ef = Utils.Clamp(steering_m/Mathf.PI, ATC.MinEf, 1);
			//tune lowpass filter
			AAf_filter.Tau = (1-Mathf.Sqrt(Ef))*ATC.AALowPassF;
			//tune PID parameters
			angularM = Vector3.Scale(angularV, VSL.Physics.MoI);
			AAf = AAf_filter.Update(Mathf.Clamp(1/VSL.Torque.MaxAngularA_m, ATC.MinAAf, ATC.MaxAAf));
			var slow = VSL.Engines.SlowTorque? 1+VSL.Engines.TorqueResponseTime*ATC.SlowTorqueF : 1;
			PIf = AAf*Utils.ClampL(1-Ef, 0.5f)*ATC.MaxEf/slow;
			pid.P = ATC.PID.P*PIf;
			pid.I = ATC.PID.I*PIf;
			pid.D = ATC.PID.D*Utils.ClampH(Utils.ClampL(1-Ef*2, 0)+angularM.magnitude*ATC.AngularMf, 1)*AAf*AAf*slow*slow;
			//set gimbal limit
			VSL.Controls.GimbalLimit = Ef*100;
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
				Mathf.Lerp(ATC.InertiaFactor, 1, VSL.Physics.MoI.magnitude*ATC.MoIFactor);
			//update PID and set steering
			pid.Update(steering, angularV);
			SetRot(pid.Action, s);

			#if DEBUG
//			CSV(VSL.Physics.UT,
//			    AttitudeError, 
//			    steering.x, steering.y, steering.z, 
//			    angularV.x, angularV.y, angularV.z, 
//			    pid.Action.x, pid.Action.y, pid.Action.x
//			    );//debug
			if(VSL.IsActiveVessel)
				ThrottleControlledAvionics.DebugMessage = 
					string.Format("pid: {0}\nsteering: {1}%\ngimbal limit: {2}",
					              pid, pid.Action*100, VSL.Controls.GimbalLimit);
			#endif
		}

		public override void Draw()
		{
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
			if(GUILayout.Button("Auto", CFG.AT[Attitude.Custom]? Styles.enabled_button : Styles.inactive_button, GUILayout.ExpandWidth(false)))
				CFG.AT.OffIfOn(Attitude.Custom);
			GUILayout.Label(CFG.AT? string.Format("Err: {0:F1}°", AttitudeError) : "Err: N/A", 
			            Aligned? Styles.green : Styles.white, GUILayout.ExpandWidth(true));
			GUILayout.EndHorizontal();
		}
	}
}
