//  Author:
//       Allis Tauri <allista@gmail.com>
//
//  Copyright (c) 2016 Allis Tauri
//
// This work is licensed under the Creative Commons Attribution-ShareAlike 4.0 International License. 
// To view a copy of this license, visit http://creativecommons.org/licenses/by-sa/4.0/ 
// or send a letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.
//
using System;
using UnityEngine;
using AT_Utils;

namespace ThrottleControlledAvionics
{
	[CareerPart(typeof(AttitudeControl))]
	[RequireModules(typeof(SASBlocker))]
	[OverrideModules(typeof(AttitudeControl))]
	public class BearingControl : AttitudeControlBase
	{
		public new class Config : ModuleConfig
		{
			[Persistent] public bool  DrawForwardDirection = true;
			[Persistent] public float YawFactor = 60f;
			[Persistent] public float MinAAf = 0.001f;
			[Persistent] public float MaxAAf = 2;
			[Persistent] public float ADf    = 0.1f;
			[Persistent] public PID_Controller DirectionPID = new PID_Controller(0.5f, 0f, 0.5f, -175, 175);
		}
		static Config BRC { get { return Globals.Instance.BRC; } }

		readonly Timer DirectionLineTimer = new Timer();
		readonly PIDf_Controller2 bearing_pid = new PIDf_Controller2();
		public readonly FloatField Bearing = new FloatField(min:0, max:360, circle:true);

		#if DEBUG
//		Vector3d DO;
//		public Vector3d DirectionOverride { get { return DO; } 
//			set { if(!value.IsZero()) LogFST("DirOverride: {}", value); DO = value; } }
//		Vector3d FD;
//		public Vector3d ForwardDirection { get { return FD; } 
//			set { if(!value.IsZero()) LogFST("FwdOverride: {}", value); FD = value; } }
		#endif
		public Vector3d DirectionOverride;
		public Vector3d ForwardDirection;

		public BearingControl(ModuleTCA tca) : base(tca) {}

		protected override void UpdateState()
		{ 
			base.UpdateState();
			IsActive &= VSL.OnPlanet && (!DirectionOverride.IsZero() || CFG.BR && !ForwardDirection.IsZero());
		}

		public override void Init()
		{
			base.Init();
			bearing_pid.setPID(BRC.DirectionPID);
			bearing_pid.Reset();
			CFG.BR.AddSingleCallback(ControlCallback);
		}

		public void ControlCallback(Multiplexer.Command cmd)
		{
			bearing_pid.Reset();
			switch(cmd)
			{
			case Multiplexer.Command.Resume:
			case Multiplexer.Command.On:
				RegisterTo<SASBlocker>();
				NeedRadarWhenMooving();
				ForwardDirection = VSL.OnPlanetParams.Fwd;
				Bearing.Value = (float)VSL.Physics.Bearing(ForwardDirection);
				break;

			case Multiplexer.Command.Off:
				DirectionOverride = Vector3d.zero;
				UnregisterFrom<SASBlocker>();
				UnregisterFrom<Radar>();
				break;
			}
		}

		public void UpdateBearing(float bearing)
		{
			Bearing.Value = bearing;
			ForwardDirection = VSL.Physics.Direction(Bearing);
		}

		protected override void OnAutopilotUpdate(FlightCtrlState s)
		{
			//need to check all the prerequisites, because the callback is called asynchroniously
			if(!(CFG.Enabled && IsActive && VSL.refT != null))
			{ DirectionOverride = Vector3d.zero; return; }
			//allow user to intervene
			if(VSL.HasUserInput)
			{
				if(!s.yaw.Equals(0))
				{
					UpdateBearing(Bearing.Value + s.yaw*BRC.YawFactor*CFG.ControlSensitivity);
					if(CFG.HF[HFlight.CruiseControl] && !VSL.HorizontalSpeed.NeededVector.IsZero()) 
						VSL.HorizontalSpeed.SetNeeded(ForwardDirection * CFG.MaxNavSpeed);
					draw_forward_direction = BRC.DrawForwardDirection;
					VSL.HasUserInput = !(s.pitch.Equals(0) && s.roll.Equals(0));
					VSL.AutopilotDisabled = VSL.HasUserInput;
					DirectionLineTimer.Reset();
					bearing_pid.Reset();
					s.yaw = 0;
				}
			}
			if(VSL.AutopilotDisabled) 
			{ DirectionOverride = Vector3d.zero; return; }
			//turn ship's nose in the direction of needed velocity
			var axis  = up_axis;
			var laxis = VSL.LocalDir(axis);
			var cDir  = H(VSL.OnPlanetParams.Fwd);
			var nDir  = DirectionOverride.IsZero()? H(ForwardDirection) : H(DirectionOverride);
			var angle = Vector3.Angle(cDir, nDir)*Mathf.Sign(Vector3.Dot(Vector3.Cross(nDir, cDir), axis));
			var eff   = Mathf.Abs(Vector3.Dot(VSL.Engines.CurrentMaxThrustDir.normalized, VSL.Physics.Up));
			var ADf   = (float)VSL.vessel.staticPressurekPa/VSL.Torque.MaxCurrent.AngularDragResistanceAroundAxis(laxis)*BRC.ADf+1;
			AAf = Utils.Clamp(1/VSL.Torque.MaxCurrent.AngularAccelerationAroundAxis(laxis)/ADf, BRC.MinAAf, BRC.MaxAAf);
			bearing_pid.P = BRC.DirectionPID.P;
			bearing_pid.D = BRC.DirectionPID.D*AAf;
			bearing_pid.Update(angle*eff, Vector3.Dot(VSL.vessel.angularVelocity, laxis)*Mathf.Rad2Deg);
			steering = rotation2steering(world2local_rotation(Quaternion.AngleAxis(bearing_pid.Action, axis)));
			VSL.Controls.AddSteering(steering);
//			if(VSL.IsActiveVessel)
//				TCAGui.DebugMessage = 
//					Utils.Format("angle {}deg, action {}deg, action*eff {}deg\n" +
//					             "AAf {}, ADf {}, result {}",
//					              angle, bearing_pid.Action, bearing_pid.Action*eff, 
//					             1/VSL.Torque.MaxCurrent.AngularAccelerationAroundAxis(laxis), ADf, AAf);//debug
			DirectionOverride = Vector3d.zero;
		}

		bool draw_forward_direction;
		static readonly GUIContent X_cnt = new GUIContent("X", "Disable Bearing Control");
		static readonly GUIContent Enable_cnt = new GUIContent("Bearing", "Enable Bearing Control");
		public override void Draw ()
		{
			if(CFG.BR[BearingMode.Auto] || !DirectionOverride.IsZero())
			{
				GUILayout.Label("AutoBearing", Styles.green, GUILayout.ExpandWidth(false));
				#if DEBUG
				var dir = DirectionOverride.IsZero()? ForwardDirection : DirectionOverride;
				Utils.GLVec(VSL.Controls.Transform.position, dir.normalized*2500, Color.green);
				#endif
			}
			else if(CFG.BR[BearingMode.User])
			{
				if(draw_forward_direction)
				{
					Utils.GLVec(VSL.Physics.wCoM, ForwardDirection.normalized*2500, Color.green);
					draw_forward_direction = !DirectionLineTimer.TimePassed;
				}
				if(Bearing.Draw("°", increment:10))
				{ 
					ForwardDirection = VSL.Physics.Direction(Bearing);
					if(CFG.HF[HFlight.CruiseControl] && !VSL.HorizontalSpeed.NeededVector.IsZero()) 
						VSL.HorizontalSpeed.SetNeeded(ForwardDirection * CFG.MaxNavSpeed);
					
				}
				if(GUILayout.Button(X_cnt, Styles.close_button, GUILayout.ExpandWidth(false)))
					CFG.BR.Off();
			}
			else if(!CFG.BR && GUILayout.Button(Enable_cnt, Styles.active_button, GUILayout.ExpandWidth(false)))
				CFG.BR.XOn(BearingMode.User);
		}
	}
}

