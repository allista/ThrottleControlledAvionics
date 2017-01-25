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

			[Persistent] public float AAf_a = 1f;
			[Persistent] public float AAf_b = 0.01f;
			[Persistent] public float AAf_c = 0.5f;

			[Persistent] public PIDf_Controller DirectionPID = new PIDf_Controller(Mathf.PI, 0.001f, 1, -Mathf.PI, Mathf.PI);
			[Persistent] public PIDf_Controller AV_PID = new PIDf_Controller(1, 0.05f, 0, -1, 1);
		}
		static Config BRC { get { return Globals.Instance.BRC; } }

		readonly Timer DirectionLineTimer = new Timer();
		readonly PIDf_Controller bearing_pid = new PIDf_Controller();
		readonly PIDf_Controller av_pid = new PIDf_Controller();
		public readonly FloatField Bearing = new FloatField("F1", min:0, max:360, circle:true);

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
			av_pid.setPID(BRC.AV_PID);
			av_pid.Reset();
			CFG.BR.AddSingleCallback(ControlCallback);
		}

		public void ControlCallback(Multiplexer.Command cmd)
		{
			bearing_pid.Reset();
			av_pid.Reset();
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
			if(!(CFG.Enabled && IsActive && VSL.refT != null)) goto end;
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
			if(VSL.AutopilotDisabled) goto end;
			//turn ship's nose in the direction of needed velocity
			var nDir  = DirectionOverride.IsZero()? H(ForwardDirection) : H(DirectionOverride);
			if(nDir.IsZero()) goto end;
			var axis  = VSL.Engines.refT_thrust_axis;
			var laxis = VSL.LocalDir(axis);
			var cDir  = VSL.OnPlanetParams.Heading;
			var angle = Vector3.Angle(cDir, nDir)*Mathf.Sign(Vector3.Dot(Vector3.Cross(nDir, cDir), axis));
			var maxAA = VSL.Torque.MaxCurrent.AngularAccelerationAroundAxis(laxis);
			var AAf = BRC.AAf_a/(BRC.AAf_b+Mathf.Pow(maxAA, BRC.AAf_c));
			bearing_pid.D = AAf;
			bearing_pid.Update(angle/180);
			av_pid.Update(bearing_pid.Action+Vector3.Dot(VSL.vessel.angularVelocity, laxis));
			steering = laxis*av_pid.Action*Mathf.PI;
			VSL.Controls.AddSteering(steering);
//			if(VSL.IsActiveVessel)
//				TCAGui.DebugMessage = 
//					Utils.Format("BearingControl: {}\n"+
//					             "angle {}deg\n" +
//					             "action {}rad/s\n" +
//					             "maxAA: {}\n"+
//					             "AAf {}\n" +
//					             "br PID: {}\n" +
//					             "av PID: {}", 
//					             CFG.BR.state,
//					             angle, 
//					             bearing_pid.Action, 
//					             maxAA,
//					             AAf, 
//					             bearing_pid, av_pid);//debug
			end: 
			DirectionOverride = Vector3d.zero;
//			if(VSL.IsActiveVessel && string.IsNullOrEmpty(TCAGui.DebugMessage))
//				TCAGui.DebugMessage = 
//					Utils.Format("forward {}\noverride {}\nnDir {}\n" +
//					             "AutopilotDisabled {}, IsActive {}",
//					             ForwardDirection, DirectionOverride, 
//					             DirectionOverride.IsZero()? H(ForwardDirection) : H(DirectionOverride),
//					             VSL.AutopilotDisabled, IsActive);//debug
		}

		bool draw_forward_direction;
		static readonly GUIContent X_cnt = new GUIContent("X", "Disable Bearing Control");
		static readonly GUIContent Enable_cnt = new GUIContent("Bearing", "Enable Bearing Control");
		static Color dir_color = new Color(0, 1, 0, 0.5f);
		public override void Draw ()
		{
//			#if DEBUG
//			var dir = DirectionOverride.IsZero()? ForwardDirection : DirectionOverride;
//			Utils.GLVec(VSL.refT.position, dir.normalized*2500, dir_color);
//			Utils.GLVec(VSL.refT.position, refT_thrust_axis*5, Color.red);
//			Utils.GLVec(VSL.refT.position, VSL.OnPlanetParams.Fwd*5, Color.yellow);
//			Utils.GLVec(VSL.refT.position, VSL.OnPlanetParams.Heading*5, Color.magenta);
//			#endif
			if(CFG.BR[BearingMode.Auto] || !DirectionOverride.IsZero())
			{
				GUILayout.Label("AutoBearing", Styles.green, GUILayout.ExpandWidth(true));
			}
			else if(CFG.BR[BearingMode.User])
			{
				if(draw_forward_direction)
				{
					Utils.GLVec(VSL.Physics.wCoM, ForwardDirection.normalized*2500, dir_color);
					draw_forward_direction = !DirectionLineTimer.TimePassed;
				}
				if(Bearing.Draw("°", increment:10))
				{ 
					ForwardDirection = VSL.Physics.Direction(Bearing);
					if(CFG.HF[HFlight.CruiseControl] && !VSL.HorizontalSpeed.NeededVector.IsZero()) 
						VSL.HorizontalSpeed.SetNeeded(ForwardDirection * CFG.MaxNavSpeed);
					
				}
				if(GUILayout.Button(X_cnt, Styles.close_button, GUILayout.ExpandWidth(true)))
					CFG.BR.Off();
			}
			else if(!CFG.BR && GUILayout.Button(Enable_cnt, Styles.active_button, GUILayout.ExpandWidth(true)))
				CFG.BR.XOn(BearingMode.User);
		}
	}
}

