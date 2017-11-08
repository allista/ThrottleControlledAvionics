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
		}
		static Config BRC { get { return Globals.Instance.BRC; } }

		readonly Timer DirectionLineTimer = new Timer();

        [Persistent]
		public FloatField Bearing = new FloatField("F1", min:0, max:360, circle:true);

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

        public override void Disable()
        {
            CFG.BR.Off();
            DirectionOverride.Zero();
        }

		protected override void UpdateState()
		{ 
			base.UpdateState();
            IsActive &= VSL.refT != null && VSL.OnPlanet && (!DirectionOverride.IsZero() || CFG.BR && !ForwardDirection.IsZero());
		}

		public override void Init()
		{
			base.Init();
			CFG.BR.AddSingleCallback(ControlCallback);
		}

		public void ControlCallback(Multiplexer.Command cmd)
		{
            Reset();
			switch(cmd)
			{
			case Multiplexer.Command.Resume:
			case Multiplexer.Command.On:
				RegisterTo<SASBlocker>();
                NeedCPSWhenMooving();
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

		protected override void OnAutopilotUpdate()
		{
			//allow user to intervene
			if(VSL.HasUserInput)
			{
				if(!CS.yaw.Equals(0))
				{
					UpdateBearing(Bearing.Value + CS.yaw*BRC.YawFactor*CFG.ControlSensitivity);
					if(CFG.HF[HFlight.CruiseControl] && !VSL.HorizontalSpeed.NeededVector.IsZero()) 
						VSL.HorizontalSpeed.SetNeeded(ForwardDirection * CFG.MaxNavSpeed);
					draw_forward_direction = BRC.DrawForwardDirection;
					VSL.HasUserInput = !(CS.pitch.Equals(0) && CS.roll.Equals(0));
					VSL.AutopilotDisabled = VSL.HasUserInput;
					DirectionLineTimer.Reset();
                    Reset();
					CS.yaw = 0;
				}
			}
			if(!VSL.AutopilotDisabled)
            {
    			//turn ship's nose in the direction of needed velocity
    			var nDir  = DirectionOverride.IsZero()? H(ForwardDirection) : H(DirectionOverride);
    			if(!nDir.IsZero())
                {
                    var cDir = VSL.OnPlanetParams.Heading;
                    var w_axis = VSL.Engines.refT_thrust_axis;
                    var error_sign = Mathf.Sign(Vector3.Dot(Vector3.Cross(cDir, nDir), w_axis));
                    rotation_axis = VSL.LocalDir(w_axis)*error_sign;
                    angular_error = Utils.Angle2(cDir, nDir)/180;
                    var AV = Vector3.Dot(VSL.vessel.angularVelocity, rotation_axis);
                    var AM = AV*Vector3.Dot(VSL.Physics.MoI, rotation_axis.AbsComponents());
                    var MaxAA = VSL.Torque.MaxCurrent.AngularAccelerationAroundAxis(rotation_axis);
                    var iMaxAA = 1/MaxAA;
                    pid_yaw.TuneFast(AV, AM, MaxAA, iMaxAA, 1-angular_error);
                    pid_yaw.atPID.Update(angular_error*Mathf.PI* 
                                         //lower the error when the nose is up or down
                                         (1- Mathf.Abs(Vector3.Dot(VSL.OnPlanetParams.Fwd, VSL.Physics.Up))), -AV);
                    steering = rotation_axis*pid_yaw.UpdateAV(AV-pid_yaw.atPID.Action);
                    VSL.Controls.AddSteering(steering);
                }
            }
            DirectionOverride = Vector3d.zero;
		}

		bool draw_forward_direction;
		static readonly GUIContent X_cnt = new GUIContent("X", "Disable Bearing Control");
		static readonly GUIContent Enable_cnt = new GUIContent("Bearing", "Enable Bearing Control");
		static Color dir_color = new Color(0, 1, 0, 0.5f);
		public override void Draw ()
		{
			#if DEBUG
//			var dir = DirectionOverride.IsZero()? ForwardDirection : DirectionOverride;
//			Utils.GLVec(VSL.refT.position, dir.normalized*2500, dir_color);
//            Utils.GLVec(VSL.refT.position, rotation_axis*5, Color.red);
//			Utils.GLVec(VSL.refT.position, VSL.OnPlanetParams.Fwd*5, Color.yellow);
//			Utils.GLVec(VSL.refT.position, VSL.OnPlanetParams.Heading*5, Color.magenta);
			#endif
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

