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

namespace ThrottleControlledAvionics
{
	[CareerPart(typeof(AttitudeControl))]
	[RequireModules(typeof(SASBlocker))]
	[OverrideModules(typeof(AttitudeControl))]
	public class BearingControl : AutopilotModule
	{
		public class Config : ModuleConfig
		{
			new public const string NODE_NAME = "BRC";

			[Persistent] public bool  DrawForwardDirection = true;
			[Persistent] public float YawFactor = 0.6f;
			[Persistent] public float MinAAf = 0.001f;
			[Persistent] public float MaxAAf = 2;
			[Persistent] public PID_Controller DirectionPID = new PID_Controller(0.5f, 0f, 0.5f, -1, 1);
		}
		static Config BRC { get { return TCAScenario.Globals.BRC; } }

		readonly Timer DirectionLineTimer = new Timer();
		readonly PIDf_Controller pid = new PIDf_Controller();
		public readonly FloatField Bearing = new FloatField(min:0, max:360, circle:true);

		public Vector3d DirectionOverride;
		public Vector3d ForwardDirection;

		public BearingControl(ModuleTCA tca) : base(tca) {}

		public override void Init()
		{
			base.Init();
			pid.setPID(BRC.DirectionPID);
			pid.Reset();
			CFG.BR.AddSingleCallback(ControlCallback);
		}

		protected override void UpdateState() { IsActive = CFG.BR && VSL.OnPlanet; }

		public void ControlCallback(Multiplexer.Command cmd)
		{
			pid.Reset();
			switch(cmd)
			{
			case Multiplexer.Command.Resume:
			case Multiplexer.Command.On:
				RegisterTo<SASBlocker>();
				RegisterTo<Radar>(vsl => vsl.HorizontalSpeed.MoovingFast);
				ForwardDirection = VSL.OnPlanetParams.Fwd;
				Bearing.Value = (float)VSL.Physics.Bearing(ForwardDirection);
				break;

			case Multiplexer.Command.Off:
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
			if(!(CFG.Enabled && 
			     (CFG.BR || !DirectionOverride.IsZero()) &&
			     VSL.OnPlanet && VSL.refT != null && 
			     !ForwardDirection.IsZero())) return;
			//allow user to intervene
			var cDir = Vector3.ProjectOnPlane(VSL.OnPlanetParams.FwdL, VSL.Physics.UpL).normalized;
			if(VSL.HasUserInput)
			{
				if(!s.yaw.Equals(0))
				{
					UpdateBearing(Bearing.Value + s.yaw*BRC.YawFactor);
					if(CFG.HF[HFlight.CruiseControl] && !VSL.HorizontalSpeed.NeededVector.IsZero()) 
						VSL.HorizontalSpeed.SetNeeded(ForwardDirection * CFG.MaxNavSpeed);
					draw_forward_direction = BRC.DrawForwardDirection;
					VSL.AutopilotDisabled = false;
					DirectionLineTimer.Reset();
					pid.Reset();
					s.yaw = 0;
				}
			}
			//turn ship's nose in the direction of needed velocity
			var axis = VSL.OnPlanetParams.NoseUp? Vector3.up : Vector3.forward;
			var nDir = VSL.LocalDir(DirectionOverride.IsZero()? ForwardDirection : DirectionOverride);
			var angle = Vector3.Angle(cDir, nDir)/180*Mathf.Sign(Vector3.Dot(Vector3.Cross(nDir, cDir), axis));
			var AAf = Utils.Clamp(1/(Mathf.Abs(Vector3.Dot(axis, VSL.Torque.MaxAngularA))), BRC.MinAAf, BRC.MaxAAf);
			var eff = Mathf.Abs(Vector3.Dot(VSL.Engines.MaxThrust.normalized, VSL.Physics.Up));
			pid.P = BRC.DirectionPID.P*AAf;
			pid.D = BRC.DirectionPID.D*AAf*AAf;
			pid.Update(angle);
			var act = pid.Action*eff;
			if(VSL.OnPlanetParams.NoseUp) s.roll = act;
			else s.yaw = act;
			DirectionOverride = Vector3d.zero;
		}

		bool draw_forward_direction;
		static readonly GUIContent X_cnt = new GUIContent("X", "Disable Bearing Control");
		static readonly GUIContent Enable_cnt = new GUIContent("Bearing", "Enable Bearing Control");
		public override void Draw ()
		{
			if(CFG.BR[BearingMode.Auto] || !DirectionOverride.IsZero())
				GUILayout.Label("AutoBearing", Styles.green, GUILayout.ExpandWidth(false));
			else if(CFG.BR[BearingMode.User])
			{
				if(draw_forward_direction)
				{
					GLUtils.GLVec(VSL.Physics.wCoM, ForwardDirection*2500, Color.green);
					draw_forward_direction = !DirectionLineTimer.Check;
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

