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
	[RequireModules(typeof(SASBlocker))]
    [OverrideModules(typeof(AttitudeControl))]
	public class BearingControl : AutopilotModule
	{
		public class Config : ModuleConfig
		{
			new public const string NODE_NAME = "BRC";

			[Persistent] public float MinAAf = 0.001f;
			[Persistent] public float MaxAAf = 2;
			[Persistent] public PID_Controller DirectionPID = new PID_Controller(0.5f, 0f, 0.5f, -1, 1);
		}
		static Config BRC { get { return TCAScenario.Globals.BRC; } }

		readonly PIDf_Controller pid = new PIDf_Controller();

		public Vector3d ForwardDirection;

		public BearingControl(ModuleTCA tca) : base(tca) {}

		public override void Init()
		{
			base.Init();
			pid.setPID(BRC.DirectionPID);
			pid.Reset();
			CFG.HF.AddHandler(this, HFlight.NoseOnCourse);
		}

		protected override void UpdateState() 
		{ IsActive = VSL.OnPlanet && CFG.HF.Any(HFlight.Stop, HFlight.NoseOnCourse, HFlight.CruiseControl); }

		public void NoseOnCourseCallback(Multiplexer.Command cmd)
		{
			pid.Reset();
			switch(cmd)
			{
			case Multiplexer.Command.Resume:
			case Multiplexer.Command.On:
				RegisterTo<SASBlocker>();
				RegisterTo<Radar>(vsl => vsl.HorizontalSpeed.MoovingFast);
				break;

			case Multiplexer.Command.Off:
				UnregisterFrom<SASBlocker>();
				UnregisterFrom<Radar>();
				break;
			}
		}

		protected override void OnAutopilotUpdate(FlightCtrlState s)
		{
			//need to check all the prerequisites, because the callback is called asynchroniously
			if(!(CFG.Enabled && 
			     CFG.HF.Any(HFlight.Stop, HFlight.NoseOnCourse, HFlight.CruiseControl) && 
			     VSL.OnPlanet && VSL.refT != null && 
			     !ForwardDirection.IsZero())) return;
			DisableSAS();
			//allow user to intervene
			var cDir = Vector3.ProjectOnPlane(VSL.OnPlanetParams.FwdL, VSL.Physics.UpL).normalized;
			if(VSL.AutopilotDisabled) pid.Reset();
			//turn ship's nose in the direction of needed velocity
			var axis = VSL.OnPlanetParams.NoseUp? Vector3.up : Vector3.forward;
			var nDir = VSL.LocalDir(ForwardDirection);
			var angle = Vector3.Angle(cDir, nDir)/180*Mathf.Sign(Vector3.Dot(Vector3.Cross(nDir, cDir), axis));
			var AAf = Utils.Clamp(1/(Mathf.Abs(Vector3.Dot(axis, VSL.Torque.MaxAngularA))), BRC.MinAAf, BRC.MaxAAf);
			var eff = Mathf.Abs(Vector3.Dot(VSL.Engines.MaxThrust.normalized, VSL.Physics.Up));
			pid.P = BRC.DirectionPID.P*AAf;
			pid.D = BRC.DirectionPID.D*AAf*AAf;
			pid.Update(angle);
			var act = pid.Action*eff;
			if(VSL.OnPlanetParams.NoseUp) s.roll = s.rollTrim = act;
			else s.yaw = s.yawTrim = act;
		}
	}
}

