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
	public class CruiseControl : AutopilotModule
	{
		public class Config : ModuleConfig
		{
			new public const string NODE_NAME = "CC";

			[Persistent] public float UpdateDelay = 1;
			[Persistent] public float MinAAf = 0.001f;
			[Persistent] public float MaxAAf = 2;
			[Persistent] public PID_Controller DirectionPID = new PID_Controller(0.5f, 0f, 0.5f, -1, 1);
		}
		static Config CC { get { return TCAScenario.Globals.CC; } }

		readonly PIDf_Controller pid = new PIDf_Controller();
		readonly ActionDamper UpdateTimer = new ActionDamper();
		bool inited;

		public Vector3d ForwardDirection;

		public CruiseControl(ModuleTCA tca) { TCA = tca; }

		public override void Init()
		{
			base.Init();
			UpdateTimer.Period = CC.UpdateDelay;
			pid.setPID(CC.DirectionPID);
			pid.Reset();
			CFG.HF.AddHandler(this, HFlight.CruiseControl, HFlight.NoseOnCourse);
		}

		protected override void UpdateState() 
		{ 
			IsActive = VSL.OnPlanet && CFG.HF.Any(HFlight.Stop, HFlight.NoseOnCourse, HFlight.CruiseControl); 
			if(!inited && IsActive && !VSL.Up.IsZero())
			{
				UpdateNeededVelocity();
				inited = true;
			}
		}

		public void CruiseControlCallback(Multiplexer.Command cmd)
		{
			pid.Reset();
			UpdateTimer.Reset();
			switch(cmd)
			{
			case Multiplexer.Command.Resume:
				TCA.SASC.Register(this);
				TCA.RAD.Register(this);
				break;

			case Multiplexer.Command.On:
				VSL.UpdateOnPlanetStats();
				TCA.HSC.SetNeededHorVelocity(VSL.HorizontalVelocity);
				goto case Multiplexer.Command.Resume;

			case Multiplexer.Command.Off:
				TCA.HSC.SetNeededHorVelocity(Vector3d.zero);
				TCA.SASC.Unregister(this);
				TCA.RAD.Unregister(this);
				break;
			}
		}

		public void NoseOnCourseCallback(Multiplexer.Command cmd)
		{
			pid.Reset();
			switch(cmd)
			{
			case Multiplexer.Command.Resume:
			case Multiplexer.Command.On:
				TCA.SASC.Register(this);
				TCA.RAD.Register(this, vsl => vsl.TCA.RAD.MoovingFast);
				break;

			case Multiplexer.Command.Off:
				TCA.SASC.Unregister(this);
				TCA.RAD.Unregister(this);
				break;
			}
		}

		public void UpdateNeededVelocity()
		{
			TCA.HSC.SetNeededHorVelocity(CFG.NeededHorVelocity.IsZero()? 
			                         Vector3.zero : 
			                         Quaternion.FromToRotation(CFG.SavedUp, VSL.Up)*CFG.NeededHorVelocity);
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
			var cDir = Vector3.ProjectOnPlane(VSL.FwdL, VSL.UpL).normalized;
			if(VSL.AutopilotDisabled) 
			{ 
				pid.Reset();
				TCA.HSC.SetNeededHorVelocity(VSL.WorldDir(cDir) * 
				                             Utils.ClampL((float)TCA.HSC.NeededHorVelocity.magnitude-s.pitch, 0));
				return; 
			}
			//update needed velocity
			if(CFG.HF[HFlight.CruiseControl])
				UpdateTimer.Run(UpdateNeededVelocity);
			//turn ship's nose in the direction of needed velocity
			var axis = VSL.NoseUp? Vector3.up : Vector3.forward;
			var nDir = VSL.LocalDir(ForwardDirection);
			var angle = Vector3.Angle(cDir, nDir)/180*Mathf.Sign(Vector3.Dot(Vector3.Cross(nDir, cDir), axis));
			var AAf = Utils.Clamp(1/(Mathf.Abs(Vector3.Dot(axis, VSL.MaxAngularA))), CC.MinAAf, CC.MaxAAf);
			var eff = Mathf.Abs(Vector3.Dot(VSL.MaxThrust.normalized, VSL.Up));
			pid.P = CC.DirectionPID.P*AAf;
			pid.D = CC.DirectionPID.D*AAf*AAf;
			pid.Update(angle);
			var act = pid.Action*eff;
			if(VSL.NoseUp) s.roll = s.rollTrim = act;
			else s.yaw = s.yawTrim = act;

//			CSV(angle, AAf, AAf*AAf, eff, pid.Action, act);//debug
		}
	}
}

