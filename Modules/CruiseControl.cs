//   CruiseControl.cs
//
//  Author:
//       Allis Tauri <allista@gmail.com>
//
//  Copyright (c) 2015 Allis Tauri
//
// This work is licensed under the Creative Commons Attribution 4.0 International License. 
// To view a copy of this license, visit http://creativecommons.org/licenses/by/4.0/ 
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

		public CruiseControl(VesselWrapper vsl) { VSL = vsl; }

		public override void Init()
		{
			base.Init();
			UpdateTimer.Period = CC.UpdateDelay;
			pid.setPID(CC.DirectionPID);
			pid.Reset();
			CFG.HF.AddCallback(HFlight.CruiseControl, Enable);
			CFG.HF.AddCallback(HFlight.NoseOnCourse, NoseOnCourse);
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

		public override void Enable(bool enable = true)
		{
			pid.Reset();
			UpdateTimer.Reset();
			if(enable) VSL.UpdateOnPlanetStats();
			BlockSAS(enable);
			VSL.SetNeededHorVelocity(enable? VSL.HorizontalVelocity : Vector3d.zero);
		}

		public void NoseOnCourse(bool enable = true)
		{
			pid.Reset();
			if(enable) VSL.UpdateOnPlanetStats();
			BlockSAS(enable);
		}

		public void UpdateNeededVelocity()
		{
			VSL.SetNeededHorVelocity(CFG.NeededHorVelocity.IsZero()? 
			                         Vector3.zero : 
			                         Quaternion.FromToRotation(CFG.SavedUp, VSL.Up)*CFG.NeededHorVelocity);
		}

		protected override void OnAutopilotUpdate(FlightCtrlState s)
		{
			//need to check all the prerequisites, because the callback is called asynchroniously
			if(!(CFG.Enabled && 
			     CFG.HF.Any(HFlight.Stop, HFlight.NoseOnCourse, HFlight.CruiseControl) && 
			     VSL.OnPlanet && VSL.refT != null && 
			     !VSL.ForwardDirection.IsZero())) return;
			DisableSAS();
			//allow user to intervene
			var cDir = Vector3.ProjectOnPlane(VSL.FwdL, VSL.UpL).normalized;
			if(VSL.AutopilotDisabled) 
			{ 
				pid.Reset();
				VSL.SetNeededHorVelocity(VSL.WorldDir(cDir) * 
				                         Utils.ClampL((float)VSL.NeededHorVelocity.magnitude-s.pitch, 0));
				return; 
			}
			//update needed velocity
			if(CFG.HF[HFlight.CruiseControl])
				UpdateTimer.Run(UpdateNeededVelocity);
			//turn ship's nose in the direction of needed velocity
			var axis = VSL.NoseUp? Vector3.up : Vector3.forward;
			var nDir = VSL.LocalDir(VSL.ForwardDirection);
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

