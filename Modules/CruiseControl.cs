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

			[Persistent] public float Delay = 1;
			[Persistent] public PID_Controller DirectionPID = new PID_Controller(0.5f, 0f, 0.5f, -1, 1);
//			[Persistent] public float AeroCoefficient = 1;
		}
		static Config CC { get { return TCAScenario.Globals.CC; } }

		readonly PIDf_Controller pid = new PIDf_Controller();
		bool inited;

		public CruiseControl(VesselWrapper vsl) { VSL = vsl; }

		public override void Init()
		{
			base.Init();
			pid.setPID(CC.DirectionPID);
			pid.Reset();
			CFG.HF.AddCallback(HFlight.CruiseControl, Enable);
			CFG.HF.AddCallback(HFlight.NoseOnCourse, NoseOnCourse);
		}

		public override void UpdateState() 
		{ 
			IsActive = CFG.HF[HFlight.CruiseControl] && VSL.OnPlanet; 
			if(!inited && IsActive && !VSL.Up.IsZero())
			{
				UpdateNeededVelocity();
				inited = true;
			}
		}

		void set_needed_velocity(bool set = true)
		{ 
			CFG.Starboard = set? VSL.CurrentStarboard : Vector3.zero;
			CFG.NeededHorVelocity = set? VSL.HorizontalVelocity : Vector3d.zero;
		}

		public override void Enable(bool enable = true)
		{
			pid.Reset();
			if(enable) 
			{
				CFG.Nav.Off();
				VSL.UpdateOnPlanetStats();
			}
			BlockSAS(enable);
			set_needed_velocity(enable);
		}

		public void NoseOnCourse(bool enable = true)
		{
			pid.Reset();
			if(enable) VSL.UpdateOnPlanetStats();
			BlockSAS(enable);
		}

		public void UpdateNeededVelocity()
		{
			CFG.NeededHorVelocity = CFG.Starboard.IsZero()? 
										Vector3.zero : 
										Quaternion.FromToRotation(Vector3.up, VSL.Up) * Vector3.Cross(Vector3.up, CFG.Starboard);
		}

		protected override void Update(FlightCtrlState s)
		{
			//need to check all the prerequisites, because the callback is called asynchroniously
			if(!(CFG.Enabled && 
			     (CFG.HF[HFlight.NoseOnCourse] || CFG.HF[HFlight.CruiseControl]) && 
			     VSL.OnPlanet && VSL.refT != null )) return;
			VSL.ActionGroups.SetGroup(KSPActionGroup.SAS, false);
			//allow user to intervene
			if(UserIntervening(s)) { pid.Reset(); return; }
			var hDir = VSL.refT.InverseTransformDirection(Vector3.ProjectOnPlane(VSL.Fwd, VSL.Up).normalized);
			var lHv  = VSL.refT.InverseTransformDirection(CFG.NeededHorVelocity);
			var attitude_error = Quaternion.FromToRotation(hDir, lHv);
			var angle = Utils.CenterAngle(VSL.NoseUp? attitude_error.eulerAngles.y : attitude_error.eulerAngles.z)/180;
			var AAf = 1/(VSL.NoseUp? VSL.MaxAngularA.y : VSL.MaxAngularA.z);
			var eff = Mathf.Clamp01(1-Mathf.Abs(Vector3.Dot(VSL.Fwd, VSL.Up)));
			pid.P = CC.DirectionPID.P*AAf;
			pid.D = CC.DirectionPID.D*AAf;
			pid.Update(angle);
			if(VSL.NoseUp) s.roll = s.rollTrim = -pid.Action*eff;
			else s.yaw = s.yawTrim = -pid.Action*eff;
//			DebugUtils.logVectors("NoseOnCourse", Vector3.right, Vector3.up, 
//			                      VSL.refT.InverseTransformDirection(VSL.Fwd), VSL.refT.InverseTransformDirection(VSL.Up), 
//			                      hDir, lHv);
//			DebugUtils.CSV(angle*180, pid.Action, AAf, VSL.NoseUp);//debug
		}
	}
}

