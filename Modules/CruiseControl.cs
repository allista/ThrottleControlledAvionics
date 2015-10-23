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
			VSL.NeededHorVelocity = set? VSL.HorizontalVelocity : Vector3d.zero;
		}

		public override void Enable(bool enable = true)
		{
			pid.Reset();
			UpdateTimer.Reset();
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
			VSL.NeededHorVelocity = CFG.Starboard.IsZero()? 
				Vector3.zero : 
				Quaternion.FromToRotation(Vector3.up, VSL.Up) * Vector3.Cross(Vector3.up, CFG.Starboard);
		}

		protected override void Update(FlightCtrlState s)
		{
			//need to check all the prerequisites, because the callback is called asynchroniously
			if(!(CFG.Enabled && 
			     CFG.HF.Any(HFlight.Stop, HFlight.NoseOnCourse, HFlight.CruiseControl) && 
			     VSL.OnPlanet && VSL.refT != null && 
			     !VSL.ForwardDirection.IsZero())) return;
			VSL.ActionGroups.SetGroup(KSPActionGroup.SAS, false);
			//allow user to intervene
			if(UserIntervening(s)) { pid.Reset(); return; }
			//update needed velocity
			if(CFG.HF[HFlight.CruiseControl])
				UpdateTimer.Run(UpdateNeededVelocity);
			//turn ship's nose in the direction of needed velocity
			var cDir = Vector3.ProjectOnPlane(VSL.FwdL, VSL.UpL).normalized;
			var nDir = VSL.refT.InverseTransformDirection(VSL.ForwardDirection);
			var attitude_error = Quaternion.FromToRotation(cDir, nDir);
			var angle = Utils.CenterAngle(VSL.NoseUp? attitude_error.eulerAngles.y : attitude_error.eulerAngles.z)/180;
			var AAf = Utils.Clamp(1/(Vector3.Scale(VSL.NoseUp? Vector3.up : Vector3.forward, VSL.MaxAngularA).magnitude), CC.MinAAf, CC.MaxAAf);
			var eff = Mathf.Abs(Vector3.Dot(VSL.MaxThrust.normalized, VSL.Up));
			pid.P = CC.DirectionPID.P*AAf;
			pid.D = CC.DirectionPID.D*AAf*AAf;
			pid.Update(angle);
			if(VSL.NoseUp) s.roll = s.rollTrim = -pid.Action*eff;
			else s.yaw = s.yawTrim = -pid.Action*eff;
		}
	}
}

