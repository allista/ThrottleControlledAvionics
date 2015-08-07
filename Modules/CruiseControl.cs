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
using System.Collections.Generic;
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
		static Config CC { get { return TCAConfiguration.Globals.CC; } }

		readonly PIDf_Controller pid = new PIDf_Controller();
		bool inited;

		public CruiseControl(VesselWrapper vsl) { VSL = vsl; }

		public override void Init()
		{
			base.Init();
			pid.setPID(CC.DirectionPID);
			pid.Reset();
		}

		public override void UpdateState() 
		{ 
			IsActive = CFG.CruiseControl && VSL.OnPlanet; 
			if(!inited && IsActive && !VSL.Up.IsZero())
			{
				CFG.NeededHorVelocity = needed_hor_velocity;
				inited = true;
			}
		}

		Vector3 needed_hor_velocity 
		{ 
			get 
			{ 
				return CFG.Starboard.IsZero()? 
					Vector3.zero : 
					Quaternion.FromToRotation(Vector3.up, VSL.Up) * Vector3.Cross(Vector3.up, CFG.Starboard);
			}
		}

		void set_needed_velocity(bool set = true)
		{ 
			CFG.Starboard = set? VSL.CurrentStarboard : Vector3.zero;
			CFG.NeededHorVelocity = set? VSL.HorizontalVelocity : Vector3d.zero;
		}

		IEnumerator<YieldInstruction> update_needed_velocity()
		{
			if(IsActive) CFG.NeededHorVelocity = needed_hor_velocity;
			yield return new WaitForSeconds(CC.Delay);
		}

		public override void Enable(bool enable = true)
		{
			CFG.CruiseControl = enable;
			pid.Reset();
			if(CFG.CruiseControl) 
			{
				CFG.KillHorVel = false;
				CFG.GoToTarget = false;
				CFG.FollowPath = false;
				VSL.UpdateHorizontalStats();
			}
			BlockSAS(CFG.CruiseControl);
			set_needed_velocity(CFG.CruiseControl);
		}

		public IEnumerator<YieldInstruction> NeededVelocityUpdater;
		public IEnumerator<YieldInstruction> UpdateNeededVelocity()
		{
			NeededVelocityUpdater = update_needed_velocity();
			return NeededVelocityUpdater;
		}

		protected override void Update(FlightCtrlState s)
		{
			//need to check all the prerequisites, because the callback is called asynchroniously
			if(!(CFG.Enabled && CFG.CruiseControl && VSL.OnPlanet && VSL.refT != null )) return;
			//allow user to intervene
			if(UserIntervening(s)) { pid.Reset(); return; }
			var hDir = Vector3.ProjectOnPlane(VSL.Fwd, VSL.Up).normalized;
			var attitude_error = Quaternion.FromToRotation(hDir, CFG.NeededHorVelocity);
			var angle = Utils.CenterAngle(attitude_error.eulerAngles.z)/180;
			pid.Update(angle);
			if(VSL.NoseUp) s.roll = s.rollTrim = pid.Action;
			else s.yaw = s.yawTrim = pid.Action;
		}
	}
}

