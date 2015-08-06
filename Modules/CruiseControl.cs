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
			if(!inited && IsActive && !VSL.up.IsZero())
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
					Quaternion.FromToRotation(Vector3.up, VSL.up) * Vector3.Cross(Vector3.up, CFG.Starboard);
			}
		}

		void set_needed_velocity(bool set = true)
		{ 
			CFG.Starboard = set? 
				Quaternion.FromToRotation(VSL.up, Vector3.up)*Vector3d.Cross(VSL.HorizontalVelocity, VSL.up) :
				Vector3.zero;
			CFG.NeededHorVelocity = set? VSL.HorizontalVelocity : Vector3d.zero;
		}

		IEnumerator<YieldInstruction> update_needed_velocity()
		{
			if(IsActive) CFG.NeededHorVelocity = needed_hor_velocity;
			yield return new WaitForSeconds(CC.Delay);
		}

		public void Enable(bool enable = true)
		{
			CFG.CruiseControl = enable;
			pid.Reset();
			BlockSAS(CFG.CruiseControl);
			if(CFG.CruiseControl) VSL.UpdateHorizontalStats();
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
			if(!Mathfx.Approx(s.yaw, s.yawTrim, 0.1f)) { pid.Reset(); return; }
			var hDir = Vector3.ProjectOnPlane(VSL.refT.up, VSL.up).normalized;
			var attitude_error = Quaternion.FromToRotation(hDir, CFG.NeededHorVelocity);
			var angle = Utils.CenterAngle(attitude_error.eulerAngles.z);
			pid.Update(angle/180);
			s.yaw = s.yawTrim = pid.Action;
		}
	}
}

