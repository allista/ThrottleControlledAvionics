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
	[CareerPart]
	[RequireModules(typeof(SASBlocker))]
	[OverrideModules(typeof(BearingControl),
	                 typeof(CruiseControl))]
	public class VTOLControl : AttitudeControlBase
	{
		public new class Config : ModuleConfig
		{
			[Persistent] public float MaxAngle = 45f;
		}
		static Config VTOL { get { return Globals.Instance.VTOL; } }

		BearingControl BRC;

		public VTOLControl(ModuleTCA tca) : base(tca) {}

		public override void Init()
		{
			base.Init();
			CFG.CTRL.AddHandler(this, ControlMode.VTOL);
		}

		public void VTOLCallback(Multiplexer.Command cmd)
		{
			switch(cmd)
			{
			case Multiplexer.Command.Resume:
				RegisterTo<SASBlocker>(vsl => vsl.OnPlanet);
				break;

			case Multiplexer.Command.On:
				VSL.UpdateOnPlanetStats();
				goto case Multiplexer.Command.Resume;

			case Multiplexer.Command.Off:
				UnregisterFrom<SASBlocker>();
				break;
			}
		}

		protected override void UpdateState() 
		{ 
			base.UpdateState();
			IsActive &= VSL.OnPlanet && CFG.CTRL[ControlMode.VTOL] && VSL.refT != null; 
		}

		protected override void correct_steering()
		{
			if(BRC != null && BRC.IsActive)
				steering = Vector3.ProjectOnPlane(steering, VSL.LocalDir(VSL.Engines.CurrentThrustDir));
		}

		protected override void OnAutopilotUpdate(FlightCtrlState s)
		{
			if(!IsActive) return;
			if(VSL.HasUserInput) 
			{ 
				Quaternion rot = Quaternion.identity;
				var angle = VTOL.MaxAngle*VSL.OnPlanetParams.TWRf;
				var pitch_roll = Mathf.Abs(s.pitch)+Mathf.Abs(s.roll);
				if(!s.pitch.Equals(0)) 
					rot = Quaternion.AngleAxis(Mathf.Abs(s.pitch)/pitch_roll*s.pitch*angle, VSL.Controls.Transform.right) * rot;
				if(!s.roll.Equals(0)) 
					rot = Quaternion.AngleAxis(Mathf.Abs(s.roll)/pitch_roll*s.roll*angle, fwd_axis) * rot;
				s.pitch = s.roll = 0;
				VSL.HasUserInput = false;
				VSL.AutopilotDisabled = true;
				rot *= Quaternion.FromToRotation(-VSL.Physics.Up, VSL.Engines.CurrentThrustDir);
				#if DEBUG
				needed_thrust = rot.Inverse() * VSL.Engines.CurrentThrustDir;
				#endif
				steering = rotation2steering(world2local_rotation(rot));
				if(!s.yaw.Equals(0))
					steering += rotation2steering(world2local_rotation(Quaternion.AngleAxis(s.yaw*60, VSL.Engines.CurrentThrustDir)));
				s.yaw = 0;
				VSL.Controls.SetAttitudeError(steering.magnitude*Mathf.Rad2Deg);
				tune_steering();
				VSL.Controls.GimbalLimit = 0;
				VSL.Controls.AddSteering(steering);
			}
			else if(!(VSL.LandedOrSplashed || CFG.AT))
			{ 
				#if DEBUG
				needed_thrust = -VSL.Physics.Up;
				#endif
				compute_steering(Rotation.Local(VSL.Engines.CurrentThrustDir, -VSL.Physics.Up, VSL)); 
				tune_steering();
				VSL.Controls.AddSteering(steering);
			}

		}

		#if DEBUG
		Vector3 needed_thrust;

		public override void Draw()
		{
			base.Draw();
			if(!IsActive || CFG.CTRL.Not(ControlMode.VTOL)) return;
			if(!VSL.Engines.MaxThrust.IsZero())
				Utils.GLVec(VSL.Physics.wCoM, VSL.Engines.MaxThrust.normalized*20, Color.red);
			if(!needed_thrust.IsZero())
				Utils.GLVec(VSL.Physics.wCoM, needed_thrust.normalized*20, Color.yellow);
			if(!steering.IsZero())
				Utils.GLVec(VSL.Physics.wCoM, VSL.WorldDir(steering.normalized*20), Color.cyan);
			
			Utils.GLVec(VSL.Physics.wCoM, VSL.Controls.Transform.up*3, Color.green);
			Utils.GLVec(VSL.Physics.wCoM, VSL.Controls.Transform.forward*3, Color.blue);
			Utils.GLVec(VSL.Physics.wCoM, VSL.Controls.Transform.right*3, Color.red);
		}
		#endif
	}
}

