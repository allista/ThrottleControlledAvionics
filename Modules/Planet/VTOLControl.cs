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
    public class VTOLControl : GeneralAttitudeControl
	{
		public new class Config : ModuleConfig
		{
			[Persistent] public float MaxAngle = 45f;
		}
		static Config VTOL { get { return Globals.Instance.VTOL; } }

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

        public override void Disable() {}

		protected override void UpdateState() 
		{ 
			base.UpdateState();
			IsActive &= VSL.OnPlanet && CFG.CTRL[ControlMode.VTOL] && VSL.refT != null; 
		}

		protected override void correct_steering()
		{
			if(BRC != null && BRC.IsActive)
                steering = Vector3.ProjectOnPlane(steering, VSL.LocalDir(VSL.Engines.refT_thrust_axis));
		}

		protected override void OnAutopilotUpdate()
		{
            var needed_thrust = -VSL.Physics.Up;
            rotation_axis = Vector3.zero;
			if(VSL.HasUserInput) 
			{ 
				var angle = VTOL.MaxAngle*VSL.OnPlanetParams.TWRf;
				var pitch_roll = Mathf.Abs(CS.pitch)+Mathf.Abs(CS.roll);
                if(!CS.pitch.Equals(0)) 
                    needed_thrust = Quaternion.AngleAxis(-Mathf.Abs(CS.pitch)/pitch_roll*CS.pitch*angle, VSL.refT.right) * needed_thrust;
				if(!CS.roll.Equals(0)) 
                    needed_thrust = Quaternion.AngleAxis(-Mathf.Abs(CS.roll)/pitch_roll*CS.roll*angle, VSL.Engines.refT_forward_axis) * needed_thrust;
                compute_rotation(Rotation.Local(VSL.Engines.CurrentDefThrustDir, needed_thrust, VSL));
				if(!CS.yaw.Equals(0))
                {
                    rotation_axis = (rotation_axis*VSL.Controls.AttitudeError/angle-VSL.LocalDir(needed_thrust.normalized*CS.yaw*Mathf.PI/3)).normalized;
                    VSL.Controls.SetAttitudeError(Mathf.Min(VSL.Controls.AttitudeError+Math.Abs(CS.yaw)*30, 180));
                }
				compute_steering();
				VSL.Controls.AddSteering(steering);
				VSL.HasUserInput = false;
				VSL.AutopilotDisabled = true;
				CS.yaw = CS.pitch = CS.roll = 0;
			}
			else if(!(VSL.LandedOrSplashed || CFG.AT))
			{ 
                compute_rotation(Rotation.Local(VSL.Engines.CurrentDefThrustDir, needed_thrust, VSL)); 
				compute_steering();
				VSL.Controls.AddSteering(steering);
			}
		}

		#if DEBUG
		Vector3 needed_thrust;

		public override void Draw()
		{
			base.Draw();
			if(!IsActive || CFG.CTRL.Not(ControlMode.VTOL)) return;
            Utils.GLVec(VSL.refT.position, VSL.Engines.MaxThrust.normalized*20, Color.yellow);
            Utils.GLVec(VSL.refT.position, needed_thrust.normalized*20, Color.red);
            Utils.GLVec(VSL.refT.position, VSL.WorldDir(VSL.vessel.angularVelocity*20), Color.cyan);
            Utils.GLVec(VSL.refT.position, VSL.WorldDir(rotation_axis*25), Color.green);
//			if(!steering.IsZero())
//				Utils.GLVec(VSL.Physics.wCoM, VSL.WorldDir(steering.normalized*20), Color.cyan);
			
//			Utils.GLVec(VSL.Physics.wCoM, VSL.refT.up*3, Color.green);
//			Utils.GLVec(VSL.Physics.wCoM, VSL.refT.forward*3, Color.blue);
//			Utils.GLVec(VSL.Physics.wCoM, VSL.refT.right*3, Color.red);
		}
		#endif
	}
}

