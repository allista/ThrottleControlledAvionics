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
	[OverrideModules(typeof(BearingControl),
	                 typeof(CruiseControl))]
	public class VTOLControl : AttitudeControlBase
	{
		public new class Config : ModuleConfig
		{
			[Persistent] public float MaxAngle = 45f;
		}
		static Config VTOL { get { return TCAScenario.Globals.VTOL; } }

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
		{ IsActive = CFG.Enabled && VSL.OnPlanet && CFG.CTRL[ControlMode.VTOL] && VSL.refT != null; }

		void set_steering(Quaternion rot)
		{ 
			rot *= world2local_rotation(Quaternion.FromToRotation(-VSL.Physics.Up, VSL.Engines.MaxThrust));
			#if DEBUG
			needed_thrust = VSL.WorldDir(rot.Inverse() * VSL.LocalDir(VSL.Engines.MaxThrust));
			#endif
			steering = rotation2steering(rot); 
		}

		void level()
		{ 
			#if DEBUG
			needed_thrust = -VSL.Physics.Up;
			#endif
			steering = rotation2steering(world2local_rotation(Quaternion.FromToRotation(-VSL.Physics.Up, VSL.Engines.MaxThrust))); 
		}

		protected override void OnAutopilotUpdate(FlightCtrlState s)
		{
			UpdateState();
			if(!IsActive) return;
			VSL.Controls.GimbalLimit = 100;
			if(VSL.HasUserInput) 
			{ 
				Quaternion rot = Quaternion.identity;
				var angle = VTOL.MaxAngle*TWR_factor;
				if(!s.yaw.Equals(0))
				{
					rot = Quaternion.AngleAxis(s.yaw*90, VSL.OnPlanetParams.NoseUp? Vector3.up : Vector3.forward) * rot;
					if(s.roll.Equals(0) && !s.pitch.Equals(0))
						rot = Quaternion.AngleAxis(s.yaw*90, VSL.OnPlanetParams.NoseUp? Vector3.forward : Vector3.up) * rot;
					s.yaw = 0;
				}
				if(!s.pitch.Equals(0)) 
				{
					rot = Quaternion.AngleAxis(s.pitch*angle, Vector3.right) * rot;
					s.pitch = 0;
				}
				if(!s.roll.Equals(0)) 
				{
					rot = Quaternion.AngleAxis(s.roll*angle, VSL.OnPlanetParams.NoseUp? Vector3.forward : Vector3.up) * rot;
					s.roll = 0;
				}
				VSL.AutopilotDisabled = true;
				set_steering(rot);
				Steer(s);
			}
			else if(!VSL.LandedOrSplashed && !CFG.AT) { level(); Steer(s); }
		}

		#if DEBUG
		Vector3 needed_thrust;

		public override void Draw()
		{
			base.Draw();
			if(!IsActive || CFG.CTRL.Not(ControlMode.VTOL)) return;
			if(!VSL.Engines.MaxThrust.IsZero())
				GLUtils.GLVec(VSL.Physics.wCoM, VSL.Engines.MaxThrust.normalized*20, Color.red);
			if(!needed_thrust.IsZero())
				GLUtils.GLVec(VSL.Physics.wCoM, needed_thrust.normalized*20, Color.yellow);
		}
		#endif
	}
}

