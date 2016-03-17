//  Author:
//       Allis Tauri <allista@gmail.com>
//
//  Copyright (c) 2016 Allis Tauri
//
// This work is licensed under the Creative Commons Attribution-ShareAlike 4.0 International License. 
// To view a copy of this license, visit http://creativecommons.org/licenses/by-sa/4.0/ 
// or send a letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.
//

namespace ThrottleControlledAvionics
{
	public abstract class LandingTrajectoryAutopilot : TargetedTrajectoryCalculator<LandingTrajectory, WayPoint>
	{
		public new class Config : TCAModule.ModuleConfig
		{
			[Persistent] public float Dtol               = 1000f; //m
			[Persistent] public float FlyOverAlt         = 1000;  //m
			[Persistent] public float ApproachAlt        = 250;   //m
			[Persistent] public float BrakeEndSpeed      = 100;   //m/s
			[Persistent] public float CorrectionOffset   = 20f;   //s
			[Persistent] public float CorrectionTimer    = 10f;   //s
		}
		protected static Config LTRJ { get { return TCAScenario.Globals.LTRJ; } }

		protected LandingTrajectoryAutopilot(ModuleTCA tca) : base(tca) {}

		enum LandingStage { None, Start, Decelerate, Approach, Land }
		LandingStage landing_stage;

		protected double TargetAltitude { get { return Target.SurfaceAlt(Body)+LTRJ.FlyOverAlt; } }

		public override void Init()
		{
			base.Init();
			Dtol = LTRJ.Dtol;
			CorrectionTimer.Period = LTRJ.CorrectionTimer;
		}

		protected override void reset()
		{
			base.reset();
			landing_stage = LandingStage.None;
		}

		protected override void setup_target()
		{
			Target = Target2WP();
			Target.UpdateCoordinates(Body);
			SetTarget(Target);
		}

		protected bool landing { get { return landing_stage != LandingStage.None; } }

		protected void start_landing()
		{
			if(trajectory != null) trajectory.UpdateOrbit(VesselOrbit);
			else trajectory = new LandingTrajectory(VSL, Vector3d.zero, VSL.Physics.UT, Target, TargetAltitude);
			ManeuverAutopilot.AddNode(VSL, trajectory.BrakeDeltaV-trajectory.BrakeDeltaV.normalized*LTRJ.BrakeEndSpeed, 
			                          trajectory.BrakeStartUT);
			landing_stage = LandingStage.Decelerate;
			CFG.AP1.On(Autopilot1.Maneuver);
			MAN.MinDeltaV = 1f;
		}

		protected bool do_land()
		{
			switch(landing_stage)
			{
			case LandingStage.Decelerate:
				if(VSL.VerticalSpeed.Absolute < 0 &&
				   VSL.Altitude.Relative < LTRJ.FlyOverAlt)
				{ CFG.AP1.Off(); clear_nodes(); }
				if(CFG.AP1[Autopilot1.Maneuver]) break;
				CFG.AltitudeAboveTerrain = true;
				CFG.BlockThrottle = true;
				CFG.VF.On(VFlight.AltitudeControl);
				CFG.HF.On(HFlight.Level);
				CFG.DesiredAltitude = LTRJ.ApproachAlt;
				landing_stage = LandingStage.Approach;
				break;
			case LandingStage.Approach:
				CFG.Nav.OnIfNot(Navigation.GoToTarget);
				if(CFG.Nav[Navigation.GoToTarget]) break;
				CFG.AP1.OnIfNot(Autopilot1.Land);
				landing_stage = LandingStage.Land;
				break;
			case LandingStage.Land:
				if(CFG.AP1[Autopilot1.Land]) break;
				CFG.AP2.Off();
				return true;
			}
			return false;
		}
	}
}

