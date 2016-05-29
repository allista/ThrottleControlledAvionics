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
			[Persistent] public float LowPassF           = 1f;    //s
			[Persistent] public float ObstacleBrakeF     = 1.1f;
		}
		protected static Config LTRJ { get { return TCAScenario.Globals.LTRJ; } }

		protected LandingTrajectoryAutopilot(ModuleTCA tca) : base(tca) {}

		enum LandingStage { None, Start, Wait, Decelerate, Coast, HardLanding, Approach, Land }
		LandingStage landing_stage;

		protected LowPassFilterD DistanceFilter = new LowPassFilterD();
		protected Timer DecelerationTimer = new Timer(0.5);
		protected bool FullStop;
		protected AtmoSim sim;

		protected AttitudeControl ATC;
		protected ThrottleControl THR;
		protected BearingControl  BRC;

		protected double TargetAltitude { get { return Target.SurfaceAlt(Body); } }

		public override void Init()
		{
			base.Init();
			Dtol = LTRJ.Dtol;
			CorrectionTimer.Period = LTRJ.CorrectionTimer;
			DistanceFilter.Tau = LTRJ.LowPassF;
		}

		protected override void reset()
		{
			base.reset();
			landing_stage = LandingStage.None;
			DistanceFilter.Reset();
			DecelerationTimer.Reset();
			FullStop = false;
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
			Status("Preparing for final deceleration...");
			sim = new AtmoSim(Body, VSL);
			if(trajectory != null) trajectory.UpdateOrbit(VesselOrbit, false);
			else trajectory = new LandingTrajectory(VSL, Vector3d.zero, VSL.Physics.UT, Target, TargetAltitude);
			landing_stage = LandingStage.Wait;
		}

		protected bool correct_trajectory()
		{
			if(!CorrectionTimer.Check) 
			{
				if(TimeWarp.WarpMode == TimeWarp.Modes.HIGH)
					WRP.WarpToTime = VSL.Physics.UT+60;
				else WRP.StopWarp();
				return false;
			}
			CorrectionTimer.Reset();
			trajectory.UpdateOrbit(VesselOrbit);
			if(trajectory.DistanceToTarget >= LTRJ.Dtol)
			{ 
				WRP.StopWarp(); 
				fine_tune_approach(); 
				return false; 
			}
			return !Body.atmosphere;
		}

		double distance_from_ground(Orbit orb, double UT)
		{
			var pos = orb.getRelativePositionAtUT(UT);
			return pos.magnitude-VSL.Geometry.H-Body.Radius - Utils.TerrainAltitude(Body, pos.xzy+Body.position);
		}

		protected double obstacle_ahead(LandingTrajectory trj)
		{
			if(trj == null) return -1;
			var start = trj.StartUT;
			var stop = start + (trj.NewOrbit.timeToAp < trj.NewOrbit.period/2? 
			                    trj.NewOrbit.timeToAp : Math.Min(trj.BrakeDuration*LTRJ.ObstacleBrakeF,
			                                                     trj.BrakeDuration+trj.BrakeStartUT-VSL.Physics.UT-5));
			var UT = start;
			var dT = (stop-start);
			double dist;
			while(dT > 0.01)
			{
				var d1p = UT+dT > stop?  double.MaxValue : distance_from_ground(trj.NewOrbit, UT+dT);
				var d1m = UT-dT < start? double.MaxValue : distance_from_ground(trj.NewOrbit, UT-dT);
				if(d1p < d1m) { dist = d1p; UT += dT; }
				else { dist = d1m; UT -= dT; }
				if(dist < 0) return -dist;
				dT /= 2;
			}
			return -1;
		}

		void approach()
		{
			CFG.BR.Off();
			CFG.BlockThrottle = true;
			CFG.AltitudeAboveTerrain = true;
			CFG.VF.On(VFlight.AltitudeControl);
			CFG.DesiredAltitude = LTRJ.ApproachAlt < VSL.Altitude.Relative? 
				LTRJ.ApproachAlt : Utils.ClampL(VSL.Altitude.Relative/2, VSL.Geometry.H*2);
			landing_stage = LandingStage.Approach;
		}

		void decelerate()
		{
			WRP.StopWarp();
			DistanceFilter.Set(trajectory.DistanceToTarget);
			DecelerationTimer.Reset();
			landing_stage = LandingStage.Decelerate; 
			CFG.AltitudeAboveTerrain = true;
		}

		Vector3d BrakeDirection(Vector3d velocity, Vector3d target_position, double error_factor = 100)
		{ return velocity-Vector3d.Exclude(VSL.Physics.Up, target_position-trajectory.AtTargetPos.xzy)/error_factor; }

		protected bool do_land()
		{
			if(VSL.LandedOrSplashed) { CFG.AP2.Off(); return true; }
			VSL.Engines.ActivateEnginesIfNeeded();
			VSL.Engines.ActivateNextStageOnFlameout();
			if(VSL.Engines.MaxThrustM.Equals(0) && !VSL.Engines.HaveNextStageEngines) 
				landing_stage = LandingStage.HardLanding;
			double terminal_velocity;
			switch(landing_stage)
			{
			case LandingStage.Wait:
				trajectory.UpdateOrbit(VesselOrbit, false);
				VSL.Info.Countdown = trajectory.BrakeStartUT-VSL.Physics.UT-Body.atmDensityASL-1;
				if(VSL.Info.Countdown <= 0) { FullStop = false; decelerate(); }
				if(obstacle_ahead(trajectory) > 0) { FullStop = true; decelerate(); break; }
				VSL.Info.TTB = (float)trajectory.BrakeDuration;
				CFG.AT.OnIfNot(Attitude.Custom);
				ATC.SetThrustDirW(BrakeDirection(VesselOrbit.vel.xzy, Target.WorldPos(Body), 
				                                 Utils.ClampL(VSL.Info.Countdown*5, 100)));
				if(ATC.Aligned) WRP.WarpToTime = VSL.Physics.UT+VSL.Info.Countdown;
				else WRP.StopWarp();
				break;
			case LandingStage.Decelerate:
				var last_distance = DistanceFilter.Value;
				trajectory.UpdateOrbit(VesselOrbit, false);
				DistanceFilter.Update(trajectory.DistanceToTarget);
				if(FullStop) Status("red", "Possible collision detected.\nPerforming full deceleration.");
				else Status("Decelerating. Landing site error: {0:F1}m", DistanceFilter.Value);
				var TargetPos = Target.WorldPos(Body);
				var radial_speed = Vector3d.Dot(VSL.HorizontalSpeed.Vector, TargetPos-VSL.Physics.wCoM);
				if(last_distance-DistanceFilter > 0.01 || radial_speed < 0) DecelerationTimer.Reset();
				if(VSL.Controls.HaveControlAuthority &&
				   (FullStop || !DecelerationTimer.Check && DistanceFilter > LTRJ.Dtol) &&
				   VSL.HorizontalSpeed.Absolute > LTRJ.BrakeEndSpeed)
				{
					CFG.AT.OnIfNot(Attitude.Custom);
					ATC.SetThrustDirW(BrakeDirection(VSL.vessel.srf_velocity, TargetPos));
					THR.DeltaV = Utils.ClampL((float)VSL.vessel.srfSpeed-LTRJ.BrakeEndSpeed+1, 0);
					VSL.BrakesOn();
					break;
				}
				landing_stage = LandingStage.Coast;
				VSL.BrakesOn(false);
				THR.Throttle = 0;
				break;
			case LandingStage.Coast:
				CFG.HF.OnIfNot(HFlight.Level);
				CFG.BR.OnIfNot(BearingMode.Auto);
				trajectory.UpdateOrbit(VesselOrbit, false);
				BRC.ForwardDirection = Vector3d.Exclude(VSL.Physics.Up, Target.WorldPos(Body)-VSL.Physics.wCoM);
				Status("Coasting. Landing site error: {0:F1} m", trajectory.DistanceToTarget);
				var tts = sim.FreeFallTime(out terminal_velocity);
				VSL.Info.Countdown = tts-VSL.Engines.TTB((float)terminal_velocity)-1;
				if(VSL.Info.Countdown <= 0 || 
				   VSL.Altitude.Relative < 2*(2-ATC.AttitudeFactor)*LTRJ.FlyOverAlt)
				{
					if(VSL.OnPlanetParams.MaxTWR < 1 || 
					   VSL.Engines.MaxDeltaV < terminal_velocity*10 ||
					   !VSL.Controls.HaveControlAuthority && 
					   VSL.Torque.AngularAcceleration(VSL.Torque.MaxTorquePossible).magnitude < Utils.TwoPI)
						landing_stage = LandingStage.HardLanding;
					else approach();
				}
				break;
			case LandingStage.HardLanding:
				CFG.VTOLAssistON = true;
				CFG.AT.OnIfNot(Attitude.Custom);
				ATC.SetThrustDirW(VSL.vessel.srf_velocity);
				VSL.Info.Countdown = sim.FreeFallTime(out terminal_velocity);
				VSL.BrakesOn();
				if(VSL.Engines.MaxThrustM > 0 && VSL.Controls.HaveControlAuthority)
				{
					VSL.Info.Countdown -= VSL.Engines.MaxDeltaV > terminal_velocity? 
						VSL.Engines.TTB((float)terminal_velocity) : VSL.Engines.SuicidalBurnTime;
					if(VSL.Info.Countdown < 0 && 
					   (!VSL.OnPlanetParams.HaveParachutes || 
					    VSL.OnPlanetParams.ParachutesActive && VSL.OnPlanetParams.ParachutesDeployed))
					{
						CFG.BlockThrottle = true;
						CFG.AltitudeAboveTerrain = true;
						CFG.VF.OffIfOn(VFlight.AltitudeControl);
						CFG.VerticalCutoff = -0.5f;
					}
					else THR.Throttle = 0;
					Status("yellow", "Unable to perform full landing sequence.\nWill deceletate just before landing...");
				}
				if(Body.atmosphere && VSL.OnPlanetParams.HaveUsableParachutes)
				{
					VSL.OnPlanetParams.ActivateParachutes();
					if(CFG.AutoParachutes) Status("yellow", "Waiting for safe speed to deploy parachutes...");
					else Status("red", "Automatic parachute deployment is disabled.\nActivate parachutes manually when needed.");
				}
				if(!VSL.OnPlanetParams.HaveParachutes && 
				   !VSL.Engines.HaveNextStageEngines && 
				   (VSL.Engines.MaxThrustM.Equals(0) || !VSL.Controls.HaveControlAuthority))
					Status("red", "Collision is imminent.\nImpact speed: {0:F1} m/s", terminal_velocity);
				break;
			case LandingStage.Approach:
				Status("Approaching the target...");
				CFG.Nav.OnIfNot(Navigation.GoToTarget);
				if(CFG.Nav[Navigation.GoToTarget]) break;
				CFG.AP1.OnIfNot(Autopilot1.Land);
				landing_stage = LandingStage.Land;
				break;
			case LandingStage.Land:
				Status("Landing on the nearest flat surface...");
				if(CFG.AP1[Autopilot1.Land]) break;
				break;
			}
			return false;
		}
	}
}

