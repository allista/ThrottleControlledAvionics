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
	public abstract class LandingTrajectoryAutopilot : TargetedTrajectoryCalculator<LandingTrajectory>
	{
		public new class Config : TCAModule.ModuleConfig
		{
			[Persistent] public float Dtol               = 1000f; //m
			[Persistent] public float FlyOverAlt         = 1000;  //m
			[Persistent] public float ApproachAlt        = 250;   //m
			[Persistent] public float BrakeEndSpeed      = 100;   //m/s
			[Persistent] public float MinBrakeOffset     = 10;    //m/s
			[Persistent] public float FinalBrakeOffset   = 5;     //m/s
			[Persistent] public float CorrectionOffset   = 20f;   //s
			[Persistent] public float CorrectionTimer    = 10f;   //s
			[Persistent] public float CorrectionMinDv    = 0.5f;  //m/s
			[Persistent] public float ObstacleBrakeF     = 1.1f;
			[Persistent] public float HoverTimeThreshold = 60f;
		}
		protected static Config LTRJ { get { return TCAScenario.Globals.LTRJ; } }

		protected LandingTrajectoryAutopilot(ModuleTCA tca) : base(tca) {}

		public enum LandingStage { None, Start, Wait, Decelerate, Coast, HardLanding, SoftLanding, Approach, Land }
		[Persistent] public LandingStage landing_stage;

		protected Timer DecelerationTimer = new Timer(0.5);
		protected ManeuverExecutor Executor;
		protected AtmoSim sim;
		protected bool FullStop;

		protected AttitudeControl ATC;
		protected ThrottleControl THR;
		protected BearingControl  BRC;

		protected double TargetAltitude { get { return CFG.Target.SurfaceAlt(Body); } }

		protected override LandingTrajectory CurrentTrajectory
		{ get { return new LandingTrajectory(VSL, Vector3d.zero, VSL.Physics.UT, CFG.Target, TargetAltitude, false); } }

		public override void Init()
		{
			base.Init();
			Dtol = LTRJ.Dtol;
			CorrectionTimer.Period = LTRJ.CorrectionTimer;
			sim = new AtmoSim(Body, VSL);
			Executor = new ManeuverExecutor(TCA);
		}

		protected override void reset()
		{
			base.reset();
			landing_stage = LandingStage.None;
			DecelerationTimer.Reset();
			FullStop = false;
		}

		protected bool landing { get { return landing_stage != LandingStage.None; } }

		protected void start_landing()
		{
			update_trajectory();
			VSL.Controls.StopWarp();
			VSL.Controls.Aligned = false;
			landing_stage = LandingStage.Wait;
		}

		protected bool correct_trajectory()
		{
			if(!CorrectionTimer.Check) 
			{
				if(TimeWarp.WarpMode == TimeWarp.Modes.HIGH)
					VSL.Controls.WarpToTime = VSL.Physics.UT+60;
				else VSL.Controls.StopWarp();
				return false;
			}
			CorrectionTimer.Reset();
			trajectory.UpdateOrbit(VesselOrbit, true);
			if(trajectory.DistanceToTarget >= LTRJ.Dtol)
			{ 
				fine_tune_approach(); 
				return false; 
			}
			return !Body.atmosphere;
		}

		protected void add_correction_node_if_needed()
		{
			var dV_threshold = Utils.ClampL(LTRJ.CorrectionMinDv * (1-CFG.Target.AngleTo(VSL)/Math.PI), 
			                                GLB.THR.MinDeltaV*2);
			if(trajectory.ManeuverDeltaV.magnitude > dV_threshold)
			{
				clear_nodes(); 
				add_trajectory_node();
				CFG.AP1.OnIfNot(Autopilot1.Maneuver);
				VSL.Controls.StopWarp();
			}
		}

		double distance_from_ground(Orbit orb, double UT)
		{
			var pos = orb.getRelativePositionAtUT(UT);
			return pos.magnitude-VSL.Geometry.H*2-Body.Radius - Utils.TerrainAltitude(Body, pos.xzy+Body.position);
		}

		protected double obstacle_ahead(LandingTrajectory trj)
		{
			if(trj == null) return -1;
			var start = trj.StartUT;
			var stop = trj.NewOrbit.trueAnomaly > 180? 
				Math.Min(start+trj.BrakeDuration*LTRJ.ObstacleBrakeF, trj.AtTargetUT-1) :
				start+trj.NewOrbit.timeToAp;
			var UT = start;
			var dT = (stop-start);
			double dist;
//			Utils.LogF("Start {}, Stop {}", start-VSL.Physics.UT, stop-VSL.Physics.UT);//debug
			while(dT > 0.01)
			{
				var d1p = UT+dT > stop?  double.MaxValue : distance_from_ground(trj.NewOrbit, UT+dT);
				var d1m = UT-dT < start? double.MaxValue : distance_from_ground(trj.NewOrbit, UT-dT);
//				Utils.LogF("d1 {}, d2 {}, dT {}, T {}", d1p, d1m, dT, UT-VSL.Physics.UT);//debug
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
			CFG.Nav.On(Navigation.GoToTarget);
			landing_stage = LandingStage.Approach;
		}

		void decelerate()
		{
			VSL.Controls.StopWarp();
			DecelerationTimer.Reset();
			landing_stage = LandingStage.Decelerate; 
			CFG.AltitudeAboveTerrain = true;
		}

		void land()
		{
			CFG.AP1.On(Autopilot1.Land);
			landing_stage = LandingStage.Land;
		}

		double compute_terminal_velocity()
		{
			double terminal_velocity = 0;
			if(VSL.VerticalSpeed.Absolute > -100 || VSL.Altitude.Relative < 100+VSL.Geometry.H) 
			{
				terminal_velocity = Utils.ClampL(-VSL.VerticalSpeed.Absolute, 0.1f);
				VSL.Info.Countdown = (VSL.Altitude.Relative-VSL.Geometry.H)/terminal_velocity;
			}
			else VSL.Info.Countdown = sim.FreeFallTime(out terminal_velocity);
			return terminal_velocity;
		}

		void setup_for_deceleration()
		{
			CFG.VTOLAssistON = true;
			CFG.AltitudeAboveTerrain = true;
			CFG.AT.OnIfNot(Attitude.Custom);
			ATC.SetThrustDirW(VSL.vessel.srf_velocity);
		}

		Vector3d CorrectedBrakeVelocity(Vector3d vel, Vector3d pos)
		{ return vel-Vector3d.Project(vel, pos)*(1-Utils.Clamp(0.5*Body.atmDensityASL, 0.1, 0.9)); }

		Vector3d CorrectBrakeDirection(Vector3d vel, Vector3d pos)
		{ 
			var tpos = CFG.Target.RelSurfPos(Body);
			return QuaternionD.AngleAxis(Utils.ProjectionAngle(Vector3d.Exclude(pos, vel), 
			                                                   trajectory.SurfacePoint.RelSurfPos(Body)-tpos, 
			                                                   Vector3d.Cross(pos, tpos)),
			                             VSL.Physics.Up) * vel; 
		}

		protected bool do_land()
		{
			if(VSL.LandedOrSplashed) 
			{ 
				THR.Throttle = 0; 
				SetTarget();
				ClearStatus(); 
				CFG.AP2.Off(); 
				return true; 
			}
			VSL.Engines.ActivateEngines();
			if(VSL.Engines.MaxThrustM.Equals(0) && !VSL.Engines.HaveNextStageEngines) 
				landing_stage = LandingStage.HardLanding;
			float rel_Ve;
			double terminal_velocity;
			Vector3d brake_pos, brake_vel;
			switch(landing_stage)
			{
			case LandingStage.Wait:
				update_trajectory();
				Status("Preparing for deceleration...");
				brake_pos = VesselOrbit.getRelativePositionAtUT(trajectory.BrakeStartUT).xzy;
				brake_vel = CorrectedBrakeVelocity(VesselOrbit.getOrbitalVelocityAtUT(trajectory.BrakeStartUT).xzy, brake_pos);
				brake_vel = CorrectBrakeDirection(brake_vel, brake_pos);
				CFG.AT.OnIfNot(Attitude.Custom);
				ATC.SetThrustDirW(brake_vel);
				VSL.Info.Countdown = trajectory.BrakeEndUT-VSL.Physics.UT-1
					-Math.Max(MatchVelocityAutopilot.BrakingOffset((float)brake_vel.magnitude, VSL, out VSL.Info.TTB), 
					          LTRJ.MinBrakeOffset*(1-Utils.ClampH(Body.atmDensityASL, 1)));
				if(VSL.Info.Countdown <= 0) { FullStop = false; decelerate(); break; }
				if(obstacle_ahead(trajectory) > 0) { FullStop = true; decelerate(); break; }
				if(VSL.Controls.Aligned) VSL.Controls.WarpToTime = VSL.Physics.UT+VSL.Info.Countdown;
				else VSL.Controls.StopWarp();
				break;
			case LandingStage.Decelerate:
				var last_distance = trajectory.DistanceToTarget;
				update_trajectory();
				if(FullStop) Status("red", "Possible collision detected.\nPerforming full deceleration.");
				else Status("white", "Decelerating. Landing site error: {0}", Utils.FormatBigValue((float)trajectory.DistanceToTarget, "m"));
				if(VSL.Controls.HaveControlAuthority && 
				   (VSL.vessel.ctrlState.mainThrottle.Equals(0) || 
				    last_distance-trajectory.DistanceToTarget > 0.01 || 
				    Vector3d.Dot(VSL.HorizontalSpeed.Vector, CFG.Target.WorldPos(Body)-VSL.Physics.wCoM) < 0)) 
				    DecelerationTimer.Reset();
				if(FullStop || !DecelerationTimer.Check && trajectory.DistanceToTarget > LTRJ.Dtol)
				{ 
					brake_pos = VesselOrbit.pos.xzy;
					brake_vel = CorrectedBrakeVelocity(VesselOrbit.vel.xzy, brake_pos);
					brake_vel = CorrectBrakeDirection(brake_vel, brake_pos);
					if(Executor.Execute(-brake_vel, (float)Utils.ClampL(LTRJ.BrakeEndSpeed*Body.GeeASL, GLB.THR.MinDeltaV)))
						break; 
				}
				landing_stage = LandingStage.Coast;
				THR.Throttle = 0;
				break;
			case LandingStage.Coast:
				update_trajectory();
				Status("white", "Coasting. Landing site error: {0}", Utils.FormatBigValue((float)trajectory.DistanceToTarget, "m"));
				CFG.HF.OnIfNot(HFlight.Level);
				CFG.BR.OnIfNot(BearingMode.Auto);
				BRC.ForwardDirection = Vector3d.Exclude(VSL.Physics.Up, CFG.Target.WorldPos(Body)-VSL.Physics.wCoM);
				terminal_velocity = compute_terminal_velocity();
				VSL.Info.Countdown -= MatchVelocityAutopilot.BrakingOffset((float)terminal_velocity, VSL, out VSL.Info.TTB)
					+Utils.ClampH(VSL.Torque.MinRotationTime(Vector3.Angle(VSL.Engines.MaxThrust, VSL.vessel.srf_velocity)), TRJ.ManeuverOffset);
				if(VSL.Info.Countdown <= 0)
				{
					FullStop = false;
					rel_Ve = VSL.Engines.RelVeASL;
					if(rel_Ve <= 0 || !VSL.Controls.HaveControlAuthority && !VSL.Torque.HavePotentialControlAuthority)
					{
						landing_stage = LandingStage.HardLanding;
						break;
					}
					var fuel_left = VSL.Engines.GetAvailableFuelMass();
					var fuel_needed = VSL.Engines.FuelNeeded((float)terminal_velocity, rel_Ve);
					if(fuel_needed >= fuel_left ||
					   VSL.Engines.MaxHoverTimeASL(fuel_left-fuel_needed) < LTRJ.HoverTimeThreshold)
					{
						landing_stage = LandingStage.HardLanding;
						break;
					}
					landing_stage = LandingStage.SoftLanding;
				}
				break;
			case LandingStage.HardLanding:
				VSL.BrakesOn();
				setup_for_deceleration();
				terminal_velocity = compute_terminal_velocity();
				Status("yellow", "Landing...");
				if(VSL.Engines.MaxThrustM > 0 && 
				   (VSL.Controls.HaveControlAuthority || VSL.Torque.HavePotentialControlAuthority))
				{
					rel_Ve = VSL.Engines.RelVeASL;
					var fuel_left = VSL.Engines.GetAvailableFuelMass();
					var fuel_needed = rel_Ve > 0? VSL.Engines.FuelNeeded((float)terminal_velocity, rel_Ve) : fuel_left*2;
					VSL.Info.Countdown -= fuel_left > fuel_needed ? 
						MatchVelocityAutopilot.BrakingOffset((float)terminal_velocity, VSL) : 
						MatchVelocityAutopilot.BrakingOffset(VSL.Engines.DeltaV(fuel_left), VSL);
					if(FullStop ||
					   (VSL.Info.Countdown < 0 && 
					    (!VSL.OnPlanetParams.HaveParachutes || 
					     VSL.OnPlanetParams.ParachutesActive && VSL.OnPlanetParams.ParachutesDeployed)))
					{
						FullStop = true;
						THR.Throttle = VSL.VerticalSpeed.Absolute < -0.5? 1 : VSL.OnPlanetParams.GeeVSF;
					}
					else 
					{
						THR.Throttle = 0;
						Status("yellow", "Have some fuel left.\nWill deceletate just before landing...");
					}
				}
				if(Body.atmosphere && VSL.OnPlanetParams.HaveUsableParachutes)
				{
					VSL.OnPlanetParams.ActivateParachutes();
					if(!VSL.OnPlanetParams.ParachutesActive)
					{
						var dir = VSL.Geometry.MinAreaDirection;
						ATC.SetCustomRotationW(dir, Quaternion.AngleAxis(360*TimeWarp.fixedDeltaTime, VSL.Physics.Up) * Vector3.Project(dir, VSL.Physics.Up).normalized);
						if(CFG.AutoParachutes) Status("yellow", "Waiting for safe speed to deploy parachutes.\nTrying to decelerate using drag...");
						else Status("red", "Automatic parachute deployment is disabled.\nActivate parachutes manually when needed.");
					}
				}
				if(!VSL.OnPlanetParams.HaveParachutes && 
				   !VSL.Engines.HaveNextStageEngines && 
				   (VSL.Engines.MaxThrustM.Equals(0) || !VSL.Controls.HaveControlAuthority))
				{
					ATC.SetCustomRotationW(VSL.Geometry.MaxAreaDirection, VSL.Physics.Up);
					Status("red", "Collision is imminent.\nImpact speed: {0}", Utils.FormatBigValue((float)terminal_velocity, "m/s"));
				}
				break;
			case LandingStage.SoftLanding:
				Status("Final deceleration...");
				CFG.HF.Off();
				update_trajectory();
				setup_for_deceleration();
				compute_terminal_velocity();
				if(VSL.Controls.HaveControlAuthority)
				{
					if(!FullStop)
					{
						VSL.Info.TTB = VSL.Engines.TTB((float)VSL.vessel.srfSpeed);
						VSL.Info.Countdown -= VSL.Info.TTB+LTRJ.FinalBrakeOffset;
						if(VSL.Info.Countdown > 0) break;
						FullStop = true;
					}
					if(FullStop && 
					   Executor.Execute(-VSL.vessel.srf_velocity, LTRJ.BrakeEndSpeed) && 
					   VSL.HorizontalSpeed.Absolute-VSL.VerticalSpeed.Absolute > LTRJ.BrakeEndSpeed)
						break;
				}
				THR.Throttle = 0;
				if(CFG.Target.DistanceTo(VSL.vessel)-VSL.Geometry.R > LTRJ.Dtol) approach();
				else land();
				break;
			case LandingStage.Approach:
				Status("Approaching the target...");
				if(!CFG.Nav[Navigation.GoToTarget]) land();
				break;
			case LandingStage.Land: break;
			}
			return false;
		}

		#if DEBUG
		protected override void update_trajectory()
		{
			base.update_trajectory();
			NavigationPanel.CustomMarkersWP.Add(trajectory.SurfacePoint);
//			NavigationPanel.CustomMarkersVec.Add(trajectory.NewOrbit.getPositionAtUT(trajectory.BrakeEndUT));
		}
		#endif
	}
}

