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
	public abstract class LandingTrajectoryAutopilot : TargetedTrajectoryCalculator<LandingTrajectory>
	{
		public new class Config : TCAModule.ModuleConfig
		{
			[Persistent] public float Dtol               = 1000f; //m
			[Persistent] public float FlyOverAlt         = 1000;  //m
			[Persistent] public float ApproachAlt        = 250;   //m

			[Persistent] public float BrakeThrustThreshod = 100;   //m/s
			[Persistent] public float BrakeEndSpeed      = 10;    //m/s
			[Persistent] public float MinBrakeOffset     = 10;    //m/s
			[Persistent] public float FinalBrakeOffset   = 5;     //m/s

			[Persistent] public float CorrectionOffset   = 20f;   //s
			[Persistent] public float CorrectionTimer    = 10f;   //s
			[Persistent] public float CorrectionMinDv    = 0.5f;  //m/s
			[Persistent] public float CorrectionF        = 2.0f;

			[Persistent] public float ObstacleBrakeF     = 1.1f;
			[Persistent] public float HoverTimeThreshold = 60f;   //s
			[Persistent] public float DropBallastThreshold = 0.5f;//dP/P_asl
			[Persistent] public float DPressureThreshold = 3f;    //kPa
			[Persistent] public float MachThreshold      = 0.9f;

		}
		protected static Config LTRJ { get { return Globals.Instance.LTRJ; } }

		protected LandingTrajectoryAutopilot(ModuleTCA tca) : base(tca) {}

		public enum LandingStage { None, Start, Wait, Decelerate, Coast, HardLanding, SoftLanding, Approach, Land }
		[Persistent] public LandingStage landing_stage;

		protected Timer DecelerationTimer = new Timer(0.5);
		protected Timer CollisionTimer = new Timer(1);
		protected Timer StageTimer = new Timer(5);
		protected ManeuverExecutor Executor;
		protected AtmoSim sim;
		protected bool FullStop;
		protected double PressureASL;

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
			StageTimer.action = () => 
			{
				VSL.ActivateNextStage();
				Message("Have to drop ballast to decelerate...");
			};
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
			FullStop = false;
			update_trajectory();
			VSL.Controls.StopWarp();
			VSL.Controls.Aligned = false;
			CFG.AltitudeAboveTerrain = false;
			landing_stage = LandingStage.Wait;
			PressureASL = Body.GetPressure(0);
		}

		protected bool correct_trajectory()
		{
			if(TimeWarp.WarpMode == TimeWarp.Modes.HIGH)
				VSL.Controls.WarpToTime = VSL.Physics.UT+(VSL.Info.Countdown > 0? 
				                                          Utils.ClampH(VSL.Info.Countdown, 60) : 60);
			else VSL.Controls.StopWarp();
			if(!CorrectionTimer.Check) return false;
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
			var pos = BodyRotationAtdT(Body, VSL.Physics.UT-UT)*orb.getRelativePositionAtUT(UT);
			return pos.magnitude-VSL.Geometry.H*2-Body.Radius - Body.TerrainAltitude(pos.xzy+Body.position);
		}

		protected double obstacle_ahead(LandingTrajectory trj)
		{
			if(trj == null) return -1;
			var start = trj.StartUT;
			var stop = trj.BrakeEndUT;
			var UT = start;
			var dT = (stop-start);
			double dist = 1;
//			Utils.Log("Start {}, Stop {}", start-VSL.Physics.UT, stop-VSL.Physics.UT);//debug
			while(dT > 0.01)
			{
				var d1p = UT+dT > stop?  double.MaxValue : distance_from_ground(trj.NewOrbit, UT+dT);
				var d1m = UT-dT < start? double.MaxValue : distance_from_ground(trj.NewOrbit, UT-dT);
//				Utils.Log("d1 {}, d2 {}, dT {}, T {}", d1p, d1m, dT, UT-VSL.Physics.UT);//debug
				if(d1p < d1m) { dist = d1p; UT += dT; }
				else { dist = d1m; UT -= dT; }
				if(dist < 0) return -dist;
				dT /= 2;
			}
			return -dist;
		}

		void approach()
		{
			CFG.BR.Off();
			CFG.BlockThrottle = true;
			CFG.AltitudeAboveTerrain = true;
			CFG.VF.On(VFlight.AltitudeControl);
			CFG.DesiredAltitude = LTRJ.ApproachAlt < VSL.Altitude.Relative/2? 
				LTRJ.ApproachAlt : Utils.ClampL(VSL.Altitude.Relative/2, VSL.Geometry.H*2);
			SetTarget(CFG.Target);
			CFG.Nav.On(Navigation.GoToTarget);
			if(CFG.Target.IsVessel) CFG.Target.Radius = 7;
			landing_stage = LandingStage.Approach;
		}

		void decelerate()
		{
			VSL.Controls.StopWarp();
			DecelerationTimer.Reset();
			landing_stage = LandingStage.Decelerate; 
		}

		void land()
		{
			CFG.AP1.On(Autopilot1.Land);
			landing_stage = LandingStage.Land;
		}

		double compute_terminal_velocity()
		{
			double terminal_velocity = 0;
			if(Body.atmosphere && (VSL.VerticalSpeed.Absolute > -100 || VSL.Altitude.Relative < 100+VSL.Geometry.H))
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

		protected override void update_trajectory()
		{
			base.update_trajectory();
			NavigationPanel.CustomMarkersWP.Add(trajectory.SurfacePoint);
		}

		void nose_to_target()
		{
			CFG.BR.OnIfNot(BearingMode.Auto);
			BRC.ForwardDirection = Vector3d.Exclude(VSL.Physics.Up, CFG.Target.WorldPos(Body)-VSL.Physics.wCoM);
		}

		void correct_attitude_with_thrusters(float turn_time)
		{
			if(VSL.Torque.MaxPossible.AngularDragResistance > VSL.Torque.MaxCurrent.AngularDragResistance*1.1)
				THR.Throttle = (float)Utils.ClampH(Utils.ClampL(VSL.vessel.dynamicPressurekPa/LTRJ.DPressureThreshold, 1)*
				                                   turn_time/Utils.ClampL(VSL.Info.Countdown, 1), 1);
		}

		bool correct_landing_site()
		{
			if(VSL.Controls.HaveControlAuthority)
			{
				var err = CFG.Target.VectorTo(trajectory.SurfacePoint, Body);
				ATC.SetThrustDirW(VSL.vessel.srf_velocity+err.ClampMagnitudeL(LTRJ.Dtol*(float)Body.GeeASL));
				THR.Throttle = Mathf.Max(THR.Throttle, 
				                         Utils.ClampH((float)err.magnitude/VSL.Engines.MaxAccel/LTRJ.Dtol*LTRJ.CorrectionF, 
				                                      VSL.OnPlanetParams.GeeVSF));
			}
			return trajectory.DistanceToTarget < LTRJ.Dtol;
		}

		Vector3d CorrectedBrakeVelocity(Vector3d obt_vel, Vector3d obt_pos)
		{ 
			return (obt_vel-Vector3d.Project(obt_vel, obt_pos) *
			        (1-Utils.Clamp(0.5*(Body.atmDensityASL+VSL.vessel.dynamicPressurekPa/LTRJ.DPressureThreshold), 0.1, 1)) +
			        Vector3d.Cross(Body.zUpAngularVelocity, obt_pos))
				.xzy;
		}

		Vector3d CorrectBrakeDirection(Vector3d vel, Vector3d pos)
		{ 
			var tpos = CFG.Target.RelSurfPos(Body);
			return QuaternionD.AngleAxis(Utils.ProjectionAngle(Vector3d.Exclude(pos, vel), 
			                                                   trajectory.SurfacePoint.RelSurfPos(Body)-tpos, 
			                                                   Vector3d.Cross(pos, tpos)),
			                             VSL.Physics.Up) * vel; 
		}

		#if DEBUG
		void log_flight()
		{
			var v = VSL.vessel;
			CSV(
				VSL.Altitude.Absolute,
				v.staticPressurekPa,
				v.atmDensity,
				v.atmDensity/Body.atmDensityASL,
				v.atmosphericTemperature,
				VSL.Physics.G,
				v.srfSpeed,
				VSL.HorizontalSpeed.Absolute,
				Mathf.Abs(VSL.VerticalSpeed.Absolute),
				v.mach,
				v.dynamicPressurekPa,
				VSL.Controls.AttitudeError
			);
		}
		#endif

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
			Vector3d brake_pos, brake_vel, obt_vel;
			switch(landing_stage)
			{
			case LandingStage.Wait:
				THR.Throttle = 0;
				update_trajectory();
				nose_to_target();
				Status("Preparing for deceleration...");
				obt_vel = VesselOrbit.getOrbitalVelocityAtUT(trajectory.BrakeStartUT);
				brake_pos = VesselOrbit.getRelativePositionAtUT(trajectory.BrakeStartUT);
				brake_vel = CorrectedBrakeVelocity(obt_vel, brake_pos);
				brake_vel = CorrectBrakeDirection(brake_vel, brake_pos.xzy);
				CFG.AT.OnIfNot(Attitude.Custom);
				ATC.SetThrustDirW(brake_vel);
				VSL.Info.Countdown = trajectory.BrakeEndUT-VSL.Physics.UT-1
					-Math.Max(MatchVelocityAutopilot.BrakingOffset((float)obt_vel.magnitude, VSL, out VSL.Info.TTB), 
					          LTRJ.MinBrakeOffset*(1-Utils.ClampH(Body.atmDensityASL, 1)));
				if(VSL.Info.Countdown > 1)
					correct_attitude_with_thrusters(VSL.Torque.MaxPossible.MinRotationTime(VSL.Controls.AttitudeError));
				if(obstacle_ahead(trajectory) > 0) { FullStop = true; decelerate(); break; }
				if(VSL.Info.Countdown <= 0) { FullStop = false; decelerate(); break; }
				if(VSL.Controls.Aligned) VSL.Controls.WarpToTime = VSL.Physics.UT+VSL.Info.Countdown;
				else VSL.Controls.StopWarp();
				break;
			case LandingStage.Decelerate:
				update_trajectory();
				nose_to_target();
				if(FullStop)
				{
					Status("red", "Possible collision detected.");
					var obstacle = obstacle_ahead(trajectory);
					if(obstacle > 0 && Executor.Execute(VSL.Physics.Up*obstacle)) break; 
					start_landing();
					break;
				}
				else
				{
					Status("white", "Decelerating. Landing site error: {0}", 
					       Utils.formatBigValue((float)trajectory.DistanceToTarget, "m"));
					if(VSL.Controls.HaveControlAuthority) DecelerationTimer.Reset();
					if(Vector3d.Dot(VSL.HorizontalSpeed.Vector, CFG.Target.WorldPos(Body)-VSL.Physics.wCoM) < 0)
					{ if(Executor.Execute(-VSL.vessel.srf_velocity, LTRJ.BrakeEndSpeed)) break; }
					else if(!DecelerationTimer.Check && 
					        trajectory.DistanceToTarget > LTRJ.Dtol/10 &&
					        Vector3d.Dot(CFG.Target.VectorTo(trajectory.SurfacePoint, Body), VSL.HorizontalSpeed.Vector) > 0)
					{ 
						brake_vel = CorrectedBrakeVelocity(VesselOrbit.vel, VesselOrbit.pos);
						brake_vel = CorrectBrakeDirection(brake_vel, VesselOrbit.pos.xzy);
						//this is nice smoothing, but is dangerous on a low decending trajectory
						if(-VSL.Altitude.Relative/VSL.VerticalSpeed.Absolute-VSL.Torque.MaxCurrent.TurnTime > VSL.vessel.srfSpeed/VSL.Engines.MaxAccel)
							brake_vel = brake_vel.normalized*VSL.HorizontalSpeed.Absolute*Utils.ClampH(trajectory.DistanceToTarget/CFG.Target.DistanceTo(VSL.vessel), 1);
						if(Executor.Execute(-brake_vel, (float)Utils.ClampL(LTRJ.BrakeEndSpeed*Body.GeeASL, GLB.THR.MinDeltaV))) break; 
					}
				}
				landing_stage = LandingStage.Coast;
				break;
			case LandingStage.Coast:
				Status("white", "Coasting. Landing site error: {0}", Utils.formatBigValue((float)trajectory.DistanceToTarget, "m"));
				THR.Throttle = 0;
				update_trajectory();
				nose_to_target();
				setup_for_deceleration();
				correct_landing_site();
				terminal_velocity = compute_terminal_velocity();
				VSL.Info.TTB = VSL.Engines.TTB((float)VSL.vessel.srfSpeed);
				VSL.Info.Countdown -= Math.Max(VSL.Info.TTB+VSL.Torque.NoEngines.TurnTime+VSL.vessel.dynamicPressurekPa, TRJ.ManeuverOffset);
				if(VSL.Info.Countdown <= 0)
				{
					FullStop = false;
					rel_Ve = VSL.Engines.RelVeASL;
					if(rel_Ve <= 0)
					{
						Message("Not enough thrust to land properly.\nPerforming emergency landing...");
						landing_stage = LandingStage.HardLanding;
						break;
					}
					if(!(VSL.Controls.HaveControlAuthority || VSL.Torque.HavePotentialControlAuthority))
					{
						Message("Lacking control authority to land properly.\nPerforming emergency landing...");
						landing_stage = LandingStage.HardLanding;
						break;
					}
					var fuel_left = VSL.Engines.GetAvailableFuelMass();
					var fuel_needed = VSL.Engines.FuelNeeded((float)terminal_velocity, rel_Ve);
					if(!CheatOptions.InfinitePropellant && 
					   (fuel_needed >= fuel_left ||
					    VSL.Engines.MaxHoverTimeASL(fuel_left-fuel_needed) < LTRJ.HoverTimeThreshold))
					{
						Message("Not enough fuel to land properly.\nPerforming emergency landing...");
						landing_stage = LandingStage.HardLanding;
						break;
					}
					landing_stage = LandingStage.SoftLanding;
				}
				break;
			case LandingStage.HardLanding:
				Status("yellow", "Emergency Landing...");
				update_trajectory();
				VSL.BrakesOn();
				CFG.BR.Off();
				setup_for_deceleration();
				terminal_velocity = compute_terminal_velocity();
				if(VSL.Engines.MaxThrustM > 0 && 
				   (VSL.Controls.HaveControlAuthority || VSL.Torque.HavePotentialControlAuthority))
				{
					rel_Ve = VSL.Engines.RelVeASL;
					var fuel_left = VSL.Engines.GetAvailableFuelMass();
					var fuel_needed = rel_Ve > 0? VSL.Engines.FuelNeeded((float)terminal_velocity, rel_Ve) : fuel_left*2;
					VSL.Info.Countdown -= fuel_left > fuel_needed ? 
						MatchVelocityAutopilot.BrakingOffset((float)terminal_velocity, VSL) : 
						MatchVelocityAutopilot.BrakingOffset(VSL.Engines.DeltaV(fuel_left), VSL);
					if((VSL.Info.Countdown < 0 && 
					    (!VSL.OnPlanetParams.HaveParachutes || 
					     VSL.OnPlanetParams.ParachutesActive && VSL.OnPlanetParams.ParachutesDeployed)))
						THR.Throttle = VSL.VerticalSpeed.Absolute < -5? 1 : VSL.OnPlanetParams.GeeVSF;
					else THR.Throttle = 0;
					Status("yellow", "Have some fuel left.\nWill deceletate just before landing...");
				}
				if(Body.atmosphere && VSL.OnPlanetParams.HaveUsableParachutes)
				{
					VSL.OnPlanetParams.ActivateParachutes();
					if(!VSL.OnPlanetParams.ParachutesActive)
					{
						ATC.SetCustomRotationW(VSL.Geometry.MaxAreaDirection, VSL.Physics.Up);
						StageTimer.RunIf(Body.atmosphere && //!VSL.Controls.HaveControlAuthority &&
						                 VSL.vessel.currentStage-1 > VSL.OnPlanetParams.NearestParachuteStage &&
						                 VSL.vessel.dynamicPressurekPa > LTRJ.DropBallastThreshold*PressureASL && 
						                 VSL.vessel.mach > LTRJ.MachThreshold);
						if(CFG.AutoParachutes) Status("yellow", "Waiting for safe speed to deploy parachutes.\n" +
						                              "Trying to decelerate using drag...");
						else Status("red", "Automatic parachute deployment is disabled.\nActivate parachutes manually when needed.");
					}
				}
				if(!VSL.OnPlanetParams.HaveParachutes && 
				   !VSL.Engines.HaveNextStageEngines && 
				   (VSL.Engines.MaxThrustM.Equals(0) || !VSL.Controls.HaveControlAuthority))
				{
					ATC.SetCustomRotationW(VSL.Geometry.MaxAreaDirection, VSL.Physics.Up);
					Status("red", "Collision is imminent.\nImpact speed: {0}", Utils.formatBigValue((float)terminal_velocity, "m/s"));
				}
				break;
			case LandingStage.SoftLanding:
				THR.Throttle = 0;
				update_trajectory();
				setup_for_deceleration();
				compute_terminal_velocity();
				nose_to_target();
				if(FullStop)
				{
					Status("white", "Final deceleration. Landing site error: {0}", 
					       Utils.formatBigValue((float)trajectory.DistanceToTarget, "m"));
					ATC.SetThrustDirW(VSL.vessel.srf_velocity+CFG.Target.VectorTo(trajectory.SurfacePoint, Body)*Body.GeeASL);
					if(VSL.Altitude.Relative > GLB.LND.WideCheckAltitude)
					{
						var brake_spd = Mathf.Max(VSL.HorizontalSpeed.Absolute, -VSL.VerticalSpeed.Absolute);
						var min_thrust = Utils.Clamp(brake_spd/(VSL.Engines.MaxAccel-VSL.Physics.G)/
						                             Utils.ClampL((float)VSL.Info.Countdown, 0.01f), 
						                             VSL.OnPlanetParams.GeeVSF, 1);
						THR.Throttle = Utils.Clamp(brake_spd/LTRJ.BrakeThrustThreshod, min_thrust, 1);
					}
					else THR.Throttle = 1;
					if(VSL.vessel.srfSpeed > LTRJ.BrakeEndSpeed) break;
					THR.Throttle = 0;
				}
				else
				{
					var turn_time = VSL.Torque.MaxPossible.MinRotationTime(VSL.Controls.AttitudeError);
					VSL.Info.TTB = VSL.Engines.TTB(Mathf.Abs(VSL.VerticalSpeed.Absolute));
					VSL.Info.Countdown -= VSL.Info.TTB+turn_time+LTRJ.FinalBrakeOffset;
					correct_attitude_with_thrusters(turn_time);
					correct_landing_site();
					if(VSL.Controls.InvAlignmentFactor > 0.5) 
						Status("white", "Final deceleration: correcting attitude.\nLanding site error: {0}", 
						       Utils.formatBigValue((float)trajectory.DistanceToTarget, "m"));
					else 
						Status("white", "Final deceleration: waiting for the burn.\nLanding site error: {0}", 
						       Utils.formatBigValue((float)trajectory.DistanceToTarget, "m"));
					FullStop = VSL.Info.Countdown <= 0 || VSL.vessel.srfSpeed < LTRJ.BrakeEndSpeed;
					break;
				}
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
	}
}

