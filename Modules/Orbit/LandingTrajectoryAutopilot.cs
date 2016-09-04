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

			[Persistent] public float BrakeThrustThreshod = 100;  //m/s
			[Persistent] public float BrakeEndSpeed      = 10;    //m/s
			[Persistent] public float MinBrakeOffset     = 10;    //m/s
			[Persistent] public float FinalBrakeOffset   = 5;     //m/s

			[Persistent] public float CorrectionOffset   = 20f;   //s
			[Persistent] public float CorrectionTimer    = 10f;   //s
			[Persistent] public float CorrectionMinDv    = 0.5f;  //m/s
			[Persistent] public float CorrectionThrustF  = 2.0f;
			[Persistent] public float CorrectionTimeF    = 2f;

			[Persistent] public float ObstacleBrakeF     = 1.1f;
			[Persistent] public float HoverTimeThreshold = 60f;   //s
			[Persistent] public float DropBallastThreshold = 0.5f;//dP/P_asl
			[Persistent] public float MaxDPressure       = 3f;    //kPa
			[Persistent] public float MinDPressure       = 1f;    //kPa
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
		protected double PressureASL;

		protected Timer dP_up_timer = new Timer(1);
		protected Timer dP_down_timer = new Timer(1);
		protected double dP_threshold;
		protected double landing_deadzone;
		protected double last_dP;
		protected double rel_dP;
		protected float last_Err;

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
			dP_up_timer.action = () =>
			{
				dP_threshold = Utils.ClampL(dP_threshold * 0.9, LTRJ.MinDPressure);
				last_dP = VSL.vessel.dynamicPressurekPa;
			};
			dP_down_timer.action = () =>
			{
				dP_threshold = Utils.ClampH(dP_threshold * 1.1, LTRJ.MaxDPressure);
				last_dP = VSL.vessel.dynamicPressurekPa;
			};
			sim = new AtmoSim(Body, VSL);
			Executor = new ManeuverExecutor(TCA);
		}

		protected override void reset()
		{
			base.reset();
			landing_stage = LandingStage.None;
			DecelerationTimer.Reset();
			dP_up_timer.Reset();
			dP_down_timer.Reset();
			dP_threshold = LTRJ.MaxDPressure;
			last_Err = 0;
			last_dP = 0;
			Working = false;
		}

		protected bool landing { get { return landing_stage != LandingStage.None; } }

		protected bool check_initial_trajectory()
		{
			var fuel_needed = VSL.Engines.FuelNeeded((float)trajectory.ManeuverDeltaV.magnitude) +
				VSL.Engines.FuelNeededAtAlt((float)trajectory.AtTargetVel.magnitude, 
				                                    (float)(trajectory.AtTargetPos.magnitude-Body.Radius));
			var fuel_available = VSL.Engines.GetAvailableFuelMass();
			var hover_time = fuel_needed < fuel_available? VSL.Engines.MaxHoverTimeASL(fuel_available-fuel_needed) : 0;
			var status = "";
			if(trajectory.DistanceToTarget < LTRJ.Dtol && hover_time > LTRJ.HoverTimeThreshold) return true;
			else
			{
				if(hover_time < LTRJ.HoverTimeThreshold)
				{
					status += "WARNING: Not enough fuel for powered landing.\n";
					if(Body.atmosphere && VSL.OnPlanetParams.HaveParachutes)
						status += "<i>Landing with parachutes may be possible, " +
							"but you're advised to supervise the process.</i>\n";
				}
				if(trajectory.DistanceToTarget > LTRJ.Dtol)
					status += string.Format("WARNING: Predicted landing site is too far from the target.\n" +
					                        "Error is <color=magenta><b>{0}</color></b>", 
					                        Utils.formatBigValue((float)trajectory.DistanceToTarget, "m"));
				status += "<color=red><b>Push to proceed. At your own risk.</b></color>";
				Status("yellow", status);
				return false;
			}
		}

		protected void start_landing()
		{
			Working = false;
			clear_nodes();
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
			if(!CorrectionTimer.TimePassed) return false;
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
				var d1p = UT+dT > stop?  double.MaxValue : distance_from_ground(trj.Orbit, UT+dT);
				var d1m = UT-dT < start? double.MaxValue : distance_from_ground(trj.Orbit, UT-dT);
//				Utils.Log("d1 {}, d2 {}, dT {}, T {}", d1p, d1m, dT, UT-VSL.Physics.UT);//debug
				if(d1p < d1m) { dist = d1p; UT += dT; }
				else { dist = d1m; UT -= dT; }
				if(dist < 0) return -dist;
				dT /= 2;
			}
			return -dist;
		}

		void rel_altitude_if_needed()
		{ CFG.AltitudeAboveTerrain = VSL.Altitude.Relative < 5000; }

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

		void decelerate(bool collision_detected)
		{
			VSL.Controls.StopWarp();
			DecelerationTimer.Reset();
			landing_stage = LandingStage.Decelerate; 
			Working = collision_detected;
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
			VSL.Info.CustomMarkersWP.Add(trajectory.SurfacePoint);
		}

		void nose_to_target()
		{
			CFG.BR.OnIfNot(BearingMode.Auto);
			BRC.ForwardDirection = Vector3d.Exclude(VSL.Physics.Up, CFG.Target.WorldPos(Body)-VSL.Physics.wCoM);
		}

		void correct_attitude_with_thrusters(float turn_time)
		{
			if((!VSL.Controls.HaveControlAuthority || rel_dP > 0 ||
			    VSL.Torque.NoEngines.MinRotationTime(VSL.Controls.AttitudeError) > VSL.Info.Countdown) &&
			   VSL.Controls.AttitudeError > Utils.ClampL(1-rel_dP, 0.1f) &&
			   VSL.Torque.MaxPossible.AngularDragResistance > VSL.Torque.MaxCurrent.AngularDragResistance*1.1)
				THR.Throttle = (float)Utils.ClampH((1+rel_dP) * turn_time/Utils.Clamp(VSL.Info.Countdown, 1, GLB.ATCB.MaxTimeToAlignment), 1);
		}

		Vector3d correction_direction()
		{
			var t0 = Utils.ClampL(VSL.Info.Countdown, 1e-5);
			var t1 = t0*LTRJ.CorrectionTimeF;
			var TL = trajectory.SurfacePoint.WorldPos(Body)-CFG.Target.WorldPos(Body);
			var correction = -VSL.Physics.Up*VSL.Physics.G*(1-t0*t0/t1/t1) + VSL.vessel.srf_velocity * 2 * ((t1-t0)/t1/t1);
			//overshot lies within [1; 2] interval
			correction += TL.ClampMagnitudeH((float)correction.magnitude) /
				VSL.Engines.MaxAccel*Utils.G0*Body.GeeASL * 
				Utils.ClampH(1 + rel_dP, 2) * 
				Math.Pow(Utils.ClampH(trajectory.DistanceToTarget/LTRJ.Dtol*2, 1), GLB.ANC.DistanceCurve);
			return correction.normalized;
		}

		bool correct_landing_site()
		{
			ATC.SetThrustDirW(correction_direction());
			if(VSL.Controls.HaveControlAuthority &&
			   trajectory.DistanceToTarget > landing_deadzone)
				THR.Throttle += Utils.ClampH((float)trajectory.DistanceToTarget /
				                             VSL.Engines.MaxAccel/LTRJ.Dtol*LTRJ.CorrectionThrustF, 
				                             VSL.OnPlanetParams.GeeVSF*0.9f);
			return trajectory.DistanceToTarget < landing_deadzone;
		}

		Vector3d corrected_brake_velocity(Vector3d obt_vel, Vector3d obt_pos)
		{ 
			return (obt_vel-Vector3d.Project(obt_vel, obt_pos) *
			        (1-Utils.Clamp(0.5*(Body.atmDensityASL+rel_dP), 0.1, 1)) +
			        Vector3d.Cross(Body.zUpAngularVelocity, obt_pos))
				.xzy;
		}

		Vector3d corrected_brake_direction(Vector3d vel, Vector3d pos)
		{ 
			var tpos = CFG.Target.RelSurfPos(Body);
			return QuaternionD.AngleAxis(Utils.ProjectionAngle(Vector3d.Exclude(pos, vel), 
			                                                   trajectory.SurfacePoint.RelSurfPos(Body)-tpos, 
			                                                   Vector3d.Cross(pos, tpos)),
			                             VSL.Physics.Up) * vel; 
		}

		void set_destination_vector()
		{ VSL.Info.Destination = CFG.Target.WorldPos(Body)-VSL.Physics.wCoM; }

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
			landing_deadzone = VSL.Geometry.D+CFG.Target.AbsRadius;
			if(VSL.vessel.dynamicPressurekPa > 0)
			{
				if(!dP_up_timer.RunIf(VSL.Controls.AttitudeError > last_Err ||
				                      Mathf.Abs(VSL.Controls.AttitudeError-last_Err) < 0.01f))
					dP_down_timer.RunIf(VSL.Controls.AttitudeError < last_Err &&
					                    VSL.vessel.dynamicPressurekPa < last_dP);
			}
			else dP_threshold = LTRJ.MaxDPressure;
			rel_dP = VSL.vessel.dynamicPressurekPa/dP_threshold;
			last_Err = VSL.Controls.AttitudeError;
			float rel_Ve;
			double terminal_velocity;
			Vector3d brake_pos, brake_vel, obt_vel;
			switch(landing_stage)
			{
			case LandingStage.Wait:
				Status("Preparing for deceleration...");
				THR.Throttle = 0;
				update_trajectory();
				nose_to_target();
				rel_altitude_if_needed();
				obt_vel = VesselOrbit.getOrbitalVelocityAtUT(trajectory.BrakeStartUT);
				brake_pos = VesselOrbit.getRelativePositionAtUT(trajectory.BrakeStartUT);
				brake_vel = corrected_brake_velocity(obt_vel, brake_pos);
				brake_vel = corrected_brake_direction(brake_vel, brake_pos.xzy);
				CFG.AT.OnIfNot(Attitude.Custom);
				ATC.SetThrustDirW(brake_vel);
				VSL.Info.Countdown = trajectory.BrakeEndUT-VSL.Physics.UT-1
					-Math.Max(MatchVelocityAutopilot.BrakingOffset((float)obt_vel.magnitude, VSL, out VSL.Info.TTB), 
					          LTRJ.MinBrakeOffset*(1-Utils.ClampH(Body.atmDensityASL, 1)));
				correct_attitude_with_thrusters(VSL.Torque.MaxPossible.MinRotationTime(VSL.Controls.AttitudeError));
				if(obstacle_ahead(trajectory) > 0) { decelerate(true); break; }
				if(VSL.Info.Countdown <= rel_dP) { decelerate(false); break; }
				if(VSL.Controls.CanWarp) 
					VSL.Controls.WarpToTime = VSL.Physics.UT+VSL.Info.Countdown;
				else VSL.Controls.StopWarp();
				break;
			case LandingStage.Decelerate:
				rel_altitude_if_needed();
				update_trajectory();
				nose_to_target();
				if(Working)
				{
					Status("red", "Possible collision detected.");
					correct_attitude_with_thrusters(VSL.Torque.MaxPossible.MinRotationTime(VSL.Controls.AttitudeError));
					Executor.Execute(VSL.Physics.Up*10);
					if(obstacle_ahead(trajectory) > 0) { CollisionTimer.Reset(); break; }
					if(!CollisionTimer.TimePassed) break;
					start_landing();
					break;
				}
				else
				{
					Status("white", "Decelerating. Landing site error: {0}", 
					       Utils.formatBigValue((float)trajectory.DistanceToTarget, "m"));
					compute_terminal_velocity();
					if(VSL.Controls.HaveControlAuthority) DecelerationTimer.Reset();
					if(Vector3d.Dot(VSL.HorizontalSpeed.Vector, CFG.Target.WorldPos(Body)-VSL.Physics.wCoM) < 0)
					{ if(Executor.Execute(-VSL.vessel.srf_velocity, LTRJ.BrakeEndSpeed)) break; }
					else if(!DecelerationTimer.TimePassed && 
					        trajectory.DistanceToTarget > landing_deadzone &&
					        Vector3d.Dot(CFG.Target.VectorTo(trajectory.SurfacePoint, Body), VSL.HorizontalSpeed.Vector) > 0)
					{ 
						brake_vel = corrected_brake_velocity(VesselOrbit.vel, VesselOrbit.pos);
						brake_vel = corrected_brake_direction(brake_vel, VesselOrbit.pos.xzy);
						//this is nice smoothing, but is dangerous on a low decending trajectory
						VSL.Info.TTB = VSL.Engines.TTB((float)VSL.vessel.srfSpeed);
						if(VSL.Info.Countdown-VSL.Torque.MaxCurrent.TurnTime-VSL.vessel.dynamicPressurekPa > VSL.Info.TTB)
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
				terminal_velocity = compute_terminal_velocity();
				correct_attitude_with_thrusters(VSL.Torque.MaxPossible.MinRotationTime(VSL.Controls.AttitudeError));
				correct_landing_site();
				VSL.Info.TTB = VSL.Engines.TTB((float)VSL.vessel.srfSpeed);
				VSL.Info.Countdown -= Math.Max(VSL.Info.TTB+VSL.Torque.NoEngines.TurnTime+VSL.vessel.dynamicPressurekPa, TRJ.ManeuverOffset);
				if(VSL.Info.Countdown <= 0)
				{
					Working = false;
					rel_Ve = VSL.Engines.RelVeASL;
					if(rel_Ve <= 0)
					{
						Message(10, "Not enough thrust to land properly.\nPerforming emergency landing...");
						landing_stage = LandingStage.HardLanding;
						break;
					}
					if(!(VSL.Controls.HaveControlAuthority || VSL.Torque.HavePotentialControlAuthority))
					{
						Message(10, "Lacking control authority to land properly.\nPerforming emergency landing...");
						landing_stage = LandingStage.HardLanding;
						break;
					}
					var fuel_left = VSL.Engines.GetAvailableFuelMass();
					var fuel_needed = VSL.Engines.FuelNeeded((float)terminal_velocity, rel_Ve);
					if(!CheatOptions.InfinitePropellant && 
					   (fuel_needed >= fuel_left ||
					    VSL.Engines.MaxHoverTimeASL(fuel_left-fuel_needed) < LTRJ.HoverTimeThreshold))
					{
						Message(10, "Not enough fuel to land properly.\nPerforming emergency landing...");
						landing_stage = LandingStage.HardLanding;
						break;
					}
					landing_stage = LandingStage.SoftLanding;
				}
				break;
			case LandingStage.HardLanding:
				Status("yellow", "Emergency Landing...");
				set_destination_vector();
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
						VSL.Engines.TTB((float)terminal_velocity) : 
						VSL.Engines.TTB(VSL.Engines.DeltaV(fuel_left));
					if((VSL.Info.Countdown < 0 && 
					    (!VSL.OnPlanetParams.HaveParachutes || 
					     VSL.OnPlanetParams.ParachutesActive && VSL.OnPlanetParams.ParachutesDeployed)))
						THR.Throttle = VSL.VerticalSpeed.Absolute < -5? 1 : VSL.OnPlanetParams.GeeVSF;
					else THR.Throttle = 0;
					Status("yellow", "Not enough fuel to land properly.\nWill deceletate as much as possible before impact...");
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
					if(Body.atmosphere) ATC.SetCustomRotationW(VSL.Geometry.MaxAreaDirection, VSL.Physics.Up);
					Status("red", "Crash is imminent.\nImpact speed: {0}", Utils.formatBigValue((float)terminal_velocity, "m/s"));
				}
				break;
			case LandingStage.SoftLanding:
				THR.Throttle = 0;
				set_destination_vector();
				update_trajectory();
				setup_for_deceleration();
				compute_terminal_velocity();
				nose_to_target();
				if(Working)
				{
					Status("white", "Final deceleration. Landing site error: {0}", 
					       Utils.formatBigValue((float)trajectory.DistanceToTarget, "m"));
					ATC.SetThrustDirW(correction_direction());
					if(VSL.Altitude.Relative > GLB.LND.WideCheckAltitude)
					{
						var brake_spd = Mathf.Max(VSL.HorizontalSpeed.Absolute, -VSL.VerticalSpeed.Absolute);
						var min_thrust = Utils.Clamp(brake_spd/(VSL.Engines.MaxAccel-VSL.Physics.G)/
						                             Utils.ClampL((float)VSL.Info.Countdown, 0.01f), 
						                             VSL.OnPlanetParams.GeeVSF, 1);
						THR.Throttle = Utils.Clamp(brake_spd/LTRJ.BrakeThrustThreshod, min_thrust, 1);
					}
					else THR.Throttle = 1;
					if(VSL.vessel.srfSpeed > LTRJ.BrakeEndSpeed) 
					{
						Working = THR.Throttle.Equals(1) || VSL.Info.Countdown < 10;
						break;
					}
				}
				else
				{
					var turn_time = VSL.Torque.MaxPossible.MinRotationTime(VSL.Controls.AttitudeError);
					correct_attitude_with_thrusters(turn_time);
					correct_landing_site();
					VSL.Info.TTB = VSL.Engines.TTB((float)VSL.vessel.srfSpeed);
					VSL.Info.Countdown -= VSL.Info.TTB+turn_time+LTRJ.FinalBrakeOffset;
					if(VSL.Controls.InvAlignmentFactor > 0.5) 
						Status("white", "Final deceleration: correcting attitude.\nLanding site error: {0}", 
						       Utils.formatBigValue((float)trajectory.DistanceToTarget, "m"));
					else 
						Status("white", "Final deceleration: waiting for the burn.\nLanding site error: {0}", 
						       Utils.formatBigValue((float)trajectory.DistanceToTarget, "m"));
					Working = VSL.Info.Countdown <= 0 || VSL.vessel.srfSpeed < LTRJ.BrakeEndSpeed;
					break;
				}
				THR.Throttle = 0;
				if(CFG.Target.DistanceTo(VSL.vessel)-VSL.Geometry.R > LTRJ.Dtol) approach();
				else land();
				break;
			case LandingStage.Approach:
				Status("Approaching the target...");
				set_destination_vector();
				if(!CFG.Nav[Navigation.GoToTarget]) land();
				break;
			case LandingStage.Land: 
				set_destination_vector();
				break;
			}
			return false;
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

		protected virtual void DrawDebugLines()
		{
			if(IsActive)
			{
				Utils.GLVec(VSL.refT.position, VSL.vessel.srf_velocity, Color.yellow);
				if(CFG.Target != null)
					Utils.GLLine(VSL.refT.position, CFG.Target.WorldPos(Body), Color.magenta);
			}
		}
		#endif
	}
}

