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
			[Persistent] public float ObstacleBrakeF     = 1.2f;  //s
		}
		protected static Config LTRJ { get { return TCAScenario.Globals.LTRJ; } }

		protected LandingTrajectoryAutopilot(ModuleTCA tca) : base(tca) {}

		enum LandingStage { None, Start, Wait, Decelerate, Coast, Approach, Land }
		LandingStage landing_stage;

		protected LowPassFilterD DistanceFilter = new LowPassFilterD();
		protected Timer DecelerationTimer = new Timer();
		protected bool FullStop;

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
			DecelerationTimer.Period = LTRJ.LowPassF;
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
			if(trajectory != null) trajectory.UpdateOrbit(VesselOrbit, false);
			else trajectory = new LandingTrajectory(VSL, Vector3d.zero, VSL.Physics.UT, Target, TargetAltitude);
			landing_stage = LandingStage.Wait;
		}

		protected bool correct_trajectory()
		{
			Status("Correcting trajectory...");
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
			{ WRP.StopWarp(); start_correction(); return false; }
			return !Body.atmosphere;
		}

		double distance_from_ground(Orbit orb, double UT)
		{
			var pos = orb.getRelativePositionAtUT(UT);
			return pos.magnitude-VSL.Geometry.H-Body.Radius - Utils.TerrainAltitude(Body, pos+Body.position);
		}

		protected double obstacle_ahead(LandingTrajectory trj)
		{
			if(trj == null) return -1;
			var start = trj.StartUT;
			var stop = start + (trj.NewOrbit.timeToAp < trj.NewOrbit.period/2? 
			                    trj.NewOrbit.timeToAp : trj.BrakeDuration*LTRJ.ObstacleBrakeF);
			var UT = start;
			var dT = (stop-start);
			double dist;
			while(dT > 0.01)
			{
				var d1p = UT+dT > stop?  double.MaxValue : distance_from_ground(trj.NewOrbit, UT+dT);
				var d1m = UT-dT < start? double.MaxValue : distance_from_ground(trj.NewOrbit, UT-dT);
				if(d1p < d1m) { dist = d1p; UT += dT; }
				else { dist = d1m; UT -= dT; }
//				LogF("dist {}, T {}", dist, UT-VSL.Physics.UT);//debug
				if(dist < 0) return -dist;
				dT /= 2;
			}
			return -1;
		}

		void approach()
		{
			CFG.BR.Off();
			CFG.AltitudeAboveTerrain = true;
			CFG.BlockThrottle = true;
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

		protected bool do_land()
		{
			switch(landing_stage)
			{
			case LandingStage.Wait:
				trajectory.UpdateOrbit(VesselOrbit, false);
				if(obstacle_ahead(trajectory) > 0) { FullStop = true; decelerate(); break; }
				VSL.Info.Countdown = trajectory.BrakeStartUT-VSL.Physics.UT-1;
				VSL.Info.TTB = (float)trajectory.BrakeDuration;
				CFG.AT.OnIfNot(Attitude.Custom);
				ATC.SetCustomRotationW(VSL.Engines.MaxThrust, trajectory.AtTargetVel.xzy);
				if(ATC.Aligned) WRP.WarpToTime = VSL.Physics.UT+VSL.Info.Countdown;
				else WRP.StopWarp();
				if(VSL.Info.Countdown <= 0) decelerate();
				break;
			case LandingStage.Decelerate:
				var last_distance = DistanceFilter.Value;
				trajectory.UpdateOrbit(VesselOrbit, false);
				DistanceFilter.Update(trajectory.DistanceToTarget);
				Status("Decelerating. Landing site error: {0:F1}m", trajectory.DistanceToTarget);
				CSV(last_distance, DistanceFilter, last_distance-DistanceFilter);//debug
				var TargetPos = Target.WorldPos(Body);
				if(last_distance-DistanceFilter > 0.01 || 
				   Vector3d.Dot(VSL.HorizontalSpeed.Vector, 
				                VSL.Physics.wCoM-TargetPos) < 0) 
					DecelerationTimer.Reset();
				if((FullStop || !DecelerationTimer.Check && 
				    trajectory.DistanceToTarget > LTRJ.Dtol) &&
				   VSL.HorizontalSpeed.Absolute > LTRJ.BrakeEndSpeed &&
				   Vector3d.Angle(VSL.Engines.MaxThrust, -VSL.Physics.Up) > 10)
				{
					CFG.AT.OnIfNot(Attitude.Custom);
					ATC.SetCustomRotationW(VSL.Engines.MaxThrust,
					                       VesselOrbit.vel.xzy-
					                       Vector3d.Exclude(VSL.Physics.Up, TargetPos-trajectory.AtTargetPos.xzy)/100);
					if(ATC.Aligned) THR.DeltaV = Utils.ClampL((float)VSL.vessel.srfSpeed-LTRJ.BrakeEndSpeed+1, 0);
					break;
				}
				landing_stage = LandingStage.Coast;
				THR.Throttle = 0;
				break;
			case LandingStage.Coast:
				CFG.HF.OnIfNot(HFlight.Level);
				CFG.BR.OnIfNot(BearingMode.Auto);
				trajectory.UpdateOrbit(VesselOrbit, false);
				BRC.ForwardDirection = Vector3d.Exclude(VSL.Physics.Up, trajectory.AtTargetPos.xzy-VSL.Physics.wCoM);
				VSL.Info.Countdown = trajectory.TimeToSurface;
				Status(string.Format("Coasting. Landing site error: {0:F1}m", trajectory.DistanceToTarget));
				if(VSL.Altitude.Relative > 2*(2-ATC.AttitudeFactor)*LTRJ.FlyOverAlt) break;
				approach();
				break;
			case LandingStage.Approach:
				Status("Approaching the target...");
				CFG.Nav.OnIfNot(Navigation.GoToTarget);
				if(CFG.Nav[Navigation.GoToTarget]) break;
				CFG.AP1.OnIfNot(Autopilot1.Land);
				landing_stage = LandingStage.Land;
				break;
			case LandingStage.Land:
				ClearStatus();
				if(CFG.AP1[Autopilot1.Land]) break;
				CFG.AP2.Off();
				return true;
			}
			return false;
		}
	}
}

