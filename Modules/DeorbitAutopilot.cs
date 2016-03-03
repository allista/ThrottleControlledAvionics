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
using System.Collections.Generic;
using UnityEngine;

namespace ThrottleControlledAvionics
{
	[CareerPart]
	[RequireModules(typeof(ManeuverAutopilot), 
	                typeof(AutoLander))]
	public class DeorbitAutopilot : TCAModule
	{
		public class Config : TCAModule.ModuleConfig
		{
			[Persistent] public float StartOffset        = 60f;   //s
			[Persistent] public float StartPeR           = 0.5f;  //of planet radius
			[Persistent] public float dVtol              = 0.01f; //m/s
			[Persistent] public float Dtol               = 1000f; //m
			[Persistent] public float FlyOverAlt         = 1000;  //m
			[Persistent] public float ApproachAlt        = 500;   //m
			[Persistent] public float BrakeEndSpeed      = 100;   //m/s
			[Persistent] public int   MaxIterations      = 1000;
			[Persistent] public int   PerFrameIterations = 10;

		}
		static Config DEO { get { return TCAScenario.Globals.DEO; } }

		public DeorbitAutopilot(ModuleTCA tca) : base(tca) {}

		Orbit VesselOrbit { get { return VSL.vessel.orbitDriver.orbit; } }
		CelestialBody Body { get { return VSL.vessel.orbitDriver.orbit.referenceBody; } }

		public static Orbit NewOrbit(Orbit old, Vector3d dV, double UT)
		{
			var obt = new Orbit();
			var pos = old.getRelativePositionAtUT(UT);
			var vel = old.getOrbitalVelocityAtUT(UT)+dV;
			obt.UpdateFromStateVectors(pos, vel, old.referenceBody, UT);
			return obt;
		}

		public static Vector3d dV4Pe(Orbit old, double PeR, double UT, Vector3d add_dV = default(Vector3d))
		{
			var oldPeR = old.PeR;
			var vel = old.getOrbitalVelocityAtUT(UT);
			var up = oldPeR < PeR;
			var pg = up? 1 : -1;
			var velN = vel.normalized * pg;
			var min_dV = 0.0;
			var max_dV = 0.0;
			if(up)
			{
				max_dV = 1;
				while(NewOrbit(old, velN*max_dV, UT).PeR < PeR)
				{ max_dV *= 2; if(max_dV > 100000) break; }
			}
			else max_dV = vel.magnitude+add_dV.magnitude;
			while(max_dV-min_dV > DEO.dVtol)
			{
				var dV = (max_dV+min_dV)/2;
				var nPe = NewOrbit(old, velN*dV+add_dV, UT).PeR;
				if(up && nPe > PeR || !up && nPe < PeR) max_dV = dV;
				else min_dV = dV;
			}
			return (max_dV+min_dV)/2*velN+add_dV;
		}

		public static Orbit NewPeR(Orbit old, double PeR, double UT)
		{
			var dV = dV4Pe(old, PeR, UT);
			return NewOrbit(old, dV, UT);
		}

		public override void Init()
		{
			base.Init();
			CFG.AP2.AddHandler(this, Autopilot2.Deorbit);
		}

		IEnumerator<LandingTrajectory> compute_landing_trajectory(WayPoint target, double PeR)
		{
//			Log("\nComputing trajectory for PeR={0}\nFor target: {1}", PeR, target);//debug
			var targetAlt = target.SurfaceAlt(Body)+DEO.FlyOverAlt;
			var StartUT = VSL.Physics.UT+DEO.StartOffset;
			LandingTrajectory site = null;
			LandingTrajectory best = null;
			Vector3d dVn = Vector3d.zero;
			var maxI = DEO.MaxIterations;
			var frameI = DEO.PerFrameIterations;
			do {
				if(site != null) 
				{
					if(Math.Abs(site.DeltaLon) > Math.Abs(site.LandingTargetInclination))
					{
						StartUT = site.StartUT+site.DeltaLon/360*VesselOrbit.period;
						if(StartUT-VSL.Physics.UT < DEO.StartOffset) StartUT += VesselOrbit.period;
						if(site.ManeuverDuration > StartUT-VSL.Physics.UT) StartUT = VSL.Physics.UT+site.ManeuverDuration+DEO.StartOffset;
					}
					else dVn += VesselOrbit.GetOrbitNormal().normalized*
							VesselOrbit.vel.magnitude*Math.Sin(site.LandingTargetInclination*Mathf.Deg2Rad/2);
//					Log("StartUT change: {0}\ndVn {1}", StartUT-site.StartUT, dVn);//debug
				}
				site = new LandingTrajectory(VSL, dV4Pe(VesselOrbit, Body.Radius*PeR, StartUT, dVn), StartUT, targetAlt);
				var targetPos = site.CBRotation * Body.GetRelSurfacePosition(target.Lat, target.Lon, 0);;
				site.UpdateDeltaLon(target.Lon);
				site.DistanceToTarget = target.AngleTo(site.SurfacePoint)*Body.Radius;
				site.LandingTargetInclination = 90-Vector3d.Angle(site.LandingOrbit.GetOrbitNormal().xzy, targetPos);
				if(best == null || site < best) best = site;
				frameI--;
				maxI--;
//				Log("target: {0}\n{1}", target, site);//debug
				if(frameI < 0)
				{
					yield return null;
					frameI = DEO.PerFrameIterations;
				}
			} while(best.DistanceToTarget > DEO.Dtol && maxI > 0);
			yield return best;
		}

		double CurrentPeR;
		bool compute_node;
		LandingTrajectory trajectory;
		IEnumerator<LandingTrajectory> trajectory_calculator;
		bool trajectory_computed(WayPoint target, double PeR)
		{
			if(trajectory != null) return true;
			if(trajectory_calculator == null)
				trajectory_calculator = compute_landing_trajectory(target, PeR);
			if(trajectory_calculator.MoveNext())
				trajectory = trajectory_calculator.Current;
			else trajectory_calculator = null;
			return trajectory != null;
		}

		void reset()
		{
			Working = false;
			Target = null;
			stage = Stage.None;
			trajectory = null;
			trajectory_calculator = null;
			CurrentPeR = DEO.StartPeR;
			CFG.AP1.Off();
		}

		public void DeorbitCallback(Multiplexer.Command cmd)
		{
			switch(cmd)
			{
			case Multiplexer.Command.Resume:
				break;

			case Multiplexer.Command.On:
				reset();
				if(VSL.Target == null) return;
				VSL.vessel.patchedConicSolver.maneuverNodes
					.ForEach(n => VSL.vessel.patchedConicSolver.RemoveManeuverNode(n));
				VSL.vessel.patchedConicSolver.UpdateFlightPlan();
				Target = Target2WP();
				Target.UpdateCoordinates(Body);
				SetTarget(Target);
				compute_node = true;
				goto case Multiplexer.Command.Resume;

			case Multiplexer.Command.Off:
				reset();
				break;
			}
		}

		WayPoint Target;
		enum Stage { None, Deorbit, Decelerate, Approach, Land, Waiting }
		Stage stage;

		protected override void Update()
		{
			if(Target == null || VSL.orbit.referenceBody == null) return;
			if(CFG.Target == null) SetTarget(Target);
			if(compute_node && trajectory_computed(CFG.Target, CurrentPeR))
			{
				if(trajectory.DistanceToTarget < DEO.Dtol || CurrentPeR >= 1)
				{
					compute_node = false;
					ManeuverAutopilot.AddNode(VSL, trajectory.ManeuverDeltaV, trajectory.StartUT);
					if(trajectory.DistanceToTarget < DEO.Dtol) 
					{ CFG.AP1.On(Autopilot1.Maneuver); Working = true; stage = Stage.Deorbit; }
					else 
					{
						ThrottleControlledAvionics.StatusMessage = "Predicted landing site is too far from the target.\n" +
							"To proceed, activate maneuver execution manually.";
						stage = Stage.Waiting;
					}
				}
				else 
				{
					CurrentPeR += 0.1;
					if(CurrentPeR < 1) trajectory = null;
				}
				return;
			}
			if(Working) 
			{ 
				switch(stage)
				{
				case Stage.Waiting:
					if(!CFG.AP1[Autopilot1.Maneuver]) break;
					stage = Stage.Deorbit;
					break;
				case Stage.Deorbit:
					if(CFG.AP1[Autopilot1.Maneuver]) break;
					trajectory.UpdateOrbit(VesselOrbit);
					ManeuverAutopilot.AddNode(VSL, trajectory.BrakeDeltaV-trajectory.BrakeDeltaV.normalized*DEO.BrakeEndSpeed, 
					                          trajectory.BrakeStartUT);
					CFG.AP1.On(Autopilot1.Maneuver);
					stage = Stage.Decelerate;
					break;
				case Stage.Decelerate:
					if(VSL.Altitude.Relative < DEO.FlyOverAlt)
						CFG.AP1.Off();
					if(CFG.AP1[Autopilot1.Maneuver]) break;
					CFG.AltitudeAboveTerrain = true;
					CFG.BlockThrottle = true;
					CFG.VF.On(VFlight.AltitudeControl);
					CFG.HF.On(HFlight.Level);
					CFG.DesiredAltitude = DEO.ApproachAlt;
					stage = Stage.Approach;
					break;
				case Stage.Approach:
					CFG.Nav.OnIfNot(Navigation.GoToTarget);
					if(CFG.Nav[Navigation.GoToTarget]) break;
					CFG.AP1.OnIfNot(Autopilot1.Land);
					stage = Stage.Land;
					break;
				case Stage.Land:
					if(CFG.AP1[Autopilot1.Land]) break;
					CFG.AP2.Off();
					break;
				}
				return;
			}
		}

		public override void Draw()
		{
			if(compute_node) GUILayout.Label("Computing...", Styles.grey_button, GUILayout.ExpandWidth(false));
			else if(Utils.ButtonSwitch("Land at Target", CFG.AP2[Autopilot2.Deorbit],
			                           "Compute and perform a deorbit maneuver, then land at the target.", 
			                           GUILayout.ExpandWidth(false)))
				CFG.AP2.XToggle(Autopilot2.Deorbit);
		}
	}
}

