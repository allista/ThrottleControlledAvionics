//   PointNavigator.cs
//
//  Author:
//       Allis Tauri <allista@gmail.com>
//
//  Copyright (c) 2015 Allis Tauri
//
// This work is licensed under the Creative Commons Attribution 4.0 International License. 
// To view a copy of this license, visit http://creativecommons.org/licenses/by/4.0/ 
// or send a letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.

using System;
using UnityEngine;

namespace ThrottleControlledAvionics
{
	public class PointNavigator : TCAModule
	{
		public class Config : ModuleConfig
		{
			new public const string NODE_NAME = "PN";

			[Persistent] public float MinDistance        = 30;
			[Persistent] public float MinTime            = 10;
			[Persistent] public float OnPathMinDistance  = 300;
			[Persistent] public float MinSpeed           = 10;
			[Persistent] public float MaxSpeed           = 300;
			[Persistent] public float DeltaSpeed         = 10f;
			[Persistent] public float DeltaSpeedF        = 0.5f;
			[Persistent] public float FallingSensitivity = 10f;
			[Persistent] public float FallingCorrection  = 3f;
			[Persistent] public float DistanceF          = 50;
			[Persistent] public float DirectNavThreshold = 1;
			[Persistent] public float GCNavStep          = 0.1f;
			[Persistent] public PID_Controller DistancePID = new PID_Controller(0.5f, 0f, 0.5f, 0, 100);

			public override void Init()
			{
				base.Init();
				DirectNavThreshold *= Mathf.Deg2Rad;
				GCNavStep *= Mathf.Deg2Rad;
			}
		}
		static Config PN { get { return TCAConfiguration.Globals.PN; } }
		public PointNavigator(VesselWrapper vsl) { VSL = vsl; }

		readonly PIDf_Controller pid = new PIDf_Controller();
		WayPoint target;
		float DeltaSpeed;

		public override void Init()
		{
			base.Init();
			pid.setPID(PN.DistancePID);
			pid.Reset();
			if(CFG.GoToTarget) GoToTarget();
			else if(CFG.FollowPath) FollowPath();
		}

		public override void UpdateState() { IsActive = (CFG.GoToTarget || CFG.FollowPath) && VSL.OnPlanet; }

		public void GoToTarget(bool enable = true)
		{
			if(enable && !VSL.HasTarget) return;
			CFG.GoToTarget = enable;
			if(CFG.GoToTarget) 
			{
				CFG.FollowPath = false;
				start_to(new WayPoint(VSL.vessel.targetObject));
			}
			else finish();
		}

		public void FollowPath(bool enable = true)
		{
			if(enable && CFG.Waypoints.Count == 0) return;
			CFG.FollowPath = enable;
			if(CFG.FollowPath) 
			{
				CFG.GoToTarget = false;
				start_to(CFG.Waypoints.Peek());	
			}
			else finish();
		}

		void start_to(WayPoint wp)
		{
			target = wp;
			FlightGlobals.fetch.SetVesselTarget(wp.GetTarget());
			BlockSAS();
			pid.Reset();
			VSL.UpdateHorizontalStats();
			CFG.CruiseControl = true;
			CFG.KillHorVel = false;
		}

		void finish()
		{
			target = null;
			FlightGlobals.fetch.SetVesselTarget(null);
			CFG.Starboard = Vector3.zero;
			CFG.NeededHorVelocity = Vector3d.zero;
			CFG.CruiseControl = false;
			CFG.KillHorVel = true;
			CFG.GoToTarget = false;
			CFG.FollowPath = false;
		}

		public void Update()
		{
			if(!IsActive || target == null) return;
			target.Update(VSL.vessel.mainBody);
			//calculate direct distance
			var vdir = Vector3.ProjectOnPlane(target.GetTransform().position-VSL.vessel.transform.position, VSL.Up);
			var distance = vdir.magnitude;
			//if it is greater that the threshold (in radians), use Great Circle navigation
			if(distance/VSL.vessel.mainBody.Radius > PN.DirectNavThreshold)
			{
				var next = target.PointFrom(VSL.vessel, 0.1);
				distance = (float)target.DistanceTo(VSL.vessel);
				vdir = Vector3.ProjectOnPlane(VSL.vessel.mainBody.GetWorldSurfacePosition(next.Lat, next.Lon, VSL.vessel.altitude)
				                              -VSL.vessel.transform.position, VSL.Up);
			}
			vdir.Normalize();
			//check if we have arrived to the target
			if(distance < PN.MinDistance) 
			{
				if(CFG.FollowPath)
				{
					while(CFG.Waypoints.Count > 0 && CFG.Waypoints.Peek() == target) CFG.Waypoints.Dequeue();
					if(CFG.Waypoints.Count > 0) { start_to(CFG.Waypoints.Peek()); return; }
				}
				//keep trying if we came in too fast
				if(distance/Vector3d.Dot(VSL.HorizontalVelocity, vdir) > PN.MinTime)
				{ finish(); return; }
			}
			//don't slow down on intermediate waypoints too much
			if(CFG.FollowPath && CFG.Waypoints.Count > 1 && distance < PN.OnPathMinDistance)
				distance = PN.OnPathMinDistance;
			//tune the pid and update needed velocity
			pid.Min = 0;
			pid.Max = CFG.MaxNavSpeed;
			pid.D   = PN.DistancePID.D*VSL.M/Utils.ClampL(VSL.Thrust.magnitude/TCAConfiguration.G, 1);
			pid.Update(distance*PN.DistanceF);
			//increase the needed velocity slowly
			var cur_vel = Utils.ClampL((float)Vector3d.Dot(VSL.vessel.srf_velocity, vdir), 1);
			var mtwr = Utils.ClampL(VSL.MaxTWR, 0.1f);
			DeltaSpeed = PN.DeltaSpeed*mtwr*Mathf.Pow(PN.DeltaSpeed/(cur_vel+PN.DeltaSpeed), PN.DeltaSpeedF);
			//make a correction if falling or flyin too low
			if(IsStateSet(TCAState.LoosingAltitude))
			{
				DeltaSpeed /= 1-Utils.ClampH(VSL.VerticalSpeed, 0)*PN.FallingSensitivity
					/(CFG.AltitudeAboveTerrain? VSL.Altitude : 1);
				if(DeltaSpeed < 1) 
				{
					var a = Utils.ClampL(PN.FallingCorrection/-VSL.VerticalSpeed, PN.FallingCorrection);
					cur_vel *= (a-1+DeltaSpeed)/a;
				}
			}
			if(CFG.ControlAltitude)
			{
				var alt_error = (CFG.DesiredAltitude-VSL.Altitude);
				if(CFG.AltitudeAboveTerrain) alt_error /= VSL.Altitude*9;
				if(alt_error > 0) DeltaSpeed /= 1+alt_error;
			}
			//set needed velocity and starboard
			CFG.NeededHorVelocity = pid.Action-cur_vel > DeltaSpeed? vdir*(cur_vel+DeltaSpeed) : vdir*pid.Action;
			CFG.Starboard = VSL.GetStarboard(CFG.NeededHorVelocity);
//			Utils.Log("Delta {0}; NHV {1}", 
//			          DeltaSpeed, CFG.NeededHorVelocity);//debug
//			Utils.Log("Distance: {0}, max {1}, err {2}, nvel {3}", 
//			          distance, PN.OnPathMinDistance, distance*PN.DistanceF, pid.Action);//debug
		}
	}
}

