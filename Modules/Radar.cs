//   Radar.cs
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
	public class Radar : TCAModule
	{
		public class Config : ModuleConfig
		{
			new public const string NODE_NAME = "RAD";

			[Persistent] public float ViewAngle          = 15;
			[Persistent] public float MaxViewAngle       = 45;
			[Persistent] public float LookAheadTime      = 30;
			[Persistent] public float MinLookAheadTime   = 5;
			[Persistent] public float AvoidFactor        = 3;
			[Persistent] public float ClosingSpeedFactor = 10;
			[Persistent] public float NHVf               = 1;
		}
		static Config RAD { get { return TCAConfiguration.Globals.RAD; } }

		public Radar(VesselWrapper vsl) { VSL = vsl; }

		Vector3 Dir;
		float   TWRf;
		float   ViewAngle;
		float   LookAheadTime;
		float   MaxDistance;
		float   DistanceAhead, DistnanceAbove, DistanceBelow;
		float   ClosingSpeed;
		float   TimeAhead;
		bool    Avoiding;
		float   SavedVSpeed;
		float   NeededVSpeed;

		float ray_distance(float angle, float dist)
		{
			if(VSL.refT == null) return -1;
			RaycastHit raycastHit;
			if(Physics.Raycast(VSL.vessel.vesselTransform.position, 
			                   Quaternion.AngleAxis(angle, VSL.refT.right)*Dir,
			                   out raycastHit, dist, 32768))
				return raycastHit.distance;
			return -1;
		}

		public override void UpdateState() { IsActive = CFG.ControlAltitude && VSL.OnPlanet; }
		public override void Reset() { if(Avoiding) CFG.VerticalCutoff = SavedVSpeed; }

		public void Update()
		{
			if(!IsActive) return;
			Dir = Vector3.Cross(VSL.refT.right, VSL.Up).normalized;
			ClosingSpeed = Vector3.Dot(VSL.vessel.srf_velocity, Dir);
			if(ClosingSpeed <= 0) return;
			//look right ahead
			TWRf = Utils.ClampL(VSL.MaxTWR, 0.1f);
			ViewAngle     = Utils.ClampH(RAD.ViewAngle*TWRf/ClosingSpeed*RAD.ClosingSpeedFactor, RAD.MaxViewAngle);
			LookAheadTime = Utils.ClampL(RAD.LookAheadTime/TWRf, RAD.MinLookAheadTime);
			MaxDistance   = ClosingSpeed*LookAheadTime;
			DistanceAhead = ray_distance(0, MaxDistance);
			if(DistanceAhead > 0)
			{
				TimeAhead = DistanceAhead/ClosingSpeed;
				SetState(TCAState.ObstacleAhead);
				if(!Avoiding)
				{
					DistnanceAbove = ray_distance(-ViewAngle, MaxDistance*(1+Mathf.Tan(ViewAngle*Mathf.Deg2Rad)));
					if(DistnanceAbove > 0)
					{
						//try to fly over the obstacle
						Avoiding = true;
						SavedVSpeed = CFG.VerticalCutoff;
					}
				}
				else
				{
					NeededVSpeed = ClosingSpeed*Mathf.Sin(ViewAngle*Mathf.Deg2Rad*RAD.AvoidFactor);
					CFG.NHVf = TimeAhead/LookAheadTime*RAD.NHVf;
				}
			}
			else if(Avoiding)
			{
				//check if the obstacle is avoided
				var old_below = DistanceBelow;
				DistanceBelow = ray_distance(RAD.ViewAngle, MaxDistance*(1+Mathf.Tan(ViewAngle*Mathf.Deg2Rad)));
				if(TimeAhead > 0 && DistanceBelow <= old_below) 
				{
					NeededVSpeed = 0;
					TimeAhead -= TimeWarp.fixedDeltaTime;
				}
				else 
				{
					Avoiding = false;
					CFG.VerticalCutoff = SavedVSpeed;
				}
			}
			if(Avoiding) 
			{
				CFG.VerticalCutoff = NeededVSpeed;
				SetState(TCAState.AvoidingObstacle);
			}

			Utils.Log("Radar: ClosingSpeed {0}; ViewAngle {1}; LookAhead {2}s; MaxDist {3}; Dist {4}; Avoiding {5}; DAbove {6}; DBelow {7}; NeededSpeed {8}",
			          ClosingSpeed, ViewAngle, LookAheadTime, MaxDistance, DistanceAhead, Avoiding, DistnanceAbove, DistanceBelow, NeededVSpeed);//debug

//			var NHVf = (VSL.SpeedAhead < 0? Mathf.Clamp01(VSL.DistanceAhead/-VSL.SpeedAhead/VSL.TimeAhead) : 1);

//			//calculate altitude, vertical speed and acceleration
//			var upV = 0f;
//			if(CFG.ControlAltitude)
//			{
//				//update vessel altitude
//				var old_alt = Altitude;
//				var old_ahead = DistanceAhead;
//				UpdateAltitude();
//				if(CFG.AltitudeAboveTerrain)
//				{ //use relative vertical speed instead of absolute
//					upV = VerticalSpeedDisplay = (Altitude - old_alt)/TimeWarp.fixedDeltaTime;
//					if(old_ahead > 0 && DistanceAhead > 0)
//					{
//						SpeedAhead = (DistanceAhead - old_ahead)/TimeWarp.fixedDeltaTime;
//						if(SpeedAhead < 0 && DistanceAhead/-SpeedAhead < TimeAhead) 
//						{
//							SetState(TCAState.ObstacleAhead);
//							upV = Mathf.Min(VerticalSpeedDisplay, SpeedAhead);
//						}
//					}
//					else SpeedAhead = 0;
//					upV = Utils.WAverage(VerticalSpeed, upV);
//					Utils.Log("Alt {0}; Ahead {1}; SpA {2}; upV {3}; VDisp {4}; time2crash {5}", 
//					          Altitude, DistanceAhead, SpeedAhead, upV, VerticalSpeedDisplay,
//					          DistanceAhead/SpeedAhead);//debug
//				} 
//				else { upV = CoM_verticalSpeed; VerticalSpeedDisplay = upV; }
//			} else { upV = CoM_verticalSpeed; VerticalSpeedDisplay = upV; }
//			VerticalAccel = Utils.WAverage(VerticalAccel, (upV-VerticalSpeed)/TimeWarp.fixedDeltaTime);
//			VerticalSpeed = upV;
		}
	}
}

