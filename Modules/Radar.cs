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
	public abstract class RadarBase : TCAModule
	{
		protected Vector3  Dir;
		protected static   int RadarMask = (1 << 15 | 1 << LayerMask.NameToLayer("Parts") | 1);

		protected Vector3  Ori(Vector3 dir) { return VSL.C+dir*(VSL.R+0.1f); }
	}

	public class Radar : TCAModule
	{
		public class Config : ModuleConfig
		{
			new public const string NODE_NAME = "RAD";

			[Persistent] public float MaxViewAngle       = 15;
			[Persistent] public float LookAheadTime      = 20;
			[Persistent] public float AltitudeFilter     = 0.1f;
			[Persistent] public int   NumRays            = 30;
			[Persistent] public float MinAltitudeFactor  = 2;
			[Persistent] public float MinClosingSpeed    = 4;
			[Persistent] public float MinAltitude        = 10;
			[Persistent] public float NHVf               = 0.5f;
			public float DeltaAngle;

			public override void Init()
			{
				base.Init();
				MaxViewAngle = Utils.ClampH(MaxViewAngle, 80);
				DeltaAngle = MaxViewAngle/NumRays;
			}
		}
		static Config RAD { get { return TCAScenario.Globals.RAD; } }

		public Radar(VesselWrapper vsl) { VSL = vsl; }

		Vector3  Dir;
		float    ViewAngle;
		float    MaxDistance;
		float    DistanceAhead;
		float    ClosingSpeed;
		float    CollisionSpeed = -1;
		readonly Hit BestHit     = new Hit();
		readonly Hit DetectedHit = new Hit();
		static   int RadarMask = (1 << 15 | 1 << LayerMask.NameToLayer("Parts") | 1);
		int      Ray;

		public Vector3d SurfaceVelocity { get { return VSL.vessel.srf_velocity; } }

		float ray_distance(float angle, float dist)
		{
			if(VSL.refT == null) return -1;
			RaycastHit raycastHit;
			var dir = Quaternion.AngleAxis(angle, VSL.refT.right)*Dir;
			if(Physics.Raycast(VSL.C+dir*(VSL.R+0.1f), dir,
			                   out raycastHit, dist, RadarMask))
				return raycastHit.distance+VSL.R+0.1f;
			return -1;
		}

		public override void Init()
		{
			base.Init();
			#if DEBUG
//			RenderingManager.AddToPostDrawQueue(1, RadarBeam);
			#endif
		}

		#if DEBUG
		public void RadarBeam()
		{
			var d = Quaternion.AngleAxis(ViewAngle, VSL.refT.right)*Dir;
			var c = VSL.C+d*(VSL.R+0.1f);
			GLUtils.GLTriangleMap(new Vector3[] { VSL.CoM-VSL.refT.right*0.1f, VSL.CoM+VSL.refT.right*0.1f, VSL.CoM+Dir*MaxDistance }, Color.green);
			GLUtils.GLTriangleMap(new Vector3[] { c-VSL.refT.right*0.1f, c+VSL.refT.right*0.1f, c+d*MaxDistance }, DistanceAhead > 0? Color.magenta : Color.red);
		}

		public override void Reset()
		{
			base.Reset();
//			RenderingManager.RemoveFromPostDrawQueue(1, RadarBeam);
		}
		#endif

		public override void UpdateState() 
		{ 
			IsActive = VSL.OnPlanet && !VSL.LandedOrSplashed && 
				CFG.VF[VFlight.AltitudeControl] && CFG.AltitudeAboveTerrain; 
		}

		void reset()
		{
			BestHit.Reset();
			DetectedHit.Reset();
			CollisionSpeed = -1;
			VSL.AltitudeAhead = -1;
			CFG.NHVf = 1;
			Ray = 0;
		}

		public void Update()
		{
			if(!IsActive) return;
			if(VSL.HorizontalSpeed <= RAD.MinClosingSpeed && 
				(CFG.HF.Any(HFlight.Stop, HFlight.Anchor, HFlight.AnchorHere) || CFG.DesiredAltitude < RAD.MinAltitude))
			{ reset(); return; }
			//closing speed and starting ray direction
			Dir = Vector3.Cross(VSL.refT.right, VSL.Up).normalized;
			if(VSL.RelVerticalSpeed < 0 && VSL.AbsVerticalSpeed < 0 && Vector3.Dot(SurfaceVelocity, Dir) >= 0)
				Dir = Vector3.Lerp(Dir, 
				                   Vector3.ProjectOnPlane(SurfaceVelocity, VSL.refT.right).normalized,
				                   RAD.LookAheadTime/(VSL.Altitude/-VSL.RelVerticalSpeed)/VSL.MaxTWR);
			ClosingSpeed = Vector3.Dot(SurfaceVelocity, Dir);
			if(ClosingSpeed < RAD.MinClosingSpeed) ClosingSpeed = RAD.MinClosingSpeed;
			//update state if previously detected something
			if(CollisionSpeed > 0) SetState(VSL.AbsVerticalSpeed < 0? TCAState.GroundCollision : TCAState.ObstacleAhead);
			if(DetectedHit.Altitude >= 0)
				VSL.AltitudeAhead = VSL.AltitudeAhead < 0? DetectedHit.Altitude 
					: Utils.EWA(VSL.AltitudeAhead, DetectedHit.Altitude, RAD.AltitudeFilter);
			//cast the ray
			if(Ray > RAD.NumRays) reset();
			ViewAngle     = RAD.DeltaAngle*Ray;
			MaxDistance   = (CollisionSpeed < ClosingSpeed? ClosingSpeed : CollisionSpeed)*RAD.LookAheadTime;
			DistanceAhead = ray_distance(ViewAngle, MaxDistance);
//			Utils.Log("Alt {0}, AltAhead {1}, MaxDist {2}, Dist {3}, Angle {4}, Ray {5}", VSL.Altitude, VSL.AltitudeAhead, MaxDistance, DistanceAhead, ViewAngle, Ray);//debug
			Ray++;
			if(DistanceAhead < 0) return;
			//check the hit
			BestHit.Update(DistanceAhead, ViewAngle);
			if(BestHit.Altitude.Equals(0) || BestHit < DetectedHit || Ray > RAD.NumRays)
			{   //reset the ray if something is found
				DetectedHit.Copy(BestHit);
				BestHit.Reset();
				Ray = 0;
			}
//			DebugUtils.CSV(BestHit.Altitude, DetectedHit.Altitude, VSL.AltitudeAhead);
			if(DetectedHit.Altitude < 0) return;
			//check for possible collision
			if(VSL.AltitudeAhead-VSL.H*RAD.MinAltitudeFactor*(CollisionSpeed < 0? 1 : 2) < 0) //deadzone of twice the detection height
			{
				if(CollisionSpeed < 0) CollisionSpeed = ClosingSpeed;
				CFG.NHVf = Utils.ClampH(DetectedHit.Distance/ClosingSpeed/RAD.LookAheadTime*VSL.MaxTWR*RAD.NHVf, 1);
			} 
			else { CollisionSpeed = -1; CFG.NHVf = 1; }
//			Utils.Log("ALtAhead {0}, dH {1}, Obstacle {2}, NHVf {3}, DetectedSpeed {4}", 
//			          VSL.AltitudeAhead, dH, 
//			          IsStateSet(TCAState.ObstacleAhead)||IsStateSet(TCAState.GroundCollision), 
//			          CFG.NHVf, CollisionSpeed);//debug
		}

		class Hit
		{
			public float Distance = -1;
			public float Altitude = -1;
			public bool  Valid { get; private set; }

			public Hit() {}
			public Hit(float dist, float angle)
			{
				Distance = dist;
				Altitude = dist*Mathf.Sin(angle*Mathf.Deg2Rad);
				Valid = dist > 0;
			}

			public void Copy(Hit h)
			{
				Distance = h.Distance;
				Altitude = h.Altitude;
				Valid = h.Valid;
			}

			public bool Update(Hit h)
			{ 
				if(h < this) { Copy(h); return true; }
				return false;
			}
			public bool Update(float dist, float angle) 
			{ return Update(new Hit(dist, angle)); }


			public void Reset()
			{
				Distance = -1;
				Altitude = -1;
				Valid = false;
			}

			public static bool operator <(Hit h1, Hit h2) { return !h2.Valid || h1.Valid && h1.Altitude < h2.Altitude; }
			public static bool operator >(Hit h1, Hit h2) { return !h1.Valid || h2.Valid && h1.Altitude > h2.Altitude; }
		}
	}
}

