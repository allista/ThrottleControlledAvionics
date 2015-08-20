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

			[Persistent] public float MaxViewAngle       = 15;
			[Persistent] public float LookAheadTime      = 20;
			[Persistent] public float AltitudeFilter     = 0.1f;
			[Persistent] public int   NumRays            = 30;
			[Persistent] public float MinAltitudeFactor  = 2;
			[Persistent] public float MinClosingSpeed    = 4;
			[Persistent] public float NHVf               = 0.5f;
			public float DeltaAngle;

			public override void Init()
			{
				base.Init();
				MaxViewAngle = Utils.ClampH(MaxViewAngle, 80);
				DeltaAngle = MaxViewAngle/NumRays;
			}
		}
		static Config RAD { get { return TCAConfiguration.Globals.RAD; } }

		public Radar(VesselWrapper vsl) { VSL = vsl; }

		Vector3  Dir;
		float    ViewAngle;
		float    MaxDistance;
		float    DistanceAhead;
		float    ClosingSpeed;
		float    CollisionSpeed = -1;
		readonly Hit BestHit     = new Hit();
		readonly Hit DetectedHit = new Hit();
		int      Ray;

		public Vector3d SurfaceVelocity { get { return VSL.vessel.srf_velocity; } }

		float ray_distance(float angle, float dist)
		{
			if(VSL.refT == null) return -1;
			RaycastHit raycastHit;
			if(Physics.Raycast(VSL.wCoM, 
			                   Quaternion.AngleAxis(angle, VSL.refT.right)*Dir,
			                   out raycastHit, dist, 32768))
				return raycastHit.distance;
			return -1;
		}

		public override void Init()
		{
			base.Init();
			#if DEBUG
			RenderingManager.AddToPostDrawQueue(1, RadarBeam);
			#endif
		}

		#if DEBUG
		public void RadarBeam()
		{
			var d = Quaternion.AngleAxis(ViewAngle, VSL.refT.right)*Dir;
			GLUtils.GLTriangleMap(new Vector3[] { VSL.CoM-VSL.refT.right, VSL.CoM+VSL.refT.right, VSL.CoM+Dir*MaxDistance }, Color.green);
			GLUtils.GLTriangleMap(new Vector3[] { VSL.CoM-VSL.refT.right, VSL.CoM+VSL.refT.right, VSL.CoM+d*MaxDistance }, DistanceAhead > 0? Color.magenta : Color.red);
		}

		public override void Reset()
		{
			base.Reset();
			RenderingManager.RemoveFromPostDrawQueue(1, RadarBeam);
		}
		#endif

		public override void UpdateState() { IsActive = CFG.VF[VFlight.AltitudeControl] && CFG.AltitudeAboveTerrain && VSL.OnPlanet; }

		public static Vector3[] BoundCorners(Bounds b)
		{
			var edges = new Vector3[8];
			Vector3 min = b.min;
			Vector3 max = b.max;
			edges[0] = new Vector3(min.x, min.y, min.z); //left-bottom-back
			edges[1] = new Vector3(min.x, min.y, max.z); //left-bottom-front
			edges[2] = new Vector3(min.x, max.y, min.z); //left-top-back
			edges[3] = new Vector3(min.x, max.y, max.z); //left-top-front
			edges[4] = new Vector3(max.x, min.y, min.z); //right-bottom-back
			edges[5] = new Vector3(max.x, min.y, max.z); //right-bottom-front
			edges[6] = new Vector3(max.x, max.y, min.z); //right-top-back
			edges[7] = new Vector3(max.x, max.y, max.z); //right-top-front
			return edges;
		}

		float lowest_vessel_point()
		{
			var lowest_h = 0f;
			var parts = VSL.vessel.parts;
			var down = Vector3.Cross(VSL.refT.right, Dir);
			for(int i = 0, partsCount = parts.Count; i < partsCount; i++)
			{
				Part p = parts[i];
				if(p == null) continue;
				foreach(var m in p.FindModelComponents<MeshFilter>())
				{
					//skip meshes without renderer
					if(m.renderer == null || !m.renderer.enabled) continue;
					var bounds = BoundCorners(m.sharedMesh.bounds);
					for(int j = 0, boundsLength = bounds.Length; j < boundsLength; j++)
					{
						var h = Vector3.Dot(m.transform.TransformPoint(bounds[j])-VSL.wCoM, down);
						if(h > lowest_h) lowest_h = h;
					}
				}
			}
			return lowest_h;
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
			//closing speed and starting ray direction
			Dir = Vector3.Cross(VSL.refT.right, VSL.Up).normalized;
			if(VSL.RelVerticalSpeed < 0 && VSL.AbsVerticalSpeed < 0)
				Dir = Vector3.Lerp(Dir,
				                   Vector3.ProjectOnPlane(SurfaceVelocity, VSL.refT.right).normalized,
				                   RAD.LookAheadTime/(VSL.Altitude/-VSL.RelVerticalSpeed)/VSL.MaxTWR);
			ClosingSpeed = Vector3.Dot(SurfaceVelocity, Dir);
			if(VSL.HorizontalSpeed <= RAD.MinClosingSpeed) { reset(); return; }
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
			var dH = lowest_vessel_point();
			if(VSL.AltitudeAhead-dH*RAD.MinAltitudeFactor*(CollisionSpeed < 0? 1 : 2) < 0) //deadzone of twice the detection height
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

