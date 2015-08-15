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
			[Persistent] public float LookAheadFilter    = 1;
			[Persistent] public int   NumRays            = 30;
			[Persistent] public float MinAltitudeFactor  = 2;
			[Persistent] public float MinClosingSpeed    = 4;
			[Persistent] public float NHVf               = 0.5f;
			public float DeltaAngle;

			public override void Init()
			{
				base.Init();
				DeltaAngle = MaxViewAngle/NumRays;
			}
		}
		static Config RAD { get { return TCAConfiguration.Globals.RAD; } }

		public Radar(VesselWrapper vsl) { VSL = vsl; }

		Vector3 Dir;
		float   ViewAngle;
		float   MaxDistance;
		float   DistanceAhead;
		float   ClosingSpeed;
		double  DetectedTime;
		float   DetectedSpeed = -1;
		int     Ray;

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

		#if DEBUG
		public void RadarBeam()
		{
			var d = Quaternion.AngleAxis(ViewAngle, VSL.refT.right)*Dir;
			GLUtils.GLTriangleMap(new Vector3[] { VSL.CoM-VSL.refT.right, VSL.CoM+VSL.refT.right, VSL.CoM+Dir*MaxDistance }, Color.green);
			GLUtils.GLTriangleMap(new Vector3[] { VSL.CoM-VSL.refT.right, VSL.CoM+VSL.refT.right, VSL.CoM+d*MaxDistance }, Color.red);
		}

		public override void Init()
		{
			base.Init();
			RenderingManager.AddToPostDrawQueue(1, RadarBeam);
		}

		public override void Reset()
		{
			base.Reset();
			RenderingManager.RemoveFromPostDrawQueue(1, RadarBeam);
		}
		#endif

		public override void UpdateState() { IsActive = CFG.ControlAltitude && CFG.AltitudeAboveTerrain && VSL.OnPlanet; }

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
			DetectedTime = -1; 
			DetectedSpeed = -1;
			VSL.AltitudeAhead = -1;
			CFG.NHVf = 1;
			Ray = 0;
		}

		public void Update()
		{
			if(!IsActive) return;
			Dir = VSL.AbsVerticalSpeed < 0?
				Vector3.ProjectOnPlane(SurfaceVelocity, VSL.refT.right).normalized :
				Vector3.Cross(VSL.refT.right, VSL.Up).normalized;
			ClosingSpeed = Vector3.Dot(SurfaceVelocity, Dir);
			if(ClosingSpeed <= RAD.MinClosingSpeed) { reset(); return; }
			//set state if previously detected an obstacle
			if(DetectedSpeed > 0) SetState(VSL.AbsVerticalSpeed < 0? TCAState.GroundCollision : TCAState.ObstacleAhead);
			//cast the ray
			ViewAngle     = RAD.DeltaAngle*Ray++;
			MaxDistance   = (DetectedSpeed < ClosingSpeed? ClosingSpeed : DetectedSpeed)*RAD.LookAheadTime;
			DistanceAhead = ray_distance(ViewAngle, MaxDistance);
//			Utils.Log("Alt {0}, MaxDist {1}, Dist {2}, Angle {3}", VSL.Altitude, MaxDistance, DistanceAhead, ViewAngle);//debug
			if(DistanceAhead < 0) { if(Ray > RAD.NumRays) reset(); return; }
			//reset the ray if something is found
			Ray = 0;
			//filter out terrain noise
			if(DetectedTime < 0) { DetectedTime = Planetarium.GetUniversalTime(); return; }
			if(Planetarium.GetUniversalTime() - DetectedTime < RAD.LookAheadFilter) return;
			//compute altitude and check for possible collision
			VSL.AltitudeAhead = DistanceAhead*Mathf.Sin(ViewAngle*Mathf.Deg2Rad);
			var dH = lowest_vessel_point();
			if(VSL.AltitudeAhead-dH*RAD.MinAltitudeFactor*(DetectedSpeed < 0? 1 : 2) < 0) //deadzone of twice the detection height
			{
				if(DetectedSpeed < 0) DetectedSpeed = ClosingSpeed;
				CFG.NHVf = Utils.ClampH(DistanceAhead/ClosingSpeed/RAD.LookAheadTime*VSL.MaxTWR*RAD.NHVf, 1);
			} 
			else { DetectedSpeed = -1; CFG.NHVf = 1; }
//			Utils.Log("ALtAhead {0}, dH {1}, Obstacle {2}, NHVf {3}, DetectedSpeed {4}", 
//			          VSL.AltitudeAhead, dH, 
//			          IsStateSet(TCAState.ObstacleAhead)||IsStateSet(TCAState.GroundCollision), 
//			          CFG.NHVf, DetectedSpeed);//debug
		}
	}
}

