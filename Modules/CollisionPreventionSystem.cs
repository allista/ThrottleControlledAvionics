//   CollisionPreventionSystem.cs
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
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace ThrottleControlledAvionics
{
	public class CollisionPreventionSystem : TCAModule
	{
		public class Config : ModuleConfig
		{
			new public const string NODE_NAME = "CPS";

			[Persistent] public float SafeDistance = 30f;
			[Persistent] public float SafeTime     = 5f;
			[Persistent] public float MaxAvoidanceSpeed = 10f;

			[Persistent] public PID_Controller PID = new PID_Controller(0.5f, 0f, 0.5f, 0, 10);
		}
		static Config CPS { get { return TCAScenario.Globals.CPS; } }

		public CollisionPreventionSystem(VesselWrapper vsl) { VSL = vsl; }

		public override void UpdateState() 
		{ IsActive = CFG.HF.Any(HFlight.CruiseControl, HFlight.NoseOnCourse, HFlight.Move) && VSL.OnPlanet && !CFG.NeededHorVelocity.IsZero(); }

		static int RadarMask = (1 | 1 << LayerMask.NameToLayer("Parts"));
		List<Vector3d> Distances = new List<Vector3d>();
		IEnumerator scanner;
		PIDvd_Controller pid = new PIDvd_Controller();
		Vector3d obstacle_direction, avoidance_direction;
		Vector3 Dir;

		public override void Init()
		{
			base.Init();
			pid.setPID(CPS.PID);
			pid.Max = CPS.MaxAvoidanceSpeed;
			pid.Reset();
			#if DEBUG
			RenderingManager.AddToPostDrawQueue(1, RadarBeam);
			#endif
		}

		#if DEBUG
		public void RadarBeam()
		{
			var c = VSL.C+Dir*(VSL.R+0.1f);
			GLUtils.GLTriangleMap(new Vector3[] { c-VSL.refT.right*0.1f, c+VSL.refT.right*0.1f, c+Dir*CPS.SafeDistance }, Color.green);
			if(!CFG.CourseCorrection.IsZero())
				GLUtils.GLTriangleMap(new Vector3[] { VSL.wCoM-VSL.refT.forward*0.1f, VSL.wCoM+VSL.refT.forward*0.1f, VSL.wCoM+CFG.CourseCorrection.normalized*10 }, Color.magenta);
		}

		public override void Reset()
		{
			base.Reset();
			RenderingManager.RemoveFromPostDrawQueue(1, RadarBeam);
		}
		#endif

		bool DistanceTo(Vessel v, out Vector3d d)
		{ 
			d = Vector3d.zero;
			Dir = (v.CurrentCoM-VSL.wCoM).normalized;
			var mdist = CPS.SafeDistance;
			var dv = v.srf_velocity-VSL.vessel.srf_velocity;
			if(Vector3d.Dot(dv, VSL.vessel.srf_velocity) < 0)
				mdist += (float)dv.magnitude*CPS.SafeTime;
			RaycastHit raycastHit;
			if(Physics.Raycast(VSL.C+Dir*(VSL.R+0.1f), Dir,
				out raycastHit, mdist, RadarMask))
			{
				d = Dir * -raycastHit.distance/mdist;
				return true;
			}
			return false;
		}

		IEnumerator vessel_scanner()
		{
			var distances = new List<Vector3d>();
			var vi = FlightGlobals.Vessels.GetEnumerator();
			while(true)
			{
				try { if(!vi.MoveNext()) break; }
				catch { break; }
				var v = vi.Current;
				if(v == null || v.packed || !v.loaded) continue;
				Vector3d d;
				if(DistanceTo(v, out d))
					distances.Add(d);
				yield return null;
			}
			Distances = distances;
		}

		bool scan()
		{
			if(scanner == null) scanner = vessel_scanner();
			if(scanner.MoveNext()) return true;
			scanner = null;
			return false;
		}

		public void Update()
		{
			CFG.CourseCorrection = Vector3d.zero;
			if(!IsActive) return;
			scan();
//			Log("Distances: {0}", Distances.Count);
			if(Distances.Count == 0) { pid.Reset(); return; }
			obstacle_direction = Vector3d.zero;
			for(int i = 0, DistancesCount = Distances.Count; i < DistancesCount; i++)
			{
				var d = Distances[i];
				if(d.IsZero()) continue;
				obstacle_direction += d 
					* Utils.ClampL(1/d.magnitude - 1, 0) 
					* CPS.MaxAvoidanceSpeed/DistancesCount;
			}
//			if(obstacle_direction.sqrMagnitude < 0.25) return;
			obstacle_direction  = Vector3d.Exclude(VSL.Up, obstacle_direction);
			avoidance_direction = Vector3d.Cross(VSL.Up, CFG.NeededHorVelocity.normalized);
			pid.Update(Vector3d.Dot(obstacle_direction, avoidance_direction) >= 0? 
				avoidance_direction*obstacle_direction.magnitude :
				-avoidance_direction*obstacle_direction.magnitude);
			CFG.CourseCorrection = pid.Action;
//			Log("CourseCorrection:\n{0}\n{1}", 
//			    CFG.CourseCorrection, 
//			    Vector3d.Dot(obstacle_direction, avoidance_direction) >= 0? 
//			    avoidance_direction*obstacle_direction.magnitude :
//			    -avoidance_direction*obstacle_direction.magnitude);//debug
		}
	}
}

