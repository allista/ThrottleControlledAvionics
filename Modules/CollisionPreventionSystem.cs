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
			[Persistent] public float MaxAvoidanceSpeed = 100f;
			[Persistent] public float LatAvoidMinVelSqr = 0.25f;
			[Persistent] public float LookAheadTime = 1f;

			[Persistent] public PID_Controller PID = new PID_Controller(0.5f, 0f, 0.5f, 0, 10);
		}
		static Config CPS { get { return TCAScenario.Globals.CPS; } }

		public CollisionPreventionSystem(VesselWrapper vsl) { VSL = vsl; }

		public override void UpdateState() 
		{ IsActive = CFG.HF && VSL.OnPlanet && !CFG.NeededHorVelocity.IsZero(); }

		static int RadarMask = (1 | 1 << LayerMask.NameToLayer("Parts"));
		List<Vector3d> Corrections = new List<Vector3d>();
		IEnumerator scanner;
		PIDvd_Controller pid = new PIDvd_Controller();
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
//			var c = VSL.C+Dir.normalized*(VSL.R+0.1f);
//			GLUtils.GLTriangleMap(new Vector3[] { c-VSL.refT.right*0.1f, c+VSL.refT.right*0.1f, c+Dir }, Color.green);
			if(!CFG.CourseCorrection.IsZero())
				GLUtils.GLTriangleMap(new Vector3[] { VSL.wCoM-VSL.refT.forward*0.1f, VSL.wCoM+VSL.refT.forward*0.1f, VSL.wCoM+CFG.CourseCorrection }, Color.magenta);
		}

		public override void Reset()
		{
			base.Reset();
			RenderingManager.RemoveFromPostDrawQueue(1, RadarBeam);
		}
		#endif

		bool Corection(Vessel v, out Vector3d c)
		{
			c = Vector3d.zero;
			//calculate distance
			Dir = (v.CurrentCoM-VSL.wCoM);
			var Dist = Dir.magnitude;
			Dir.Normalize();
			//first try to get TCA from other vessel and get vessel's R
			var vR = 0f;
			var tca = ModuleTCA.EnabledTCA(v);
			if(tca != null) vR = tca.VSL.R;
			else //do a raycast
			{
				RaycastHit raycastHit;
				if(Physics.Raycast(VSL.C+Dir*(VSL.R+0.1f), Dir,
				               out raycastHit, Dist, RadarMask))
					vR = Utils.ClampL(Dist-raycastHit.distance-VSL.R-0.1f, 0);
			}
			//filter vessels on non-collision courses
			var dV  = VSL.vessel.srf_velocity-v.srf_velocity+(VSL.vessel.acceleration-v.acceleration)*CPS.LookAheadTime;
			var dVn = dV.normalized;
			var cosA = Vector3.Dot(Dir, dVn);
			if(cosA <= 0) goto check_distance;
			//calculate minimum separation
			var alpha = Mathf.Acos(Mathf.Clamp(cosA, -1, 1));
			var separation = Dist*Mathf.Sin(alpha);
			var min_sep = (vR+VSL.R)*2;
			if(separation > min_sep) goto check_distance;
			//evaluate time to collision
			var vDist = dVn*Dist*Mathf.Cos(alpha);
			var vTime = vDist.magnitude/dV.magnitude;
			if(vTime > CPS.SafeTime/Utils.Clamp(Mathf.Abs(Vector3.Dot(VSL.wMaxAngularA, vDist.normalized)), 0.01f, 1f)) 
				goto check_distance;
			//calculate course correction
			c = (vDist-Dir*Dist).normalized * (min_sep-separation) 
				/ vTime;// * (10 - 9*Utils.Clamp((Dist-VSL.R-vR)/CPS.SafeDistance, 0, 1));
//			Log("c: {0}\nvTime {1}\ncollision {2}", c, vTime, Dist <= (vR+VSL.R));//debug
			return true;
			//if distance is not safe, correct course anyway
			check_distance:
			if(Dist-VSL.R-vR >= CPS.SafeDistance) return !c.IsZero();
			Vector3d dc;
			var dir_avoid = Dir*Utils.ClampH(Dist-CPS.SafeDistance, -0.1f);
			if(CFG.NeededHorVelocity.sqrMagnitude > CPS.LatAvoidMinVelSqr)
			{
				var lat_avoid = Vector3d.Cross(VSL.Up, CFG.NeededHorVelocity.normalized);
				dc = Vector3d.Dot(dir_avoid, lat_avoid) >= 0? 
					lat_avoid*dir_avoid.magnitude :
					lat_avoid*-dir_avoid.magnitude;
			}
			else dc = dir_avoid;
			dc /= CPS.SafeTime/2;
			if(dc.sqrMagnitude > c.sqrMagnitude) c = dc;
			return true;
		}

		IEnumerator vessel_scanner()
		{
			var corrections = new List<Vector3d>();
			var vi = FlightGlobals.Vessels.GetEnumerator();
			while(true)
			{
				try { if(!vi.MoveNext()) break; }
				catch { break; }
				var v = vi.Current;
				if(v == null || v.packed || !v.loaded || v == VSL.vessel) continue;
				Vector3d c;
				if(Corection(v, out c))
					corrections.Add(c);
				yield return null;
			}
			Corrections = corrections;
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
			if(!IsActive || VSL.refT == null) return;
			scan();
			if(Corrections.Count == 0) { pid.Reset(); return; }
			var correction = Vector3d.zero;
			for(int i = 0, count = Corrections.Count; i < count; i++)
				correction += Corrections[i];
			pid.Update(correction);
			//correct needed vertical speed
			if(CFG.VF[VFlight.AltitudeControl])
				CFG.VerticalCutoff += (float)Vector3d.Dot(pid.Action, VSL.Up);
			//correct horizontal course
			CFG.CourseCorrection = Vector3d.Exclude(VSL.Up, pid.Action);

//			Log("CourseCorrection:\n{0}\n{1}", 
//			    CFG.CourseCorrection, 
//			    Vector3d.Dot(obstacle_direction, avoidance_direction) >= 0? 
//			    avoidance_direction*obstacle_direction.magnitude :
//			    -avoidance_direction*obstacle_direction.magnitude);//debug
		}
	}
}

