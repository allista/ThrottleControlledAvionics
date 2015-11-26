//  Author:
//       Allis Tauri <allista@gmail.com>
//
//  Copyright (c) 2015 Allis Tauri
//
// This work is licensed under the Creative Commons Attribution-ShareAlike 4.0 International License. 
// To view a copy of this license, visit http://creativecommons.org/licenses/by-sa/4.0/ 
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

			[Persistent] public float MinDistance       = 5f;
			[Persistent] public float SafeDistance      = 30f;
			[Persistent] public float SafeTime          = 5f;
			[Persistent] public float MaxAvoidanceSpeed = 100f;
			[Persistent] public float LatAvoidMinVelSqr = 0.25f;
			[Persistent] public float LookAheadTime     = 1f;
			[Persistent] public float ManeuverTimer     = 3f;
			[Persistent] public float LowPassF          = 0.5f;
		}
		static Config CPS { get { return TCAScenario.Globals.CPS; } }

		public CollisionPreventionSystem(ModuleTCA tca) { TCA = tca; }

		protected override void UpdateState() 
		{ 
			IsActive = CFG.HF && VSL.OnPlanet && !VSL.LandedOrSplashed && VSL.refT != null; 
			if(IsActive) return;
			Correction = Vector3.zero;
		}

		static int RadarMask = (1 | 1 << LayerMask.NameToLayer("Parts"));
		readonly HashSet<Guid> Dangerous = new HashSet<Guid>();
		List<Vector3d> Corrections = new List<Vector3d>();
		Vector3 Correction;
		IEnumerator scanner;
		readonly LowPassFilterV filter = new LowPassFilterV();
		readonly Timer ManeuverTimer = new Timer();

		public override void Init()
		{
			base.Init();
			ManeuverTimer.Period = CPS.ManeuverTimer;
			#if DEBUG
			RenderingManager.AddToPostDrawQueue(1, RadarBeam);
			#endif
		}

		#if DEBUG
//		Vector3 Dir, DeltaV, Maneuver;
		bool Collided;
		public void RadarBeam()
		{
			if(!IsActive || VSL == null || VSL.vessel == null) return;
			if(!filter.Value.IsZero())
				GLUtils.GLVec(VSL.wCoM, filter.Value, Color.magenta);
//			if(VSL.IsActiveVessel)
//			{
//				if(!Dir.IsZero())
//					GLUtils.GLVec(VSL.wCoM, Dir, Color.cyan);
//				if(!DeltaV.IsZero())
//					GLUtils.GLVec(VSL.wCoM, DeltaV, Color.green);
//				if(!Maneuver.IsZero() && !DeltaV.IsZero())
//					GLUtils.GLVec(VSL.wCoM+DeltaV, Maneuver, Color.yellow);
//				if(!Maneuver.IsZero() && !DeltaV.IsZero())
//					GLUtils.GLVec(VSL.wCoM, Maneuver+DeltaV, Color.red);
//			}
			GLUtils.GLBounds(VSL.EnginesExhaust, VSL.refT, Collided? Color.red : Color.white);
//			for(int i = 0, VSLEnginesCount = VSL.Engines.Count; i < VSLEnginesCount; i++)
//			{
//				var e = VSL.Engines[i];
//				for(int j = 0, eenginethrustTransformsCount = e.engine.thrustTransforms.Count; j < eenginethrustTransformsCount; j++)
//				{
//					var t = e.engine.thrustTransforms[j];
//					GLUtils.GLVec(t.position, t.forward * e.engine.exhaustDamageMaxRange, Color.yellow);
//				}
//			}
		}

		public override void Reset()
		{
			base.Reset();
			RenderingManager.RemoveFromPostDrawQueue(1, RadarBeam);
		}
		#endif

		static float SafeTime(VesselWrapper vsl, Vector3d dVn)
		{ return CPS.SafeTime/Utils.Clamp(Mathf.Abs(Vector3.Dot(vsl.wMaxAngularA, dVn)), 0.01f, 1f); }

		public static bool AvoidStatic(VesselWrapper vsl, Vector3d dir, float dist, Vector3d dV, out Vector3d maneuver)
		{
			maneuver = Vector3d.zero;
			var dVn = dV.normalized;
			var cosA = Mathf.Clamp(Vector3.Dot(dir, dVn), -1, 1);
			if(cosA <= 0) return false;
			var sinA = Mathf.Sqrt(1-cosA*cosA);
			var vdist = dist*cosA;
			var min_separation = dist*sinA;
			var sep_threshold = vsl.R*2;
			if(min_separation > sep_threshold ||
			   min_separation > vsl.R && vdist < min_separation) return false;
			maneuver = (dVn*cosA-dir).normalized;
			var vTime = dist*cosA/dV.magnitude;
			if(vTime > SafeTime(vsl, dVn)) return false;
			maneuver *= (sep_threshold-min_separation) / Math.Sqrt(vTime);
			return true;
		}

		#if !DEBUG
		public static
		#endif
		bool AvoideVessel(VesselWrapper vsl, Vector3d dir, float dist, Vector3d dV, 
		                                float r2, Bounds exhaust2, Transform refT2,
		                                out Vector3d maneuver, float threshold = -1)
		{
			maneuver = Vector3d.zero;
			//filter vessels on non-collision courses
			var dVn = dV.normalized;
			var cosA = Mathf.Clamp(Vector3.Dot(dir, dVn), -1, 1);
			var collision_dist = Mathf.Max(vsl.R, dist-vsl.DistToExhaust(vsl.wCoM+dir*dist)) +
				Mathf.Max(r2, dist-Mathf.Sqrt(exhaust2.SqrDistance(refT2.InverseTransformPoint(vsl.wCoM))));
//			vsl.Log("R {0}, R2 {1}; r {2}, r2 {3}", vsl.R, r2, 
//			        dist-vsl.DistToExhaust(vsl.wCoM+dir*dist), 
//			        dist-Mathf.Sqrt(exhaust2.SqrDistance(refT2.InverseTransformPoint(vsl.wCoM))));//debug
//			Utils.Log("{0}: cosA: {1}, threshold {2}", vsl.vessel.vesselName, cosA, threshold);//debug
			if(cosA <= 0) goto check_distance;
			if(threshold < 0) threshold = CPS.MinDistance;
			var sinA = Mathf.Sqrt(1-cosA*cosA);
			var min_separation = dist*sinA;
			var sep_threshold = collision_dist+threshold;
//			Utils.Log("{0}: min_sep > sep_thresh: {1} > {2}, threshold {3}", 
//			          vsl.vessel.vesselName, min_separation, sep_threshold, threshold);//debug
			if(min_separation > sep_threshold) goto check_distance;
			//calculate time to collision
			var vDist = 0f;
			var alpha = Mathf.Acos(cosA);
			var dVm   = dV.magnitude;
			if(sinA <= 0) vDist = dist-sep_threshold;
			else if(dist > sep_threshold) vDist = sep_threshold*Mathf.Sin(Mathf.Asin(min_separation/sep_threshold)-alpha)/sinA;
			var vTime = Utils.ClampL(vDist, 0.1f)/dVm;
//				Utils.Log("{0}: vTime > SafeTime: {1} > {2}", 
//				          vsl.vessel.vesselName, vTime, CPS.SafeTime/Utils.Clamp(Mathf.Abs(Vector3.Dot(vsl.wMaxAngularA, dVn)), 0.01f, 1f));//debug
			if(vTime > SafeTime(vsl, dVn)) goto check_distance;
			//calculate maneuver
			Vector3d side;
			if(cosA < 0.9) side = (dVn*cosA-dir).normalized;
			else if(Math.Abs(Vector3d.Dot(dVn, vsl.Up)) < 0.9) 
				side = Vector3d.Cross(dVn, vsl.Up).normalized;
			else side = Vector3d.Cross(dVn, vsl.Fwd).normalized;
//			if(dist > sep_threshold)
//			{
//				var beta = Mathf.Asin(sep_threshold/dist)-alpha;
//				maneuver = (Mathf.Sin(beta)*side + dVn*(Mathf.Cos(beta)-1)).normalized;
//			}
//			else 
			maneuver  = side;
			maneuver += vsl.Up*Mathf.Sign(Vector3.Dot(dir, vsl.Up))*(min_separation/sep_threshold-1);
			maneuver *= (sep_threshold-min_separation) / Math.Sqrt(vTime);
//			maneuver = (-vsl.Up*Mathf.Sign(Vector3.Dot(dir, vsl.Up))*(1-min_separation/sep_threshold) + 
//			            (dVn*cosA-dir).normalized).normalized * (sep_threshold-min_separation) / vTime;
//			vsl.Log("\ndist {0}\ndV {1}\nmaneuver: {2}\n" +
//			        "vTime {3}, vDist {4}, min sep {5}, sep_thresh {6}", 
//			        dir*dist, dV, maneuver, vTime, vDist, min_separation, sep_threshold);//debug
			#if DEBUG
			Collided |= dist < collision_dist;
			#endif
			//if distance is not safe, correct course anyway
			check_distance: 
			var collision = !maneuver.IsZero();
			dist -= collision_dist;
			if(dist < threshold)
			{
				var dist_to_safe = Utils.ClampH(dist-threshold, -0.01f);
				var dc = dir*dist_to_safe;
				if(vsl.NeededHorVelocity.sqrMagnitude > CPS.LatAvoidMinVelSqr)
				{
					var lat_avoid = Vector3d.Cross(vsl.Up, vsl.NeededHorVelocity.normalized);
					dc = Vector3d.Dot(dc, lat_avoid) >= 0? 
						lat_avoid*dist_to_safe :
						lat_avoid*-dist_to_safe;
				}
				if(dc.sqrMagnitude > 0.25) maneuver += dc/CPS.SafeTime*2;
//				vsl.Log("adding safe distance correction: {0}", dc/CPS.SafeTime*2);//debug
			}
//			#if DEBUG
//			Dir = dir*dist;
//			DeltaV = dV;
//			Maneuver = maneuver;
//			#endif
			return collision;
		}

		bool ComputeManeuver(Vessel v, out Vector3d maneuver)
		{
			maneuver = Vector3d.zero;
			//calculate distance
			var dir = (v.CurrentCoM-VSL.wCoM);
			var dist = dir.magnitude;
			dir /= dist;
			//first try to get TCA from other vessel and get vessel's R and Exhaust
			var vR = 0f;
			var vE = new Bounds();
			Transform vT = null;
			var tca = ModuleTCA.EnabledTCA(v);
			if(tca != null) 
			{
//				if(tca.CPS != null && 
//				   tca.CPS.IsActive && 
//				   VSL.M > tca.VSL.M &&
//				   VSL.vessel.srfSpeed > v.srfSpeed) //test 
//					return false;
				vR = tca.VSL.R;
				vE = tca.VSL.EnginesExhaust;
				vT = tca.VSL.refT;
			}
			else //do a raycast
			{
				RaycastHit raycastHit;
				if(Physics.SphereCast(VSL.C+dir*(VSL.R+0.1f), VSL.R, dir,
				               out raycastHit, dist, RadarMask))
					vR = (raycastHit.point-v.CurrentCoM).magnitude;
				vE = v.EnginesExhaust();
				vT = v.ReferenceTransform;
			}
			//compute course correction
			var dV = VSL.vessel.srf_velocity-v.srf_velocity+(VSL.vessel.acceleration-v.acceleration)*CPS.LookAheadTime;
			var thrershold = -1f;
			if(v.LandedOrSplashed) thrershold = 0;
			else if(Dangerous.Contains(v.id)) thrershold = CPS.SafeDistance;
			if(AvoideVessel(VSL, dir, dist, dV, vR, vE, vT, out maneuver, thrershold))
				Dangerous.Add(v.id);
			else Dangerous.Remove(v.id);
			return !maneuver.IsZero();
		}

		IEnumerator vessel_scanner()
		{
			#if DEBUG
			Collided = false;
			#endif
			var corrections = new List<Vector3d>();
			var vi = FlightGlobals.Vessels.GetEnumerator();
			while(true)
			{
				try { if(!vi.MoveNext()) break; }
				catch { break; }
				var v = vi.Current;
				if(v == null || v.packed || !v.loaded || v == VSL.vessel) continue;
				Vector3d maneuver;
				if(ComputeManeuver(v, out maneuver))
					corrections.Add(maneuver);
				yield return null;
			}
			Corrections = corrections;
		}

		bool scan()
		{
			if(scanner == null) scanner = vessel_scanner();
			if(scanner.MoveNext()) return false;
			scanner = null;
			return true;
		}

		protected override void Update()
		{
			if(!IsActive) return;
			if(scan())
			{
				var correction = Vector3d.zero;
				for(int i = 0, count = Corrections.Count; i < count; i++)
					correction += Corrections[i];
				if(correction.IsZero())
				{ 
					if(ManeuverTimer.Check) 
					{ 
						Dangerous.Clear();
						Correction = Vector3.zero;
						filter.Reset();
						return; 
					}
				}
				else 
				{
					ManeuverTimer.Reset();
					Correction = correction;
				}
			}
			if(Correction.IsZero()) return;
			filter.Update(Correction.ClampComponents(-CPS.MaxAvoidanceSpeed, CPS.MaxAvoidanceSpeed));
			//correct needed vertical speed
			if(CFG.VF[VFlight.AltitudeControl])
			{
				var dVSP = (float)Vector3d.Dot(filter.Value, VSL.Up);
				if(dVSP > 0 || 
				   VSL.RelAltitude-VSL.H +
				   (dVSP+VSL.RelVerticalSpeed)*CPS.LookAheadTime > 0)
					CFG.VerticalCutoff += dVSP;
//				else dVSP = 0;//debug
//				Log("\nCorrection {0}\nAction {1}\ndVSP {2}\ncorrectins:{3}", 
//				    Correction, filter.Value, dVSP,
//				    Corrections.Aggregate("\n", (s, v) => s+v+"\n"));//debug
			}
			//correct horizontal course
			VSL.CourseCorrections.Add(Vector3d.Exclude(VSL.Up, filter.Value));
		}
	}
}