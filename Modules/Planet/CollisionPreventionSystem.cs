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
using AT_Utils;

namespace ThrottleControlledAvionics
{
	[CareerPart]
    [OptionalModules(typeof(HorizontalSpeedControl),
                     typeof(VerticalSpeedControl))]
    [OverrideModules(typeof(AltitudeControl),
                     typeof(VTOLAssist))]
    public class CollisionPreventionSystem : TCAService
	{
		public class Config : ModuleConfig
		{
			[Persistent] public float MinDistance       = 5f;
			[Persistent] public float SafeDistance      = 30f;
			[Persistent] public float SafeTime          = 5f;
			[Persistent] public float MaxAvoidanceSpeed = 100f;
			[Persistent] public float LatAvoidMinVelSqr = 0.25f;
			[Persistent] public float LookAheadTime     = 1f;
			[Persistent] public float ManeuverTimer     = 3f;
			[Persistent] public float LowPassF          = 0.5f;
		}
		static Config CPS { get { return Globals.Instance.CPS; } }

		public CollisionPreventionSystem(ModuleTCA tca) : base(tca) {}

        #pragma warning disable 169
		HorizontalSpeedControl HSC;
        VerticalSpeedControl VSC;
        #pragma warning restore 169

        public override void Disable() 
        { 
            Correction = Vector3.zero; 
        }

        protected override void Resume()
        {
            base.Resume();
            Correction = Vector3.zero;
        }

		protected override void UpdateState() 
		{ 
			base.UpdateState();
            IsActive &= CFG.UseCPS && VSL.OnPlanet && !VSL.LandedOrSplashed && VSL.refT != null && HasActiveClients;
		}

		static int RadarMask = (1 | 1 << LayerMask.NameToLayer("Parts"));
		readonly HashSet<Guid> Dangerous = new HashSet<Guid>();
		List<Vector3d> Corrections = new List<Vector3d>();
		Vector3 Correction;
		IEnumerator scanner;
		readonly LowPassFilterV filter = new LowPassFilterV();
		readonly Timer ManeuverTimer = new Timer();

        public Vector3 CourseCorrection { get { return Correction.IsZero()? Correction : filter.Value; } }

		public override void Init()
		{
			base.Init();
			filter.Tau = CPS.LowPassF;
			ManeuverTimer.Period = CPS.ManeuverTimer;
		}

		#if DEBUG
//		Vector3 Dir, DeltaV, Maneuver;
		bool Collided;
		public void RadarBeam()
		{
			if(!IsActive || VSL == null || VSL.vessel == null) return;
			Utils.GLVec(VSL.refT.position, filter.Value, Color.magenta);
//			if(VSL.IsActiveVessel)
//			{
//				if(!Dir.IsZero())
//					Utils.GLVec(VSL.Physics.wCoM, Dir, Color.cyan);
//				if(!DeltaV.IsZero())
//					Utils.GLVec(VSL.Physics.wCoM, DeltaV, Color.green);
//				if(!Maneuver.IsZero() && !DeltaV.IsZero())
//					Utils.GLVec(VSL.Physics.wCoM+DeltaV, Maneuver, Color.yellow);
//				if(!Maneuver.IsZero() && !DeltaV.IsZero())
//					Utils.GLVec(VSL.Physics.wCoM, Maneuver+DeltaV, Color.red);
//			}
			Utils.GLDrawBounds(VSL.Geometry.B, VSL.refT, Collided? Color.red : Color.white);
//			for(int i = 0, VSLEnginesCount = VSL.Engines.Count; i < VSLEnginesCount; i++)
//			{
//				var e = VSL.Engines[i];
//				for(int j = 0, eenginethrustTransformsCount = e.engine.thrustTransforms.Count; j < eenginethrustTransformsCount; j++)
//				{
//					var t = e.engine.thrustTransforms[j];
//					Utils.GLVec(t.position, t.forward * e.engine.exhaustDamageMaxRange, Color.yellow);
//				}
//			}
		}
		#endif

		static float SafeTime(VesselWrapper vsl, Vector3d dVn)
		{ return CPS.SafeTime/Utils.Clamp(Mathf.Abs(Vector3.Dot(vsl.Torque.MaxCurrent.AA, vsl.LocalDir(dVn))), 0.01f, 1f); }

		public static bool AvoidStatic(VesselWrapper vsl, Vector3d dir, float dist, Vector3d dV, out Vector3d maneuver)
		{
			maneuver = Vector3d.zero;
			var dVn = dV.normalized;
			var cosA = Mathf.Clamp(Vector3.Dot(dir, dVn), -1, 1);
			if(cosA <= 0) return false;
			var sinA = Mathf.Sqrt(1-cosA*cosA);
			var vdist = dist*cosA;
			var min_separation = dist*sinA;
			if(min_separation > vsl.Geometry.D ||
			   min_separation > vsl.Geometry.R && vdist < min_separation) return false;
			maneuver = (dVn*cosA-dir).normalized;
			var vTime = dist*cosA/dV.magnitude;
			if(vTime > SafeTime(vsl, dVn)) return false;
			maneuver *= (vsl.Geometry.D-min_separation) / Math.Sqrt(vTime);
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
			var collision_dist = threshold.Equals(0)?
				2*dist-vsl.Geometry.DistToBounds(vsl.Physics.wCoM+dir*dist)-
				Mathf.Sqrt(exhaust2.SqrDistance(refT2.InverseTransformPoint(vsl.Physics.wCoM))) :
				Mathf.Max(vsl.Geometry.R, dist-vsl.Geometry.DistToBounds(vsl.Physics.wCoM+dir*dist)) +
				Mathf.Max(r2, dist-Mathf.Sqrt(exhaust2.SqrDistance(refT2.InverseTransformPoint(vsl.Physics.wCoM))));
//			vsl.Log("R {}, R2 {}; r {}, r2 {}", vsl.Geometry.R, r2, 
//			        dist-vsl.DistToBounds(vsl.wCoM+dir*dist), 
//			        dist-Mathf.Sqrt(exhaust2.SqrDistance(refT2.InverseTransformPoint(vsl.wCoM))));//debug
//			Utils.Log("{}: cosA: {}, threshold {}", vsl.vessel.vesselName, cosA, threshold);//debug
			if(cosA <= 0) goto check_distance;
			if(threshold < 0) threshold = CPS.MinDistance;
			var sinA = Mathf.Sqrt(1-cosA*cosA);
			var min_separation = dist*sinA;
			var sep_threshold = collision_dist+threshold;
//			Utils.Log("{}: min_sep > sep_thresh: {} > {}, threshold {}", 
//			          vsl.vessel.vesselName, min_separation, sep_threshold, threshold);//debug
			if(min_separation > sep_threshold) goto check_distance;
			//calculate time to collision
			var vDist = 0f;
			var alpha = Mathf.Acos(cosA);
			var dVm   = dV.magnitude;
			if(sinA <= 0) vDist = dist-sep_threshold;
			else if(dist > sep_threshold) vDist = sep_threshold*Mathf.Sin(Mathf.Asin(min_separation/sep_threshold)-alpha)/sinA;
			var vTime = Utils.ClampL(vDist, 0.1f)/dVm;
//				Utils.Log("{}: vTime > SafeTime: {} > {}", 
//				          vsl.vessel.vesselName, vTime, CPS.SafeTime/Utils.Clamp(Mathf.Abs(Vector3.Dot(vsl.wMaxAngularA, dVn)), 0.01f, 1f));//debug
			if(vTime > SafeTime(vsl, dVn)) goto check_distance;
			//calculate maneuver
			Vector3d side;
			if(cosA < 0.9) side = (dVn*cosA-dir).normalized;
			else if(Math.Abs(Vector3d.Dot(dVn, vsl.Physics.Up)) < 0.9) 
				side = Vector3d.Cross(dVn, vsl.Physics.Up).normalized;
			else side = Vector3d.Cross(dVn, vsl.OnPlanetParams.Fwd).normalized;
//			if(dist > sep_threshold)
//			{
//				var beta = Mathf.Asin(sep_threshold/dist)-alpha;
//				maneuver = (Mathf.Sin(beta)*side + dVn*(Mathf.Cos(beta)-1)).normalized;
//			}
//			else 
			maneuver  = side;
			maneuver += vsl.Physics.Up*Mathf.Sign(Vector3.Dot(dir, vsl.Physics.Up))*(min_separation/sep_threshold-1);
			maneuver *= (sep_threshold-min_separation) / Math.Sqrt(vTime);
//			maneuver = (-vsl.Up*Mathf.Sign(Vector3.Dot(dir, vsl.Up))*(1-min_separation/sep_threshold) + 
//			            (dVn*cosA-dir).normalized).normalized * (sep_threshold-min_separation) / vTime;
//			vsl.Log("\ndist {}\ndV {}\nmaneuver: {}\n" +
//			        "vTime {}, vDist {}, min sep {}, sep_thresh {}", 
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
				if(vsl.HorizontalSpeed.NeededVector.sqrMagnitude > CPS.LatAvoidMinVelSqr)
				{
					var lat_avoid = Vector3d.Cross(vsl.Physics.Up, vsl.HorizontalSpeed.NeededVector.normalized);
					dc = Vector3d.Dot(dc, lat_avoid) >= 0? 
						lat_avoid*dist_to_safe :
						lat_avoid*-dist_to_safe;
				}
				if(dc.sqrMagnitude > 0.25) maneuver += dc/CPS.SafeTime*2;
//				vsl.Log("adding safe distance correction: {}", dc/CPS.SafeTime*2);//debug
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
			var dir = (v.CurrentCoM-VSL.Physics.wCoM);
			var dist = dir.magnitude;
			dir /= dist;
			//first try to get TCA from other vessel and get vessel's R and Exhaust
			var vR = 0f;
			var vB = new Bounds();
			Transform vT = null;
			var tca = ModuleTCA.EnabledTCA(v);
			if(tca != null) 
			{
//				if(tca.CPS != null && 
//				   tca.CPS.IsActive && 
//				   VSL.Physics.M > tca.VSL.Physics.M &&
//				   VSL.vessel.srfSpeed > v.srfSpeed) //test 
//					return false;
				vR = tca.VSL.Geometry.R;
				vB = tca.VSL.Geometry.B;
				vT = tca.VSL.refT;
			}
			else //do a raycast
			{
				RaycastHit raycastHit;
				if(Physics.SphereCast(VSL.Geometry.C+dir*(VSL.Geometry.R+0.1f), VSL.Geometry.R, dir,
				               out raycastHit, dist, RadarMask))
					vR = (raycastHit.point-v.CurrentCoM).magnitude;
				vT = v.ReferenceTransform;
				vB = v.BoundsWithExhaust(vT);
			}
			//compute course correction
			var dV = VSL.vessel.srf_velocity-v.srf_velocity+(VSL.vessel.acceleration-v.acceleration)*CPS.LookAheadTime;
			var thrershold = -1f;
			if(v.LandedOrSplashed) thrershold = 0;
			else if(Dangerous.Contains(v.id)) thrershold = CPS.SafeDistance;
			if(AvoideVessel(VSL, dir, dist, dV, vR, vB, vT, out maneuver, thrershold))
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
				if(v.isEVA) //ignore kerbals on our own ladders
				{
					var eva = v.GetComponent<KerbalEVA>();
					if(eva != null && eva.LadderPart != null && 
					   eva.LadderPart.vessel == VSL.vessel)
						continue;
				}
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
			if(scan())
			{
				var correction = Vector3d.zero;
				for(int i = 0, count = Corrections.Count; i < count; i++)
					correction += Corrections[i];
				if(correction.IsZero())
				{ 
					if(ManeuverTimer.TimePassed) 
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
            if(VSC != null && CFG.VF[VFlight.AltitudeControl])
			{
				var dVSP = (float)Vector3d.Dot(filter.Value, VSL.Physics.Up);
				if(dVSP > 0 || 
                   VSL.Altitude.Relative-VSL.Geometry.H +
                   (dVSP+VSL.VerticalSpeed.Relative)*CPS.LookAheadTime > 0)
                    VSC.SetpointOverride = CFG.VerticalCutoff+dVSP;
//				else dVSP = 0;//debug
//				Log("\nCorrection {}\nAction {}\ndVSP {}\ncorrectins: {}", 
//				    Correction, filter.Value, dVSP,
//				    Corrections.Aggregate("\n", (s, v) => s+v+"\n"));//debug
			}
			//correct horizontal course
            if(HSC != null) 
                HSC.AddWeightedCorrection(Vector3d.Exclude(VSL.Physics.Up, filter.Value));
		}
	}
}