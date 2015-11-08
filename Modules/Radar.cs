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
using System.Collections;
using UnityEngine;

namespace ThrottleControlledAvionics
{
	public class Radar : TCAModule
	{
		public class Config : ModuleConfig
		{
			new public const string NODE_NAME = "RAD";

			[Persistent] public float UpViewAngle       = 15;
			[Persistent] public float DownViewAngle     = 15;
			[Persistent] public float MinAngleDelta     = 0.1f;
			[Persistent] public float UpViewSlope       = 5;
			[Persistent] public float LookAheadTime     = 20;
			[Persistent] public int   NumRays           = 30;
			[Persistent] public float MinAltitudeFactor = 2;
			[Persistent] public float MinClosingSpeed   = 4;
			[Persistent] public float MaxClosingSpeed   = 10;
			[Persistent] public float MinDistanceAhead  = 10;
			[Persistent] public float PitchRollAAf      = -10;
			[Persistent] public float MinAltitude       = 10;
			[Persistent] public float NHVf              = 0.5f;
			[Persistent] public float ManeuverTimer     = 3f;
			public float AngleDelta;

			public override void Init()
			{
				base.Init();
				UpViewAngle = Utils.ClampH(UpViewAngle, 80);
				DownViewAngle = Utils.ClampH(DownViewAngle, 80);
				AngleDelta = (UpViewAngle+DownViewAngle)/NumRays;
			}
		}
		static Config RAD { get { return TCAScenario.Globals.RAD; } }

		[Flags]
		public enum Mode 
		{ 
			Off = 0, 
			Vertical = 1 << 0, 
			Horizontal = 1 << 1, 
			Both = Vertical|Horizontal 
		}

		public Radar(VesselWrapper vsl) 
		{ 
			VSL = vsl;
			CurHit = new Sweep(vsl);
			BestHit = new Sweep(vsl);
			DetectedHit = new Sweep(vsl);
		}

		static   int RadarMask = (1 << LayerMask.NameToLayer("Local Scenery"));
		//normal radar
		Mode     mode;
		Vector3  Dir;
		Vector3d SurfaceVelocity;
		float    ViewAngle;
		float    AngleDelta;
		float    MaxDistance;
		Vector3  RelObstaclePosition;
		float    DistanceAhead;
		float    ClosingSpeed;
		float    CollisionSpeed = -1;
		readonly Sweep CurHit;
		readonly Sweep BestHit;
		readonly Sweep DetectedHit;
		bool     last_hit_valid;
		//side maneuver
		bool     side_collision;
		Vector3d side_maneuver;
		readonly Timer ManeuverTimer = new Timer();
		//terrain altitude predictor
		float    LookAheadTime = 1;
		float    LookAheadTimeDelta = 1;
		float    BestHeight = float.MinValue;
		float    HeightAhead = float.MinValue;
		Vector3d Probe;

		public override void Init()
		{
			base.Init();
			ManeuverTimer.Period = RAD.ManeuverTimer;
			#if DEBUG
//			RenderingManager.AddToPostDrawQueue(1, RadarBeam);
			#endif
		}

		#if DEBUG
		public void RadarBeam()
		{
			if(VSL == null || VSL.vessel == null || !IsActive) return;
			GLUtils.GLLine(VSL.wCoM, Probe, Color.green);
			CurHit.Draw();
		}

		public override void Reset()
		{
			base.Reset();
//			RenderingManager.RemoveFromPostDrawQueue(1, RadarBeam);
		}
		#endif

		public override void UpdateState() 
		{ 
			IsActive = VSL.OnPlanet && !VSL.LandedOrSplashed;
			if(IsActive)
			{
				mode = Mode.Off;
				if(CFG.HF) 
					mode |= Mode.Horizontal;
				if(CFG.VF[VFlight.AltitudeControl] && CFG.AltitudeAboveTerrain)
					mode |= Mode.Vertical;
				IsActive &= mode != Mode.Off;
			}
			if(IsActive) return;
			reset();
		}

		void reset()
		{
			rewind();
			DetectedHit.Reset();
			CollisionSpeed = -1;
			DistanceAhead  = -1;
			VSL.TimeAhead  = -1;
			VSL.AltitudeAhead = float.MinValue;
			RelObstaclePosition = Vector3.zero;
		}

		void rewind()
		{
			CurHit.Reset();
			BestHit.Reset();
			ViewAngle = -RAD.UpViewAngle;
			AngleDelta = RAD.AngleDelta;
			last_hit_valid = false;
			ManeuverTimer.RunIf(() => side_maneuver.Zero(), !side_collision);
			side_collision = false;
		}

		void ProbeHeightAhead()
		{
			var pqs = VSL.mainBody.pqsController;
			if(pqs == null) return;
			Probe = VSL.wCoM+VSL.HorizontalVelocity*LookAheadTime;
			//CelestialBody.GetRelSurfacePosition is broken
			var height = pqs.GetSurfaceHeight(QuaternionD.AngleAxis(VSL.mainBody.directRotAngle, Vector3d.up) * (Probe-pqs.transformPosition))-pqs.radius;
			if(VSL.mainBody.ocean && height < 0) height = 0;
//			Log("MainBody {0}: LoodAheadTime {1}, height {2}, BestHeight {3}, HeightAhead {4}, TimeDelta {5}\nprobe {6}",
//			    VSL.mainBody.bodyName, LookAheadTime, height, BestHeight, HeightAhead, LookAheadTimeDelta, Probe-VSL.wCoM);//debug
			if(height > BestHeight) BestHeight = (float)height;
			if(BestHeight > HeightAhead) HeightAhead = BestHeight;
			if(VSL.AbsAltitude-height <= VSL.H)
			{
				LookAheadTime -= LookAheadTimeDelta;
				LookAheadTimeDelta /= 2;
			}
			LookAheadTime += LookAheadTimeDelta;
			if(LookAheadTime > RAD.LookAheadTime) 
			{
				HeightAhead = BestHeight;
				LookAheadTime = 1;
				LookAheadTimeDelta = 1;
				BestHeight = float.MinValue;
			}
			else if(LookAheadTimeDelta < 0.1)
			{
				if(!BestHit.Equals(float.MinValue))
					HeightAhead = BestHeight;
				LookAheadTime = 1;
				LookAheadTimeDelta = 1;
				BestHeight = float.MinValue;
			}
		}

		public void Update()
		{
			if(!IsActive) return;
			var zero_needed = VSL.NeededHorVelocity.IsZero();
			if(CollisionSpeed < 0 && VSL.HorizontalSpeed < RAD.MinClosingSpeed && 
			   (zero_needed || CFG.DesiredAltitude < RAD.MinAltitude || IsStateSet(TCAState.Landing)))
			{ reset(); return; }
			//closing speed and starting ray direction
			Dir = Vector3.zero;
			SurfaceVelocity = VSL.PredictedSrfVelocity(GLB.CPS.LookAheadTime);
			if((DistanceAhead < 0 || DistanceAhead > RAD.MinDistanceAhead ||
		        Vector3.Dot(RelObstaclePosition, VSL.NeededHorVelocity) < 0) &&
		       (VSL.HorizontalSpeed >= RAD.MaxClosingSpeed ||
			    zero_needed && 
		        VSL.HorizontalSpeed >= RAD.MinClosingSpeed))
			{
				Dir = VSL.HorizontalVelocity.normalized;
				if(VSL.IsStateSet(TCAState.LoosingAltitude))
					Dir = Vector3.Lerp(Dir, SurfaceVelocity.normalized,
					                   RAD.LookAheadTime/(VSL.Altitude/-VSL.RelVerticalSpeed)/VSL.MaxDTWR);
			}
			else Dir = zero_needed? 
					Vector3d.Exclude(VSL.Up, VSL.Fwd).normalized : 
					VSL.NeededHorVelocity.normalized;
			ClosingSpeed = Utils.ClampL(Vector3.Dot(SurfaceVelocity, Dir), RAD.MinClosingSpeed);
			//cast the ray
			MaxDistance = (CollisionSpeed < ClosingSpeed? ClosingSpeed : CollisionSpeed)*RAD.LookAheadTime;
			if(ViewAngle < 0) MaxDistance = MaxDistance/Mathf.Cos(ViewAngle*Mathf.Deg2Rad)*(1+ClosingSpeed/RAD.UpViewSlope*Utils.ClampL(-ViewAngle/RAD.UpViewAngle, 0));
			CurHit.Cast(Dir, ViewAngle, MaxDistance);
			//check the hit
			if(CurHit.Valid)
			{
				if(CurHit.Maneuver == Sweep.ManeuverType.Horizontal &&
				   (mode & Mode.Horizontal) == Mode.Horizontal)
				{
					last_hit_valid = false;
					//check if it is indeed a collision
					if(CurHit.Altitude < VSL.H*RAD.MinAltitudeFactor)
					{
						//queue avoiding maneuver with CPS
						side_collision = true;
						var ray = CurHit.SideCollisionRay;
						var collision_point = Vector3.ProjectOnPlane(ray.CollisionPoint-VSL.wCoM, VSL.Up);
						var dist = collision_point.magnitude;
						Vector3d maneuver;
						if(CollisionPreventionSystem.AvoidStatic(VSL, collision_point/dist, dist, 
						                                         Vector3d.Exclude(VSL.Up, SurfaceVelocity), out maneuver))
						{
							if(Vector3d.Dot(side_maneuver, maneuver) > 0 ||
							   side_maneuver.sqrMagnitude < maneuver.sqrMagnitude)
								side_maneuver = maneuver;
						}
					}
				}
				else if((mode & Mode.Vertical) == Mode.Vertical)
				{
					if(CurHit > BestHit) BestHit.Copy(CurHit);
					if(BestHit.Valid && BestHit > DetectedHit)
						DetectedHit.Copy(BestHit);
					//rewind the ray one step and decrease the delta if direct collision detected
					if(last_hit_valid) ViewAngle -= 2*AngleDelta;
					else
					{
						last_hit_valid = true;
						ViewAngle -= AngleDelta;
						AngleDelta /= 2;
					}
				}
			}
			else 
			{
				if(last_hit_valid) AngleDelta /= 2;
				last_hit_valid = false;
			}
			//if on side collision course, correct it
			if(!side_maneuver.IsZero()) VSL.CourseCorrections.Add(side_maneuver);
			//handle direct collisions if we're able to
			if((mode & Mode.Vertical) == Mode.Vertical)
			{
				//probe for surface height
				ProbeHeightAhead();
				//update collision info if detected something
				VSL.TimeAhead = -1;
				DistanceAhead = -1;
				RelObstaclePosition = Vector3.zero;
				if(DetectedHit.Valid) VSL.AltitudeAhead = DetectedHit.Altitude;
				if(HeightAhead > VSL.AltitudeAhead) VSL.AltitudeAhead = HeightAhead;
				//check for possible stright collision
				if(VSL.AbsAltitude-VSL.AltitudeAhead-VSL.H*RAD.MinAltitudeFactor*(CollisionSpeed < 0? 1 : 2) < 0) //deadzone of twice the detection height
				{ 
					if(CollisionSpeed < ClosingSpeed) CollisionSpeed = ClosingSpeed;
					RelObstaclePosition = Vector3.ProjectOnPlane(DetectedHit.CenterCollisionRay.CollisionPoint-VSL.wCoM, VSL.Up);
					DistanceAhead = Utils.ClampL(RelObstaclePosition.magnitude-VSL.R, 0.1f);
					VSL.TimeAhead = DistanceAhead/Vector3.Dot(SurfaceVelocity, RelObstaclePosition.normalized);
					var dV = Vector3d.zero;
					if(DistanceAhead > RAD.MinDistanceAhead)
						dV = Vector3d.Project(SurfaceVelocity, RelObstaclePosition) *
							-Math.Sqrt(1-Utils.ClampH(DistanceAhead/ClosingSpeed/RAD.LookAheadTime*VSL.MaxTWR*RAD.NHVf, 1));
					else if(DistanceAhead > RAD.MinDistanceAhead/2)
						dV = -VSL.NeededHorVelocity;
					else if(Vector3d.Dot(SurfaceVelocity, RelObstaclePosition) > 0)
						dV = Vector3d.Project(SurfaceVelocity, RelObstaclePosition) *
							-RAD.MinDistanceAhead/DistanceAhead*RAD.PitchRollAAf/VSL.MaxPitchRollAA_m;
					else dV = -VSL.NeededHorVelocity;
					VSL.CourseCorrections.Add(dV);
				}
				else CollisionSpeed = -1;
			}
			//update angle for the next ray and check bounds
			ViewAngle += AngleDelta;
			if(ViewAngle > RAD.DownViewAngle) 
			{ 
				if(BestHit.Valid) 
				{ 
					DetectedHit.Copy(BestHit); 
					rewind(); 
				}
				else reset(); 
			}
			else if(AngleDelta < RAD.MinAngleDelta) 
			{
				if(BestHit.Valid) 
					DetectedHit.Copy(BestHit); 
				rewind();
			}
		}

		public struct Ray
		{
			RaycastHit hit;
			float max_distance;
			public Vector3 Ori { get; private set; }
			public Vector3 Dir { get; private set; }
			public float Distance { get { return Valid? hit.distance : float.MaxValue; } }
			public Vector3 CollisionPoint { get { return Valid? hit.point : Vector3.zero; } }
			public bool  Valid { get; private set; }

			public void Reset() { Valid = false; hit = default(RaycastHit); }

			/// <summary>
			/// Cast a ray from ori in the dir with max dist. and optional radius (using SphereCast)
			/// </summary>
			/// <param name="ori">Ori in worldspace</param>
			/// <param name="dir">Dir in worldspace; must be normalized</param>
			/// <param name="dist">Maximum distance</param>
			/// <param name="radius">Radius of the ray</param>
			public bool Cast(Vector3 ori, Vector3 dir, float dist, float radius)
			{
				Ori = ori; Dir = dir; max_distance = dist;
				Valid = Physics.SphereCast(Ori, radius, Dir, out hit, dist, RadarMask);
				return Valid;
			}

			/// <summary>
			/// Calculates altitude of the hit point relative the specified base direction
			/// </summary>
			/// <param name="base_dir">Base direction; must be normalized</param>
			/// <param name="angle">Initial angle between the casting dir and base_dir; 
			/// used to define the sign of the returned altitude</param>
			public float Altitude(Vector3 base_dir, float angle)
			{
				return Valid? 
					Mathf.Sign(angle)*
					hit.distance*Mathf.Sin(
						Mathf.Acos(Mathf.Clamp(Vector3.Dot(base_dir, (hit.point-Ori).normalized), -1, 1))) : 
					float.MaxValue;
			}

			#if DEBUG
			public void Draw()
			{
				GLUtils.GLLine(Ori, Valid? hit.point : Ori+Dir*max_distance, 
				                   Valid? Color.magenta : Color.red);
			}
			#endif
		}

		public class Sweep
		{
			public enum ManeuverType { None, Horizontal, Vertical }

			VesselWrapper VSL;
			Ray L, C, R;

			public float Distance { get; private set; }
			public float Altitude { get; private set; }
			public bool  Valid { get; private set; }

			public Sweep(VesselWrapper vsl)	{ VSL = vsl; }

			public void Copy(Sweep s)
			{
				VSL = s.VSL;
				L = s.L; C = s.C; R = s.R;
				Distance = s.Distance;
				Altitude = s.Altitude;
				Valid = s.Valid;
			}

			public bool Update(Sweep s)
			{
				if(s < this) { Copy(s); return true; }
				return false;
			}

			public void Reset() 
			{ 
				Valid = false; 
				Altitude = 0; 
				L.Reset(); C.Reset(); R.Reset(); 
			}

			public ManeuverType Maneuver
			{
				get
				{
					if(L.Valid && !R.Valid || R.Valid && !L.Valid) 
						return ManeuverType.Horizontal;
					if(C.Valid || R.Valid && L.Valid)
						return ManeuverType.Vertical;
					return ManeuverType.None;
				}
			}

			/// <summary>
			/// Gets the side collision ray. 
			/// Only valid if the Maneuver equals ManeuverType.Horizontal.
			/// </summary>
			/// <value>The side collision ray.</value>
			public Ray SideCollisionRay
			{
				get
				{
					if(L.Valid && !R.Valid) return L;
					if(!L.Valid && R.Valid) return R;
					return default(Ray);
				}
			}

			public Ray CenterCollisionRay
			{
				get
				{
					if(C.Valid) return C;
					if(L.Valid && R.Valid) 
						return R.Distance < L.Distance? R : L;
					return default(Ray);
				}
			}

			public void Cast(Vector3 dir, float angle, float dist)
			{
				Reset();
				if(VSL.refT == null) return;
				//cast the rays
				var side = Vector3.Cross(VSL.Up, dir)*VSL.R*1.5f;
				var cast_dir = Quaternion.AngleAxis(angle, side)*dir;
				Valid |= L.Cast(VSL.wCoM-side, cast_dir, dist, VSL.R);
				Valid |= C.Cast(VSL.wCoM,      cast_dir, dist, VSL.R);
				Valid |= R.Cast(VSL.wCoM+side, cast_dir, dist, VSL.R);
				if(Valid) 
					Altitude = VSL.AbsAltitude-Mathf.Min(L.Altitude(dir, angle), 
					                                     C.Altitude(dir, angle), 
					                                     R.Altitude(dir, angle));
			}

			public static Sweep Cast(VesselWrapper vsl, Vector3 dir, float angle, float dist)
			{
				var s = new Sweep(vsl);
				s.Cast(dir, angle, dist);
				return s;
			}

			public static bool operator <(Sweep s1, Sweep s2) { return !s1.Valid || s2.Valid && s1.Altitude < s2.Altitude; }
			public static bool operator >(Sweep s1, Sweep s2) { return !s2.Valid || s1.Valid && s1.Altitude > s2.Altitude; }

			public override string ToString()
			{ return string.Format("[Hit: Valid={0}, Maneuver {1}, Altitude={2}]", Valid, Maneuver, Altitude); }

			#if DEBUG
			public void Draw() 
			{ 
//				if(!Valid) return;
				L.Draw(); C.Draw(); R.Draw(); 
			}
			#endif
		}
	}
}

