//  Author:
//       Allis Tauri <allista@gmail.com>
//
//  Copyright (c) 2015 Allis Tauri
//
// This work is licensed under the Creative Commons Attribution-ShareAlike 4.0 International License. 
// To view a copy of this license, visit http://creativecommons.org/licenses/by-sa/4.0/ 
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
			[Persistent] public float MaxLittleSteps    = 5;
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

		public Radar(ModuleTCA tca)
		{ 
			TCA = tca;
			CurHit = new Sweep(VSL);
			BestHit = new Sweep(VSL);
			DetectedHit = new Sweep(VSL);
			Altimeter = new PQS_Altimeter(VSL);
		}

		public float AltitudeAhead;
		public float TimeAhead;

		static   int RadarMask = (1 << LayerMask.NameToLayer("Local Scenery"));
		//normal radar
		Mode     mode;
		Vector3  Dir;
		Vector3d SurfaceVelocity;
		float    ViewAngle;
		float    AngleDelta;
		int      LittleSteps;
		float    MaxDistance;
		TerrainPoint Obstacle;
		Vector3  RelObstaclePosition;
		float    DistanceAhead;
		float    ClosingSpeed;
		float    CollisionSpeed = -1;
		Ray      ForwardRay;
		readonly Sweep CurHit;
		readonly Sweep BestHit;
		readonly Sweep DetectedHit;
		bool     LastHitValid;
		//side maneuver
		bool     SideCollision;
		Vector3d SideManeuver;
		readonly Timer ManeuverTimer = new Timer();
		//altimeter
		readonly PQS_Altimeter Altimeter;

		public override void Init()
		{
			base.Init();
			ManeuverTimer.Period = RAD.ManeuverTimer;
			reset();
			#if DEBUG
//			RenderingManager.AddToPostDrawQueue(1, RadarBeam);
			#endif
		}

		#if DEBUG
		public void RadarBeam()
		{
			if(VSL == null || VSL.vessel == null || !IsActive) return;
			ForwardRay.Draw();
			Altimeter.Draw();
			CurHit.Draw();
			DetectedHit.Draw();
		}

		public override void Reset()
		{
			base.Reset();
//			RenderingManager.RemoveFromPostDrawQueue(1, RadarBeam);
		}
		#endif

		protected override void UpdateState() 
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
			TimeAhead      = -1;
			AltitudeAhead  = float.MinValue;
			RelObstaclePosition = Vector3.zero;
		}

		void rewind()
		{
			CurHit.Reset();
			BestHit.Reset();
			ForwardRay.Reset();
			ViewAngle = -RAD.UpViewAngle;
			AngleDelta = RAD.AngleDelta;
			LittleSteps = 0;
			LastHitValid = false;
			ManeuverTimer.RunIf(() => SideManeuver.Zero(), !SideCollision);
			SideCollision = false;
		}

		protected override void Update()
		{
			if(!IsActive) return;
			var zero_needed = TCA.HSC.NeededHorVelocity.IsZero();
			if(CollisionSpeed < 0 && VSL.HorizontalSpeed < RAD.MinClosingSpeed && 
			   (zero_needed && !CFG.Nav.Any(Navigation.FollowPath, Navigation.FollowTarget, Navigation.GoToTarget) ||
			    CFG.HF[HFlight.Stop] || CFG.Nav.Any(Navigation.Anchor, Navigation.AnchorHere) || 
			    !VSL.AltitudeAboveGround || IsStateSet(TCAState.Landing)))
			{ reset(); return; }
			//check boundary conditions
			if(ViewAngle > RAD.DownViewAngle) 
			{ 
				if(BestHit.Valid) 
				{ 
					DetectedHit.Copy(BestHit); 
					rewind(); 
				}
				else reset(); 
			}
			else if(AngleDelta < RAD.MinAngleDelta || LittleSteps > RAD.MaxLittleSteps) 
			{
				if(BestHit.Valid) 
					DetectedHit.Copy(BestHit); 
				rewind();
			}
			//calculate closing speed and initial ray direction
			var alt_threshold = VSL.AbsAltitude-VSL.H*RAD.MinAltitudeFactor*(CollisionSpeed < 0? 1 : 2);
			Dir = Vector3.zero;
			SurfaceVelocity = VSL.PredictedSrfVelocity(GLB.CPS.LookAheadTime);
			if((DistanceAhead < 0 || DistanceAhead > RAD.MinDistanceAhead ||
		        Vector3.Dot(RelObstaclePosition, TCA.HSC.NeededHorVelocity) < 0) &&
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
					TCA.HSC.NeededHorVelocity.normalized;
			Dir.Normalize();
			ClosingSpeed = Utils.ClampL(Vector3.Dot(SurfaceVelocity, Dir), RAD.MinClosingSpeed);
			//cast the sweep and the fwd ray
			MaxDistance = (CollisionSpeed < ClosingSpeed? ClosingSpeed : CollisionSpeed)*RAD.LookAheadTime;
			if(ViewAngle < 0) MaxDistance = MaxDistance/Mathf.Cos(ViewAngle*Mathf.Deg2Rad)*(1+ClosingSpeed/RAD.UpViewSlope*Utils.ClampL(-ViewAngle/RAD.UpViewAngle, 0));
			CurHit.Cast(Dir, ViewAngle, MaxDistance);
			ForwardRay.Cast(VSL.wCoM, SurfaceVelocity.normalized, (float)SurfaceVelocity.magnitude*GLB.CPS.LookAheadTime*3, VSL.R*2);
			//check the hit
			if(CurHit.BeforeDestination(SurfaceVelocity))
			{
				if(CurHit.Maneuver == Sweep.ManeuverType.Horizontal &&
				   (mode & Mode.Horizontal) == Mode.Horizontal)
				{
					LastHitValid = false;
					//check if it is indeed a collision
					if(CurHit.Altitude > alt_threshold)
					{
						//queue avoiding maneuver with CPS
						SideCollision = true;
						var collision_point = Vector3.ProjectOnPlane(CurHit.Obstacle.RelPosition(VSL.wCoM), VSL.Up);
						var dist = collision_point.magnitude;
						Vector3d maneuver;
						if(CollisionPreventionSystem.AvoidStatic(VSL, collision_point/dist, dist, 
						                                         Vector3d.Exclude(VSL.Up, SurfaceVelocity), out maneuver))
						{
							if(Vector3d.Dot(SideManeuver, maneuver) > 0 ||
							   SideManeuver.sqrMagnitude < maneuver.sqrMagnitude)
								SideManeuver = maneuver;
						}
					}
				}
				else
				{
					if(CurHit > BestHit) BestHit.Copy(CurHit);
					if(BestHit.Valid && BestHit > DetectedHit) DetectedHit.Copy(BestHit);
					//rewind the ray one step and decrease the delta if direct collision detected
					if(LastHitValid) ViewAngle -= 2*AngleDelta;
					else
					{
						LastHitValid = true;
						ViewAngle -= AngleDelta;
						AngleDelta /= 2;
					}
				}
			}
			else if(!CurHit.Valid)
			{
				if(LastHitValid) AngleDelta /= 2;
				LastHitValid = false;
			}
			if(AngleDelta < RAD.AngleDelta) LittleSteps++;
			//if on side collision course, correct it
			if(!SideManeuver.IsZero()) TCA.HSC.CourseCorrections.Add(SideManeuver);
			//probe for surface height
			Altimeter.ProbeHeightAhead(Dir);
			//update collision info if detected something
			TimeAhead = -1;
			DistanceAhead = -1;
			RelObstaclePosition = Vector3.zero;
			Obstacle = DetectedHit.Obstacle;
			if(Altimeter.BeforeDestination(SurfaceVelocity) && 
			   DetectedHit.Obstacle < Altimeter.Obstacle)
				Obstacle = Altimeter.Obstacle;
			if(ForwardRay.Valid)
			{
				ForwardRay.ClaculateAltitude(VSL.Up, VSL.AbsAltitude);
				if(ForwardRay.Altitude > Obstacle.Altitude || ForwardRay.Altitude > alt_threshold)
					Obstacle = new TerrainPoint(ForwardRay.Altitude, ForwardRay.CollisionPoint);
			}
			if(Obstacle.Valid) AltitudeAhead = (float)Obstacle.Altitude;
//			Log("\nCurHit {0}\nBestHit {1}\nDetectedHit {2}\nRObstacle {3}\nAObstacle {4}\nForwardRay {5}",
//			    CurHit, BestHit, DetectedHit, Obstacle, Altimeter.Obstacle, ForwardRay);//debug
			//check for possible stright collision
			if(AltitudeAhead > alt_threshold) //deadzone of twice the detection height
			{ 
				if(CollisionSpeed < ClosingSpeed) CollisionSpeed = ClosingSpeed;
				RelObstaclePosition = Vector3.ProjectOnPlane(Obstacle.RelPosition(VSL.wCoM), VSL.Up);
				DistanceAhead = Utils.ClampL(RelObstaclePosition.magnitude-VSL.R, 0.1f);
				TimeAhead = DistanceAhead/Vector3.Dot(SurfaceVelocity, RelObstaclePosition.normalized);
				Vector3d dV;
				if(DistanceAhead > RAD.MinDistanceAhead)
					dV = Vector3d.Project(SurfaceVelocity, RelObstaclePosition) *
						-Math.Sqrt(1-Utils.ClampH(DistanceAhead/ClosingSpeed/RAD.LookAheadTime*VSL.MaxTWR*RAD.NHVf, 1));
				else if(DistanceAhead > RAD.MinDistanceAhead/2)
					dV = -TCA.HSC.NeededHorVelocity;
				else if(Vector3d.Dot(SurfaceVelocity, RelObstaclePosition) > 0)
					dV = Vector3d.Project(SurfaceVelocity, RelObstaclePosition) *
						-RAD.MinDistanceAhead/DistanceAhead*RAD.PitchRollAAf/VSL.MaxPitchRollAA_m;
				else dV = -TCA.HSC.NeededHorVelocity;
				TCA.HSC.CourseCorrections.Add(dV);
			}
			else CollisionSpeed = -1;
			//update angle for the next ray
			ViewAngle += AngleDelta;
		}

		public struct TerrainPoint
		{
			public double Altitude { get; private set; }
			public Vector3d Position { get; private set; }
			public bool Valid { get { return !Altitude.Equals(double.MinValue); } }

			public TerrainPoint(double alt, Vector3d pos)
			{ Altitude = alt; Position = pos; }

			public void Reset() 
			{ Altitude = double.MinValue; Position = Vector3d.zero; }

			public void Update(Vector3d position, CelestialBody body)
			{
				Position = position;
				//CelestialBody.GetRelSurfacePosition is broken
				Altitude = body.pqsController
					.GetSurfaceHeight(QuaternionD.AngleAxis(body.directRotAngle, Vector3d.up) * 
					                      (Position-body.pqsController.transformPosition))
					-body.pqsController.radius;
				if(body.ocean && Altitude < 0) Altitude = 0;
			}

			public Vector3 RelPosition(Vector3 ori) { return Position-ori; }

			public float DistanceTo(Vector3 pos)
			{ return Valid? (float)(Position-pos).magnitude : -1; }

			public bool CanCollideWith(VesselWrapper vsl, float threshold = 0)
			{ return vsl.AbsAltitude-Altitude-vsl.H-threshold < 0; }

			public static bool operator <(TerrainPoint p1, TerrainPoint p2)
			{ return p1.Altitude < p2.Altitude;	}
			public static bool operator >(TerrainPoint p1, TerrainPoint p2)
			{ return p1.Altitude > p2.Altitude;	}

			public bool BeforeDestination(VesselWrapper vsl, Vector3d vel)
			{ 
				return Valid &&
					(vsl.TCA.PN.Destination.IsZero() ||
					 Vector3.Dot(RelPosition(vsl.wCoM+vsl.TCA.PN.Destination), vsl.TCA.PN.Destination) < 0 ||
					 Vector3.Dot(vel, vsl.TCA.PN.Destination) < 0);
			}

			public override string ToString()
			{ return string.Format("[TerrainPoint: Valid={0}, Altitude={1}, Position={2}]", Valid, Altitude, Position); }
		}

		public struct Ray
		{
			RaycastHit hit;
			#if DEBUG
			float max_distance;
			#endif
			public Vector3 Ori { get; private set; }
			public Vector3 Dir { get; private set; }
			public float Altitude { get; private set; }
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
				#if DEBUG
				max_distance = dist;
				#endif
				Ori = ori; Dir = dir;
				Valid = Physics.SphereCast(Ori, radius, Dir, out hit, dist, RadarMask);
				return Valid;
			}

			/// <summary>
			/// Calculates altitude of the hit point relative the specified base direction
			/// </summary>
			/// <param name="base_dir">Base direction; must be normalized</param>
			/// <param name="angle">Initial angle between the casting dir and base_dir; 
			/// used to define the sign of the returned altitude</param>
			public void ClaculateAltitude(Vector3 base_dir, float angle)
			{
				Altitude = Valid? 
					Mathf.Sign(angle)*
					hit.distance*Mathf.Sin(
						Mathf.Acos(Mathf.Clamp(Vector3.Dot(base_dir, (hit.point-Ori).normalized), -1, 1))) : 
					float.MaxValue;
			}

			public void ClaculateAltitude(Vector3d up, float start_alt)
			{
				var rel_pos = hit.point-Ori;
				Altitude = Valid? 
					start_alt+Mathf.Sign(Vector3.Dot(rel_pos, up)) *
					Vector3.Project(rel_pos, up).magnitude :
					float.MaxValue;
			}

			#if DEBUG
			public void Draw()
			{
				GLUtils.GLLine(Ori, Valid? hit.point : Ori+Dir*max_distance, 
				                   Valid? Color.magenta : Color.red);
			}
			#endif

			public override string ToString()
			{
				return string.Format("[Ray: Valid={0}, Ori={1}, Dir={2}, Altitude={3}, Distance={4}, CollisionPoint={5}]", 
				                     Valid, Ori, Dir, Altitude, Distance, CollisionPoint);
			}
		}

		public class Sweep
		{
			public enum ManeuverType { None, Horizontal, Vertical }

			VesselWrapper VSL;
			Ray L, C, R;

			public ManeuverType Maneuver { get; private set; }
			public TerrainPoint Obstacle;
			public float Altitude { get; private set; }
			public bool  Valid { get; private set; }

			public bool BeforeDestination(Vector3d vel)
			{ return Obstacle.BeforeDestination(VSL, vel); }

			public Sweep(VesselWrapper vsl)	{ VSL = vsl; }

			public void Copy(Sweep s)
			{
				VSL = s.VSL;
				L = s.L; C = s.C; R = s.R;
				Obstacle = s.Obstacle;
				Altitude = s.Altitude;
				Maneuver = s.Maneuver;
				Valid = s.Valid;
			}

			public void Reset() 
			{ 
				Valid = false; 
				Altitude = float.MinValue; 
				Maneuver = ManeuverType.None;
				Obstacle.Reset(); L.Reset(); C.Reset(); R.Reset(); 
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
				{
					L.ClaculateAltitude(dir, angle);
					C.ClaculateAltitude(dir, angle);
					R.ClaculateAltitude(dir, angle);
					Altitude = VSL.AbsAltitude-Mathf.Min(L.Altitude, C.Altitude, R.Altitude);
					Ray ray;
					if(L.Valid && !R.Valid) { ray = L; Maneuver = ManeuverType.Horizontal; }
					else if(!L.Valid && R.Valid) { ray = R; Maneuver = ManeuverType.Horizontal; }
					else if(C.Valid) { ray = C; Maneuver = ManeuverType.Vertical; }
					else if(L.Valid && R.Valid) { ray = R.Distance < L.Distance? R : L; Maneuver = ManeuverType.Vertical; }
					else { VSL.Log("Unknown ManevuerType of a Valid Sweep. This should never happen."); return; }
					Obstacle = new TerrainPoint(Altitude, ray.CollisionPoint);
				}
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
			{ return string.Format("[Hit: Valid={0}, Maneuver {1}, Altitude {2}, Obstacle {3}]", Valid, Maneuver, Altitude, Obstacle); }

			#if DEBUG
			public void Draw() 
			{ L.Draw(); C.Draw(); R.Draw(); }
			#endif
		}

		public class PQS_Altimeter
		{
			readonly VesselWrapper VSL;
			float LookAheadTime = 1;
			float LookAheadTimeDelta = 1;

			TerrainPoint CurPoint;
			TerrainPoint BestPoint;
			public TerrainPoint Obstacle { get; private set; }
			public float Altitude { get { return (float)Obstacle.Altitude; } }

			public PQS_Altimeter(VesselWrapper vsl) { VSL = vsl; }

			void rewind()
			{
				LookAheadTime = 1;
				LookAheadTimeDelta = 1;
				BestPoint.Reset();
			}

			public bool BeforeDestination(Vector3d vel)
			{ return Obstacle.BeforeDestination(VSL, vel); }

			public void ProbeHeightAhead(Vector3 Dir)
			{
				if(VSL.mainBody == null || VSL.mainBody.pqsController == null) return;
				if(LookAheadTime > RAD.LookAheadTime) 
				{
					Obstacle = BestPoint;
					rewind();
				}
				else if(LookAheadTimeDelta < 0.1)
				{
					if(BestPoint.Valid)
						Obstacle = BestPoint;
					rewind();
				}
				CurPoint.Update(VSL.wCoM+Dir*(VSL.R+Utils.ClampL(VSL.HorizontalSpeed, 0.1f)*LookAheadTime), VSL.mainBody);
				if(CurPoint > BestPoint) BestPoint = CurPoint;
				if(BestPoint > Obstacle) Obstacle = BestPoint;
				if(VSL.AbsAltitude-CurPoint.Altitude <= VSL.H)
				{
					LookAheadTime -= LookAheadTimeDelta;
					LookAheadTimeDelta /= 2;
				}
				LookAheadTime += LookAheadTimeDelta;
			}

			public override string ToString()
			{ return string.Format("[PQS_Altimeter:\nCurPoint={0}\nBestPoint={1}\nObstacle={2}]", CurPoint, BestPoint, Obstacle); }

			#if DEBUG
			public void Draw() 
			{ GLUtils.GLLine(VSL.wCoM, CurPoint.Position, Color.green); }
			#endif
		}
	}
}

