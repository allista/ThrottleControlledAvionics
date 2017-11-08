
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
using AT_Utils;

namespace ThrottleControlledAvionics
{
	[CareerPart]
    [OptionalModules(typeof(AltitudeControl),
                     typeof(HorizontalSpeedControl))]
    [ModuleInputs(typeof(Anchor),
                  typeof(PointNavigator))]
	public class Radar : TCAService
	{
		public class Config : ModuleConfig
		{
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
		static Config RAD { get { return Globals.Instance.RAD; } }

		[Flags]
		public enum Mode 
		{ 
			Off = 0, 
			Vertical = 1 << 0, 
			Horizontal = 1 << 1, 
			Both = Vertical|Horizontal 
		}

		HorizontalSpeedControl HSC;

		public Radar(ModuleTCA tca) : base(tca)
		{ 
			CurHit = new Sweep(VSL);
			BestHit = new Sweep(VSL);
			DetectedHit = new Sweep(VSL);
			Altimeter = new PQS_Altimeter(VSL);
		}

		public float TimeAhead { get; private set; }
		public float DistanceAhead { get; private set; }

		public static readonly int RadarMask = (1 << LayerMask.NameToLayer("Local Scenery"));
		//normal radar
		Mode     mode;
		Vector3  Dir;
		Vector3d SurfaceVelocity;
		float    ViewAngle;
		float    AngleDelta;
		int      LittleSteps;
		float    MaxDistance;
		TerrainPoint Obstacle;
		TerrainPoint VelocityHit;
		Vector3  RelObstaclePosition;
		Vector3d NeededHorVelocity;
		float    LookAheadTime;
		float    ClosingSpeed;
		float    CollisionSpeed = -1;
		Ray      VelocityRay;
		Ray      DirectPathRay;
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
			Reset();
		}

		#if DEBUG
		public void DrawDebugLines()
		{
			if(VSL == null || VSL.vessel == null || !IsActive) return;
			VelocityRay.Draw(Color.yellow);
			DirectPathRay.Draw(Color.cyan);
			Altimeter.Draw();
			CurHit.Draw(Color.red);
			if(DetectedHit.Valid)
				DetectedHit.Draw(Color.clear);
		}
		#endif

        public override void Disable()
        {
            Reset();
        }

		protected override void UpdateState() 
		{ 
			base.UpdateState();
			IsActive &= VSL.OnPlanet && !VSL.LandedOrSplashed;
			if(IsActive)
			{
				mode = Mode.Off;
				if(CFG.HF && !CFG.HF[HFlight.Level])
					mode |= Mode.Horizontal;
				if(CFG.VF[VFlight.AltitudeControl] && CFG.AltitudeAboveTerrain)
					mode |= Mode.Vertical;
				IsActive = CollisionSpeed > 0 || mode != Mode.Off && HasActiveClients;
			}
		}

		protected override void Reset()
		{
			base.Reset();
			rewind();
			DetectedHit.Reset();
			Altimeter.Reset();
			CollisionSpeed = -1;
			DistanceAhead  = -1;
			TimeAhead      = -1;
			VSL.Altitude.Ahead  = float.MinValue;
			RelObstaclePosition = Vector3.zero;
		}

		void rewind()
		{
			CurHit.Reset();
			BestHit.Reset();
			VelocityRay.Reset();
			DirectPathRay.Reset();
			ViewAngle = -RAD.UpViewAngle;
			AngleDelta = RAD.AngleDelta;
			LittleSteps = 0;
			LastHitValid = false;
			ManeuverTimer.RunIf(() => SideManeuver.Zero(), !SideCollision);
			SideCollision = false;
		}

		protected override void Update()
		{
			NeededHorVelocity = HSC == null? Vector3d.zero : VSL.HorizontalSpeed.NeededVector;
			var zero_needed = NeededHorVelocity.sqrMagnitude <= 0.01;
			//check boundary conditions
			if(ViewAngle > RAD.DownViewAngle) 
			{ 
				if(BestHit.Valid) 
				{ 
					DetectedHit.Copy(BestHit); 
					rewind(); 
				}
				else Reset(); 
			}
			else if(AngleDelta < RAD.MinAngleDelta || LittleSteps > RAD.MaxLittleSteps) 
			{
				if(BestHit.Valid) 
					DetectedHit.Copy(BestHit); 
				rewind();
			}
			//calculate closing speed and initial ray direction
			Dir = Vector3.zero;
			LookAheadTime = Utils.ClampL(RAD.LookAheadTime/VSL.OnPlanetParams.MaxTWR*VSL.Engines.AccelerationTime90, 10);
			SurfaceVelocity = VSL.PredictedSrfVelocity(GLB.CPS.LookAheadTime);
			var SurfaceVelocityDir = SurfaceVelocity.normalized;
			var SurfaceSpeed = (float)SurfaceVelocity.magnitude;
			var alt_threshold = VSL.Altitude.Absolute - 
				Mathf.Min(Utils.ClampL(CFG.DesiredAltitude-1, 0), VSL.Geometry.H*RAD.MinAltitudeFactor*(CollisionSpeed < 0? 1 : 2));
			if((DistanceAhead < 0 || DistanceAhead > RAD.MinDistanceAhead ||
		        Vector3.Dot(RelObstaclePosition, NeededHorVelocity) < 0) &&
			   (VSL.HorizontalSpeed >= RAD.MaxClosingSpeed ||
			    zero_needed && 
			    VSL.HorizontalSpeed >= RAD.MinClosingSpeed))
			{
				Dir = VSL.HorizontalSpeed.normalized;
				if(VSL.IsStateSet(TCAState.LoosingAltitude))
					Dir = Vector3.Lerp(Dir, SurfaceVelocityDir,
					                   LookAheadTime/(VSL.Altitude/-VSL.VerticalSpeed.Relative)/VSL.OnPlanetParams.MaxDTWR);
			}
			else Dir = zero_needed? 
					Vector3d.Exclude(VSL.Physics.Up, VSL.OnPlanetParams.Fwd).normalized : 
					NeededHorVelocity.normalized;
			Dir.Normalize();
			ClosingSpeed = Utils.ClampL(SurfaceSpeed > NeededHorVelocity.magnitude? 
			                            Vector3.Dot(SurfaceVelocity, Dir) : 
			                            Vector3.Dot(NeededHorVelocity, Dir), 0);
			//calculate ray length
			var ray_speed = CollisionSpeed < ClosingSpeed? ClosingSpeed : CollisionSpeed;
			if(VSL.Info.Destination.IsZero()) MaxDistance = ray_speed*LookAheadTime;
			else MaxDistance = VSL.Info.Destination.magnitude + ray_speed*GLB.CPS.LookAheadTime;
			if(ViewAngle < 0) 
			{
				MaxDistance /= Mathf.Cos(ViewAngle*Mathf.Deg2Rad);
				if(ClosingSpeed > RAD.MinClosingSpeed)
					MaxDistance *= Utils.ClampL((ClosingSpeed-RAD.MinClosingSpeed)/RAD.UpViewSlope*(-ViewAngle/RAD.UpViewAngle), 1);
			}
			MaxDistance = Mathf.Max(MaxDistance, VSL.Geometry.D);
			//cast the sweep, the velocity and direct path rays
			if(ViewAngle < 0 && !zero_needed)
			{
				DirectPathRay.Cast(VSL.Physics.wCoM, Dir, MaxDistance, VSL.Geometry.R*1.1f);
				if(!DirectPathRay.Valid || !DirectPathRay.BeforeDestination(VSL, NeededHorVelocity))
					ViewAngle = 0;
			}
			CurHit.Cast(Dir, ViewAngle, MaxDistance);
			VelocityRay.Cast(VSL.Physics.wCoM, VSL.vessel.srf_vel_direction, (float)VSL.vessel.srfSpeed*GLB.CPS.LookAheadTime*3, VSL.Geometry.R*1.1f);
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
						var collision_point = Vector3.ProjectOnPlane(CurHit.Obstacle.RelPosition(VSL.Physics.wCoM), VSL.Physics.Up);
						var dist = collision_point.magnitude;
						Vector3d maneuver;
						if(CollisionPreventionSystem.AvoidStatic(VSL, collision_point/dist, dist, 
						                                         Vector3d.Exclude(VSL.Physics.Up, SurfaceVelocity), out maneuver))
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
			if(VelocityRay.Valid)
			{
				VelocityRay.ClaculateAltitude(VSL.Physics.Up, VSL.Altitude.Absolute);
				VelocityHit = new TerrainPoint(VelocityRay.Altitude, VelocityRay.CollisionPoint);
				if(VelocityRay.Altitude > Obstacle.Altitude || VelocityRay.Altitude > alt_threshold)
					Obstacle = VelocityHit;
			}
			if(Obstacle.Valid) 
			{
				VSL.Altitude.Ahead = (float)Obstacle.Altitude;
				RelObstaclePosition = Obstacle.HorPosition(VSL);
				DistanceAhead = Utils.ClampL(RelObstaclePosition.magnitude-VSL.Geometry.R, 0.1f);
			}
//			if(Obstacle.Valid) VSL.Info.AddCustopWaypoint(Obstacle.Position, "Obstacle: "+Utils.formatBigValue((float)Obstacle.Altitude, "m"));//debug
			//if on side collision course, correct it
			if(HSC != null && !SideManeuver.IsZero()) 
				HSC.AddWeightedCorrection(SideManeuver);
//			Log("Obstacle! {}, SrfSpeed {}, NeedeSpeed {}, ClosingSpeed {}, MaxDistance {}\n" +
//				"CurHit {}\nBestHit {}\nDetectedHit {}\nRayObstacle {}\nAltObstacle {}\nForwardRay {}",
//			    VSL.Altitude.Ahead > alt_threshold, SurfaceSpeed, NeededHorVelocity.magnitude, ClosingSpeed, MaxDistance,
//			    CurHit, BestHit, DetectedHit, Obstacle, Altimeter.Obstacle, ForwardRay);//debug
			//check for possible stright collision
			if(VSL.Altitude.Ahead > alt_threshold)
			{ 
				if(CollisionSpeed < ClosingSpeed) CollisionSpeed = ClosingSpeed;

				TimeAhead = DistanceAhead/Utils.ClampL(Vector3.Dot(SurfaceVelocity, RelObstaclePosition.normalized), 1e-5f);
//				Log("DistAhead {}, Speed {}, Time {}", DistanceAhead, 
//				    Utils.ClampL(Vector3.Dot(SurfaceVelocity, RelObstaclePosition.normalized), 1e-5f),
//				    TimeAhead);//debug
				if(HSC != null)
				{
					Vector3d dV;
					if(DistanceAhead > RAD.MinDistanceAhead)
						dV = Vector3d.Project(SurfaceVelocity, RelObstaclePosition) *
							-Math.Sqrt(1-Utils.ClampH(DistanceAhead/ClosingSpeed/LookAheadTime*VSL.OnPlanetParams.MaxTWR*RAD.NHVf, 1));
					else if(DistanceAhead > RAD.MinDistanceAhead/2)
						dV = -NeededHorVelocity;
					else if(Vector3d.Dot(SurfaceVelocity, RelObstaclePosition) > 0)
						dV = Vector3d.Project(SurfaceVelocity, RelObstaclePosition) *
							-RAD.MinDistanceAhead/DistanceAhead*RAD.PitchRollAAf/VSL.Torque.MaxPitchRoll.AA_rad;
					else dV = -NeededHorVelocity;
					HSC.AddRawCorrection(dV);
				}
			}
			else 
			{
				if(VelocityHit.Valid) 
				{
//					VSL.Info.AddCustopWaypoint(DirectHit.Position, "DirectHit");//debug
					VSL.Altitude.LowerThreshold = (float)VelocityHit.Altitude;
					var rel_pos = VelocityHit.HorPosition(VSL);
					if(HSC != null &&
                       !VSL.HorizontalSpeed.MoovingFast && !VSL.Info.Destination.IsZero() && 
                       VelocityHit.Altitude-VSL.Altitude.TerrainAltitude > 1 &&
					   Vector3.Dot(VSL.Info.Destination, rel_pos-VSL.Info.Destination) < 0)
					{
						var dV = rel_pos.normalized*GLB.HSC.TranslationMaxDeltaV;
					    if(Vector3.Dot(rel_pos, VSL.Info.Destination) > 0)
							HSC.AddRawCorrection(Vector3.Project(dV, VSL.Info.Destination)*2-dV);
						else HSC.AddRawCorrection(-dV);
					}
					if(!VelocityRay.Valid && 
					   (VSL.HorizontalSpeed.MoovingFast || rel_pos.magnitude > VSL.Geometry.R))
						VelocityHit.Reset();
				}
				CollisionSpeed = -1;
			}
			//update angle for the next ray
			ViewAngle += AngleDelta;
		}

		public struct TerrainPoint
		{
			public double Altitude { get; private set; }
			public Vector3d Position { get; private set; }
			public bool Valid { get { return !Altitude.Equals(double.MinValue); } }

			public TerrainPoint(double alt, Vector3d pos) : this()
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

			public Vector3 HorPosition(VesselWrapper vsl) 
			{ return Vector3.ProjectOnPlane(Position-vsl.Physics.wCoM, vsl.Physics.Up); }

			public float DistanceTo(Vector3 pos)
			{ return Valid? (float)(Position-pos).magnitude : -1; }

			public bool CanCollideWith(VesselWrapper vsl, float threshold = 0)
			{ return vsl.Altitude.Absolute-Altitude-vsl.Geometry.H-threshold < 0; }

			public static bool operator <(TerrainPoint p1, TerrainPoint p2)
			{ return p1.Altitude < p2.Altitude;	}
			public static bool operator >(TerrainPoint p1, TerrainPoint p2)
			{ return p1.Altitude > p2.Altitude;	}

			public bool BeforeDestination(VesselWrapper VSL, Vector3d vel)
			{ 
				return Valid &&
					(VSL.Info.Destination.IsZero() ||
					 Vector3.Dot(RelPosition(VSL.Physics.wCoM+VSL.Info.Destination), VSL.Info.Destination) < 0 ||
					 Vector3.Dot(vel, VSL.Info.Destination) < 0);
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

			public bool BeforeDestination(VesselWrapper VSL, Vector3d vel)
			{
				var point = new TerrainPoint(0, CollisionPoint);
				return point.BeforeDestination(VSL, vel);
			}

			#if DEBUG
			public void Draw(Color idle_color)
			{
				Utils.GLLine(Ori, Valid? hit.point : Ori+Dir*max_distance, 
				             Valid? Color.magenta : idle_color);
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

			public Sweep(VesselWrapper vsl) { VSL = vsl; }

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
				var side = Vector3.Cross(VSL.Physics.Up, dir)*VSL.Geometry.R*1.5f;
				var cast_dir = Quaternion.AngleAxis(angle, side)*dir;
				Valid |= L.Cast(VSL.Physics.wCoM-side, cast_dir, dist, VSL.Geometry.R);
				Valid |= C.Cast(VSL.Physics.wCoM,      cast_dir, dist, VSL.Geometry.R);
				Valid |= R.Cast(VSL.Physics.wCoM+side, cast_dir, dist, VSL.Geometry.R);
				if(Valid) 
				{
					L.ClaculateAltitude(dir, angle);
					C.ClaculateAltitude(dir, angle);
					R.ClaculateAltitude(dir, angle);
					Altitude = VSL.Altitude.Absolute-Mathf.Min(L.Altitude, C.Altitude, R.Altitude);
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
			public void Draw(Color idle_color) 
			{ L.Draw(idle_color); C.Draw(idle_color); R.Draw(idle_color); }
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

			public void Reset()
			{
				CurPoint.Reset(); 
				BestPoint.Reset(); 
				Obstacle.Reset();
			}

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
				if(VSL.Body == null || VSL.Body.pqsController == null) return;
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
				CurPoint.Update(VSL.Physics.wCoM+Dir*(VSL.Geometry.R+Utils.ClampL(VSL.HorizontalSpeed, 0.1f)*LookAheadTime), VSL.Body);
				if(CurPoint > BestPoint) BestPoint = CurPoint;
				if(BestPoint > Obstacle) Obstacle = BestPoint;
				if(VSL.Altitude.Absolute-CurPoint.Altitude <= VSL.Geometry.H)
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
			{ Utils.GLLine(VSL.Physics.wCoM, CurPoint.Position, Color.green); }
			#endif
		}
	}
}

