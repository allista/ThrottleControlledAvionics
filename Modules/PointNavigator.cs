//  Author:
//       Allis Tauri <allista@gmail.com>
//
//  Copyright (c) 2015 Allis Tauri
//
// This work is licensed under the Creative Commons Attribution-ShareAlike 4.0 International License. 
// To view a copy of this license, visit http://creativecommons.org/licenses/by-sa/4.0/ 
// or send a letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.

using System;
using System.Collections.Generic;
using UnityEngine;

namespace ThrottleControlledAvionics
{
	[CareerPart]
	[RequireModules(typeof(HorizontalSpeedControl), 
	                typeof(BearingControl))]
	[OptionalModules(typeof(AltitudeControl))]
	public class PointNavigator : TCAModule
	{
		public class Config : ModuleConfig
		{
			new public const string NODE_NAME = "PN";

			[Persistent] public float MinDistance          = 30;
			[Persistent] public float MinTime              = 10;
			[Persistent] public float OnPathMinDistance    = 300;
			[Persistent] public float MinSpeed             = 10;
			[Persistent] public float MaxSpeed             = 300;
			[Persistent] public float DistanceF            = 2;
			[Persistent] public float DirectNavThreshold   = 1;
			[Persistent] public float GCNavStep            = 0.1f;
			[Persistent] public float LookAheadTime        = 2f;
			[Persistent] public float BearingCutoff        = 60f;
			[Persistent] public float FormationSpeedCutoff = 5f;
			[Persistent] public float FormationFactor      = 0.2f;
			[Persistent] public float FormationBreakTime   = 10f;
			[Persistent] public float FormationUpdateTimer = 60f;
			[Persistent] public float TakeoffAltitude      = 100f;
			[Persistent] public float BrakeOffset          = 1.5f;
			[Persistent] public float PitchRollAAf         = 100f;
			[Persistent] public float FollowerMaxAwaySpeed = 15f;

			[Persistent] public PID_Controller DistancePID = new PID_Controller(0.5f, 0f, 0.5f, 0, 100);

			public float BearingCutoffCos;
			public float FormationSpeedSqr;

			public override void Init()
			{
				base.Init();
				DirectNavThreshold *= Mathf.Deg2Rad;
				GCNavStep *= Mathf.Deg2Rad;
				BearingCutoffCos = Mathf.Cos(BearingCutoff*Mathf.Deg2Rad);
				FormationSpeedSqr = FormationSpeedCutoff*FormationSpeedCutoff;
			}
		}
		static Config PN { get { return TCAScenario.Globals.PN; } }
		public PointNavigator(ModuleTCA tca) : base(tca) {}

		public Vector3 Destination;
		public bool Maneuvering { get; private set; }
		public List<FormationNode> Formation;

		readonly PIDf_Controller pid = new PIDf_Controller();
		readonly Timer ArrivedTimer = new Timer();

		Vessel tVSL;
		PointNavigator tPN;
		SortedList<Guid, PointNavigator> all_followers = new SortedList<Guid, PointNavigator>();
		FormationNode fnode;
		Vector3 formation_offset { get { return fnode == null? Vector3.zero : fnode.Offset; } }
		bool CanManeuver = true;
		Timer FormationBreakTimer = new Timer();
		Timer FormationUpdateTimer = new Timer();
		bool keep_formation;

		HorizontalSpeedControl HSC;

		public override void Init()
		{
			base.Init();
			pid.setPID(PN.DistancePID);
			pid.Reset();
			ArrivedTimer.Period = PN.MinTime;
			FormationBreakTimer.Period = PN.FormationBreakTime;
			FormationUpdateTimer.Period = PN.FormationUpdateTimer;
			CFG.Nav.AddCallback(GoToTargetCallback, Navigation.GoToTarget, Navigation.FollowTarget);
			CFG.Nav.AddCallback(FollowPathCallback, Navigation.FollowPath);
			#if DEBUG
			RenderingManager.AddToPostDrawQueue(1, RadarBeam);
			#endif
		}

		protected override void UpdateState() 
		{ 
			IsActive = CFG.Nav && VSL.OnPlanet; 
			if(IsActive) return;
			Destination = Vector3.zero;
		}

		public void GoToTargetCallback(Multiplexer.Command cmd)
		{
			switch(cmd)
			{
			case Multiplexer.Command.Resume:
				if(CFG.Target != null) 
					start_to(CFG.Target);
				break;

			case Multiplexer.Command.On:
				if(!VSL.HasTarget) return;
				var wp = VSL.Target as WayPoint ?? 
					new WayPoint(VSL.Target);
				start_to(wp);
				break;

			case Multiplexer.Command.Off:
				finish(); break;
			}
		}

		public void FollowPathCallback(Multiplexer.Command cmd)
		{
			switch(cmd)
			{
			case Multiplexer.Command.Resume:
			case Multiplexer.Command.On:
				if(CFG.Waypoints.Count > 0)
					start_to(CFG.Waypoints.Peek());
				break;

			case Multiplexer.Command.Off:
				finish(); break;
			}
		}

		void start_to(WayPoint wp)
		{
			VSL.UpdateOnPlanetStats();
			if(VSL.LandedOrSplashed) 
			{
				CFG.AltitudeAboveTerrain = true;
				CFG.VF.OnIfNot(VFlight.AltitudeControl);
				CFG.DesiredAltitude = PN.TakeoffAltitude+VSL.Geometry.H;
			}				
			else if(CFG.VTOLAssistON) 
				VSL.vessel.ActionGroups.SetGroup(KSPActionGroup.Gear, false);
			reset_formation();
			SetTarget(wp);
			pid.Reset();
			CFG.HF.On(HFlight.NoseOnCourse);
			RegisterTo<Radar>();
		}

		void finish()
		{
			SetTarget(null);
			Destination = Vector3d.zero;
			CFG.Nav.Off();
			CFG.HF.On(HFlight.Stop);
			UnregisterFrom<Radar>();
			reset_formation();
		}

		bool on_arrival()
		{
			if(CFG.Target == null) return false;
			if(CFG.Target.Pause) { PauseMenu.Display(); CFG.Target.Pause = false; }
			if(CFG.Target.Land)	{ CFG.AP.XOn(Autopilot.Land); return true; }
			return false;
		}

		void reset_formation()
		{
			Maneuvering = false;
			Formation = null;
			fnode = null;			
		}

		public void UpdateFormation(List<FormationNode> formation)
		{
			if(formation == null) reset_formation();
			else { Formation = formation; fnode = null; }
		}

		void update_formation_info()
		{
			tVSL = CFG.Target.GetVessel();
			if(tVSL == null) { reset_formation(); CanManeuver = false; return; }
			if(tPN == null || !tPN.TCA.Valid || tPN.VSL.vessel != tVSL)
			{
				var tTCA = ModuleTCA.EnabledTCA(tVSL);
				if(tTCA != null) tPN = tTCA.GetModule<PointNavigator>();
			}
			var only_count = false;
			if(tVSL.srf_velocity.sqrMagnitude < PN.FormationSpeedSqr)
			{ reset_formation(); CanManeuver = false; only_count = true; }
			//update followers
			var can_maneuver = true;
			all_followers.Clear();
			for(int i = 0, num_vessels = FlightGlobals.Vessels.Count; i < num_vessels; i++)
			{
				var v = FlightGlobals.Vessels[i];
				if(v == null || v.packed || !v.loaded) continue;
				var tca = ModuleTCA.EnabledTCA(v);
				if(tca != null && 
				   (tca.vessel == VSL.vessel || 
				    tca.CFG.Nav[Navigation.FollowTarget] && 
				    tca.CFG.Target.GetTarget() != null && 
				    tca.CFG.Target.GetTarget() == CFG.Target.GetTarget()))
				{
					var vPN = tca.GetModule<PointNavigator>();
					if(vPN == null) continue;
					all_followers.Add(v.id, vPN);
					if(v.id != VSL.vessel.id)
						can_maneuver &= !vPN.Maneuvering || 
							(Maneuvering && VSL.vessel.id.CompareTo(v.id) > 0);
				}
			}
			if(only_count) return;
			CanManeuver = can_maneuver;
			var follower_index = all_followers.IndexOfKey(VSL.vessel.id);
			if(follower_index == 0)
			{
				var forward = tVSL == null? Vector3d.zero : -tVSL.srf_velocity.normalized;
				var side = Vector3d.Cross(VSL.Physics.Up, forward).normalized;
				var num_offsets = all_followers.Count+(all_followers.Count%2);
				if(Formation == null || Formation.Count != num_offsets || FormationUpdateTimer.Check)
				{
					FormationUpdateTimer.Reset();
					Formation = new List<FormationNode>(num_offsets);
					for(int i = 0; i < num_offsets; i++) 
						Formation.Add(new FormationNode(tVSL, i, forward, side, PN.MinDistance));
					all_followers.ForEach(p => p.Value.UpdateFormation(Formation));
				}
				else for(int i = 0; i < num_offsets; i++) 
					Formation[i].Update(forward, side, PN.MinDistance);
			}
			keep_formation = Formation != null;
			if(Formation == null || fnode != null) return;
			//compute follower offset
			var min_d   = -1f;
			var min_off = 0;
			for(int i = 0; i < Formation.Count; i++)
			{
				var node = Formation[i];
				if(node.Follower != null) continue;
				var d = node.Distance(VSL.vessel);
				if(min_d < 0 || min_d > d)
				{
					min_d = d;
					min_off = i;
				}
			}
			Formation[min_off].Follower = VSL.vessel;
			fnode = Formation[min_off];
		}

		protected override void Update()
		{
			if(!IsActive || CFG.Target == null || CFG.Nav.Paused) return;
			CFG.Target.Update(VSL.mainBody);
			//update things that are needed to fly in formation
			if(CFG.Nav[Navigation.FollowTarget]) update_formation_info();
			else { tVSL = null; tPN = null; }
			//calculate direct distance
			var vdir = Vector3.ProjectOnPlane(CFG.Target.GetTransform().position+formation_offset-VSL.Physics.wCoM, VSL.Physics.Up);
			var distance = Utils.ClampL(vdir.magnitude-VSL.Geometry.R, 0);
			//update destination
			if(CFG.Nav.Any(Navigation.GoToTarget, Navigation.FollowPath)) Destination = vdir;
			else if(tPN != null) Destination = tPN.Destination;
			//handle flying in formation
			var tvel = Vector3.zero;
			var vel_is_set = false;
			var end_distance = CFG.Target.Land?  CFG.Target.Distance/4 : CFG.Target.Distance;
			var dvel = VSL.HorizontalSpeed.Vector;
			if(tVSL != null && tVSL.loaded)
			{
				if(formation_offset.IsZero()) end_distance *= all_followers.Count/2f;
				tvel = Vector3d.Exclude(VSL.Physics.Up, tVSL.srf_velocity);
				dvel -= tvel;
				var tvel_m = tvel.magnitude;
				var dir2vel_cos = Vector3.Dot(vdir.normalized, tvel.normalized);
				var lat_dir  = Vector3.ProjectOnPlane(vdir-VSL.HorizontalSpeed.Vector*PN.LookAheadTime, tvel);
				var lat_dist = lat_dir.magnitude;
				FormationBreakTimer.RunIf(() => keep_formation = false, 
				                          tvel_m < PN.FormationSpeedCutoff);
				Maneuvering = CanManeuver && lat_dist > CFG.Target.Distance && distance < CFG.Target.Distance*3;
				if(keep_formation && tvel_m > 0 &&
				   (!CanManeuver || 
				    dir2vel_cos <= PN.BearingCutoffCos || 
				    lat_dist < CFG.Target.Distance*3))
				{
					if(CanManeuver) HSC.CourseCorrections.Add( 
						lat_dir.normalized*Utils.ClampH(lat_dist/CFG.Target.Distance, 1) * 
						tvel_m*PN.FormationFactor*(Maneuvering? 1 : 0.5f));
					distance = Utils.ClampL(Mathf.Abs(dir2vel_cos)*distance-VSL.Geometry.R, 0);
					if(dir2vel_cos < 0)
					{
						if(distance < CFG.Target.Distance)
							HSC.CourseCorrections.Add(tvel*Utils.Clamp(-distance/CFG.Target.Distance*PN.FormationFactor, -PN.FormationFactor, 0));
						else if(Vector3.Dot(vdir, dvel) < 0 && 
						        (dvel.magnitude > PN.FollowerMaxAwaySpeed ||
						         distance > CFG.Target.Distance*5))
						{
							keep_formation = true;
							VSL.HorizontalSpeed.SetNeeded(vdir);
							return;
						}
						else HSC.CourseCorrections.Add(tvel*(PN.FormationFactor-1));
						distance = 0;
					}
					vdir = tvel;
				}
			}
			//if the distance is greater that the threshold (in radians), use Great Circle navigation
			if(distance/VSL.mainBody.Radius > PN.DirectNavThreshold)
			{
				var next = CFG.Target.PointFrom(VSL.vessel, 0.1);
				distance = (float)CFG.Target.DistanceTo(VSL.vessel);
				vdir = Vector3.ProjectOnPlane(VSL.mainBody.GetWorldSurfacePosition(next.Lat, next.Lon, VSL.vessel.altitude)
				                              -VSL.vessel.transform.position, VSL.Physics.Up);
				tvel = Vector3.zero;
			}
			else if(!VSL.IsActiveVessel && distance > GLB.UnpackDistance) 
				VSL.SetUnpackDistance(distance*1.2f);
			vdir.Normalize();
			//check if we have arrived to the target and stayed long enough
			if(distance < end_distance)
			{
				CFG.HF.OnIfNot(VSL.HorizontalSpeed.NeededVector.sqrMagnitude > 1? HFlight.NoseOnCourse : HFlight.Move);
				if(CFG.Nav[Navigation.FollowTarget])
				{
					if(tvel.sqrMagnitude > 1)
					{
						//set needed velocity and starboard to match that of the target
						keep_formation = true;
						VSL.HorizontalSpeed.SetNeeded(tvel);
						HSC.CourseCorrections.Add((tvel-VSL.HorizontalSpeed.Vector)*0.9f);
					}
					else VSL.HorizontalSpeed.SetNeeded(Vector3d.zero);
					vel_is_set = true;
				}
				else if(CFG.Nav[Navigation.FollowPath] && CFG.Waypoints.Count > 0)
				{
					if(CFG.Waypoints.Peek() == CFG.Target)
					{
						if(CFG.Waypoints.Count > 1)
						{ 
							CFG.Waypoints.Dequeue();
							if(on_arrival()) return;
							start_to(CFG.Waypoints.Peek());
							return;
						}
						else if(ArrivedTimer.Check)
						{ 
							CFG.Waypoints.Clear();
							if(on_arrival()) return;
							finish();
							return;
						}
					}
					else 
					{ 
						if(on_arrival()) return;
						start_to(CFG.Waypoints.Peek()); 
						return; 
					}
				}
				else if(ArrivedTimer.Check) { if(on_arrival()) return; finish(); return; }
			}
			else 
			{
				CFG.HF.OnIfNot(HFlight.NoseOnCourse);
				ArrivedTimer.Reset();
			}
			//if we need to make a sharp turn, stop and turn, then go on
			if(Vector3.Dot(vdir, VSL.OnPlanetParams.Fwd) < PN.BearingCutoffCos &&
			   Vector3d.Dot(VSL.HorizontalSpeed.normalized, vdir) < PN.BearingCutoffCos)
			{
				VSL.HorizontalSpeed.SetNeeded(vdir);
				CFG.HF.OnIfNot(HFlight.NoseOnCourse);
				Maneuvering = false;
				vel_is_set = true;
			}
			if(vel_is_set) return;
			//don't slow down on intermediate waypoints too much
			if(!CFG.Target.Land && CFG.Nav[Navigation.FollowPath] && 
			   CFG.Waypoints.Count > 1 && distance < PN.OnPathMinDistance)
			{
				WayPoint next_wp = null;
				if(CFG.Waypoints.Peek() == CFG.Target)
				{
					var iwp = CFG.Waypoints.GetEnumerator();
					try 
					{ 
						iwp.MoveNext(); iwp.MoveNext();
						next_wp = iwp.Current;
					} 
					catch {}
				}
				else next_wp = CFG.Waypoints.Peek();
				if(next_wp != null)
				{
					next_wp.Update(VSL.mainBody);
					var next_dist = Vector3.ProjectOnPlane(next_wp.GetTransform().position-CFG.Target.GetTransform().position, VSL.Physics.Up);
					var angle2next = Vector3.Angle(vdir, next_dist);
					var minD = Utils.ClampL(PN.OnPathMinDistance*(1-angle2next/180/VSL.Torque.MaxPitchRollAA_m*PN.PitchRollAAf), CFG.Target.Distance);
					if(minD > distance) distance = minD;
				}
				else distance = PN.OnPathMinDistance;
			}
			//tune maximum speed
			var cur_vel = (float)Vector3d.Dot(dvel, vdir);
			pid.Min = 0;
			pid.Max = CFG.MaxNavSpeed;
			if(cur_vel > 0)
			{
				var lateral_thrust = VSL.OnPlanetParams.ManualThrustLimits.Project(VSL.LocalDir(vdir)).magnitude;
				var down_thrust = VSL.Engines.MaxThrustM*VSL.OnPlanetParams.MinVSFtwr*0.707f;
				var brake_thrust = lateral_thrust >= down_thrust? lateral_thrust : down_thrust;
				var eta = distance/cur_vel;
				var max_speed = 0f;
				if(brake_thrust > 0)
				{
					var brake_time = cur_vel/brake_thrust*VSL.Physics.M;
					max_speed = brake_thrust/VSL.Physics.M*eta;
					if(eta <= brake_time*PN.BrakeOffset)
						HSC.CourseCorrections.Add((eta/brake_time/PN.BrakeOffset-1)*VSL.HorizontalSpeed.Vector);
				}
				if(max_speed < CFG.MaxNavSpeed) pid.Max = max_speed;
			}
			//update the needed velocity
			pid.Update(distance*PN.DistanceF);
			var nV = vdir*pid.Action;
			//correcto for Follow Target program
			if(CFG.Nav[Navigation.FollowTarget] && Vector3d.Dot(tvel, vdir) > 0) nV += tvel;
			VSL.HorizontalSpeed.SetNeeded(nV);
		}

		#if DEBUG
		public void RadarBeam()
		{
			if(VSL == null || VSL.vessel == null) return;
			if(CFG.Target != null && CFG.Target.GetTransform() != null && CFG.Nav[Navigation.FollowTarget])
				GLUtils.GLLine(VSL.Physics.wCoM, CFG.Target.GetTransform().position+formation_offset,
				               Maneuvering? Color.yellow : Color.cyan);
		}

		public override void Reset()
		{
			base.Reset();
			RenderingManager.RemoveFromPostDrawQueue(1, RadarBeam);
		}
		#endif
	}

	public class FormationNode
	{
		public readonly Vessel Target;
		public readonly int Index;
		public Vector3 Offset { get; private set; }
		public Vessel Follower;

		public FormationNode(Vessel target, int i, Vector3 forward, Vector3 side, float dist)
		{
			Index = i+1;
			Target = target;
			Update(forward, side, dist);
		}

		public void Update(Vector3 forward, Vector3 side, float dist)
		{
			if(Index % 2 == 0) Offset = (forward+side)*dist*Index/2;
			else Offset = (forward-side)*dist*(Index+1)/2;
		}

		public float Distance(Vessel vsl)
		{ return (vsl.transform.position - Target.transform.position - Offset).magnitude; }

		public override string ToString()
		{
			return string.Format("[{0}]: Target {1}, Follower {2}, Offset {3}", 
			                     Index, Target.vesselName, Follower == null? "empty" : Follower.vesselName, Offset);
		}
	}
}

