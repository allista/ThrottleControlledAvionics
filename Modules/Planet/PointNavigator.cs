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
using AT_Utils;

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
			[Persistent] public float MinDistance          = 3;
			[Persistent] public float MinTime              = 10;
			[Persistent] public float OnPathMinDistance    = 10;
			[Persistent] public float MinSpeed             = 0;
			[Persistent] public float MaxSpeed             = 500;
			[Persistent] public float DistanceFactor       = 0.1f;
			[Persistent] public float AngularAccelFactor   = 0.2f;

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
			[Persistent] public PID_Controller LateralPID = new PID_Controller(0.5f, 0f, 0.5f, -100, 100);

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
		static Config PN { get { return Globals.Instance.PN; } }
		public PointNavigator(ModuleTCA tca) : base(tca) {}

		public bool Maneuvering { get; private set; }
		public List<FormationNode> Formation;

		readonly PIDf_Controller DistancePID = new PIDf_Controller();
		readonly PIDvd_Controller LateralPID = new PIDvd_Controller();
		readonly Timer ArrivedTimer = new Timer();

		Vessel tVSL;
		ModuleTCA tTCA;
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
			DistancePID.setPID(PN.DistancePID);
			DistancePID.Reset();
			LateralPID.setPID(PN.LateralPID);
			LateralPID.Reset();
			ArrivedTimer.Period = PN.MinTime;
			FormationBreakTimer.Period = PN.FormationBreakTime;
			FormationUpdateTimer.Period = PN.FormationUpdateTimer;
			CFG.Nav.AddCallback(GoToTargetCallback, Navigation.GoToTarget, Navigation.FollowTarget);
			CFG.Nav.AddCallback(FollowPathCallback, Navigation.FollowPath);
		}

		protected override void UpdateState() 
		{ 
			base.UpdateState();
			IsActive &= CFG.Nav.Any(Navigation.GoToTarget, Navigation.FollowPath, Navigation.FollowTarget) && VSL.OnPlanet; 
		}

		public void GoToTargetCallback(Multiplexer.Command cmd)
		{
			switch(cmd)
			{
			case Multiplexer.Command.Resume:
				if(CFG.Target != null) 
					start_to(CFG.Target);
				else finish();
				break;

			case Multiplexer.Command.On:
				var wp = Target2WP();
				if(wp == null) finish();
				else start_to(wp);
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
				else finish();
				break;

			case Multiplexer.Command.Off:
				finish(); 
				break;
			}
		}

		void start_to(WayPoint wp)
		{
			VSL.UpdateOnPlanetStats();
			wp.Update(VSL);
			if(CFG.Nav[Navigation.GoToTarget] && wp.CloseEnough(VSL))
			{ 
				CFG.Nav.Off(); 
				return; 
			}
			if(VSL.LandedOrSplashed) 
			{
				CFG.AltitudeAboveTerrain = true;
				CFG.VF.OnIfNot(VFlight.AltitudeControl);
				CFG.DesiredAltitude = PN.TakeoffAltitude+VSL.Geometry.H;
			}				
			else if(CFG.VTOLAssistON) 
				VSL.GearOn(false);
			reset_formation();
			SetTarget(wp);
			DistancePID.Reset();
			LateralPID.Reset();
			CFG.HF.OnIfNot(HFlight.NoseOnCourse);
			RegisterTo<Radar>();
		}

		void finish()
		{
			CFG.Nav.Off();
			CFG.HF.OnIfNot(HFlight.Stop);
			UnregisterFrom<Radar>();
			reset_formation();
		}

		bool on_arrival()
		{
			if(CFG.Target == null) return false;
			if(CFG.Target.Pause) { PauseMenu.Display(); CFG.Target.Pause = false; }
			if(CFG.Target.Land)	{ CFG.AP1.XOn(Autopilot1.Land); return true; }
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
				tTCA = ModuleTCA.EnabledTCA(tVSL);
				if(tTCA != null) tPN = tTCA.GetModule<PointNavigator>();
			}
			var only_count = false;
			if(tVSL.srf_velocity.sqrMagnitude < PN.FormationSpeedSqr)
			{ reset_formation(); CanManeuver = false; only_count = true; }
			//update followers
			var offset = 0f;
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
					if(offset < vPN.VSL.Geometry.R)
						offset = vPN.VSL.Geometry.R;
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
				offset *= 2;
				var target_size = tTCA != null? tTCA.VSL.Geometry.D : Utils.ClampL(Math.Pow(tVSL.totalMass, 1/3f), 1);
				if(offset < target_size) offset = (float)target_size;
				offset *= PN.MinDistance;
				if(Formation == null || Formation.Count != num_offsets || FormationUpdateTimer.TimePassed)
				{
					FormationUpdateTimer.Reset();
					Formation = new List<FormationNode>(num_offsets);
					for(int i = 0; i < num_offsets; i++) 
						Formation.Add(new FormationNode(tVSL, i, forward, side, offset));
					all_followers.ForEach(p => p.Value.UpdateFormation(Formation));
				}
				else for(int i = 0; i < num_offsets; i++) 
					Formation[i].Update(forward, side, offset);
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
			CFG.Target.Update(VSL);
			//update things that are needed to fly in formation
			if(CFG.Nav[Navigation.FollowTarget]) update_formation_info();
			else { tVSL = null; tPN = null; }
			//calculate direct distance
			var vdir = Vector3.ProjectOnPlane(CFG.Target.GetTransform().position+formation_offset-VSL.Physics.wCoM, VSL.Physics.Up);
			var distance = Utils.ClampL(vdir.magnitude-VSL.Geometry.R, 0);
			//update destination
			if(tPN != null && !tPN.VSL.Info.Destination.IsZero()) 
				VSL.Info.Destination = tPN.VSL.Info.Destination;
			else VSL.Info.Destination = vdir;
			//handle flying in formation
			var tvel = Vector3.zero;
			var vel_is_set = false;
			var end_distance = CFG.Target.AbsRadius;
			if(CFG.Target.Land) end_distance /= 4;
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
				Maneuvering = CanManeuver && lat_dist > CFG.Target.AbsRadius && distance < CFG.Target.AbsRadius*3;
				if(keep_formation && tvel_m > 0 &&
				   (!CanManeuver || 
				    dir2vel_cos <= PN.BearingCutoffCos || 
				    lat_dist < CFG.Target.AbsRadius*3))
				{
					if(CanManeuver) 
						HSC.AddWeightedCorrection(lat_dir.normalized*Utils.ClampH(lat_dist/CFG.Target.AbsRadius, 1) * 
						                          tvel_m*PN.FormationFactor*(Maneuvering? 1 : 0.5f));
					distance = Utils.ClampL(Mathf.Abs(dir2vel_cos)*distance-VSL.Geometry.R, 0);
					if(dir2vel_cos < 0)
					{
						if(distance < CFG.Target.AbsRadius)
							HSC.AddRawCorrection(tvel*Utils.Clamp(-distance/CFG.Target.AbsRadius*PN.FormationFactor, -PN.FormationFactor, 0));
						else if(Vector3.Dot(vdir, dvel) < 0 && 
						        (dvel.magnitude > PN.FollowerMaxAwaySpeed ||
						         distance > CFG.Target.AbsRadius*5))
						{
							keep_formation = true;
							VSL.HorizontalSpeed.SetNeeded(vdir);
							return;
						}
						else HSC.AddRawCorrection(tvel*(PN.FormationFactor-1));
						distance = 0;
					}
					vdir = tvel;
				}
			}
			//if the distance is greater that the threshold (in radians), use Great Circle navigation
			if(distance/VSL.Body.Radius > PN.DirectNavThreshold)
			{
				var next = CFG.Target.PointFrom(VSL.vessel, 0.1);
				distance = (float)CFG.Target.DistanceTo(VSL.vessel);
				vdir = Vector3.ProjectOnPlane(VSL.Body.GetWorldSurfacePosition(next.Lat, next.Lon, VSL.vessel.altitude)
				                              -VSL.vessel.transform.position, VSL.Physics.Up);
				tvel = Vector3.zero;
			}
			else if(!VSL.IsActiveVessel && distance > GLB.UnpackDistance) 
				VSL.SetUnpackDistance(distance*1.2f);
			vdir.Normalize();
			//check if we have arrived to the target and stayed long enough
			if(distance < end_distance)
			{
				var prev_needed_speed = VSL.HorizontalSpeed.NeededVector.magnitude;
				if(prev_needed_speed < 1 && !CFG.HF[HFlight.Move]) CFG.HF.OnIfNot(HFlight.Move);
				else if(prev_needed_speed > 10 && !CFG.HF[HFlight.NoseOnCourse]) CFG.HF.OnIfNot(HFlight.NoseOnCourse);
				if(CFG.Nav[Navigation.FollowTarget])
				{
					if(tvel.sqrMagnitude > 1)
					{
						//set needed velocity and starboard to match that of the target
						keep_formation = true;
						VSL.HorizontalSpeed.SetNeeded(tvel);
						HSC.AddRawCorrection((tvel-VSL.HorizontalSpeed.Vector)*0.9f);
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
						else if(ArrivedTimer.TimePassed)
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
				else if(ArrivedTimer.TimePassed) { if(on_arrival()) return; finish(); return; }
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
			var cur_vel = (float)Vector3d.Dot(dvel, vdir);
			if(!vel_is_set)
			{
				//don't slow down on intermediate waypoints too much
				var min_dist = PN.OnPathMinDistance*VSL.Geometry.R;
				if(!CFG.Target.Land && CFG.Nav[Navigation.FollowPath] && 
				   CFG.Waypoints.Count > 1 && distance < min_dist)
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
						next_wp.Update(VSL);
						var next_dist = Vector3.ProjectOnPlane(next_wp.GetTransform().position-CFG.Target.GetTransform().position, VSL.Physics.Up);
						var angle2next = Vector3.Angle(vdir, next_dist);
						var minD = Utils.ClampL(min_dist*(1-angle2next/180/VSL.Torque.MaxPitchRoll.AA_rad*PN.PitchRollAAf), CFG.Target.AbsRadius);
						if(minD > distance) distance = minD;
					}
					else distance = min_dist;
				}
				else if(CFG.Nav.Not(Navigation.FollowTarget))
					distance = Utils.ClampL(distance-end_distance+VSL.Geometry.D, 0);
				//tune maximum speed and PID
				if(CFG.MaxNavSpeed < 10) CFG.MaxNavSpeed = 10;
				DistancePID.Min = 0;
				DistancePID.Max = CFG.MaxNavSpeed;
				if(cur_vel > 0)
				{
					var brake_thrust = Mathf.Max(VSL.Engines.ManualThrustLimits.Project(VSL.LocalDir(vdir)).magnitude,
					                             VSL.Physics.mg*VSL.OnPlanetParams.TWRf);
					var eta = distance/cur_vel;
					var max_speed = 0f;
					if(brake_thrust > 0)
					{
						var brake_accel = brake_thrust/VSL.Physics.M;
						var brake_time = cur_vel/brake_accel;
						max_speed = brake_accel*eta;
						if(eta <= brake_time*PN.BrakeOffset)
							HSC.AddRawCorrection((eta/brake_time/PN.BrakeOffset-1)*VSL.HorizontalSpeed.Vector);
					}
					if(max_speed < CFG.MaxNavSpeed) DistancePID.Max = max_speed;
				}
				//update the needed velocity
				DistancePID.Update(distance*PN.DistanceFactor);
				var nV = vdir*DistancePID.Action;
				//correcto for Follow Target program
				if(CFG.Nav[Navigation.FollowTarget] && Vector3d.Dot(tvel, vdir) > 0) nV += tvel;
				VSL.HorizontalSpeed.SetNeeded(nV);
			}
			//correct for lateral movement
			var latV = -Vector3d.Exclude(vdir, VSL.HorizontalSpeed.Vector);
			var latF = (float)Math.Min((latV.magnitude/Math.Max(VSL.HorizontalSpeed.Absolute, 0.1)), 1);
			LateralPID.P = PN.LateralPID.P*latF;
			LateralPID.I = Mathf.Min(PN.LateralPID.I, latF);
			LateralPID.D = PN.LateralPID.D*latF;
			LateralPID.Update(latV);
			HSC.AddWeightedCorrection(LateralPID.Action);
//			LogF("\ndir v {}\nlat v {}\nact v {}\nlatPID {}", 
//			     VSL.HorizontalSpeed.NeededVector, latV,
//			     LateralPID.Action, LateralPID);//debug
		}

		#if DEBUG
		public void RadarBeam()
		{
			if(VSL == null || VSL.vessel == null) return;
			if(CFG.Target != null && CFG.Target.GetTransform() != null && CFG.Nav[Navigation.FollowTarget])
				Utils.GLLine(VSL.Physics.wCoM, CFG.Target.GetTransform().position+formation_offset,
				               Maneuvering? Color.yellow : Color.cyan);
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

