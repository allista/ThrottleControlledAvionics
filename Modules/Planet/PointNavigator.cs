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

            [Persistent] public float MaxSpeedFilterUp     = 1000f;
            [Persistent] public float MaxSpeedFilterDown   = 100f;
            [Persistent] public float RotationAccelPhase   = 0.1f;
            [Persistent] public float CorrectionEasingRate = 0.8f;

			[Persistent] public PIDf_Controller DistancePID = new PIDf_Controller(0.5f, 0f, 0.5f, 0, 100);
			[Persistent] public PIDvd_Controller LateralPID = new PIDvd_Controller(0.5f, 0f, 0.5f, -100, 100);
            [Persistent] public PIDf_Controller CorrectionPID = new PIDf_Controller(0.5f, 0f, 0.5f, -100, 0);

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

        AsymmetricFiterF max_speed = new AsymmetricFiterF();
		readonly PIDf_Controller DistancePID = new PIDf_Controller();
		readonly PIDvd_Controller LateralPID = new PIDvd_Controller();
        readonly PIDf_Controller CorrectionPID = new PIDf_Controller();
		readonly Timer ArrivedTimer = new Timer();
		readonly Timer SharpTurnTimer = new Timer();

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

        #pragma warning disable 169
		HorizontalSpeedControl HSC;
		AltitudeControl ALT;
		AutoLander LND;
		Radar RAD;
        #pragma warning restore 169

		public override void Init()
		{
			base.Init();
			DistancePID.setPID(PN.DistancePID);
			DistancePID.Reset();
			LateralPID.setPID(PN.LateralPID);
			LateralPID.Reset();
            CorrectionPID.setPID(PN.CorrectionPID);
            CorrectionPID.Reset();
			ArrivedTimer.Period = PN.MinTime;
			FormationBreakTimer.Period = PN.FormationBreakTime;
			FormationUpdateTimer.Period = PN.FormationUpdateTimer;
			CFG.Nav.AddCallback(GoToTargetCallback, Navigation.GoToTarget, Navigation.FollowTarget);
			CFG.Nav.AddCallback(FollowPathCallback, Navigation.FollowPath);
            max_speed.TauUp = PN.MaxSpeedFilterUp;
            max_speed.TauDown = PN.MaxSpeedFilterDown;
            max_speed.Set(CFG.MaxNavSpeed);
		}

        public override void Disable()
        {
            CFG.Nav.OffIfOn(Navigation.GoToTarget, Navigation.FollowPath, Navigation.FollowTarget);
        }

		protected override void UpdateState() 
		{ 
			base.UpdateState();
            IsActive &= CFG.Target  && VSL.OnPlanet &&
                CFG.Nav.Any(Navigation.GoToTarget, Navigation.FollowPath, Navigation.FollowTarget);
		}

		public void GoToTargetCallback(Multiplexer.Command cmd)
		{
			switch(cmd)
			{
			case Multiplexer.Command.Resume:
                if(CFG.Target) 
					start_to(CFG.Target);
				else finish();
				break;

			case Multiplexer.Command.On:
				var wp = VSL.TargetAsWP;
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
				if(CFG.Path.Count > 0)
					start_to(CFG.Path.Peek());
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
            max_speed.Set(CFG.MaxNavSpeed);
			reset_formation();
			SetTarget(wp);
			DistancePID.Reset();
			LateralPID.Reset();
            CorrectionPID.Reset();
			CFG.HF.OnIfNot(HFlight.NoseOnCourse);
            NeedCPS();
		}

		void finish()
		{
			CFG.Nav.Off();
			CFG.HF.OnIfNot(HFlight.Stop);
            ReleaseCPS();
			reset_formation();
			SetTarget();
		}

		bool on_arrival()
		{
            if(CFG.Target == null || !CFG.Target.Valid) return false;
			if(CFG.Target.Land && LND != null)	
			{ 
				if(!CFG.Target.IsVessel)
					LND.StartFromTarget();
                VSL.Controls.PauseWhenStopped = CFG.Target.Pause;
				CFG.Target.Pause = false;
				CFG.AP1.XOn(Autopilot1.Land);
				return true; 
			}
			else if(CFG.Target.Pause) 
			{ 
				CFG.Target.Pause = false; 
				CFG.HF.XOn(HFlight.Stop); 
                VSL.Controls.PauseWhenStopped = true;
				return true;
			}
			return false;
		}

		void reset_formation()
		{
			Maneuvering = false;
			Formation = null;
			fnode = null;	
            tVSL = null;
            tPN = null;
		}

		public void UpdateFormation(List<FormationNode> formation)
		{
			if(formation == null) reset_formation();
			else { Formation = formation; fnode = null; }
		}

		void update_formation_info()
		{
			tVSL = CFG.Target.GetVessel();
			if(tVSL == null) 
            { 
                reset_formation(); 
                CanManeuver = false; 
                return; 
            }
			if(tPN == null || !tPN.Valid)
			{
				tTCA = ModuleTCA.EnabledTCA(tVSL);
				tPN = tTCA != null? tTCA.GetModule<PointNavigator>() : null;
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
            if(CFG.Nav.Paused) return;
			//differentiate between flying in formation and going to the target
			var vdistance = 0f; //vertical distance to the target
			if(CFG.Nav[Navigation.FollowTarget]) 
                update_formation_info();
			else 
			{ 
				VSL.Altitude.LowerThreshold = (float)CFG.Target.Pos.Alt;
				if(ALT != null && CFG.VF[VFlight.AltitudeControl] && CFG.AltitudeAboveTerrain)
					vdistance = VSL.Altitude.LowerThreshold+CFG.DesiredAltitude-VSL.Altitude.Absolute;
				tVSL = null; 
                tPN = null; 
			}
			//calculate direct distance
			var vdir = Vector3.ProjectOnPlane(CFG.Target.GetTransform().position+formation_offset-VSL.Physics.wCoM, VSL.Physics.Up);
            var hdistance = Utils.ClampL(vdir.magnitude-VSL.Geometry.R, 0);
			var bearing_threshold = Utils.Clamp(1/VSL.Torque.MaxCurrent.AngularAccelerationAroundAxis(VSL.Engines.CurrentDefThrustDir), 
			                                    PN.BearingCutoffCos, 0.98480775f); //10deg yaw error
			//update destination
            if(tPN != null && tPN.Valid && !tPN.VSL.Info.Destination.IsZero()) 
				VSL.Info.Destination = tPN.VSL.Info.Destination;
			else VSL.Info.Destination = vdir;
			//handle flying in formation
			var tvel = Vector3.zero;
			var vel_is_set = false;
			var end_distance = CFG.Target.AbsRadius;
			var dvel = VSL.HorizontalSpeed.Vector;
			if(CFG.Target.Land) end_distance /= 4;
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
				Maneuvering = CanManeuver && lat_dist > CFG.Target.AbsRadius && hdistance < CFG.Target.AbsRadius*3;
				if(keep_formation && tvel_m > 0 &&
				   (!CanManeuver || 
				    dir2vel_cos <= bearing_threshold || 
				    lat_dist < CFG.Target.AbsRadius*3))
				{
					if(CanManeuver) 
						HSC.AddWeightedCorrection(lat_dir.normalized*Utils.ClampH(lat_dist/CFG.Target.AbsRadius, 1) * 
						                          tvel_m*PN.FormationFactor*(Maneuvering? 1 : 0.5f));
					hdistance = Utils.ClampL(Mathf.Abs(dir2vel_cos)*hdistance-VSL.Geometry.R, 0);
					if(dir2vel_cos < 0)
					{
						if(hdistance < CFG.Target.AbsRadius)
							HSC.AddRawCorrection(tvel*Utils.Clamp(-hdistance/CFG.Target.AbsRadius*PN.FormationFactor, -PN.FormationFactor, 0));
						else if(Vector3.Dot(vdir, dvel) < 0 && 
						        (dvel.magnitude > PN.FollowerMaxAwaySpeed ||
						         hdistance > CFG.Target.AbsRadius*5))
						{
							keep_formation = true;
							VSL.HorizontalSpeed.SetNeeded(vdir);
							return;
						}
						else HSC.AddRawCorrection(tvel*(PN.FormationFactor-1));
						hdistance = 0;
					}
					vdir = tvel;
				}
			}
			//if the distance is greater that the threshold (in radians), use the Great Circle navigation
			if(hdistance/VSL.Body.Radius > PN.DirectNavThreshold)
			{
				var next = CFG.Target.PointFrom(VSL.vessel, 0.1);
				hdistance = (float)CFG.Target.DistanceTo(VSL.vessel);
				vdir = Vector3.ProjectOnPlane(VSL.Body.GetWorldSurfacePosition(next.Lat, next.Lon, VSL.vessel.altitude)
				                              -VSL.vessel.transform.position, VSL.Physics.Up);
				tvel = Vector3.zero;
			}
			else if(!VSL.IsActiveVessel && hdistance > GLB.UnpackDistance) 
				VSL.SetUnpackDistance(hdistance*1.2f);
			vdir.Normalize();
			//check if we have arrived to the target and stayed long enough
			if(hdistance < end_distance)
			{
				if(CFG.Nav[Navigation.FollowTarget])
				{
					var prev_needed_speed = VSL.HorizontalSpeed.NeededVector.magnitude;
					if(prev_needed_speed < 1 && !CFG.HF[HFlight.Move]) 
                        CFG.HF.OnIfNot(HFlight.Move);
					else if(prev_needed_speed > 10 && !CFG.HF[HFlight.NoseOnCourse]) 
                        CFG.HF.OnIfNot(HFlight.NoseOnCourse);
					if(tvel.sqrMagnitude > 1)
					{
						//set needed velocity and starboard to match that of the target
						keep_formation = true;
						VSL.HorizontalSpeed.SetNeeded(tvel);
						HSC.AddRawCorrection((tvel-VSL.HorizontalSpeed.Vector)*0.9f);
					}
					else 
                        VSL.HorizontalSpeed.SetNeeded(Vector3d.zero);
					vel_is_set = true;
				}
				else
				{
                    CFG.HF.OnIfNot(HFlight.Move);
                    VSL.Altitude.DontCorrectIfSlow();
                    VSL.HorizontalSpeed.SetNeeded(Vector3d.zero);
                    vel_is_set = true;
                    if(vdistance <= 0 && vdistance > -VSL.Geometry.R)
					{
						if(CFG.Nav[Navigation.FollowPath] && CFG.Path.Count > 0)
						{
							if(CFG.Path.Peek() == CFG.Target)
							{
								if(CFG.Path.Count > 1)
								{ 
									CFG.Path.Dequeue();
									if(on_arrival()) return;
									start_to(CFG.Path.Peek());
									return;
								}
								if(ArrivedTimer.TimePassed)
								{ 
									CFG.Path.Clear();
									if(on_arrival()) return;
									finish();
									return;
								}
							}
							else 
							{ 
								if(on_arrival()) return;
								start_to(CFG.Path.Peek()); 
								return; 
							}
						}
						else if(ArrivedTimer.TimePassed) 
						{ 
                            if(on_arrival()) return; 
                            finish(); 
                            return; 
                        }
					}
				}
			}
			else 
			{
				ArrivedTimer.Reset();
				CFG.HF.OnIfNot(HFlight.NoseOnCourse);
				//if we need to make a sharp turn, stop and turn, then go on
				var heading_dir = Vector3.Dot(VSL.OnPlanetParams.Heading, vdir);
				var hvel_dir = Vector3d.Dot(VSL.HorizontalSpeed.normalized, vdir);
                var sharp_turn_allowed = !CFG.Nav[Navigation.FollowTarget] ||
                    hdistance < end_distance*Utils.ClampL(all_followers.Count/2, 2);
				if(heading_dir < bearing_threshold && 
                   hvel_dir < bearing_threshold &&
                   sharp_turn_allowed)
					SharpTurnTimer.Start();
				if(SharpTurnTimer.Started)
				{
					VSL.HorizontalSpeed.SetNeeded(vdir);
					Maneuvering = false;
					vel_is_set = true;
                    if(heading_dir < bearing_threshold || sharp_turn_allowed &&
					   VSL.HorizontalSpeed.Absolute > 1 && Math.Abs(hvel_dir) < PN.BearingCutoffCos)
						SharpTurnTimer.Restart();
					else if(SharpTurnTimer.TimePassed) 
						SharpTurnTimer.Reset();
				}
//				Log("timer: {}\nheading*dir {} < {}, vel {} > 1, vel*dir {} < {}",
//				    SharpTurnTimer,
//				    Vector3.Dot(VSL.OnPlanetParams.Heading, vdir), bearing_threshold,
//				    VSL.HorizontalSpeed.Absolute, Vector3d.Dot(VSL.HorizontalSpeed.normalized, vdir), bearing_threshold);//debug
			}
			var cur_vel = (float)Vector3d.Dot(dvel, vdir);
			if(!vel_is_set)
			{
				//don't slow down on intermediate waypoints too much
				var min_dist = PN.OnPathMinDistance*VSL.Geometry.R;
				if(!CFG.Target.Land && CFG.Nav[Navigation.FollowPath] && 
				   CFG.Path.Count > 1 && hdistance < min_dist)
				{
					WayPoint next_wp = null;
					if(CFG.Path.Peek() == CFG.Target)
					{
						var iwp = CFG.Path.GetEnumerator();
						try 
						{ 
							iwp.MoveNext(); iwp.MoveNext();
							next_wp = iwp.Current;
						} 
						catch {}
					}
					else next_wp = CFG.Path.Peek();
					if(next_wp != null)
					{
						next_wp.Update(VSL);
						var next_dist = Vector3.ProjectOnPlane(next_wp.GetTransform().position-CFG.Target.GetTransform().position, VSL.Physics.Up);
						var angle2next = Utils.Angle2(vdir, next_dist);
						var minD = Utils.ClampL(min_dist*(1-angle2next/180/VSL.Torque.MaxPitchRoll.AA_rad*PN.PitchRollAAf), CFG.Target.AbsRadius);
						if(minD > hdistance) hdistance = minD;
					}
					else hdistance = min_dist;
				}
				else if(CFG.Nav.Not(Navigation.FollowTarget))
					hdistance = Utils.ClampL(hdistance-end_distance+VSL.Geometry.D, 0);
				//tune maximum speed and PID
				if(CFG.MaxNavSpeed < 10) CFG.MaxNavSpeed = 10;
                DistancePID.Min = GLB.HSC.TranslationMinDeltaV+0.1f;
				DistancePID.Max = CFG.MaxNavSpeed;
                if(CFG.Nav[Navigation.FollowTarget])
                {
                    DistancePID.P = PN.DistancePID.P/2;
                    DistancePID.D = DistancePID.P/2;
                }
                else
                {
                    DistancePID.P = PN.DistancePID.P;
                    DistancePID.D = PN.DistancePID.D;
                }
                if(cur_vel > 0)
				{
                    var mg2 = VSL.Physics.mg*VSL.Physics.mg;
                    var brake_thrust = Mathf.Min(VSL.Physics.mg, VSL.Engines.MaxThrustM/2*VSL.OnPlanetParams.TWRf);
                    var max_thrust = Mathf.Min(Mathf.Sqrt(brake_thrust*brake_thrust + mg2), VSL.Engines.MaxThrustM*0.99f);
					var manual_thrust = VSL.Engines.ManualThrustLimits.Project(VSL.LocalDir(vdir)).magnitude;
					if(manual_thrust > brake_thrust) brake_thrust = manual_thrust;
					else manual_thrust = -1;
                    if(brake_thrust > 0)
					{
                        var brake_accel = brake_thrust/VSL.Physics.M;
						var prep_time = 0f;
						if(manual_thrust < 0) 
						{
							var brake_angle = Utils.Angle2(VSL.Engines.CurrentDefThrustDir, vdir)-45;
							if(brake_angle > 0)
							{
                                //count rotation of the vessel to braking position
                                var axis = Vector3.Cross(VSL.Engines.CurrentDefThrustDir, vdir);
                                if(VSL.Torque.Slow)
                                {
                                    prep_time = VSL.Torque.NoEngines.RotationTime3Phase(brake_angle, axis, PN.RotationAccelPhase);
                                    //also count time needed for the engines to get to full thrust
                                    prep_time += Utils.LerpTime(VSL.Engines.Thrust.magnitude, VSL.Engines.MaxThrustM, max_thrust, VSL.Engines.AccelerationSpeed);
                                }
								else 
                                    prep_time = VSL.Torque.MaxCurrent.RotationTime2Phase(brake_angle, axis, VSL.OnPlanetParams.GeeVSF);
							}
						}
                        var prep_dist = cur_vel*prep_time+CFG.Target.AbsRadius;
                        var eta = hdistance/cur_vel;
                        max_speed.TauUp = PN.MaxSpeedFilterUp/eta/brake_accel;
                        max_speed.TauDown = eta*brake_accel/PN.MaxSpeedFilterDown;
                        max_speed.Update(prep_dist < hdistance?
                                         (1+Mathf.Sqrt(1+2/brake_accel*(hdistance-prep_dist)))*brake_accel : 
                                         2*brake_accel);
                        CorrectionPID.Min = -VSL.HorizontalSpeed.Absolute;
                        if(max_speed < cur_vel) 
                            CorrectionPID.Update(max_speed-cur_vel);
                        else
                        {
                            CorrectionPID.IntegralError *= (1-TimeWarp.fixedDeltaTime*PN.CorrectionEasingRate);
                            CorrectionPID.Update(0);
                        }
                        HSC.AddRawCorrection(CorrectionPID.Action*VSL.HorizontalSpeed.Vector.normalized);
					}
                    if(max_speed < CFG.MaxNavSpeed)
                        DistancePID.Max = Mathf.Max(DistancePID.Min, max_speed);
				}
				//take into account vertical distance and obstacle
				var rel_ahead = VSL.Altitude.Ahead-VSL.Altitude.Absolute;
//				Log("vdist {}, rel.ahead {}, vF {}, aF {}", vdistance, rel_ahead,
//				    Utils.ClampL(1 - Mathf.Atan(vdistance/hdistance)/Utils.HalfPI, 0),
//				    Utils.ClampL(1 - rel_ahead/RAD.DistanceAhead, 0));//debug
				vdistance = Mathf.Max(vdistance, rel_ahead);
				if(vdistance > 0)
                    hdistance *= Utils.ClampL(1 - Mathf.Atan(vdistance/hdistance)/(float)Utils.HalfPI, 0);
				if(RAD != null && rel_ahead > 0 && RAD.DistanceAhead > 0)
					hdistance *= Utils.ClampL(1 - rel_ahead/RAD.DistanceAhead, 0);
				//update the needed velocity
				DistancePID.Update(hdistance);
				var nV = vdir*DistancePID.Action;
				//correcto for Follow Target program
				if(CFG.Nav[Navigation.FollowTarget] && Vector3d.Dot(tvel, vdir) > 0) nV += tvel;
				VSL.HorizontalSpeed.SetNeeded(nV);
			}
			//correct for lateral movement
			var latV = -Vector3d.Exclude(vdir, VSL.HorizontalSpeed.Vector);
			var latF = (float)Math.Min((latV.magnitude/Math.Max(VSL.HorizontalSpeed.Absolute, 0.1)), 1);
			LateralPID.P = PN.LateralPID.P*latF;
			LateralPID.I = Math.Min(PN.LateralPID.I, latF);
			LateralPID.D = PN.LateralPID.D*latF;
			LateralPID.Update(latV);
			HSC.AddWeightedCorrection(LateralPID.Action);
//			Log("\ndir v {}\nlat v {}\nact v {}\nlatPID {}", 
//			     VSL.HorizontalSpeed.NeededVector, latV,
//			     LateralPID.Action, LateralPID);//debug
		}

		#if DEBUG
		public void RadarBeam()
		{
			if(VSL == null || VSL.vessel == null) return;
			if(CFG.Target && CFG.Target.GetTransform() != null && CFG.Nav[Navigation.FollowTarget])
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

