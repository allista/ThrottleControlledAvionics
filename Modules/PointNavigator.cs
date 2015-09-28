//   PointNavigator.cs
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
	public class PointNavigator : TCAModule
	{
		public class Config : ModuleConfig
		{
			new public const string NODE_NAME = "PN";

			[Persistent] public float MinDistance        = 30;
			[Persistent] public float MinTime            = 10;
			[Persistent] public float OnPathMinDistance  = 300;
			[Persistent] public float MinSpeed           = 10;
			[Persistent] public float MaxSpeed           = 300;
			[Persistent] public float DeltaSpeed         = 10f;
			[Persistent] public float DeltaSpeedF        = 0.5f;
			[Persistent] public float FallingSensitivity = 10f;
			[Persistent] public float FallingCorrection  = 3f;
			[Persistent] public float AltErrSensitivity  = 10f;
			[Persistent] public float DistanceF          = 2;
			[Persistent] public float DirectNavThreshold = 1;
			[Persistent] public float GCNavStep          = 0.1f;
			[Persistent] public float AngularAccelF      = 10f;
			[Persistent] public float MaxAccelF          = 10f;
			[Persistent] public float LookAheadTime      = 2f;
			[Persistent] public PID_Controller DistancePID = new PID_Controller(0.5f, 0f, 0.5f, 0, 100);

			public override void Init()
			{
				base.Init();
				DirectNavThreshold *= Mathf.Deg2Rad;
				GCNavStep *= Mathf.Deg2Rad;
			}
		}
		static Config PN { get { return TCAScenario.Globals.PN; } }
		public PointNavigator(VesselWrapper vsl) { VSL = vsl; }

		float DeltaSpeed;
		readonly PIDf_Controller pid = new PIDf_Controller();
		readonly Timer ArrivedTimer = new Timer();
		readonly EWA AccelCorrection = new EWA();

		SortedList<Guid, ModuleTCA> all_followers = new SortedList<Guid, ModuleTCA>();
		Vector3 follower_offset = Vector3.zero;

		public override void Init()
		{
			base.Init();
			pid.setPID(PN.DistancePID);
			pid.Reset();
			ArrivedTimer.Period = PN.MinTime;
			CFG.Nav.AddCallback(Navigation.GoToTarget, GoToTarget);
			CFG.Nav.AddCallback(Navigation.FollowTarget, GoToTarget);
			CFG.Nav.AddCallback(Navigation.FollowPath, FollowPath);
			if(CFG.Target != null) SetTarget(CFG.Target);
		}

		public override void UpdateState() { IsActive = CFG.Nav && VSL.OnPlanet; }

		public void GoToTarget(bool enable = true)
		{
			if(enable && !VSL.HasTarget) return;
			if(enable) 
			{
				var wp = VSL.vessel.targetObject as WayPoint ?? 
					new WayPoint(VSL.vessel.targetObject);
				start_to(wp);
			}
			else finish();
		}

		public void FollowPath(bool enable = true)
		{
			if(enable && CFG.Waypoints.Count == 0) return;
			if(enable) start_to(CFG.Waypoints.Peek());	
			else finish();
		}

		IEnumerator updater;
		IEnumerator offset_updater()
		{
			var tvsl = CFG.Target.GetVessel();
			if(tvsl.srf_velocity.sqrMagnitude < 0.25)
			{ follower_offset = Vector3.zero; yield break; }
			//update followers
			var vi = FlightGlobals.Vessels.GetEnumerator();
			while(true)
			{
				try { if(!vi.MoveNext()) break; }
				catch { yield break; }
				var v = vi.Current;
				if(v == null) continue;
				if(v.packed || !v.loaded)
				{
					all_followers.Remove(v.id);
					continue;
				}
				var tca = ModuleTCA.EnabledTCA(v);
				if(tca != null && tca.CFG.Nav[Navigation.FollowTarget] && tca.CFG.Target.GetTarget() != null && tca.CFG.Target.GetTarget() == CFG.Target.GetTarget())
					all_followers[v.id] = tca;
				else
					all_followers.Remove(v.id);
				yield return null;
			}
			//compute follower offset from index
			var follower_index = 0;
			follower_offset = Vector3.zero;
			try { follower_index = all_followers.IndexOfKey(VSL.vessel.id)+1; }
			catch { yield break; }
			var forward = tvsl == null? Vector3d.zero : -tvsl.srf_velocity.normalized;
			var side = Vector3d.Cross(VSL.Up, forward).normalized;
			if(follower_index % 2 == 0)
				follower_offset += (forward+side)*PN.MinDistance*follower_index/2;
			else 
				follower_offset += (forward-side)*PN.MinDistance*(follower_index+1)/2;
		}

		bool update_follower_offset()
		{
			if(updater == null) updater = offset_updater();
			if(updater.MoveNext()) return true;
			updater = null;
			return false;
		}

		void start_to(WayPoint wp)
		{
			SetTarget(wp);
			pid.Reset();
			follower_offset = Vector3.zero;
			VSL.UpdateOnPlanetStats();
			CFG.HF.On(HFlight.NoseOnCourse);
			VSL.ActionGroups.SetGroup(KSPActionGroup.Gear, false);
		}

		void finish()
		{
			SetTarget(null);
			CFG.HF.On(HFlight.Stop);
		}

		bool on_arrival()
		{
			if(CFG.Target == null) return false;
			if(CFG.Target.Pause) { PauseMenu.Display(); CFG.Target.Pause = false; }
			if(CFG.Target.Land)	{ CFG.AP.OnIfNot(Autopilot.Land); return true; }
			return false;
		}

		public void Update()
		{
			if(!IsActive || CFG.Target == null) return;
			CFG.Target.Update(VSL.mainBody);
			//calculate direct distance
			if(CFG.Nav[Navigation.FollowTarget]) update_follower_offset();
			var tvsl = CFG.Target.GetVessel();
			var vdir = Vector3.ProjectOnPlane(CFG.Target.GetTransform().position+follower_offset-VSL.vessel.transform.position, VSL.Up);
			var distance = Utils.ClampL(vdir.magnitude-VSL.R, 0);
			var tvel = Vector3d.zero;
			if(tvsl != null && tvsl.loaded && CFG.Nav[Navigation.FollowTarget])
			{
				tvel = Vector3d.Exclude(VSL.Up, tvsl.srf_velocity+tvsl.acceleration*PN.LookAheadTime);
				if(distance < CFG.Target.Distance*3) 
				{
					distance = Vector3d.Dot(vdir, tvel) > 0? 
						Utils.ClampL((float)Vector3d.Project(vdir, tvel).magnitude-VSL.R, 0) : 0;
					vdir = tvel;
				}
			}
			//if it is greater that the threshold (in radians), use Great Circle navigation
			if(distance/VSL.mainBody.Radius > PN.DirectNavThreshold)
			{
				var next = CFG.Target.PointFrom(VSL.vessel, 0.1);
				distance = (float)CFG.Target.DistanceTo(VSL.vessel);
				vdir = Vector3.ProjectOnPlane(VSL.mainBody.GetWorldSurfacePosition(next.Lat, next.Lon, VSL.vessel.altitude)
				                              -VSL.vessel.transform.position, VSL.Up);
			}
			vdir.Normalize();
			//check if we have arrived to the target and stayed long enough
			if(distance < CFG.Target.Distance)
			{
				CFG.HF.OnIfNot(CFG.NeededHorVelocity.sqrMagnitude > 1? HFlight.NoseOnCourse : HFlight.Move);
				if(CFG.Nav[Navigation.FollowTarget])
				{
					//set needed velocity and starboard to zero if in range of a target
					if(tvsl != null && tvsl.loaded)
					{
						CFG.NeededHorVelocity = tvel;
						CFG.Starboard = VSL.GetStarboard(CFG.NeededHorVelocity);
					}
					else 
					{
						CFG.NeededHorVelocity = Vector3d.zero;
						CFG.Starboard = Vector3d.zero;
					}
					return;
				}
				if(CFG.Nav[Navigation.FollowPath] && CFG.Waypoints.Count > 0)
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
			//don't slow down on intermediate waypoints too much
			if(CFG.Nav[Navigation.FollowPath] && CFG.Waypoints.Count > 1 && distance < PN.OnPathMinDistance)
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
					var next_dist = Vector3.ProjectOnPlane(next_wp.GetTransform().position-CFG.Target.GetTransform().position, VSL.Up);
					var angle2next = Vector3.Angle(vdir, next_dist);
					var minD = PN.OnPathMinDistance*(1-angle2next/190);
					if(minD > distance) distance = minD;
//					Log("distance {0}; min_distance {1}; angle2next {2}", distance, PN.OnPathMinDistance, angle2next);//debug
				}
				else distance = PN.OnPathMinDistance;
			}
			//tune the pid and update needed velocity
			AccelCorrection.Update(Mathf.Clamp(VSL.MaxThrust.magnitude*VSL.MinVSFtwr/VSL.M/PN.MaxAccelF, 0.01f, 1)*
			                       Mathf.Clamp(VSL.MaxPitchRollAA_m/PN.AngularAccelF, 0.01f, 1), 0.01f);
			pid.Min = 0;
			pid.Max = CFG.MaxNavSpeed;
			pid.P   = PN.DistancePID.P*AccelCorrection;
			pid.D   = PN.DistancePID.D*(2-AccelCorrection);
			pid.Update(distance*PN.DistanceF);
			//increase the needed velocity slowly
			var cur_vel   = Utils.ClampL((float)Vector3d.Dot(VSL.vessel.srf_velocity, vdir), 1);
			DeltaSpeed = PN.DeltaSpeed*AccelCorrection*Mathf.Pow(PN.DeltaSpeed/(cur_vel+PN.DeltaSpeed), PN.DeltaSpeedF);
			//make a correction if falling or flyin too low
			var mtwr = Utils.ClampL(VSL.MaxTWR, 0.1f);
			var V = (CFG.AltitudeAboveTerrain? VSL.RelVerticalSpeed : VSL.AbsVerticalSpeed);
			if(V < 0 && IsStateSet(TCAState.LoosingAltitude))
			{
				DeltaSpeed /= 1-Utils.ClampH(V, 0)
					/(CFG.AltitudeAboveTerrain? VSL.Altitude : 1)
					*PN.FallingSensitivity/mtwr;
				if(DeltaSpeed < 1) 
				{
					var a = Utils.ClampL(PN.FallingCorrection/mtwr/-V, PN.FallingCorrection);
					cur_vel *= (a-1+DeltaSpeed)/a;
				}
			}
			if(CFG.VF[VFlight.AltitudeControl])
			{
				var alt_error = (CFG.DesiredAltitude-VSL.Altitude);
				if(CFG.AltitudeAboveTerrain) alt_error /= VSL.Altitude;
				if(alt_error > 0) DeltaSpeed /= 1+alt_error*PN.AltErrSensitivity/mtwr;
			}
			//set needed velocity and starboard
			CFG.NeededHorVelocity = pid.Action-cur_vel > DeltaSpeed? vdir*(cur_vel+DeltaSpeed) : vdir*pid.Action;
			//correcto for Follow Target program
			if(tvsl != null && tvsl.loaded && CFG.Nav[Navigation.FollowTarget])
				if(Vector3d.Dot(tvel, vdir) > 0) CFG.NeededHorVelocity += tvel;
			CFG.Starboard = VSL.GetStarboard(CFG.NeededHorVelocity);
//			DebugUtils.CSV(VSL.vessel.vesselName, distance, cur_vel, Vector3d.Dot(cur_vel, vdir), 
//			               DeltaSpeed, VSL.MinVSFtwr, AccelCorrection, 
//			               pid.P, pid.D, pid.Action, CFG.NeededHorVelocity.magnitude);//debug
		}
	}
}

