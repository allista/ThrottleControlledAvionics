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
using UnityEngine;

namespace ThrottleControlledAvionics
{
	public class PointNavigator : TCAModule
	{
		public class Config : ModuleConfig
		{
			new public const string NODE_NAME = "PN";

			[Persistent] public float Delay = 1;
			[Persistent] public float MinDistance = 10;
			[Persistent] public float MinSpeed = 10;
			[Persistent] public float MaxSpeed = 1000;
			[Persistent] public PID_Controller DistancePID = new PID_Controller(0.5f, 0f, 0.5f, 0, 100);
		}
		static Config PN { get { return TCAConfiguration.Globals.PN; } }
		public PointNavigator(VesselWrapper vsl) { VSL = vsl; }

		readonly PIDf_Controller pid = new PIDf_Controller();
		ITargetable target;

		public override void Init()
		{
			base.Init();
			pid.setPID(PN.DistancePID);
			pid.Reset();
			if(CFG.GoToTarget) GoToTarget();
			else if(CFG.FollowPath) FollowPath();
		}

		public override void UpdateState() { IsActive = (CFG.GoToTarget || CFG.FollowPath) && VSL.OnPlanet; }

		public void GoToTarget(bool enable = true)
		{
			if(enable == CFG.GoToTarget) return;
			if(enable && VSL.vessel.targetObject == null) return;
			CFG.GoToTarget = enable;
			if(CFG.GoToTarget) start_to(VSL.vessel.targetObject);
			else finish();
		}

		public void FollowPath(bool enable = true)
		{
			if(enable == CFG.FollowPath) return;
			if(enable && CFG.Waypoints.Count == 0) return;
			CFG.FollowPath = enable;
			if(CFG.FollowPath) start_to(CFG.Waypoints.Peek());
			else finish();
		}

		void start_to(ITargetable t)
		{
			target = t;
			if(target == null) return;
			FlightGlobals.fetch.SetVesselTarget(t);
			BlockSAS();
			pid.Reset();
			VSL.UpdateHorizontalStats();
			CFG.CruiseControl = true;
			CFG.KillHorVel = false;
		}

		void finish()
		{
			target = null;
			CFG.Starboard = Vector3.zero;
			CFG.NeededHorVelocity = Vector3d.zero;
			CFG.CruiseControl = false;
			CFG.KillHorVel = true;
			CFG.GoToTarget = false;
			CFG.FollowPath = false;
		}

		public void Update()
		{
			if(target == null) return;
			var mt = target as MapTarget;
			if(mt != null) mt.Update(VSL.vessel.mainBody);
			var dr = Vector3.ProjectOnPlane(target.GetTransform().position-VSL.vessel.transform.position, VSL.Up);
			var distance = dr.magnitude;
			if(distance < PN.MinDistance) 
			{
				if(CFG.FollowPath)
				{
					while(CFG.Waypoints.Count > 0 && CFG.Waypoints.Peek() == target) CFG.Waypoints.Dequeue();
					if(CFG.Waypoints.Count > 0) { start_to(CFG.Waypoints.Peek()); return; }
				}
			   	finish(); return;
			}
			pid.Min = 0;
			pid.Max = CFG.MaxNavSpeed;
			pid.Update(distance/PN.MinDistance);
			CFG.NeededHorVelocity = dr/distance*pid.Action;
			CFG.Starboard = VSL.GetStarboard(CFG.NeededHorVelocity);
		}
	}

	//adapted from MechJeb
	public class MapTarget : ConfigNodeObject, ITargetable
	{
		new public const string NODE_NAME = "WAYPOINT";

		[Persistent] public string Name;
		[Persistent] public double Lat;
		[Persistent] public double Lon;

		GameObject go = new GameObject();

		public MapTarget() {}
		public MapTarget(Coordinates c) 
		{ Lat = c.Lat; Lon = c.Lon; Name = c.ToString(); }

		static public MapTarget FromConfig(ConfigNode node)
		{
			var wp = new MapTarget();
			wp.Load(node);
			return wp;
		}

		//Call this every frame to make sure the target transform stays up to date
		public void Update(CelestialBody body) 
		{ go.transform.position = body.GetWorldSurfacePosition(Lat, Lon, Utils.TerrainAltitude(body, Lat, Lon)); }

		public Vector3 GetFwdVector() { return Vector3.up; }
		public string GetName() { return Name; }
		public Vector3 GetObtVelocity() { return Vector3.zero; }
		public Orbit GetOrbit() { return null; }
		public OrbitDriver GetOrbitDriver() { return null; }
		public Vector3 GetSrfVelocity() { return Vector3.zero; }
		public Transform GetTransform() { return go.transform; }
		public Vessel GetVessel() { return null; }
		public VesselTargetModes GetTargetingMode() { return VesselTargetModes.Direction; }
	}
}

