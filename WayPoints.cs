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
	//adapted from MechJeb
	public class WayPoint : ConfigNodeObject, ITargetable
	{
		new public const string NODE_NAME = "WAYPOINT";

		[Persistent] public string Name;
		[Persistent] public double Lat;
		[Persistent] public double Lon;
		//target proxy
		[Persistent] ProtoTargetInfo TargetInfo = new ProtoTargetInfo();
		ITargetable target;
		//a transform holder for simple lat-lon coordinates on the map
		GameObject go;

		public WayPoint() {}
		public WayPoint(Coordinates c) { Lat = c.Lat; Lon = c.Lon; Name = c.ToString(); go = new GameObject(); }
		public WayPoint(ITargetable t) { target = t; TargetInfo = new ProtoTargetInfo(t); }

		static public WayPoint FromConfig(ConfigNode node)
		{
			var wp = new WayPoint();
			wp.Load(node);
			return wp;
		}

		//using Spherical Law of Cosines (for other methods see http://www.movable-type.co.uk/scripts/latlong.html)
		public double AngleTo(Vessel vsl)
		{
			var fi1 = Lat*Mathf.Deg2Rad;
			var fi2 = vsl.latitude*Mathf.Deg2Rad;
			var dlambda = (vsl.longitude-Lon)*Mathf.Deg2Rad;
			return Math.Acos(Math.Sin(fi1)*Math.Sin(fi2)+Math.Cos(fi1)*Math.Cos(fi2)*Math.Cos(dlambda));
		}
		public double DistanceTo(Vessel vsl) { return AngleTo(vsl)*vsl.mainBody.Radius; }

		//Call this every frame to make sure the target transform stays up to date
		public void Update(CelestialBody body) 
		{ 
			if(target != null) UpdateCoordinates(body);
			if(TargetInfo.targetType != ProtoTargetInfo.Type.Null && 
			   HighLogic.LoadedSceneIsFlight)
			{
				target = TargetInfo.FindTarget();
				if(target == null) TargetInfo.targetType = ProtoTargetInfo.Type.Null;
				else UpdateCoordinates(body);
			}
			else
			{
				if(go == null) go = new GameObject();
				go.transform.position = body.GetWorldSurfacePosition(Lat, Lon, Utils.TerrainAltitude(body, Lat, Lon)); 
			}
		}

		public void UpdateCoordinates(CelestialBody body)
		{
			if(target == null) return;
			Name = target.GetName();
			switch(TargetInfo.targetType)
			{
			case ProtoTargetInfo.Type.Vessel:
				var v = target as Vessel;
				Lat   = v.latitude;
				Lon   = v.longitude;
				break;
			case ProtoTargetInfo.Type.PartModule:
				var m = target as PartModule;
				Lat   = m.vessel.latitude;
				Lon   = m.vessel.longitude;
				break;
			case ProtoTargetInfo.Type.Part:
				var p = target as Part;
				Lat   = p.vessel.latitude;
				Lon   = p.vessel.longitude;
				break;
			case ProtoTargetInfo.Type.Generic:
				var g = target as MonoBehaviour;
				Lat = body.GetLatitude(g.transform.position);
				Lat = body.GetLongitude(g.transform.position);
				break;
//			case ProtoTargetInfo.Type.CelestialBody:
			default:
				return;
			}
		}

		public string GetName() { return Name; }
		public ITargetable GetTarget() { return target ?? this; }
		public Vector3 GetFwdVector() { return target == null? Vector3.up : target.GetFwdVector(); }
		public Vector3 GetObtVelocity() { return target == null? Vector3.zero : target.GetObtVelocity(); }
		public Orbit GetOrbit() { return target == null? null : target.GetOrbit(); }
		public OrbitDriver GetOrbitDriver() { return target == null? null : target.GetOrbitDriver(); }
		public Vector3 GetSrfVelocity() { return target == null? Vector3.zero : target.GetSrfVelocity(); }
		public Vessel GetVessel() { return target == null? null : target.GetVessel(); }
		public VesselTargetModes GetTargetingMode() { return target == null? VesselTargetModes.Direction : target.GetTargetingMode(); }

		public Transform GetTransform() 
		{ 
			if(target == null)
			{
				if(go == null) go = new GameObject();
				return go.transform; 
			}
			return target.GetTransform(); 
		}
	}
}

