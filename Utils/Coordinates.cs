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
	//adapted from MechJeb
	public class Coordinates : ConfigNodeObject
	{
		[Persistent] public double Lat;
		[Persistent] public double Lon;

		public Coordinates(double lat, double lon) 
		{ Lat = Utils.ClampAngle(lat); Lon = Utils.ClampAngle(lon); }

		public Coordinates(Vessel vsl) : this(vsl.latitude, vsl.longitude) {}

		public static string AngleToDMS(double angle)
		{
			var d = (int)Math.Floor(Math.Abs(angle));
			var m = (int)Math.Floor(60 * (Math.Abs(angle) - d));
			var s = (int)Math.Floor(3600 * (Math.Abs(angle) - d - m / 60.0));
			return String.Format("{0:0}°{1:00}'{2:00}\"", Math.Sign(angle)*d, m, s);
		}

		public double Alt(CelestialBody body) { return body.TerrainAltitude(Lat, Lon); }
		public string Biome(CelestialBody body) { return ScienceUtil.GetExperimentBiome(body, Lat, Lon); }

		static Coordinates Search(CelestialBody body, Ray mouseRay)
		{
			Vector3d relSurfacePosition;
			Vector3d relOrigin = mouseRay.origin - body.position;
			double curRadius = body.pqsController.radiusMax;
			double lastRadius = 0;
			double error = 0;
			int loops = 0;
			float st = Time.time;
			while(loops < 50)
			{
				if(PQS.LineSphereIntersection(relOrigin, mouseRay.direction, curRadius, out relSurfacePosition))
				{
					var alt = body.pqsController.GetSurfaceHeight(relSurfacePosition);
					if(body.ocean && alt < body.Radius) alt = body.Radius;
					error = Math.Abs(curRadius - alt);
					if(error < (body.pqsController.radiusMax - body.pqsController.radiusMin) / 100)
					{
						var surfacePoint = body.position + relSurfacePosition;
						return new Coordinates(body.GetLatitude(surfacePoint), body.GetLongitude(surfacePoint));
					}
					else
					{
						lastRadius = curRadius;
						curRadius = alt;
						loops++;
					}
				}
				else
				{
					if(loops == 0) break;
					else
					{ // Went too low, needs to try higher
						curRadius = (lastRadius * 9 + curRadius) / 10;
						loops++;
					}
				}
			}
			return null;
		}

		public static Coordinates GetAtPointer(CelestialBody body)
		{
			var mouseRay = PlanetariumCamera.Camera.ScreenPointToRay(Input.mousePosition);
			mouseRay.origin = ScaledSpace.ScaledToLocalSpace(mouseRay.origin);
			return Search(body, mouseRay);
		}

		public static Coordinates GetAtPointerInFlight()
		{
			var body = FlightGlobals.currentMainBody;
			var mouseRay = FlightCamera.fetch.mainCamera.ScreenPointToRay(Input.mousePosition);
			RaycastHit raycast;
			return Physics.Raycast(mouseRay, out raycast, (float)body.Radius * 4f, 1 << 15)? 
				new Coordinates(body.GetLatitude(raycast.point), Utils.CenterAngle(body.GetLongitude(raycast.point))) : 
				Search(body, mouseRay);
		}

		public override string ToString()
		{ return string.Format("Lat: {0} Lon: {1}", AngleToDMS(Lat), AngleToDMS(Lon)); }

		public string FullDescription(CelestialBody body)
		{ return string.Format("{0}\nAlt: {1:F0}m {2}", this, Alt(body), Biome(body)); }

		public string FullDescription(Vessel vsl) { return FullDescription(vsl.mainBody);}
	}
}
