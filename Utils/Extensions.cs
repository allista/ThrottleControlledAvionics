//  Author:
//       Allis Tauri <allista@gmail.com>
//
//  Copyright (c) 2015 Allis Tauri
//
// This work is licensed under the Creative Commons Attribution-ShareAlike 4.0 International License. 
// To view a copy of this license, visit http://creativecommons.org/licenses/by-sa/4.0/ 
// or send a letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.

﻿using System;
using System.Linq;
using System.Collections.Generic;
using UnityEngine;

namespace ThrottleControlledAvionics
{
	public static class Extensions
	{
		#region Logging
		public static string Title(this Part p) { return p.partInfo != null? p.partInfo.title : p.name; }

		public static void Log(this Part p, string msg, params object[] args)
		{
			var vname = p.vessel == null? "" : p.vessel.vesselName;
			var _msg = string.Format("{0}.{1} [{2}]: {3}", 
			                         vname, p.name, p.flightID, msg);
			Utils.Log(_msg, args);
		}
		#endregion

		public static void ForEach<T>(this IEnumerable<T> E, Action<T> action)
		{
			var en = E.GetEnumerator();
			while(en.MoveNext()) action(en.Current);
		}

		#region Vector3
		public static Vector3 CubeNorm(this Vector3 v)
		{
			if(v.IsZero()) return v;
			var max = -1f;
			for(int i = 0; i < 3; i++)
			{
				var ai = Mathf.Abs(v[i]);
				if(max < ai) max = ai;
			}
			return v/max;
		}

		public static Color Normalized(this Color c)
		{
			var max = c.r > c.g? (c.r > c.b? c.r : c.b) : (c.g > c.b? c.g : c.b);
			return max.Equals(0)? c : new Color(c.r / max, c.g / max, c.b / max);
		}

		public static Vector3 Inverse(this Vector3 v, float nan=float.MaxValue) 
		{ 
			return new Vector3(
			v.x.Equals(0)? nan : 1f/v.x, 
			v.y.Equals(0)? nan : 1f/v.y, 
			v.z.Equals(0)? nan : 1f/v.z); 
		}

		public static Vector3 ClampComponents(this Vector3 v, float min, float max) 
		{ 
			return new Vector3(Mathf.Clamp(v.x, min, max), 
			                   Mathf.Clamp(v.y, min, max), 
			                   Mathf.Clamp(v.z, min, max)); 
		}

		public static Vector3 Sign(this Vector3 v)
		{ return new Vector3(Mathf.Sign(v.x), Mathf.Sign(v.y), Mathf.Sign(v.z)); }

		public static Vector3 AbsComponents(this Vector3 v)
		{ return new Vector3(Mathf.Abs(v.x), Mathf.Abs(v.y), Mathf.Abs(v.z)); }

		public static int MaxI(this Vector3 v)
		{
			var maxi = 0;
			var max  = 0f;
			for(int i = 0; i < 3; i++)
			{
				if(Mathf.Abs(v[i]) > Mathf.Abs(max))
				{ max = v[i]; maxi = i; }
			}
			return maxi;
		}

		public static int MinI(this Vector3 v)
		{
			var mini = 0;
			var min   = float.MaxValue;
			for(int i = 0; i < 3; i++)
			{
				if(Mathf.Abs(v[i]) < Mathf.Abs(min))
				{ min = v[i]; mini = i; }
			}
			return mini;
		}

		public static int MedI(this Vector3 v)
		{
			return v.x < v.y? 
				(v.x > v.z? 0 : (v.z < v.y? 2 : 1)) : 
				(v.y > v.z? 1 : (v.z < v.x? 2 : 0));
		}

		public static Vector3 Component(this Vector3 v, int i)
		{
			var ret = Vector3.zero;
			ret[i] = v[i];
			return ret;
		}

		public static Vector3 Exclude(this Vector3 v, int i)
		{
			var ret = v;
			ret[i] = 0;
			return ret;
		}

		public static Vector3 MaxComponent(this Vector3 v)
		{ return v.Component(v.MaxI()); }

		public static Vector3 MinComponent(this Vector3 v)
		{ return v.Component(v.MinI()); }
		#endregion

		#region From blizzy's Toolbar
		public static Vector2 clampToScreen(this Vector2 pos) 
		{
			pos.x = Mathf.Clamp(pos.x, 0, Screen.width - 1);
			pos.y = Mathf.Clamp(pos.y, 0, Screen.height - 1);
			return pos;
		}

		public static Rect clampToScreen(this Rect rect) 
		{
			rect.width = Mathf.Clamp(rect.width, 0, Screen.width);
			rect.height = Mathf.Clamp(rect.height, 0, Screen.height);
			rect.x = Mathf.Clamp(rect.x, 0, Screen.width - rect.width);
			rect.y = Mathf.Clamp(rect.y, 0, Screen.height - rect.height);
			return rect;
		}

		public static Rect clampToWindow(this Rect rect, Rect window) 
		{
			rect.width = Mathf.Clamp(rect.width, 0, window.width);
			rect.height = Mathf.Clamp(rect.height, 0, window.height);
			rect.x = Mathf.Clamp(rect.x, 0, window.width - rect.width);
			rect.y = Mathf.Clamp(rect.y, 0, window.height - rect.height);
			return rect;
		}
		#endregion

		#region ConfigNode
		public static void AddRect(this ConfigNode n, string name, Rect r)
		{ n.AddValue(name, ConfigNode.WriteQuaternion(new Quaternion(r.x, r.y, r.width, r.height))); }

		public static Rect GetRect(this ConfigNode n, string name)
		{ 
			try 
			{ 
				var q = ConfigNode.ParseQuaternion(n.GetValue(name)); 
				return new Rect(q.x, q.y, q.z, q.w);
			}
			catch { return default(Rect); }
		}
		#endregion

		public static bool HasModule<T>(this Part p) where T : PartModule
		{ return p.Modules.GetModules<T>().Any(); }

		public static T GetModule<T>(this Part p) where T : PartModule
		{ return p.Modules.GetModules<T>().FirstOrDefault(); }

		#region From MechJeb2
		public static bool IsPhysicallySignificant(this Part p)
		{
			bool physicallySignificant = (p.physicalSignificance != Part.PhysicalSignificance.NONE);
			// part.PhysicsSignificance is not initialized in the Editor for all part. but physicallySignificant is useful there.
			if (HighLogic.LoadedSceneIsEditor)
				physicallySignificant = physicallySignificant && p.PhysicsSignificance != 1;
			//Landing gear set physicalSignificance = NONE when they enter the flight scene
			//Launch clamp mass should be ignored.
			physicallySignificant &= !p.HasModule<ModuleLandingGear>() && !p.HasModule<LaunchClamp>();
			return physicallySignificant;
		}
		#endregion

		public static float TotalMass(this Part p) { return p.mass+p.GetResourceMass(); }

		public static Bounds EnginesExhaust(this Vessel vessel)
		{
			var CoM = vessel.CurrentCoM;
			var refT = vessel.ReferenceTransform;
			var b = new Bounds();
			var inited = false;
			for(int i = 0, vesselPartsCount = vessel.Parts.Count; i < vesselPartsCount; i++)
			{
				var p = vessel.Parts[i];
				var engines = p.Modules.GetModules<ModuleEngines>();
				for(int j = 0, enginesCount = engines.Count; j < enginesCount; j++)
				{
					var e = engines[j];
					if(!e.exhaustDamage) continue;
					for(int k = 0, tCount = e.thrustTransforms.Count; k < tCount; k++)
					{
						var t = e.thrustTransforms[k];
						var term = refT.InverseTransformDirection(t.position + t.forward * e.exhaustDamageMaxRange - CoM);
						if(inited) b.Encapsulate(term);
						else { b = new Bounds(term, Vector3.zero); inited = true; }
					}
				}
			}
			return b;
		}
	}
}

