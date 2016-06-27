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
using ModuleWheels;

namespace ThrottleControlledAvionics
{
	public static class MiscExtensions
	{
		public static Color Normalized(this Color c)
		{
			var max = c.r > c.g? (c.r > c.b? c.r : c.b) : (c.g > c.b? c.g : c.b);
			return max.Equals(0)? c : new Color(c.r / max, c.g / max, c.b / max);
		}

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
	}


	public static class VectorExtensions
	{
		public static bool IsNaN(this Vector3d v)
		{ return double.IsNaN(v.x) || double.IsNaN(v.y) || double.IsNaN(v.z); }

		public static bool IsNaN(this Vector3 v)
		{ return float.IsNaN(v.x) || float.IsNaN(v.y) || float.IsNaN(v.z); }

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

		public static Vector3 ClampMagnitudeH(this Vector3 v, float max)
		{ 
			var vm = v.magnitude;
			return vm > max? v/vm*max : v;

		}

		public static Vector3d ClampMagnitudeH(this Vector3d v, float max)
		{ 
			var vm = v.magnitude;
			return vm > max? v/vm*max : v;
		}

		public static Vector3d ClampMagnitudeL(this Vector3d v, float min)
		{ 
			var vm = v.magnitude;
			return vm < min? v/vm*min : v;
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
	}


	public static class CollectionsExtensions
	{
		public static TSource SelectMax<TSource>(this IEnumerable<TSource> s, Func<TSource, float> metric)
		{
			float max_v = -1;
			TSource max_e = default(TSource);
			foreach(TSource e in s)
			{
				float m = metric(e);
				if(m > max_v) { max_v = m; max_e = e; }
			}
			return max_e;
		}

		public static void ForEach<TSource>(this IEnumerable<TSource> E, Action<TSource> action)
		{
			var en = E.GetEnumerator();
			while(en.MoveNext()) action(en.Current);
		}

		public static void ForEach<TSource>(this TSource[] a, Action<TSource> action)
		{ for(int i = 0; i < a.Length; i++) action(a[i]); }

		public static TSource Pop<TSource>(this LinkedList<TSource> l)
		{
			TSource e = l.Last.Value;
			l.RemoveLast();
			return e;
		}

		public static TSource Min<TSource>(params TSource[] args) where TSource : IComparable
		{
			if(args.Length == 0) throw new InvalidOperationException("Min: arguments list should not be empty");
			TSource min = args[0];
			foreach(var arg in args)
			{ if(min.CompareTo(arg) < 0) min = arg; }
			return min;
		}

		public static TSource Max<TSource>(params TSource[] args) where TSource : IComparable
		{
			if(args.Length == 0) throw new InvalidOperationException("Max: arguments list should not be empty");
			TSource max = args[0];
			foreach(var arg in args)
			{ if(max.CompareTo(arg) > 0) max = arg; }
			return max;
		}

		public static TValue Next<TKey, TValue>(this SortedList<TKey,TValue> list, TKey key)
		{
			try
			{
				var i = list.IndexOfKey(key);
				var ni = (i+1) % list.Count;
				return list.Values[ni];
			} catch { return default(TValue); }
		}

		public static TValue Prev<TKey, TValue>(this SortedList<TKey,TValue> list, TKey key)
		{
			try
			{
				var i = list.IndexOfKey(key);
				var ni = i > 0? i-1 : list.Count-1;
				return list.Values[ni];
			} catch { return default(TValue); }
		}
	}


	public static class PartExtensions
	{
		#region from MechJeb2 PartExtensions
		public static bool HasModule<T>(this Part p) where T : PartModule
		{ return p.Modules.OfType<T>().Any(); }

		public static T GetModule<T>(this Part p) where T : PartModule
		{ return p.Modules.OfType<T>().FirstOrDefault(); }

		public static bool IsPhysicallySignificant(this Part p)
		{
			bool physicallySignificant = (p.physicalSignificance != Part.PhysicalSignificance.NONE);
			// part.PhysicsSignificance is not initialized in the Editor for all part. but physicallySignificant is useful there.
			if (HighLogic.LoadedSceneIsEditor)
				physicallySignificant = physicallySignificant && p.PhysicsSignificance != 1;
			//Landing gear set physicalSignificance = NONE when they enter the flight scene
			//Launch clamp mass should be ignored.
			physicallySignificant &= !p.HasModule<ModuleWheelBase>() && !p.HasModule<LaunchClamp>();
			return physicallySignificant;
		}

		public static float TotalMass(this Part p) { return p.mass+p.GetResourceMass(); }
		#endregion

		#region Find Modules or Parts
		public static Part RootPart(this Part p) 
		{ return p.parent == null ? p : p.parent.RootPart(); }

		public static List<Part> AllChildren(this Part p)
		{
			var all_children = new List<Part>{};
			foreach(Part ch in p.children) 
			{
				all_children.Add(ch);
				all_children.AddRange(ch.AllChildren());
			}
			return all_children;
		}

		public static List<Part> AllConnectedParts(this Part p)
		{
			if(p.parent != null) return p.parent.AllConnectedParts();
			var all_parts = new List<Part>{p};
			all_parts.AddRange(p.AllChildren());
			return all_parts;
		}

		public static Part AttachedPartWithModule<T>(this Part p) where T : PartModule
		{
			if(p.parent != null && p.parent.HasModule<T>()) return p.parent;
			foreach(var c in p.children) if(c.HasModule<T>()) return c;
			return null;
		}

		public static List<ModuleT> AllModulesOfType<ModuleT>(this Part part, ModuleT exception = null)
			where ModuleT : PartModule
		{
			var passages = new List<ModuleT>();
			foreach(Part p in part.AllConnectedParts())
				passages.AddRange(from m in p.Modules.OfType<ModuleT>()
				                  where exception == null || m != exception
				                  select m);
			return passages;
		}
		#endregion

		#region Resources and Phys-Props
		public static float TotalCost(this Part p) { return p.partInfo != null? p.partInfo.cost : 0; }

		public static float ResourcesCost(this Part p) 
		{ 
			return (float)p.Resources.Cast<PartResource>()
				.Aggregate(0.0, (a, b) => a + b.amount * b.info.unitCost); 
		}

		public static float MaxResourcesCost(this Part p) 
		{ 
			return (float)p.Resources.Cast<PartResource>()
				.Aggregate(0.0, (a, b) => a + b.maxAmount * b.info.unitCost); 
		}

		public static float DryCost(this Part p) { return p.TotalCost() - p.MaxResourcesCost(); }

		public static float MassWithChildren(this Part p)
		{
			float mass = p.TotalMass();
			p.children.ForEach(ch => mass += ch.MassWithChildren());
			return mass;
		}
		#endregion

		#region Actions
		public static void BreakConnectedStruts(this Part p)
		{
			//break strut connectors
			foreach(Part part in p.AllConnectedParts())
			{
				var s = part as StrutConnector;
				if(s == null || s.target == null) continue;
				if(s.parent == p || s.target == p)
				{
					s.BreakJoint();
					s.targetAnchor.gameObject.SetActive(false);
					s.direction = Vector3.zero;
				}
			}
		}

		public static void UpdateAttachedPartPos(this Part p, AttachNode node)
		{
			if(node == null) return;
			var ap = node.attachedPart; 
			if(ap == null) return;
			var an = ap.findAttachNodeByPart(p);	
			if(an == null) return;
			var dp =
				p.transform.TransformPoint(node.position) -
				ap.transform.TransformPoint(an.position);
			if(ap == p.parent) 
			{
				while (ap.parent) ap = ap.parent;
				ap.transform.position += dp;
				p.transform.position -= dp;
			} 
			else ap.transform.position += dp;
		}
		#endregion

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

		#region Misc
		//directly from Part disassembly
		public static PartModule.StartState StartState(this Part part)
		{
			var _state = PartModule.StartState.None;
			if(HighLogic.LoadedSceneIsEditor)
				_state |= PartModule.StartState.Editor;
			else if(HighLogic.LoadedSceneIsFlight)
			{
				if(part.vessel.situation == Vessel.Situations.PRELAUNCH)
				{
					_state |= PartModule.StartState.PreLaunch;
					_state |= PartModule.StartState.Landed;
				}
				if(part.vessel.situation == Vessel.Situations.DOCKED)
					_state |= PartModule.StartState.Docked;
				if(part.vessel.situation == Vessel.Situations.ORBITING ||
				   part.vessel.situation == Vessel.Situations.ESCAPING)
					_state |= PartModule.StartState.Orbital;
				if(part.vessel.situation == Vessel.Situations.SUB_ORBITAL)
					_state |= PartModule.StartState.SubOrbital;
				if(part.vessel.situation == Vessel.Situations.SPLASHED)
					_state |= PartModule.StartState.Splashed;
				if(part.vessel.situation == Vessel.Situations.FLYING)
					_state |= PartModule.StartState.Flying;
				if(part.vessel.situation == Vessel.Situations.LANDED)
					_state |= PartModule.StartState.Landed;
			}
			return _state;
		}
		#endregion
	}


	public static class PartModuleExtensions
	{
		public static string Title(this PartModule pm) 
		{ return pm.part.partInfo != null? pm.part.partInfo.title : pm.part.name; }

		public static void EnableModule(this PartModule pm, bool enable)
		{ pm.enabled = pm.isEnabled = enable; }
	}


	public static class VesselExtensions
	{
		public static Part GetPart<T>(this Vessel v) where T : PartModule
		{ return v.parts.FirstOrDefault(p => p.HasModule<T>()); }

		public static bool PartsStarted(this Vessel v)
		{ return v.parts.TrueForAll(p => p.started); }

		public static Bounds Bounds(this Vessel vessel, Transform refT)
		{
			//update physical bounds
			var b = new Bounds();
			bool inited = false;
			var parts = vessel.parts;
			for(int i = 0, partsCount = parts.Count; i < partsCount; i++)
			{
				Part p = parts[i];
				if(p == null) continue;
				var meshes = p.FindModelComponents<MeshFilter>();
				for(int mi = 0, meshesLength = meshes.Length; mi < meshesLength; mi++)
				{
					//skip meshes without renderer
					var m = meshes[mi];
					if(m.renderer == null || !m.renderer.enabled) continue;
					var bounds = Utils.BoundCorners(m.sharedMesh.bounds);
					for(int j = 0; j < 8; j++)
					{
						var c = refT.InverseTransformPoint(m.transform.TransformPoint(bounds[j]));
						if(inited) b.Encapsulate(c);
						else { b = new Bounds(c, Vector3.zero); inited = true; }
					}
				}
			}
			return b;
		}

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

		public static float Radius(this Vessel vessel)
		{ 
			if(!vessel.loaded) return (float)Math.Pow(vessel.totalMass, 1/3.0);
			var tca = ModuleTCA.EnabledTCA(vessel);
			return tca != null? tca.VSL.Geometry.R : 
				vessel.Bounds(vessel.ReferenceTransform).size.magnitude;
		}
	}
}

