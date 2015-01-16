using System;
using System.Linq;
using UnityEngine;

namespace ThrottleControlledAvionics
{
	public static class Extensions
	{
		#region Resources
		const string ElectricChargeName = "ElectricCharge";
		static PartResourceDefinition _electric_charge;

		public static PartResourceDefinition ElectricCharge
		{ 
			get
			{ 
				if(_electric_charge == null)
					_electric_charge = PartResourceLibrary.Instance.GetDefinition(ElectricChargeName);
				return _electric_charge;
			} 
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

		public static bool ElectricChargeAvailible(this Vessel v)
		{
			var ec = v.GetActiveResource(ElectricCharge);
			return ec != null && ec.amount > 0;
		}

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

		#region From MechJeb2
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
			physicallySignificant &= !p.HasModule<ModuleLandingGear>() && !p.HasModule<LaunchClamp>();
			return physicallySignificant;
		}

		public static float TotalMass(this Part p) { return p.mass+p.GetResourceMass(); }
		#endregion
	}
}

