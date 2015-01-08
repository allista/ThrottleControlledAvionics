/* Name: Throttle Controlled Avionics, Fork by Allis Tauri
 *
 * Authors: Quinten Feys & Willem van Vliet & Allis Tauri
 * License: BY: Attribution-ShareAlike 3.0 Unported (CC BY-SA 3.0): 
 * http://creativecommons.org/licenses/by-sa/3.0/
 * 
 */

using System;
using UnityEngine;
using System.IO;

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
	}

	public static class Utils
	{
		public static void writeToFile(String text)
		{
			using(var writer = new StreamWriter("DumpFile.txt", true))
				writer.Write(text + " \n");
		}

		public static string formatVector(Vector3 v)
		{ return string.Format("({0}, {1}, {2}); |v| = {3}", v.x, v.y, v.z, v.magnitude); }

		public static string formatVector(Vector3d v)
		{ return string.Format("({0}, {1}, {2}); |v| = {3}", v.x, v.y, v.z, v.magnitude); }

		public static void Log(string msg, params object[] args)
		{ 
			for(int i = 0; i < args.Length; i++) 
			{
				if(args[i] is Vector3) args[i] = formatVector((Vector3)args[i]);
				else if(args[i] is Vector3d) args[i] = formatVector((Vector3d)args[i]);
				else if(args[i] == null) args[i] = "null";
			}
			Debug.Log(string.Format("[TCA] "+msg, args)); 
		}

		public static float Asymptote01(float x, float k=1) { return 1-1/(x/k+1); }
		public static float ClampL(float x, float low)  { return x < low  ? low  : x;  }
		public static float ClampH(float x, float high) { return x < high ? high : x;  }
	}
}