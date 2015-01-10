using System;
using System.IO;
using System.Linq;
using System.Collections.Generic;
using UnityEngine;

namespace ThrottleControlledAvionics
{
	public class VesselConfig : ConfigNodeObject, IComparable<VesselConfig>
	{
		new public const string NODE_NAME = "VSLCONFIG";
		const string VesselIDName = "VesselID";

		[Persistent] public Guid  VesselID;
		[Persistent] public bool  Enabled;
		[Persistent] public float VerticalCutoff; //max. positive vertical speed m/s (configurable)

		public VesselConfig() {}
		public VesselConfig(Vessel vsl) 
		{ 
			VesselID = vsl.id; 
			VerticalCutoff = TCAConfiguration.Globals.MaxCutoff;
		}

		public override void Load(ConfigNode node)
		{
			base.Load(node);
			if(node.HasValue(VesselIDName))
				VesselID = new Guid(node.GetValue(VesselIDName));
		}

		public override void Save(ConfigNode node)
		{
			node.AddValue(VesselIDName, VesselID.ToString());
			base.Save(node);
		}

		public int CompareTo(VesselConfig other)
		{ return VesselID.CompareTo(other.VesselID); }
	}

	public class TCAGlobals : ConfigNodeObject
	{
		new public const string NODE_NAME = "TCAGLOBALS";

		public static readonly float MAX_STEERING = Mathf.Sqrt(3);

		//vertical speed limit control coefficients
		[Persistent] public float  K1 = 10f, L1 = 1f, K2 = 10f, L2 = 10f;

		[Persistent] public float  MaxCutoff            = 10f;   //max. positive vertical speed m/s (configuration limit)
		[Persistent] public float  MaxSteeringThreshold = 0.5f;  //max. positive vertical speed m/s (configuration limit)
		[Persistent] public float  TorqueThreshold      = 5f;    //engines which produce less torque (kNm) are considered to be main thrusters and excluded from TCA control
		[Persistent] public float  IdleResponseSpeed    = 0.25f; //constant response speed when attitude controls are idle
		[Persistent] public float  SteeringThreshold    = 0.01f; //too small steering vector may cause oscilations
		[Persistent] public float  StabilityCurve       = 0.3f;  /* coefficient of non-linearity of efficiency response to partial steering 
												    		               (2 means response is quadratic, 1 means response is linear, 0 means no response) */
		const string ControlsPosName = "ControlsPos", HelpPosName = "HelpPos";
		[Persistent] public Rect   ControlsPos = new Rect(50, 100, TCAGui.controlsWidth, TCAGui.controlsHeight);
		[Persistent] public Rect   HelpPos     = new Rect(Screen.width/2-TCAGui.helpWidth/2, 100, TCAGui.helpWidth, TCAGui.helpHeight);
		[Persistent] public bool   GUIVisible;

		[Persistent] public string TCA_Key = "y";

		public string Instructions = string.Empty;
		const string instructions = 
@"Welcome to TCA instructions manual.

For simple use:
	1) Put TCA on ('{0}'),
	2) Put SAS on ('t'),
	3) Launch!

Vertical Speed Limit, hovering and horizontal flight:
	* If you're using TCA to control VTOL or during vertical landing of a rocket, you may enable the Vertical Speed Limiter. 
	The limit may be set with the scroll bar in the interval from -{1:F1}m/s to {1:F1}m/s (not including). When the Limit is set, the total thrust of all controllable engines is modified in an attempt to reach the specified vertical speed. The speed limit itself is never achived, however, but approached asymptotically, so you need to set it a little higher (~0.1-0.5m/s) than desired.
	To completely disable the Speed Limit, just set it to maximum value ({1}m/s).
	* Another use of the Vertical Speed Limit is a stable horizontal flight. Consider a VTOL that has lifted off, reached some altitude and started to pitch to get some forward momentum. If the thrust of its engines will remain constant, it will start to loose altitude as it moves forward. But with the automatic speed limiting the thrust will be adjusted, and the VTOL will move more or less in a plane.

Notes:
	* Thrust of jets and turbofan engines changes very slowly. This makes using them as attitude controllers impractical. Don't use them with TCA. 
	* Solid boosters have constant thrust and thus cannot be controlled by TCA.";

		public void InitInstructions()
		{ Instructions = string.Format(instructions, TCA_Key, MaxCutoff); }

		public override void Load(ConfigNode node)
		{
			base.Load(node);
			var r = node.GetRect(ControlsPosName);
			if(r != default(Rect)) ControlsPos = r;
			r = node.GetRect(HelpPosName);
			if(r != default(Rect)) HelpPos = r;
		}

		public override void Save(ConfigNode node)
		{
			base.Save(node);
			node.AddRect(ControlsPosName, ControlsPos);
			node.AddRect(HelpPosName, HelpPos);
		}
	}

	public static class TCAConfiguration
	{
		public const string FILENAME  = "TCA.conf";
		public const string NODE_NAME = "TCACONFIG";
		public static TCAGlobals Globals = new TCAGlobals();
		public static Dictionary<Guid, VesselConfig> Configs = new Dictionary<Guid, VesselConfig>();

		#region From KSPPluginFramework
		static public string FilePath
		{
			get
			{
				//Combine the Location of the assembly and the provided string. This means we can use relative or absolute paths
				return System.IO.Path.Combine(_AssemblyFolder, 
				                              "PluginData/"+_AssemblyName+"/"+FILENAME)
											.Replace("\\","/");
			}
		}
		#region Assembly Information
		/// <summary>
		/// Name of the Assembly that is running this MonoBehaviour
		/// </summary>
		internal static String _AssemblyName
		{ get { return System.Reflection.Assembly.GetExecutingAssembly().GetName().Name; } }
		/// <summary>
		/// Full Path of the executing Assembly
		/// </summary>
		internal static String _AssemblyLocation
		{ get { return System.Reflection.Assembly.GetExecutingAssembly().Location; } }
		/// <summary>
		/// Folder containing the executing Assembly
		/// </summary>
		internal static String _AssemblyFolder
		{ get { return System.IO.Path.GetDirectoryName(_AssemblyLocation); } }
		#endregion 
		#endregion

		public static VesselConfig GetConfig(Vessel vsl)
		{
			if(!Configs.ContainsKey(vsl.id)) 
				Configs.Add(vsl.id, new VesselConfig(vsl));
			return Configs[vsl.id];
		}

		public static void Load() { Load(FilePath); }

		public static void Load(string filename)
		{
			var node = ConfigNode.Load(filename);
			if(node != null) Load(node);
			else 
			{
				Utils.Log("TCAConfiguration: unable to read "+filename);
				Globals.InitInstructions();
				Configs.Clear();
			}
		}

		public static void Load(ConfigNode node) 
		{
			if(node.HasNode(TCAGlobals.NODE_NAME))
				Globals.Load(node.GetNode(TCAGlobals.NODE_NAME));
			Globals.InitInstructions();
			Configs.Clear();
			foreach(var c in node.GetNodes(VesselConfig.NODE_NAME))
			{
				var config = new VesselConfig();
				config.Load(c);
				Configs[config.VesselID] = config;
			}
		}

		public static void Save() { Save(FilePath); }

		public static void Save(string filename)
		{
			var node = new ConfigNode(NODE_NAME);
			Save(node);
			var dir = Path.GetDirectoryName(filename);
			if(!Directory.Exists(dir)) Directory.CreateDirectory(dir);
			try { node.Save(filename); }
			catch { Utils.Log("TCAConfiguration: unable to write to "+filename); }
		}

		public static void Save(ConfigNode node) 
		{
			Globals.Save(node.AddNode(TCAGlobals.NODE_NAME));
			var current_vessels = new HashSet<Guid>(HighLogic.CurrentGame.flightState.protoVessels.Select(p => p.vesselID));
			var configs = new List<VesselConfig>(Configs.Values);
			configs.Sort();
			foreach(var c in configs)
			{
				if(current_vessels.Contains(c.VesselID))
					c.Save(node.AddNode(VesselConfig.NODE_NAME));
			}
		}
	}
}

