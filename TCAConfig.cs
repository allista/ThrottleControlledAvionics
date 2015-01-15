using System;
using System.IO;
using System.Linq;
using System.Collections.Generic;
using UnityEngine;

namespace ThrottleControlledAvionics
{
	public class TCAGlobals : ConfigNodeObject
	{
		new public const string NODE_NAME = "TCAGLOBALS";

		//control model configuration parameters
		[Persistent] public float K0 = 2f, K1 = 10f, L1 = 1f, K2 = 10f, L2 = 10f; //vertical speed limit control coefficients
		[Persistent] public float MaxCutoff             = 10f;  //max. positive vertical speed m/s (configuration limit)
		[Persistent] public float OptimizationPrecision = 0.1f; //optimize engines limits until torque error or delta torque error is less than this
		[Persistent] public int   MaxIterations         = 30;   //maximum number of optimizations per fixed frame
		[Persistent] public float MaxP                  = 2f;   //value of P slider
		[Persistent] public float MaxI                  = 2f;   //value of I slider
		//default values for PI controllers
		[Persistent] public PI_Dummy Engines = new PI_Dummy(0.4f, 0.15f); //thrustPercentage master PI controller defaults
		//UI window position and dimensions
		[Persistent] public Rect ControlsPos = new Rect(50, 100, TCAGui.controlsWidth, TCAGui.controlsHeight);
		[Persistent] public Rect HelpPos     = new Rect(Screen.width/2-TCAGui.helpWidth/2, 100, TCAGui.helpWidth, TCAGui.helpHeight);
		//key binding to toggle TCA
		[Persistent] public string TCA_Key = "y";

		//TODO: need key bindings or action groups for altitude control

		//help text
		public string Instructions = string.Empty;
		const string instructions = 
@"Welcome to TCA instructions manual.

For simple use:
	1) Put TCA on ('{0}'),
	2) Put SAS on ('t'),
	3) Launch!

Engines PI-controller tuning:
    * P (proportional) parameter controls the response speed of the engines. If the craft reacts slowly, increase it a little. If the craft vibrates after a manouver, decrease it.
    * I (integral) parameter controls the smoothing. It prevents rapid changes in thrust. If engines's trust jitters, increas it.

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
			var r = node.GetRect(Utils.PropertyName(new {ControlsPos}));
			if(r != default(Rect)) ControlsPos = r;
			r = node.GetRect(Utils.PropertyName(new {HelpPos}));
			if(r != default(Rect)) HelpPos = r;
		}

		public override void Save(ConfigNode node)
		{
			base.Save(node);
			node.AddRect(Utils.PropertyName(new {ControlsPos}), ControlsPos);
			node.AddRect(Utils.PropertyName(new {HelpPos}), HelpPos);
		}
	}


	public class VesselConfig : ConfigNodeObject, IComparable<VesselConfig>
	{
		new public const string NODE_NAME = "VSLCONFIG";

		[Persistent] public Guid  VesselID;
		[Persistent] public bool  Enabled;
		[Persistent] public bool  GUIVisible;
		[Persistent] public float VerticalCutoff; //max. positive vertical speed m/s (configurable)
		//steering
		[Persistent] public float   SteeringGain     = 1f;          //steering vector is scaled by this
		[Persistent] public Vector3 SteeringModifier = Vector3.one; //steering vector is scaled by this (pitch, roll, yaw); needed to prevent too fast roll on vtols and oscilations in wobbly ships
		[Persistent] public bool    PitchYawLinked   = true;        //if true, pitch and yaw sliders will be linked
		//engines
		[Persistent] public PIf_Controller Engines   = new PIf_Controller();

		public ConfigNode Configuration 
		{ get { var node = new ConfigNode(); Save(node); return node; } }

		public VesselConfig() //set defaults
		{
			VerticalCutoff = TCAConfiguration.Globals.MaxCutoff;
			Engines.setPI(TCAConfiguration.Globals.Engines);
		}
		public VesselConfig(Vessel vsl) : this() { VesselID = vsl.id; }

		public override void Load(ConfigNode node)
		{
			base.Load(node);
			var val = node.GetValue(Utils.PropertyName(new {VesselID}));
			if(!string.IsNullOrEmpty(val)) VesselID = new Guid(val);
		}

		public override void Save(ConfigNode node)
		{
			node.AddValue(Utils.PropertyName(new {VesselID}), VesselID.ToString());
			base.Save(node);
		}

		public int CompareTo(VesselConfig other)
		{ return VesselID.CompareTo(other.VesselID); }

		public static VesselConfig Clone(VesselConfig config)
		{
			var clone = new VesselConfig();
			clone.Load(config.Configuration);
			return clone;
		}

		public virtual void CopyFrom(VesselConfig other)
		{
			var vid = VesselID;
			Load(other.Configuration);
			VesselID = vid;
		}
	}


	public class NamedConfig : VesselConfig
	{ 
		[Persistent] public string Name = "Config"; 

		public NamedConfig() {}
		public NamedConfig(string name) { Name = name; }

		public override void CopyFrom(VesselConfig other)
		{
			var name = Name;
			base.CopyFrom(other);
			Name = name;
		}
	}


	public static class TCAConfiguration
	{
		public const string FILENAME   = "TCA.conf";
		public const string NODE_NAME  = "TCACONFIG";
		public const string VSL_NODE   = "VESSELS";
		public const string NAMED_NODE = "NAMED";
		public static TCAGlobals Globals = new TCAGlobals();
		public static Dictionary<Guid, VesselConfig> Configs = new Dictionary<Guid, VesselConfig>();
		public static SortedList<string, NamedConfig> NamedConfigs = new SortedList<string, NamedConfig>();

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

		public static NamedConfig GetConfig(string name)
		{ return NamedConfigs.ContainsKey(name) ? NamedConfigs[name] : null; }

		public static NamedConfig GetConfig(int index)
		{ return NamedConfigs.Count > index ? NamedConfigs.Values[index] : null; }

		public static bool SaveNamedConfig(string name, VesselConfig config, bool overwrite = false)
		{ 
			if(!overwrite && NamedConfigs.ContainsKey(name)) return false;
			var nconfig = new NamedConfig(name);
			nconfig.CopyFrom(config);
			NamedConfigs[name] = nconfig;
			return true;
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
			//deprecated conversion
			var n = node.HasNode(VesselConfig.NODE_NAME)? node : node.GetNode(VSL_NODE);
			if(n != null && n.HasNode(VesselConfig.NODE_NAME))
			{
				foreach(var c in n.GetNodes(VesselConfig.NODE_NAME))
				{
					var config = new VesselConfig();
					config.Load(c);
					Configs[config.VesselID] = config;
				}
			}
			NamedConfigs.Clear();
			if(node.HasNode(NAMED_NODE))
			{
				foreach(var c in node.GetNode(NAMED_NODE).GetNodes(NamedConfig.NODE_NAME))
				{
					var config = new NamedConfig();
					config.Load(c);
					NamedConfigs[config.Name] = config;
				}
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
			if(configs.Count > 0)
			{
				var n = node.AddNode(VSL_NODE);
				foreach(var c in configs)
				{
					if(current_vessels.Contains(c.VesselID))
						c.Save(n.AddNode(VesselConfig.NODE_NAME));
				}
			}
			if(NamedConfigs.Count > 0)
			{
				var n = node.AddNode(NAMED_NODE);
				foreach(var c in NamedConfigs.Keys)
					NamedConfigs[c].Save(n.AddNode(NamedConfig.NODE_NAME));
			}
		}
	}
}

