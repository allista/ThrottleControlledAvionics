using System;
using System.IO;
using System.Linq;
using System.Reflection;
using System.Collections.Generic;
using UnityEngine;

namespace ThrottleControlledAvionics
{
	public class TCAGlobals : ConfigNodeObject
	{
		new public const string NODE_NAME = "TCAGLOBALS";

		//vertical speed control parameters
		[Persistent] public float K0 = 2f, K1 = 10f, L1 = 1f, K2 = 10f, L2 = 10f; //vertical speed limit control coefficients
		[Persistent] public float MaxCutoff                = 10f;   //max. positive vertical speed m/s (configuration limit)
		[Persistent] public float MinVSF                   = 0.05f; //minimum vertical speed factor; so as not to lose control during a rapid descent
		[Persistent] public float VSFCorrectionMultiplier  = 1.2f;  //multiplier for the vertical speed factor correction; 1.2 means +20% of thrust above the minimal value sufficient for zero balance
		[Persistent] public float CutoffAdjustFactor       = 2.1f;  //factor for the VerticalCutoff adjustment
		//engine balancing parameters
		[Persistent] public int   MaxIterations            = 50;    //maximum number of optimizations per fixed frame
		[Persistent] public float OptimizationPrecision    = 0.1f;  //optimize engines limits until torque error or delta torque error is less than this
		[Persistent] public float OptimizationAngleCutoff  = 45f;   //maximum angle between torque imbalance and torque demand that is considered optimized
		[Persistent] public float OptimizationTorqueCutoff = 1f;    //maximum torque delta between imbalance and demand that is considered optimized
		//default values for PI controllers
		[Persistent] public float MaxP = 1f; //value of P slider
		[Persistent] public float MaxI = 1f; //value of I slider
		[Persistent] public PI_Dummy Engines = new PI_Dummy(0.4f, 0.2f); //thrustPercentage master PI controller defaults
		[Persistent] public PI_Dummy AngularA = new PI_Dummy(0.4f, 0.2f); //values for angular acceleration dumper
		[Persistent] public FloatCurve EnginesCurve = new FloatCurve();  //float curve for P value of Engines PI controller = F(torque/MoI)
		//steering gain curve
		[Persistent] public FloatCurve SteeringCurve = new FloatCurve(); // float curve for Pitch,Yaw,Roll steering modifiers = F(torque/MoI)
		//horizontal velocity control
		[Persistent] public float HvP = 0.9f, HvI_Factor = 20f;
		[Persistent] public float MinHvD = 0.02f, MaxHvD = 0.07f;
		[Persistent] public float MinTf = 0.1f, MaxTf = 1f;
		[Persistent] public float TWRf = 5;
		[Persistent] public float upF  = 3;
		public float TfSpan;
		[Persistent] public float InertiaFactor = 10f, AngularMomentumFactor = 0.002f;
		[Persistent] public float AccelerationFactor = 1f, MinHvThreshold = 10f;
		[Persistent] public float MoIFactor = 0.01f;
		//other
		[Persistent] public bool IntegrateIntoCareer  = true;
		[Persistent] public bool RoleSymmetryInFlight = true;
		[Persistent] public bool UseStockAppLauncher  = false;
		//help text
		public string Instructions = string.Empty;
		const string instructions = 
@"Welcome to TCA instructions manual.

For simple use:
    1) Turn TCA on ('{0}')
    2) Turn SAS on ('t')
    3) Activate the engines and throttle them up!

Engine Modes:
    * In editor or in flight (through the part menu) you may set any engine to work in one of the three modes: 
        1) Main Engine (default). TCA tries to maximize the thrust of these engines. In a perfectly balanced ship all Main Engines should have 100% thrust in the absence of control input. These engines are also used to control vertical speed.
        2) Maneuver Engine. TCA tries to minimize the thrust of these engines. In a perfectly balanced ship these engines produce thrust only in response to control input
        3) Manual Control. TCA does not change the thrust of these engines, but includes them in calculations.

Autotuning Parameters:
    * If this option is enabled, TCA calculates Steering Gains and PI coefficients as functions of maximum possible angular acceleration along each of the principal axes of a craft. Note, that calculations are based on the predefined response curves tuned for the stock SAS.
    * If you have already tuned these parameters for the ship, save its configuration before enabling, as autotuning will overwrite previous parameters.

Steering Gains:
    * Master Gain modifies the magnitude of all steering input after other modifications. If the craft reacts slowly, increase it a little. If the craft vibrates after a maneuver, decrease it.
    * Pitch, Yaw and Roll modify only corresponding inputs.
    * Linking Pitch and Yaw useful if the ship's engines are symmetrical along main axis. In that case Pitch&Yaw slider may be used in stead of Master Gain.

Engines PI-controller tuning:
    * P (proportional) parameter controls the response speed of the engines. If the craft reacts slowly, increase it a little. If the craft vibrates after a maneuver, decrease it.
    * I (integral) parameter controls the smoothing. It prevents rapid changes in thrust. If engines's trust jitters, increase it.

Vertical Speed Control, hovering and horizontal flight:
    * If you're using TCA to control VTOL or during vertical landing of a rocket, you may enable the Vertical Speed Control System.
    * The desired vertical speed may be set with the scroll bar in the (configurable) interval from -{1:F1}m/s to {1:F1}m/s (not including). Then the total thrust of all controllable engines is modified in an attempt to reach the specified vertical speed. This speed, however, is never achieved, but approached asymptotically, so you need to set it a little higher (0.1-0.5m/s) than desired.
    * To completely disable the VSC, just set it the desired speed to its maximum value ({1}m/s).
    * VSC is also very useful to maintain stable horizontal flight. Consider a VTOL that has lifted off, reached some altitude and started to pitch to get some forward momentum. If the thrust of its engines will remain constant, it will start to loose altitude as it moves forward. But with the VSC the thrust will be adjusted, and the VTOL will move more or less in a plane.

Set vertical speed with throttle controls:
    * When this option is enabled, the throttle is locked at 100% and throttle controls are used to set desired vertical speed instead. If VSC system was switched off it is automatically enabled and the desired speed is set to 0.

Kill Horizontal Velocity:
    * Enables an autopilot that tries to maneuver the craft so that the horizontal component of its velocity became zero. It includes flip-over prevention system, so whatever the speed, the autopilot decrease it carefully and steady.

Notes:
    * For safety reasons the Vertical and Horizontal speed controls are disabled in orbit, but not on suborbital trajectories, so be carefull.
    * If your ship wobbles and oscillates with TCA and SAS enabled, rebuild it with more struts, or decrease appropriate Steering Gains.
    * Thrust of jets and turbofan engines changes very slowly. Thus using them as attitude controllers is impractical. If you want to use them, switch them to Manual Control mode.
    * Solid boosters have constant thrust and thus cannot be controlled by TCA. But they are still accounted for, if present.";

		public void Init()
		{ 
			Instructions = string.Format(instructions, TCAGui.TCA_Key, MaxCutoff); 
			TfSpan = MaxTf-MinTf;
		}

		public override void Load(ConfigNode node)
		{
			base.Load(node);
			//load curves
			var n = node.GetNode(Utils.PropertyName(new {SteeringCurve}));
			if(n != null) SteeringCurve.Load(n);
			n = node.GetNode(Utils.PropertyName(new {EnginesCurve}));
			if(n != null) EnginesCurve.Load(n);
		}

		//Globals are readonly
		public override void Save(ConfigNode node) 
		{
			#if DEBUG
			base.Save(node);
			SteeringCurve.Save(node.AddNode(Utils.PropertyName(new {SteeringCurve})));
			EnginesCurve.Save(node.AddNode(Utils.PropertyName(new {EnginesCurve})));
			#endif
		}
	}


	public class VesselConfig : ConfigNodeObject, IComparable<VesselConfig>
	{
		new public const string NODE_NAME = "VSLCONFIG";

		[Persistent] public Guid  VesselID;
		[Persistent] public bool  Enabled;
		[Persistent] public bool  GUIVisible;
		[Persistent] public float VerticalCutoff; //max. positive vertical speed m/s (configurable)
		[Persistent] public bool  BlockThrottle;
		[Persistent] public float VSControlSensitivity = 0.01f;
		//steering
		[Persistent] public float   SteeringGain     = 1f;          //steering vector is scaled by this
		[Persistent] public Vector3 SteeringModifier = Vector3.one; //steering vector is scaled by this (pitch, roll, yaw); needed to prevent too fast roll on vtols and oscilations in wobbly ships
		[Persistent] public bool    PitchYawLinked   = true;        //if true, pitch and yaw sliders will be linked
		[Persistent] public bool    AutoTune         = true;        //if true, engine PI coefficients and steering modifier will be tuned automatically
		//horizontal velocity
		[Persistent] public bool    KillHorVel;
		[Persistent] public bool    SASWasEnabled;
		//engines
		[Persistent] public PIf_Controller Engines = new PIf_Controller();

		public ConfigNode Configuration 
		{ get { var node = new ConfigNode(); Save(node); return node; } }

		public bool VerticalSpeedControl { get { return VerticalCutoff < TCAConfiguration.Globals.MaxCutoff; } }

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
		public const string CONFIGNAME  = "TCA.conf";
		public const string GLOBALSNAME = "TCA.glob";
		public const string NODE_NAME   = "TCACONFIG";
		public const string VSL_NODE    = "VESSELS";
		public const string NAMED_NODE  = "NAMED";
		public static TCAGlobals Globals = new TCAGlobals();
		public static Dictionary<Guid, VesselConfig> Configs = new Dictionary<Guid, VesselConfig>();
		public static SortedList<string, NamedConfig> NamedConfigs = new SortedList<string, NamedConfig>();
		static readonly List<ConfigNode> other_games = new List<ConfigNode>();

		#region From KSPPluginFramework
		//Combine the Location of the assembly and the provided string.
		//This means we can use relative or absolute paths.
		static public string FilePath(string filename)
		{ return Path.Combine(_AssemblyFolder, "PluginData/"+_AssemblyName+"/"+filename).Replace("\\","/"); }
		/// <summary>
		/// Name of the Assembly that is running this MonoBehaviour
		/// </summary>
		internal static String _AssemblyName
		{ get { return Assembly.GetExecutingAssembly().GetName().Name; } }
		/// <summary>
		/// Full Path of the executing Assembly
		/// </summary>
		internal static String _AssemblyLocation
		{ get { return Assembly.GetExecutingAssembly().Location; } }
		/// <summary>
		/// Folder containing the executing Assembly
		/// </summary>
		internal static String _AssemblyFolder
		{ get { return Path.GetDirectoryName(_AssemblyLocation); } }
		#endregion

		static public string CurrentGame
		{ get { return HighLogic.CurrentGame.Title.Split()[0]; } }

		#region Runtime Interface
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
			if(name == string.Empty || //do not allow empty name
			   NamedConfigs.ContainsKey(name) && !overwrite) return false;
			var nconfig = new NamedConfig(name);
			nconfig.CopyFrom(config);
			NamedConfigs[name] = nconfig;
			return true;
		}
		#endregion

		#region Save/Load
		static ConfigNode loadNode(string filepath)
		{
			var node = ConfigNode.Load(filepath);
			if(node == null)
				Utils.Log("TCAConfiguration: unable to read "+filepath);
			return node;
		}

		public static void Load() { Load(FilePath(CONFIGNAME), FilePath(GLOBALSNAME)); }

		public static void Load(string configs, string globals)
		{
			var gnode = loadNode(globals);
			if(gnode != null) LoadGlobals(gnode);
			else Globals.Init();

			var cnode = loadNode(configs);
			if(cnode != null) LoadConfigs(cnode);
			else Configs.Clear();
		}

		public static void LoadGlobals()
		{
			var gnode = loadNode(FilePath(GLOBALSNAME));
			if(gnode != null) LoadGlobals(gnode);
			else Globals.Init();
		}

		public static void LoadGlobals(ConfigNode node) 
		{
			Globals.Load(node);
			Globals.Init();
		}

		public static void LoadConfigs(ConfigNode node) 
		{
			Configs.Clear();
			NamedConfigs.Clear();
			other_games.Clear();
			var game = CurrentGame;
			foreach(var n in node.GetNodes())
			{
				if(n.name == game ||
				   n.name == VSL_NODE) //deprecated conversion
				{
					foreach(var c in n.GetNodes(VesselConfig.NODE_NAME))
					{
						var config = new VesselConfig();
						config.Load(c);
						Configs[config.VesselID] = config;
					}
				}
				else if(n.name == NAMED_NODE)
				{
					foreach(var c in n.GetNodes(NamedConfig.NODE_NAME))
					{
						var config = new NamedConfig();
						config.Load(c);
						NamedConfigs[config.Name] = config;
					}
				}
				else other_games.Add(n);
			}
		}

		static void saveNode(ConfigNode node, string filepath)
		{
			var dir = Path.GetDirectoryName(filepath);
			if(!Directory.Exists(dir)) Directory.CreateDirectory(dir);
			try { node.Save(filepath); }
			catch { Utils.Log("TCAConfiguration: unable to write to "+filepath); }
		}

		public static void Save() { Save(FilePath(CONFIGNAME)); }

		public static void Save(string configs)
		{
			var cnode = new ConfigNode(NODE_NAME);
			SaveConfigs(cnode); saveNode(cnode, configs);
		}

		public static void SaveConfigs(ConfigNode node) 
		{
			//save per-vessel configurations into the current game's node
			var current_vessels = new HashSet<Guid>(HighLogic.CurrentGame.flightState.protoVessels.Select(p => p.vesselID));
			var fg = FlightGlobals.fetch;
			if(fg != null) fg.vessels.ForEach(v => current_vessels.Add(v.id));
			var configs = new List<VesselConfig>(Configs.Values);
			configs.Sort();
			if(configs.Count > 0)
			{
				var n = node.AddNode(CurrentGame);
				foreach(var c in configs)
				{
					if(current_vessels.Contains(c.VesselID))
						c.Save(n.AddNode(VesselConfig.NODE_NAME));
					else Utils.Log(
						"TCAConfiguration.SaveConfigs: vessel {0} is not present in the game. " +
						"Removing orphan configuration.", c.VesselID);
				}
			}
			//save other games
			other_games.ForEach(n => node.AddNode(n));
			if(NamedConfigs.Count > 0)
			{
				var n = node.AddNode(NAMED_NODE);
				foreach(var c in NamedConfigs.Keys)
					NamedConfigs[c].Save(n.AddNode(NamedConfig.NODE_NAME));
			}
		}
		#endregion
	}
}

