//  Author:
//       Allis Tauri <allista@gmail.com>
//
//  Copyright (c) 2015 Allis Tauri
//
// This work is licensed under the Creative Commons Attribution-ShareAlike 4.0 International License. 
// To view a copy of this license, visit http://creativecommons.org/licenses/by-sa/4.0/ 
// or send a letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.

using System;
using System.IO;
using System.Linq;
using System.Reflection;
using System.Collections.Generic;

namespace ThrottleControlledAvionics
{
	[KSPScenario(ScenarioCreationOptions.AddToAllGames, 
		new []
		{
			GameScenes.SPACECENTER,
			GameScenes.FLIGHT,
			GameScenes.EDITOR
		})]
	public class TCAScenario : ScenarioModule
	{
		public const string GLOBALSNAME = "TCA.glob";
		public const string GLOBALS_OVERRIDE = "TCA.user";
		public const string MACROSNAME  = "TCA.macro";
		public const string VSL_NODE    = "VESSELS";
		public const string NAMED_NODE  = "NAMED";

		static TCAGlobals globals;
		public static TCAGlobals Globals 
		{
			get
			{
				if(globals == null) LoadGlobals();
				return globals;
			}
		}

		static TCAMacroLibrary macros;
		public static TCAMacroLibrary Macros
		{
			get
			{
				if(macros == null)
				{
					macros = new TCAMacroLibrary();
					var node = loadNode(PluginFolder(MACROSNAME));
					if(node != null) macros.Load(node);
				}
				return macros;
			}
		}

		public static void SaveMacro(TCAMacro macro)
		{
			Macros.SaveMacro(macro, true);
			var node = new ConfigNode();
			Macros.Save(node);
			node.Save(PluginFolder(MACROSNAME));
		}

		public static Dictionary<Guid, VesselConfig> Configs = new Dictionary<Guid, VesselConfig>();
		public static SortedList<string, NamedConfig> NamedConfigs = new SortedList<string, NamedConfig>();
		public static bool ConfigsLoaded { get; private set; }

		#region ModuleInfo
		public static string ModuleStatusString()
		{ return HasTCA? "<b><color=#00ff00ff>Software Installed</color></b>" : "<color=#ff0000ff>Unavailable</color>"; }
		public static bool HasTCA { get { return !Globals.IntegrateIntoCareer || Utils.PartIsPurchased(TCAGlobals.TCA_PART); } }
		#endregion

		#region From KSPPluginFramework
		static public string PluginFolder(string filename)
		{ return Path.Combine(_AssemblyFolder, "../"+filename).Replace("\\","/"); }
		//Combine the Location of the assembly and the provided string.
		//This means we can use relative or absolute paths.
		static public string PluginData(string filename)
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

		#region Runtime Interface
		public static VesselConfig GetConfig(VesselWrapper VSL)
		{ return GetConfig(VSL.vessel); }

		public static VesselConfig GetConfig(Vessel vessel)
		{
			if(!Configs.ContainsKey(vessel.id)) 
				Configs.Add(vessel.id, new VesselConfig(vessel));
			return Configs[vessel.id];
		}

		public static NamedConfig NewNamedConfig(string name)
		{ 
			if(NamedConfigs.ContainsKey(name)) return null;
			var c = new NamedConfig(name);
			NamedConfigs[name] = c;
			return c;
		}

		public static NamedConfig GetConfig(string name)
		{ return NamedConfigs.ContainsKey(name) ? NamedConfigs[name] : null; }

		public static NamedConfig GetConfig(int index)
		{ return NamedConfigs.Count > index ? NamedConfigs.Values[index] : null; }

		public static bool SaveNamedConfig(string name, VesselConfig config, bool overwrite = false)
		{ 
			if(name == string.Empty || //do not allow empty name
				NamedConfigs.ContainsKey(name) && !overwrite) return false;
			NamedConfigs[name] = NamedConfig.FromVesselConfig(name, config);
			return true;
		}

		public static void UpdateConfig(VesselConfig config)
		{
			VesselConfig old;
			if(Configs.TryGetValue(config.VesselID, out old))
				old.CopyFrom(config);
			else Configs.Add(config.VesselID, config);
		}
		#endregion

		#region Save/Load
		static ConfigNode loadNode(string filepath)
		{
			var node = ConfigNode.Load(filepath);
			if(node == null)
				Utils.Log("TCAScenario: Unable to read "+filepath);
			return node;
		}

		public static void LoadGlobals()
		{
			globals = new TCAGlobals();
			var gnode = loadNode(PluginData(GLOBALSNAME));
			if(gnode != null) Globals.Load(gnode);
			gnode = loadNode(PluginFolder(GLOBALS_OVERRIDE));
			if(gnode != null) Globals.Load(gnode);
			Globals.Init();
		}

		public static void LoadConfigs(ConfigNode node) 
		{
			if(ConfigsLoaded) return;
			Configs.Clear();
			NamedConfigs.Clear();
			foreach(var n in node.GetNodes())
			{
				if(n.name == VSL_NODE)
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
			}
			ConfigsLoaded = true;
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
				var n = node.AddNode(VSL_NODE);
				foreach(var c in configs)
				{
					if(current_vessels.Contains(c.VesselID))
						c.Save(n.AddNode(VesselConfig.NODE_NAME));
					else Utils.Log(
						"TCAScenario: SaveConfigs: vessel {0} is not present in the game. " +
						"Removing orphan configuration.", c.VesselID);
				}
			}
			if(NamedConfigs.Count > 0)
			{
				var n = node.AddNode(NAMED_NODE);
				foreach(var c in NamedConfigs.Keys)
					NamedConfigs[c].Save(n.AddNode(NamedConfig.NODE_NAME));
			}
		}

		[Obsolete("Only needed for legacy config conversion")]
		public static void LoadLegacyConfigs(ConfigNode node) 
		{
			Configs.Clear();
			NamedConfigs.Clear();
			foreach(var n in node.GetNodes())
			{
				if(n.name == HighLogic.CurrentGame.Title.Split()[0])
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
			}
		}
		#endregion

		public override void OnLoad(ConfigNode node)
		{ 
			LoadConfigs(node);
			//deprecated: Old config conversion
			if(Configs.Count == 0 && NamedConfigs.Count == 0)
			{
				var cnode = loadNode(PluginData("TCA.conf"));
				if(cnode != null) LoadLegacyConfigs(cnode);
			}
		}

		public override void OnSave (ConfigNode node) { SaveConfigs(node); }
	}
}

