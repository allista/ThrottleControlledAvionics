//  Author:
//       Allis Tauri <allista@gmail.com>
//
//  Copyright (c) 2015 Allis Tauri
//
// This work is licensed under the Creative Commons Attribution-ShareAlike 4.0 International License. 
// To view a copy of this license, visit http://creativecommons.org/licenses/by-sa/4.0/ 
// or send a letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.

using System;
using System.Linq;
using System.Collections.Generic;
using UnityEngine;
using AT_Utils;

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
		public const string MACROSNAME  = "TCA.macro";
		public const string VSL_NODE    = "VESSELS";
		public const string NAMED_NODE  = "NAMED";

		static TCAMacroLibrary macros;
		public static TCAMacroLibrary Macros
		{
			get
			{
				if(macros == null)
				{
					macros = new TCAMacroLibrary();
					var node = loadNode(Globals.Instance.PluginFolder(MACROSNAME));
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
			node.Save(Globals.Instance.PluginFolder(MACROSNAME));
		}

        public static VesselConfig VAB_DefaultConfig { get; private set; } = new VesselConfig();
        public static VesselConfig SPH_DefaultConfig { get; private set; } = new VesselConfig();
		public static Dictionary<Guid, VesselConfig> Configs = new Dictionary<Guid, VesselConfig>();
		public static SortedList<string, NamedConfig> NamedConfigs = new SortedList<string, NamedConfig>();
		public static bool ConfigsLoaded { get; private set; }

		public static PathDB Paths = new PathDB();

		#region ModuleInfo
		public static bool HasTCA { get { return !Globals.Instance.IntegrateIntoCareer || Utils.PartIsPurchased(Globals.TCA_PART); } }
		public static bool HavePatchedConics { get; private set; }
		public static bool ModuleInstalled { get; private set; }
		public static bool HavePersistentRotation { get; private set; }
		public static string ModuleStatusString()
		{ return HasTCA? "<b><color=#00ff00ff>Software Installed</color></b>" : "<color=#ff0000ff>Unavailable</color>"; }
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

        public static VesselConfig GetDefaultConfig(EditorFacility facility)
        {
            return facility == EditorFacility.SPH?
                SPH_DefaultConfig : VAB_DefaultConfig;
        }

        public static void UpdateDefaultConfig(EditorFacility facility, VesselConfig config)
        {
            if(facility == EditorFacility.SPH)
                SPH_DefaultConfig.CopyFrom(config);
            else 
                VAB_DefaultConfig.CopyFrom(config);
        }
		#endregion

		#region Save/Load
		static ConfigNode loadNode(string filepath)
		{
			var node = ConfigNode.Load(filepath);
			if(node == null)
				Utils.Log("TCAScenario: Unable to read {}", filepath);
			return node;
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
            VAB_DefaultConfig.LoadFrom(node, "VAB_DefaultConfig");
            SPH_DefaultConfig.LoadFrom(node, "SPH_DefaultConfig");
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
						c.SaveInto(n);
//					else Utils.Log("TCAScenario: SaveConfigs: vessel {} is not present in the game. " +
//					               "Removing orphan configuration.", c.VesselID);
				}
			}
			if(NamedConfigs.Count > 0)
			{
				var n = node.AddNode(NAMED_NODE);
				foreach(var c in NamedConfigs.Keys)
					NamedConfigs[c].SaveInto(n);
			}
            VAB_DefaultConfig.SaveInto(node, "VAB_DefaultConfig");
            SPH_DefaultConfig.SaveInto(node, "SPH_DefaultConfig");
		}
		#endregion

		public override void OnLoad(ConfigNode node)
		{ 
			Globals.Load();
			LoadConfigs(node);
			//navigation paths
			var paths = node.GetNode(PathDB.NODE_NAME);
			if(paths != null) Paths.Load(paths);
			else Paths.Clear();
			//patched conics availability
			HavePatchedConics = GameVariables.Instance
				.GetOrbitDisplayMode(ScenarioUpgradeableFacilities.GetFacilityLevel(SpaceCenterFacility.TrackingStation)) 
				== GameVariables.OrbitDisplayMode.PatchedConics;
			//check if MM is successfully installed ModuleTCA in any of the parts
			ModuleInstalled = false;
			foreach(var p in PartLoader.LoadedPartsList)
			{
				if(p.partPrefab != null && p.partPrefab.HasModule<ModuleTCA>())
				{
					ModuleInstalled = true;
					break;
				}
			}
			if(!ModuleInstalled) TCAManual.ShowStatus();
			//check for PersistentRotation
			HavePersistentRotation = AssemblyLoader.loadedAssemblies.FirstOrDefault(a => a.name == Globals.Instance.PersistentRotationName) != null;
		}

		public override void OnSave(ConfigNode node) 
		{ 
			SaveConfigs(node);
			Paths.SaveInto(node);
		}

        #if DEBUG
        bool show;
        Rect pos = new Rect();
        void drawGlobalsUI(int windowID)
        {
            GUILayout.BeginVertical();
            if(GUILayout.Button(show? "Hide" : "Show", Styles.active_button, GUILayout.ExpandWidth(true)))
                show = !show;
            if(show)
            {
                Globals.Instance.UI.Draw();
                if(GUILayout.Button("Save", Styles.danger_button, GUILayout.ExpandWidth(true)))
                    Globals.Instance.CreateDefaultOverride();
            }
            GUILayout.EndVertical();
            GUIWindowBase.TooltipsAndDragWindow();
        }

        void OnGUI()
        {
            if(Event.current.type != EventType.Layout && Event.current.type != EventType.Repaint) return;
            if(Globals.Instance != null && GUIWindowBase.HUD_enabled)
            {
                Styles.Init();
                pos = GUILayout.Window(GetInstanceID(), pos, drawGlobalsUI, "Globals",
                                       GUILayout.Width(600), GUILayout.Height(show? 400 : 50))
                    .clampToScreen();
            }
        }
        #endif
	}
}

