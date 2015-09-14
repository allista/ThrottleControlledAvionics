//   EnginesPrfileEditor.cs
//
//  Author:
//       Allis Tauri <allista@gmail.com>
//
//  Copyright (c) 2015 Allis Tauri
//
// This work is licensed under the Creative Commons Attribution 4.0 International License. 
// To view a copy of this license, visit http://creativecommons.org/licenses/by/4.0/ 
// or send a letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.

using System;
using System.Collections.Generic;
using UnityEngine;

namespace ThrottleControlledAvionics
{
	[KSPAddon(KSPAddon.Startup.EditorAny, false)]
	public class EnginesProfileEditor : AddonWindowBase<EnginesProfileEditor>
	{
		const string LockName = "EnginesProfileEditor";
		const string DefaultConstractName = "Untitled Space Craft";

		NamedConfig CFG;
		bool saved;
		readonly List<EngineWrapper> Engines = new List<EngineWrapper>();

		public static bool GUIVisible 
		{ 
			get { return instance != null && instance.CFG != null && instance.CFG.GUIVisible; } 
			set { if(instance != null && instance.CFG != null) instance.CFG.GUIVisible = value; }
		}

		public override void Awake()
		{
			base.Awake();
			height = 400;
			GameEvents.onEditorShipModified.Add(OnShipModified);
			GameEvents.onEditorLoad.Add(OnShipLoad);
			GameEvents.onEditorRestart.Add(Reset);
		}

		public override void OnDestroy ()
		{
			if(EditorLogic.fetch.ship.shipName != DefaultConstractName) SaveConfig();
			GameEvents.onEditorShipModified.Remove(OnShipModified);
			GameEvents.onEditorLoad.Remove(OnShipLoad);
			GameEvents.onEditorRestart.Remove(Reset);
			base.OnDestroy();
		}

		void Reset()
		{
			TCAToolbarManager.ShowButton(false);
			Engines.Clear();
			saved = false;
			CFG = null;
		}

		void OnShipLoad(ShipConstruct ship, CraftBrowser.LoadType load_type)
		{
			if(load_type == CraftBrowser.LoadType.Merge) return;
			Utils.Log("EPE.OnShipLoad: {0}", ship.shipName);//debug
			if(UpdateEngines(ship)) GetCFG(ship);
		}

		void GetCFG(ShipConstruct ship, bool local = false)
		{
			CFG = TCAScenario.GetConfig(ship.shipName);
			if(CFG == null)
			{
				saved = !local;
				CFG = local ? new NamedConfig(ship.shipName) : 
					TCAScenario.NewNamedConfig(ship.shipName);
				CFG.EnginesProfiles.AddProfile(Engines);
			}
			else { CFG.ActiveProfile.Apply(Engines); saved = true; }
			CFG.ActiveProfile.Update(Engines);
			Utils.Log("EPE.GetCFG.CFG: {0}, local: {1}", CFG, local);//debug
		}

		bool UpdateEngines(ShipConstruct ship)
		{
			Engines.Clear();
			if(ModuleTCA.HasTCA) 
			{ 
				TCAToolbarManager.SetDefaultButton();
				TCAToolbarManager.ShowButton();
				foreach(Part p in ship.Parts)
					foreach(var module in p.Modules)
					{	
						var engine = module as ModuleEngines;
						if(engine != null) Engines.Add(new EngineWrapper(engine)); 
					}
				if(Engines.Count > 0) return true;
			}
			Reset();
			return false;
		}

		void OnShipModified(ShipConstruct ship)
		{
			if(!UpdateEngines(ship)) return;
			Utils.Log("EPE.OnShipModified: {0}", ship.shipName);//debug
			if(CFG == null) GetCFG(ship, true);
			CFG.ActiveProfile.Update(Engines);
		}

		void SaveConfig()
		{
			if(saved || CFG == null) return;
			TCAScenario.NamedConfigs.Remove(CFG.Name);
			CFG.Name = EditorLogic.fetch.ship.shipName;
			TCAScenario.NamedConfigs.Add(CFG.Name, CFG);
			saved = true;
		}

		public void Update()
		{
			if(CFG == null) return;
			if(EditorLogic.fetch.ship.shipName != CFG.Name)
			{
				Utils.Log("EPE.Udpate.Rename '{0}' to '{1}'", CFG.Name, EditorLogic.fetch.ship.shipName);//debug
				SaveConfig();
			}
		}

		protected override void DrawMainWindow(int windowID)
		{
			GUILayout.BeginVertical();
			GUILayout.Label(new GUIContent(string.Format("{0} configuration: {1}", CFG.Name, saved? "saved" : "unsaved"),
			                               "Configuration for 'Untitled Space Craft' is never saved"),
			                saved? Styles.green : Styles.red, GUILayout.ExpandWidth(true));
			CFG.EnginesProfiles.Draw(height);
			GUILayout.EndVertical();
			if(CFG.ActiveProfile.Changed)
				CFG.ActiveProfile.Apply(Engines);
			base.DrawMainWindow(windowID);
		}

		public void OnGUI()
		{
			if(Engines.Count == 0 || !CFG.GUIVisible || !showHUD) 
			{
				Utils.LockIfMouseOver(LockName, MainWindow, false);
				return;
			}
			Styles.Init();
			Utils.LockIfMouseOver(LockName, MainWindow);
			MainWindow = 
				GUILayout.Window(GetInstanceID(), 
					MainWindow, 
					DrawMainWindow, 
					TCATitle,
					GUILayout.Width(width),
					GUILayout.Height(height));
			MainWindow.clampToScreen();
		}
	}
}