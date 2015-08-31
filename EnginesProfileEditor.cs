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
		static public NamedConfig CFG { get; private set; }
		static List<EngineWrapper> Engines = new List<EngineWrapper>();

		public override void Awake()
		{
			base.Awake();
			GameEvents.onEditorShipModified.Add(OnShipModified);
			GameEvents.onEditorLoad.Add(OnShipLoad);
		}

		public override void OnDestroy ()
		{
			GameEvents.onEditorShipModified.Remove(OnShipModified);
			GameEvents.onEditorLoad.Remove(OnShipLoad);
			base.OnDestroy();
		}

		void OnShipLoad(ShipConstruct ship, CraftBrowser.LoadType load_type)
		{
			if(load_type == CraftBrowser.LoadType.Merge) return;
			if(!UpdateEngines(ship)) return;
			CFG = TCAScenario.GetConfig(ship.shipName);
			if(CFG != null) 
				CFG.EnginesProfiles.Active.Apply(Engines);
			else 
			{ 
				CFG = TCAScenario.NewNamedConfig(ship.shipName);
				CFG.EnginesProfiles.AddProfile(Engines);
			}
			CFG.EnginesProfiles.Active.Update(Engines);
		}

		static bool UpdateEngines(ShipConstruct ship)
		{
			Engines.Clear();
			if(!ModuleTCA.HasTCA) 
			{ 
				TCAToolbarManager.ShowButton(false);
				CFG = null; 
				return false; 
			}
			TCAToolbarManager.ShowButton();
			foreach(Part p in ship.Parts)
				foreach(var module in p.Modules)
				{	
					var engine = module as ModuleEngines;
					if(engine != null) Engines.Add(new EngineWrapper(engine)); 
				}
			return Engines.Count > 0;
		}

		void OnShipModified(ShipConstruct ship)
		{
			UpdateEngines(ship);
			if(!UpdateEngines(ship)) return;
			CFG.EnginesProfiles.Active.Update(Engines);
			if(ship.shipName != CFG.Name)
			{
				TCAScenario.NamedConfigs.Remove(CFG.Name);
				CFG.Name = ship.shipName;
				TCAScenario.NamedConfigs.Add(CFG.Name, CFG);
			}
		}

		protected override void DrawMainWindow(int windowID)
		{
			CFG.EnginesProfiles.Draw(height);
			base.DrawMainWindow(windowID);
		}

		public void OnGUI()
		{
			if(Engines.Count == 0 || !showHUD) return;
			Styles.Init();
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