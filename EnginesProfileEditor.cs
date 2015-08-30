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
using KSP.IO;

namespace ThrottleControlledAvionics
{
	[KSPAddon(KSPAddon.Startup.EditorAny, false)]
	public class EnginesProfileEditor : TCAGuiBase<EnginesProfileEditor>
	{
		NamedConfig CFG;
		List<EngineWrapper> Engines = new List<EngineWrapper>();

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
			base.OnDestroy ();
		}

		void OnShipLoad(ShipConstruct ship, CraftBrowser.LoadType load_type)
		{
			if(load_type == CraftBrowser.LoadType.Merge) return;
			UpdateEngines(ship);
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

		void UpdateEngines(ShipConstruct ship)
		{
			Engines.Clear();
			foreach(Part p in ship.Parts)
				foreach(var module in p.Modules)
				{	
					var engine = module as ModuleEngines;
					if(engine != null) Engines.Add(new EngineWrapper(engine)); 
				}
		}

		void OnShipModified(ShipConstruct ship)
		{
			UpdateEngines(ship);
			CFG.EnginesProfiles.Active.Update(Engines);
			if(ship.shipName != CFG.Name)
			{
				TCAScenario.NamedConfigs.Remove(CFG.Name);
				CFG.Name = ship.shipName;
				TCAScenario.NamedConfigs.Add(CFG.Name, CFG);
			}
		}

		void ProfileManagerWindow(int windowID)
		{
			CFG.EnginesProfiles.Draw(height);
			GetToolTip();
			DrawToolTip(MainWindow);
			GUI.DragWindow();
		}

		public void OnGUI()
		{
			if(!showHUD) return;
			Styles.Init();
			MainWindow = 
				GUILayout.Window(GetInstanceID(), 
					MainWindow, 
					ProfileManagerWindow, 
					TCATitle,
					GUILayout.Width(width),
					GUILayout.Height(height));
			MainWindow.clampToScreen();
		}
	}
}