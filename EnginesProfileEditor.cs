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
		NamedConfig CFG;
		List<EngineWrapper> Engines = new List<EngineWrapper>();

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
			GameEvents.onEditorRestart.Add(OnEditorRestart);
		}

		public override void OnDestroy ()
		{
			GameEvents.onEditorShipModified.Remove(OnShipModified);
			GameEvents.onEditorLoad.Remove(OnShipLoad);
			GameEvents.onEditorRestart.Remove(OnEditorRestart);
			base.OnDestroy();
		}

		void OnEditorRestart()
		{
			Engines.Clear();
			CFG = null;
		}

		void OnShipLoad(ShipConstruct ship, CraftBrowser.LoadType load_type)
		{
			if(load_type == CraftBrowser.LoadType.Merge) return;
			if(UpdateEngines(ship)) GetCFG(ship);
		}

		void GetCFG(ShipConstruct ship)
		{
			CFG = TCAScenario.GetConfig(ship.shipName);
			if(CFG != null) 
				CFG.EnginesProfiles.Active.Apply(Engines);
			else 
			{ 
				CFG = TCAScenario.NewNamedConfig(ship.shipName);
				CFG.EnginesProfiles.AddProfile(Engines);
			}
			Utils.Log("EPE.UpdateCFG.CFG: {0}", CFG);//debug
			CFG.EnginesProfiles.Active.Update(Engines);
		}

		bool UpdateEngines(ShipConstruct ship)
		{
			Engines.Clear();
			if(ModuleTCA.HasTCA) 
			{ 
				TCAToolbarManager.ShowButton();
				foreach(Part p in ship.Parts)
					foreach(var module in p.Modules)
					{	
						var engine = module as ModuleEngines;
						if(engine != null) Engines.Add(new EngineWrapper(engine)); 
					}
				if(Engines.Count > 0) return true;
			}
			TCAToolbarManager.ShowButton(false);
			CFG = null;
			return false;
		}

		void OnShipModified(ShipConstruct ship)
		{
			if(CFG == null) return;
			if(!UpdateEngines(ship)) return;
			CFG.EnginesProfiles.Active.Update(Engines);
			if(ship.shipName != CFG.Name)
			{
				Utils.Log("EPE.OnShipModified.Rename {0} to {1}", CFG.Name, ship.shipName);//debug
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