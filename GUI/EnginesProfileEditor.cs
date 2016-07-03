//  Author:
//       Allis Tauri <allista@gmail.com>
//
//  Copyright (c) 2015 Allis Tauri
//
// This work is licensed under the Creative Commons Attribution-ShareAlike 4.0 International License. 
// To view a copy of this license, visit http://creativecommons.org/licenses/by-sa/4.0/ 
// or send a letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.

using System;
using System.Collections.Generic;
using UnityEngine;
using KSP.UI.Screens;
using AT_Utils;

namespace ThrottleControlledAvionics
{
	[KSPAddon(KSPAddon.Startup.EditorAny, false)]
	public class EnginesProfileEditor : AddonWindowBase<EnginesProfileEditor>
	{
		const string LockName = "EnginesProfileEditor";
		const string DefaultConstractName = "Untitled Space Craft";

		NamedConfig CFG;
		readonly List<EngineWrapper> Engines = new List<EngineWrapper>();

		public static bool Available { get; private set; }
		static bool HasMacroProcessor, HasVTOLAssist, HasFlightStabilizer, HasAltitudeControl, HasVTOLControls;

		public override void Awake()
		{
			base.Awake();
			width = 600;
			height = 400;
			GameEvents.onEditorShipModified.Add(OnShipModified);
			GameEvents.onEditorLoad.Add(OnShipLoad);
			GameEvents.onEditorRestart.Add(Reset);
			Available = false;
			//module availability
			HasMacroProcessor = TCAModulesDatabase.ModuleAvailable(typeof(MacroProcessor));
			HasVTOLAssist = TCAModulesDatabase.ModuleAvailable(typeof(VTOLAssist));
			HasVTOLControls = TCAModulesDatabase.ModuleAvailable(typeof(VTOLControl));
			HasFlightStabilizer = TCAModulesDatabase.ModuleAvailable(typeof(FlightStabilizer));
			HasAltitudeControl = TCAModulesDatabase.ModuleAvailable(typeof(AltitudeControl));
			//update TCA part infos
			foreach(var ap in PartLoader.LoadedPartsList)
			{
				foreach(var mi in ap.moduleInfos)
				{
					if(mi.moduleName != ModuleTCA.TCA_NAME) continue;
					mi.info = TCAScenario.ModuleStatusString();
					mi.primaryInfo = "<b>TCA:</b> "+mi.info;
				}
			}
		}

		public override void OnDestroy ()
		{
			GameEvents.onEditorShipModified.Remove(OnShipModified);
			GameEvents.onEditorLoad.Remove(OnShipLoad);
			GameEvents.onEditorRestart.Remove(Reset);
			TCAMacroEditor.Exit();
			base.OnDestroy();
		}

		void Reset() { reset = true; }

		void OnShipLoad(ShipConstruct ship, CraftBrowserDialog.LoadType load_type)
		{ init_engines = load_type == CraftBrowserDialog.LoadType.Normal; }

		bool GetCFG(ShipConstruct ship)
		{
			var TCA_Modules = ModuleTCA.AllTCA(ship);
			if(TCA_Modules.Count == 0) { Reset(); return false; }
			CFG = null;
			foreach(var tca in TCA_Modules)
			{
				if(tca.CFG == null) continue;
				CFG = NamedConfig.FromVesselConfig(ship.shipName, tca.CFG);
				break;
			}
			if(CFG == null)
			{
				CFG = new NamedConfig(ship.shipName);
				CFG.EnginesProfiles.AddProfile(Engines);
			}
			else CFG.ActiveProfile.Apply(Engines);
			CFG.ActiveProfile.Update(Engines);
			UpdateCFG(TCA_Modules);
			return true;
		}

		void UpdateCFG(IList<ModuleTCA> TCA_Modules)
		{
			if(CFG == null || TCA_Modules.Count == 0) return;
			TCA_Modules.ForEach(m => m.CFG = null);
			TCA_Modules[0].CFG = CFG;
		}
		void UpdateCFG(ShipConstruct ship)
		{ UpdateCFG(ModuleTCA.AllTCA(ship)); }

		bool UpdateEngines(ShipConstruct ship)
		{
			Engines.Clear();
			if(TCAScenario.HasTCA) 
			{ 
				foreach(Part p in ship.Parts)
					foreach(var module in p.Modules)
					{	
						var engine = module as ModuleEngines;
						if(engine != null) Engines.Add(new EngineWrapper(engine)); 
					}
			}
			var ret = Engines.Count > 0;
			if(!ret) Reset();
			return ret;
		}

		void OnShipModified(ShipConstruct ship) { update_engines = true; }

		bool update_engines, init_engines, reset;
		void Update()
		{
			if(EditorLogic.fetch == null) return;
			if(reset)
			{
				Available = false;
				Engines.Clear();
				CFG = null;
				reset = false;
			}
			if(init_engines)
			{
				if(UpdateEngines(EditorLogic.fetch.ship))
					GetCFG(EditorLogic.fetch.ship);
				init_engines = false;
			}
			if(update_engines)
			{
				if(UpdateEngines(EditorLogic.fetch.ship))
				{
					if(CFG != null) UpdateCFG(EditorLogic.fetch.ship);
					else GetCFG(EditorLogic.fetch.ship);
					if(CFG != null) CFG.ActiveProfile.Update(Engines);
				}
				update_engines = false;
			}
			Available |= CFG != null && Engines.Count > 0;
			if(Available) CFG.GUIVisible = CFG.Enabled;
		}

		protected override void DrawMainWindow(int windowID)
		{
			//help button
			if(GUI.Button(new Rect(MainWindow.width - 23f, 2f, 20f, 18f), 
			              new GUIContent("?", "Help"))) TCAManual.Toggle();
			GUILayout.BeginVertical();
				if(HasMacroProcessor)
				{
					if(TCAMacroEditor.Editing)
						GUILayout.Label("Edit Macros", Styles.inactive_button, GUILayout.ExpandWidth(true));
					else if(GUILayout.Button("Edit Macros", Styles.normal_button, GUILayout.ExpandWidth(true)))
						TCAMacroEditor.Edit(CFG);
				}
				GUILayout.BeginHorizontal();
					GUILayout.BeginVertical();
						GUILayout.BeginHorizontal();
							Utils.ButtonSwitch("Enable TCA", ref CFG.Enabled, "", GUILayout.ExpandWidth(false));
							if(HasAltitudeControl)
							{
								if(Utils.ButtonSwitch("Hover", CFG.VF[VFlight.AltitudeControl], 
				                                      "Enable Altitude Control", GUILayout.ExpandWidth(false)))
									CFG.VF.Toggle(VFlight.AltitudeControl);
								Utils.ButtonSwitch("Follow Terrain", ref CFG.AltitudeAboveTerrain, 
				                                   "Enable follow terrain mode", GUILayout.ExpandWidth(false));
								Utils.ButtonSwitch("AutoThrottle", ref CFG.BlockThrottle, 
				                                   "Change altitude/vertical velocity using main throttle control", GUILayout.ExpandWidth(false));
							}
							if(HasVTOLControls)
							{
								if(Utils.ButtonSwitch("VTOL Mode", CFG.CTRL[ControlMode.VTOL], 
				                                      "Keyboard controls thrust direction instead of torque", GUILayout.ExpandWidth(false)))
									CFG.CTRL.XToggle(ControlMode.VTOL);
							}
							if(HasVTOLAssist)
								Utils.ButtonSwitch("VTOL Assist", ref CFG.VTOLAssistON, 
				                                   "Automatic assistnce with vertical takeof or landing", GUILayout.ExpandWidth(false));
							if(HasFlightStabilizer)
								Utils.ButtonSwitch("Flight Stabilizer", ref CFG.StabilizeFlight, 
				                                   "Automatic flight stabilization when vessel is out of control", GUILayout.ExpandWidth(false));
						GUILayout.EndHorizontal();
						GUILayout.BeginHorizontal();
							Utils.ButtonSwitch("AutoGear", ref CFG.AutoGear, 
			                                   "Automatically deploy/retract landing gear when needed", GUILayout.ExpandWidth(true));
							Utils.ButtonSwitch("AutoBrakes", ref CFG.AutoBrakes, 
							                   "Automatically ebable/disable brakes when needed", GUILayout.ExpandWidth(true));
							Utils.ButtonSwitch("AutoStage", ref CFG.AutoStage, 
							                   "Automatically activate next stage when previous falmeouted", GUILayout.ExpandWidth(true));
							Utils.ButtonSwitch("AutoChute", ref CFG.AutoParachutes, 
							                   "Automatically activate parachutes when needed", GUILayout.ExpandWidth(true));
						GUILayout.EndHorizontal();
					GUILayout.EndVertical();
				GUILayout.EndHorizontal();
				CFG.EnginesProfiles.Draw(height);
				if(CFG.ActiveProfile.Changed)
					CFG.ActiveProfile.Apply(Engines);
			GUILayout.EndVertical();
			base.DrawMainWindow(windowID);
		}

		public void OnGUI()
		{
			if(Engines.Count == 0 || CFG == null || !do_show) 
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
					Title,
					GUILayout.Width(width),
					GUILayout.Height(height)).clampToScreen();
		}
	}
}