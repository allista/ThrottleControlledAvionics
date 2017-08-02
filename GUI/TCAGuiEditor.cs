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
using KSP.UI.Screens;
using AT_Utils;

namespace ThrottleControlledAvionics
{
	[KSPAddon(KSPAddon.Startup.EditorAny, false)]
	public class TCAGuiEditor : AddonWindowBase<TCAGuiEditor>
	{
		const string DefaultConstractName = "Untitled Space Craft";

		public static bool Available { get; private set; }
		Dictionary<Type,bool> Modules = new Dictionary<Type, bool>();
		static Texture2D CoM_Icon;

		TCAPartsEditor PartsEditor;
        SimpleWarning warning;

		ModuleTCA TCA;
		NamedConfig CFG;
		readonly List<EngineWrapper> Engines = new List<EngineWrapper>();
		readonly EnginesDB ActiveEngines = new EnginesDB();
		static bool HaveSelectedPart { get { return EditorLogic.SelectedPart != null && EditorLogic.SelectedPart.potentialParent != null; } }

		HighlightSwitcher TCA_highlight, Engines_highlight;

		float Mass, DryMass, MinTWR, MaxTWR, MinLimit;
		Vector3 CoM = Vector3.zero;
		Vector3 WetCoM = Vector3.zero;
		Vector3 DryCoM = Vector3.zero;
		Matrix3x3f InertiaTensor = new Matrix3x3f();
		Vector3 MoI { get { return new Vector3(InertiaTensor[0, 0], InertiaTensor[1, 1], InertiaTensor[2, 2]); } }

		bool show_imbalance;
		bool use_wet_mass = true;

		public override void Awake()
		{
			base.Awake();
			width = 600;
			height = 100;
			GameEvents.onEditorShipModified.Add(OnShipModified);
			GameEvents.onEditorLoad.Add(OnShipLoad);
			GameEvents.onEditorRestart.Add(Reset);
			GameEvents.onEditorStarted.Add(Started);
			Available = false;
			show_imbalance = false;
			use_wet_mass = true;
			//icons
			CoM_Icon = GameDatabase.Instance.GetTexture(Globals.RADIATION_ICON, false);
			//highlighters
			TCA_highlight = new HighlightSwitcher(highlight_TCA, reset_TCA_highlighting);
			Engines_highlight = new HighlightSwitcher(highlight_engines, reset_engines_highlightig);

		}

		public override void OnDestroy ()
		{
			GameEvents.onEditorShipModified.Remove(OnShipModified);
			GameEvents.onEditorLoad.Remove(OnShipLoad);
			GameEvents.onEditorRestart.Remove(Reset);
			GameEvents.onEditorStarted.Remove(Started);
			TCAMacroEditor.Exit();
			base.OnDestroy();
		}

		static void UpdatePartsInfo()
		{
			//update TCA part infos
			var info = TCAScenario.ModuleStatusString();
			foreach(var ap in PartLoader.LoadedPartsList)
			{
				foreach(var mi in ap.moduleInfos)
				{
					if(mi.moduleName != ModuleTCA.TCA_NAME) continue;
					mi.primaryInfo = "<b>TCA:</b> "+info;
					mi.info = info;
				}
			}
		}

		void Started() { UpdatePartsInfo(); }

		void Reset() { reset = true; }

		void OnShipLoad(ShipConstruct ship, CraftBrowserDialog.LoadType load_type)
		{ init_engines = load_type == CraftBrowserDialog.LoadType.Normal; }

		void OnShipModified(ShipConstruct ship) 
		{ update_engines = true; }

		void update_modules()
		{
			Modules.Clear();
			TCAModulesDatabase.ValidModules
				.ForEach(t => Modules.Add(t, TCAModulesDatabase.ModuleAvailable(t, CFG)));
		}
		public static void UpdateModules() { if(Instance) Instance.update_modules(); }

		bool GetCFG()
		{
			var ship = EditorLogic.fetch.ship;
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
                CFG = NamedConfig.FromVesselConfig(ship.shipName, TCAScenario.GetDefaultConfig(ship.shipFacility));
                if(CFG.EnginesProfiles.Empty)
                    CFG.EnginesProfiles.AddProfile(Engines);
			}
			else CFG.ActiveProfile.Apply(Engines);
			UpdateCFG(TCA_Modules);
			return true;
		}

		void UpdateCFG(IList<ModuleTCA> TCA_Modules)
		{
			if(CFG == null || TCA_Modules.Count == 0) return;
			TCA_highlight.Reset();
			TCA_Modules.ForEach(m => { m.CFG = null; m.TCA_Active = false; });
			TCA = TCA_Modules[0];
			TCA.CFG = CFG;
			TCA.TCA_Active = true;
			CFG.ActiveProfile.Update(Engines);
			PartsEditor.SetCFG(CFG);
			update_modules();
		}
		void UpdateCFG() { UpdateCFG(ModuleTCA.AllTCA(EditorLogic.fetch.ship)); }

		void compute_inertia_tensor()
		{
			InertiaTensor = new Matrix3x3f();
			if(EditorLogic.RootPart)
				update_inertia_tensor(EditorLogic.RootPart);
		}

		void update_inertia_tensor(Part part)
		{
			if(!EditorLogic.RootPart) return;
			var partMass = part.mass;
			if(use_wet_mass) partMass += part.GetResourceMass();
			Vector3 partPosition = EditorLogic.RootPart.transform
				.InverseTransformDirection(part.transform.position + part.transform.rotation * part.CoMOffset - CoM);
			for(int i = 0; i < 3; i++)
			{
				InertiaTensor.Add(i, i, partMass * partPosition.sqrMagnitude);
				for(int j = 0; j < 3; j++)
					InertiaTensor.Add(i, j, -partMass * partPosition[i] * partPosition[j]);
			}
			part.children.ForEach(update_inertia_tensor);
		}

		void update_mass_and_CoM(Part part)
		{
			if(part == null) return;
			var dryMass = part.mass;
			var wetMass = dryMass+part.GetResourceMass();
			Vector3 pos = Vector3.zero;
			if (part.physicalSignificance == Part.PhysicalSignificance.FULL)
				pos = part.transform.position + part.transform.rotation * part.CoMOffset;
			else if(part.parent != null)
				pos = part.parent.transform.position + part.parent.transform.rotation * part.parent.CoMOffset;
			else if (part.potentialParent != null)
				pos = part.potentialParent.transform.position + part.potentialParent.transform.rotation * part.potentialParent.CoMOffset;
			WetCoM += pos * wetMass;
			DryCoM += pos * dryMass;
			Mass += wetMass;
			DryMass += dryMass;
			part.children.ForEach(update_mass_and_CoM);
		}

		void find_engines_recursively(Part part, List<EngineWrapper> engines)
		{
			if(part.Modules != null)
				engines.AddRange(part.Modules.GetModules<ModuleEngines>()
				                 .Select(m => new EngineWrapper(m)));
			part.children.ForEach(p => find_engines_recursively(p, engines));
		}

		bool UpdateEngines()
		{
			Engines_highlight.Reset();
			Engines.Clear();
			if(TCAScenario.HasTCA && EditorLogic.RootPart) 
				find_engines_recursively(EditorLogic.RootPart, Engines);
			var ret = Engines.Count > 0;
			if(!ret) Reset();
			return ret;
		}

		void process_active_engine(EngineWrapper e)
		{
			e.throttle = e.VSF = e.thrustMod = 1;
			e.UpdateThrustInfo();
			e.InitLimits();
			e.InitTorque(EditorLogic.RootPart.transform, CoM,
			             Globals.Instance.ENG.TorqueRatioFactor);
			e.UpdateCurrentTorque(1);
		}

		static List<Part> GetSelectedParts()
		{
			var selected_parts = new List<Part>();
			if(HaveSelectedPart && !EditorLogic.fetch.ship.Contains(EditorLogic.SelectedPart))
			{
				selected_parts.Add(EditorLogic.SelectedPart);
				selected_parts.AddRange(EditorLogic.SelectedPart.symmetryCounterparts);
			}
			return selected_parts;
		}

		void CalculateMassAndCoM(List<Part> selected_parts)
		{
			Mass = DryMass = MinTWR = MaxTWR = 0;
			CoM = WetCoM = DryCoM = Vector3.zero;
			update_mass_and_CoM(EditorLogic.RootPart);
			if(selected_parts != null)
				selected_parts.ForEach(update_mass_and_CoM);
			WetCoM /= Mass; DryCoM /= DryMass;
			CoM = use_wet_mass? WetCoM : DryCoM;
		}

		void UpdateShipStats()
		{
			MinLimit = 0;
			var thrust = Vector3.zero;
			var selected_parts = GetSelectedParts();
			CalculateMassAndCoM(selected_parts);
			if(CFG != null && CFG.Enabled && Engines.Count > 0)
			{
				ActiveEngines.Clear();
				for(int i = 0, EnginesCount = Engines.Count; i < EnginesCount; i++)
				{
					var e = Engines[i];
					var ecfg = CFG.ActiveProfile.GetConfig(e);
					if(ecfg == null || ecfg.On) ActiveEngines.Add(e);
				}
				if(selected_parts.Count > 0)
				{
					var selected_engines = new List<EngineWrapper>();
					selected_parts.ForEach(p => find_engines_recursively(p, selected_engines));
					ActiveEngines.AddRange(selected_engines);
				}
				if(ActiveEngines.Count > 0)
				{
					ActiveEngines.ForEach(process_active_engine);
					compute_inertia_tensor();
					selected_parts.ForEach(update_inertia_tensor);
					ActiveEngines.SortByRole();
					float max_limit, torque_error, angle_error;
					var imbalance = TorqueProps.CalculateImbalance(true, ActiveEngines.Manual, ActiveEngines.UnBalanced);
					if(ActiveEngines.Balanced.Count > 0)
					{
                        EngineOptimizer.OptimizeLimitsForTorque(ActiveEngines.Balanced, Vector3.zero, imbalance, MoI, true, 
						                                        out max_limit, out torque_error, out angle_error);
                        imbalance = TorqueProps.CalculateImbalance(true, ActiveEngines.Manual, ActiveEngines.UnBalanced, ActiveEngines.Balanced);
					}
					if(ActiveEngines.Steering.Count > 0)
					{
                        if(!EngineOptimizer.OptimizeLimitsForTorque(ActiveEngines.Steering, Vector3.zero, imbalance, MoI, true, 
						                                            out max_limit, out torque_error, out angle_error))
						{
							ActiveEngines.Steering.ForEach(e => e.limit = 0);
							ActiveEngines.Balanced.ForEach(e => e.limit = 0);
						}
					}
					MinLimit = 1;
					for(int i = 0, ActiveEnginesCount = ActiveEngines.Count; i < ActiveEnginesCount; i++)
					{
						var e = ActiveEngines[i];
						thrust += e.wThrustDir * e.nominalCurrentThrust(e.limit);
						if(e.Role != TCARole.MANUAL &&
						   e.Role != TCARole.MANEUVER &&
						   MinLimit > e.limit) MinLimit = e.limit;
						e.forceThrustPercentage(e.limit*100);
					}
					var T = thrust.magnitude/Utils.G0;
					MinTWR = T/Mass;
					MaxTWR = T/DryMass;
				}
			}
		}

		void AutoconfigureProfile()
		{
			CalculateMassAndCoM(GetSelectedParts());
			var EnginesCount = Engines.Count;
			//reset groups; set CoM-coaxial engines to UnBalanced role
			for(int i = 0; i < EnginesCount; i++)
			{
				var e = Engines[i];
				e.SetGroup(0);
				if(e.Role == TCARole.MANUAL || e.Role == TCARole.MANEUVER)
					continue;
				if(e.engine.throttleLocked)
				{
					e.SetRole(TCARole.MANUAL);
					e.forceThrustPercentage(100);
					continue;
				}
				e.UpdateThrustInfo();
				e.InitTorque(EditorLogic.fetch.ship[0].transform, CoM, 1);
				if(e.torqueRatio < Globals.Instance.ENG.UnBalancedThreshold) e.SetRole(TCARole.UNBALANCE);
			}
			//group symmetry-clones
			var group = 1;
			for(int i = 0; i < EnginesCount; i++)
			{
				var e = Engines[i];
				if(e.Group > 0) continue;
				if(e.part.symmetryCounterparts.Count > 0)
				{
					e.SetGroup(group);
					e.part.symmetryCounterparts.ForEach(p => p.Modules.GetModule<TCAEngineInfo>().group = group);
					group += 1;
				}
			}
			//update active profile
			CFG.ActiveProfile.Update(Engines);
		}


		bool reset, init_engines, update_engines, update_stats, autoconfigure_profile;
		void Update()
		{
			if(EditorLogic.fetch == null || EditorLogic.fetch.ship == null) return;
			update_stats |= HaveSelectedPart;
			if(reset)
			{
				Available = false;
				Modules.Clear();
				Engines.Clear();
				PartsEditor.SetCFG(null);
				CFG = null;
				reset = false;
			}
			if(init_engines)
			{
				if(UpdateEngines()) GetCFG();
				init_engines = false;
				update_stats = true;
			}
			if(update_engines)
			{
				if(UpdateEngines())
				{
					if(CFG != null) UpdateCFG();
					else GetCFG();
				}
				update_engines = false;
				update_stats = true;
			}
			if(autoconfigure_profile)
			{
				AutoconfigureProfile();
				autoconfigure_profile = false;
				update_stats = true;
			}
			if(update_stats) 
			{
				UpdateShipStats();
				update_stats = false;
			}
			Available |= CFG != null && Engines.Count > 0;
			TCA_highlight.Update(Available && doShow);
			Engines_highlight.Update(Available && doShow && show_imbalance && ActiveEngines.Count > 0);
		}

		void DrawMainWindow(int windowID)
		{
			//help button
			if(GUI.Button(new Rect(WindowPos.width - 23f, 2f, 20f, 18f), 
			              new GUIContent("?", "Help"))) TCAManual.ToggleInstance();
			GUILayout.BeginVertical();
			{
				GUILayout.BeginHorizontal();
				{
					if(GUILayout.Button(new GUIContent("Select Modules", "Select which TCA Modules should be installed on this ship"),
					                        Styles.active_button, GUILayout.ExpandWidth(true)))
						PartsEditor.Toggle();
					if(Modules[typeof(MacroProcessor)])
					{
						
						if(TCAMacroEditor.Editing)
							GUILayout.Label("Edit Macros", Styles.inactive_button, GUILayout.ExpandWidth(true));
						else if(GUILayout.Button("Edit Macros", Styles.active_button, GUILayout.ExpandWidth(true)))
							TCAMacroEditor.Edit(CFG);
					}
                    if(GUILayout.Button(new GUIContent("Save As Default", "Save current configuration as default for new ships in this facility (VAB/SPH)"),
                                        Styles.active_button, GUILayout.ExpandWidth(true)))
                        warning.Show(true);
				}
				GUILayout.EndHorizontal();
				GUILayout.BeginHorizontal();
				{
					GUILayout.BeginVertical();
					{
						GUILayout.BeginHorizontal();
						{
							if(Utils.ButtonSwitch("Enable TCA", ref CFG.Enabled, "", GUILayout.ExpandWidth(false)))
							{
								if(!CFG.Enabled) 
									Engines.ForEach(e => e.forceThrustPercentage(100));
								CFG.GUIVisible = CFG.Enabled;
							}
							if(Modules[typeof(AltitudeControl)])
							{
								if(Utils.ButtonSwitch("Hover", CFG.VF[VFlight.AltitudeControl], 
				                                      "Enable Altitude Control", GUILayout.ExpandWidth(false)))
									CFG.VF.Toggle(VFlight.AltitudeControl);
								Utils.ButtonSwitch("Follow Terrain", ref CFG.AltitudeAboveTerrain, 
				                                   "Enable follow terrain mode", GUILayout.ExpandWidth(false));
							}
							if(Modules[typeof(VTOLControl)])
							{
								if(Utils.ButtonSwitch("VTOL Mode", CFG.CTRL[ControlMode.VTOL], 
				                                      "Keyboard controls thrust direction instead of torque", GUILayout.ExpandWidth(false)))
									CFG.CTRL.XToggle(ControlMode.VTOL);
							}
							if(Modules[typeof(VTOLAssist)])
								Utils.ButtonSwitch("VTOL Assist", ref CFG.VTOLAssistON, 
				                                   "Automatic assistnce with vertical takeof or landing", GUILayout.ExpandWidth(false));
							if(Modules[typeof(FlightStabilizer)])
								Utils.ButtonSwitch("Flight Stabilizer", ref CFG.StabilizeFlight, 
				                                   "Automatic flight stabilization when vessel is out of control", GUILayout.ExpandWidth(false));
							if(Modules[typeof(HorizontalSpeedControl)])
								Utils.ButtonSwitch("H-Translation", ref CFG.CorrectWithTranslation, 
				                                   "Use translation to correct horizontal velocity", GUILayout.ExpandWidth(false));
							if(Modules[typeof(CollisionPreventionSystem)]) 
								Utils.ButtonSwitch("CPS", ref CFG.UseCPS, 
				                                   "Enable Collistion Prevention System", GUILayout.ExpandWidth(false));
						}
						GUILayout.EndHorizontal();
						GUILayout.BeginHorizontal();
						{
							Utils.ButtonSwitch("AutoThrottle", ref CFG.BlockThrottle, 
			                                   "Change altitude/vertical velocity using main throttle control", GUILayout.ExpandWidth(true));
							if(Utils.ButtonSwitch("SmartEngines", ref CFG.UseSmartEngines, 
							                      "Group engines by thrust direction and automatically use appropriate group for a meneuver", GUILayout.ExpandWidth(true)))
							{ if(CFG.UseSmartEngines) CFG.SmartEngines.OnIfNot(SmartEnginesMode.Best); }
							Utils.ButtonSwitch("AutoGear", ref CFG.AutoGear, 
			                                   "Automatically deploy/retract landing gear when needed", GUILayout.ExpandWidth(true));
							Utils.ButtonSwitch("AutoBrakes", ref CFG.AutoBrakes, 
							                   "Automatically ebable/disable brakes when needed", GUILayout.ExpandWidth(true));
							Utils.ButtonSwitch("AutoStage", ref CFG.AutoStage, 
							                   "Automatically activate next stage when previous falmeouted", GUILayout.ExpandWidth(true));
							Utils.ButtonSwitch("AutoChute", ref CFG.AutoParachutes, 
							                   "Automatically activate parachutes when needed", GUILayout.ExpandWidth(true));
						}
						GUILayout.EndHorizontal();
					}
					GUILayout.EndVertical();
				}
				GUILayout.EndHorizontal();
				if(GUILayout.Button(new GUIContent("Autoconfigure Active Profile", 
				                                   "This will overwrite any existing groups and roles"), 
				                    Styles.danger_button, GUILayout.ExpandWidth(true)))
					autoconfigure_profile = true;
				CFG.EnginesProfiles.Draw(height);
				if(CFG.ActiveProfile.Changed)
				{
					CFG.ActiveProfile.Apply(Engines);
					update_engines = true;
				}
				GUILayout.BeginHorizontal(Styles.white);
				{
					GUILayout.Label("Ship Info:");
					GUILayout.FlexibleSpace();
					GUILayout.Label("Mass:", Styles.boxed_label);
					if(Utils.ButtonSwitch(Utils.formatMass(Mass), ref use_wet_mass, "Balance engines using Wet Mass"))
						update_stats = true;
					GUILayout.Label("►");
					if(Utils.ButtonSwitch(Utils.formatMass(DryMass), !use_wet_mass, "Balance engines using Dry Mass")) 
					{
						use_wet_mass = !use_wet_mass;
						update_stats = true;
					}
					if(CFG.Enabled)
					{
						if(ActiveEngines.Count > 0)
						{
                            GUILayout.Label(new GUIContent(string.Format("TMR: {0:F2} ► {1:F2}", MinTWR, MaxTWR),
                                                           "Thrust ot Mass Ratio"), 
                                            Styles.fracStyle(Utils.Clamp(MinTWR-1, 0, 1)));
							GUILayout.Label(new GUIContent(string.Format("Balanced: {0:P1}", MinLimit),
							                               "The efficacy of the least efficient of balanced engines"),
							                Styles.fracStyle(MinLimit));
							Utils.ButtonSwitch("HL", ref show_imbalance, "Highlight engines with low efficacy deu to balancing");
						}
						else GUILayout.Label("No active engines", Styles.boxed_label);
					}
					else GUILayout.Label("TCA is disabled", Styles.boxed_label);
				}
				GUILayout.EndHorizontal();
			}
			GUILayout.EndVertical();
			TooltipsAndDragWindow();
		}

		protected override bool can_draw() { return Available; }

		static void highlight_engine(ThrusterWrapper e)
		{
			if(e.limit < 1) 
			{
				var lim = e.limit * e.limit;
				e.part.HighlightAlways(lim < 0.5f? 
				                       Color.Lerp(Color.magenta, Color.yellow, lim/0.5f) :
				                       Color.Lerp(Color.yellow, Color.cyan, (lim-0.5f)/0.5f));
			}
		}

		void highlight_engines()
		{
			for(int i = 0, EnginesCount = Engines.Count; i < EnginesCount; i++)
			{
				var e = Engines[i];
				if(e.Role != TCARole.BALANCE && e.Role != TCARole.MAIN)
					e.part.SetHighlightDefault();
			}
			ActiveEngines.Balanced.ForEach(highlight_engine);
			ActiveEngines.Main.ForEach(highlight_engine);
		}

		static void reset_engines_highlightig()
		{
			EditorLogic.fetch.ship.Parts
				.Where(p => p.Modules.Contains<ModuleEngines>())
				.ForEach(p => p.SetHighlightDefault());
		}

		void highlight_TCA()
		{
			if(TCA != null && TCA.part != null ) 
				TCA.part.HighlightAlways(Color.green);
		}

		void reset_TCA_highlighting()
		{
			if(TCA != null && TCA.part != null) 
				TCA.part.SetHighlightDefault();
		}

		protected override void draw_gui()
		{
			LockControls();
			WindowPos = 
				GUILayout.Window(GetInstanceID(), 
				                 WindowPos, 
				                 DrawMainWindow, 
				                 Title,
				                 GUILayout.Width(width),
				                 GUILayout.Height(height)).clampToScreen();
            if(warning.doShow)
            {
                var facility = EditorLogic.fetch.ship.shipFacility;
                warning.Draw("Are you sure you want to save current ship configuration as default for "+facility+"?");
                if(warning.Result == SimpleDialog.Answer.Yes) 
                    TCAScenario.UpdateDefaultConfig(facility, CFG);
            }
			PartsEditor.Draw();
			if(show_imbalance && ActiveEngines.Count > 0)
			{
				Markers.DrawWorldMarker(WetCoM, Color.yellow, "Center of Mass", CoM_Icon);
				Markers.DrawWorldMarker(DryCoM, Color.red, "Center of Dry Mass", CoM_Icon);
			}
		}

		class HighlightSwitcher
		{
			bool enabled;
			public Action Enable = delegate {};
			public Action Disable = delegate {};

			public HighlightSwitcher(Action highlight, Action disable_highliting)
			{
				Enable = highlight;
				Disable = disable_highliting;
			}

			public void Update(bool predicate)
			{
				if(predicate)
				{
					Enable();
					enabled = true;
				}
				else if(enabled)
				{
					Disable();
					enabled = false;
				}
			}

			public void Reset() { Update(false); }
		}
	}
}