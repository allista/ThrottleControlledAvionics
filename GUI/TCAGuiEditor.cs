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
		static Dictionary<Type,bool> Modules = new Dictionary<Type, bool>();
		static Texture2D CoM_Icon;

		NamedConfig CFG;
		readonly List<EngineWrapper> Engines = new List<EngineWrapper>();
		readonly EnginesProps.EnginesDB ActiveEngines = new EnginesProps.EnginesDB();
		static bool HaveSelectedPart { get { return EditorLogic.SelectedPart != null && EditorLogic.SelectedPart.potentialParent != null; } }

		float Mass, DryMass, MinTWR, MaxTWR, MinLimit;
		Vector3 CoM = Vector3.zero;
		Matrix3x3f InertiaTensor = new Matrix3x3f();
		Vector3 MoI { get { return new Vector3(InertiaTensor[0, 0], InertiaTensor[1, 1], InertiaTensor[2, 2]); } }

		bool show_imbalance, parts_highlighted;
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
			//module availability
			Modules.Clear();
			TCAModulesDatabase.ValidModules
				.ForEach(t => Modules.Add(t, TCAModulesDatabase.ModuleAvailable(t)));
			//icons
			CoM_Icon = GameDatabase.Instance.GetTexture(Globals.RADIATION_ICON, false);
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
			TCA_Modules.ForEach(m => { m.CFG = null; m.TCA_Active = false; });
			TCA_Modules[0].CFG = CFG;
			TCA_Modules[0].TCA_Active = true;
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
			var dryMass = part.mass;
			var wetMass = dryMass+part.GetResourceMass();
			var partMass = use_wet_mass? wetMass : dryMass;
			if (part.physicalSignificance == Part.PhysicalSignificance.FULL)
				CoM += (part.transform.position + part.transform.rotation * part.CoMOffset) * partMass;
			else if(part.parent != null)
				CoM += (part.parent.transform.position + part.parent.transform.rotation * part.parent.CoMOffset) * partMass;
			else if (part.potentialParent != null)
				CoM += (part.potentialParent.transform.position + part.potentialParent.transform.rotation * part.potentialParent.CoMOffset) * partMass;
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

		void UpdateShipStats()
		{
			var thrust = Vector3.zero;
			Mass = DryMass = MinTWR = MaxTWR = 0;
			CoM = Vector3.zero;
			MinLimit = 0;
			var selected_parts = new List<Part>();
			if(HaveSelectedPart && !EditorLogic.fetch.ship.Contains(EditorLogic.SelectedPart))
			{
				selected_parts.Add(EditorLogic.SelectedPart);
				selected_parts.AddRange(EditorLogic.SelectedPart.symmetryCounterparts);
			}
			update_mass_and_CoM(EditorLogic.RootPart);
			selected_parts.ForEach(update_mass_and_CoM);
			CoM /= use_wet_mass? Mass : DryMass;
			if(CFG != null && Engines.Count > 0)
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
					var imbalance = TorqueProps.CalculateImbalance(ActiveEngines.Manual, ActiveEngines.UnBalanced);
					if(ActiveEngines.Balanced.Count > 0)
					{
						EngineOptimizer.OptimizeLimitsForTorque(ActiveEngines.Balanced, Vector3.zero, imbalance, MoI, 
						                                        out max_limit, out torque_error, out angle_error);
						imbalance = TorqueProps.CalculateImbalance(ActiveEngines.Manual, ActiveEngines.UnBalanced, ActiveEngines.Balanced);
					}
					if(ActiveEngines.Steering.Count > 0)
					{
						if(!EngineOptimizer.OptimizeLimitsForTorque(ActiveEngines.Steering, Vector3.zero, imbalance, MoI, 
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


		bool reset, init_engines, update_engines, update_stats;
		void Update()
		{
			if(EditorLogic.fetch == null || EditorLogic.fetch.ship == null) return;
			update_stats |= HaveSelectedPart;
			if(reset)
			{
				reset_highlightig();
				Available = false;
				Engines.Clear();
				CFG = null;
				reset = false;
			}
			if(init_engines)
			{
				reset_highlightig();
				if(UpdateEngines()) GetCFG();
				update_stats = true;
				init_engines = false;
			}
			if(update_engines)
			{
				reset_highlightig();
				if(UpdateEngines())
				{
					if(CFG != null) UpdateCFG();
					else GetCFG();
					if(CFG != null) CFG.ActiveProfile.Update(Engines);
				}
				update_stats = true;
				update_engines = false;
			}
			if(update_stats) UpdateShipStats();
			Available |= CFG != null && Engines.Count > 0;
			if(Available) CFG.GUIVisible = CFG.Enabled;
		}

		void DrawMainWindow(int windowID)
		{
			//help button
			if(GUI.Button(new Rect(WindowPos.width - 23f, 2f, 20f, 18f), 
			              new GUIContent("?", "Help"))) TCAManual.Toggle();
			GUILayout.BeginVertical();
			{
				if(Modules[typeof(MacroProcessor)])
				{
					if(TCAMacroEditor.Editing)
						GUILayout.Label("Edit Macros", Styles.inactive_button, GUILayout.ExpandWidth(true));
					else if(GUILayout.Button("Edit Macros", Styles.normal_button, GUILayout.ExpandWidth(true)))
						TCAMacroEditor.Edit(CFG);
				}
				GUILayout.BeginHorizontal();
				{
					GUILayout.BeginVertical();
					{
						GUILayout.BeginHorizontal();
						{
							Utils.ButtonSwitch("Enable TCA", ref CFG.Enabled, "", GUILayout.ExpandWidth(false));
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
					if(ActiveEngines.Count > 0)
					{
						GUILayout.Label(string.Format("TWR: {0:F2} ► {1:F2}", MinTWR, MaxTWR), Styles.fracStyle(Utils.Clamp(MinTWR-1, 0, 1)));
						GUILayout.Label(new GUIContent(string.Format("Balanced: {0:P1}", MinLimit),
						                               "The efficacy of the least efficient of balanced engines"),
						                Styles.fracStyle(MinLimit));
						Utils.ButtonSwitch("HL", ref show_imbalance, "Highlight engines with low efficacy deu to balancing");
					}
					else GUILayout.Label("No active engines", Styles.boxed_label);
				}
				GUILayout.EndHorizontal();
			}
			GUILayout.EndVertical();
			TooltipsAndDragWindow(WindowPos);
		}

		protected override bool can_draw()
		{ return Engines.Count > 0 && CFG != null; }

		static void highlight_engine(ThrusterWrapper e)
		{
			if(e.limit < 1) 
			{
				var lim = e.limit * e.limit;
				e.part.RecurseHighlight = false;
				e.part.highlightColor = lim < 0.5f? 
					Color.Lerp(Color.magenta, Color.yellow, lim/0.5f) :
					Color.Lerp(Color.yellow, Color.cyan, (lim-0.5f)/0.5f);
				e.part.SetHighlightType(Part.HighlightType.AlwaysOn);
			}
		}

		static void reset_highlightig()
		{
			EditorLogic.fetch.ship.Parts
				.Where(p => p.Modules.Contains<ModuleEngines>())
				.ForEach(p => p.SetHighlightDefault());
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
			if(show_imbalance && ActiveEngines.Count > 0)
			{
				Markers.DrawWorldMarker(CoM, 
				                        use_wet_mass? Color.yellow : Color.red, 
				                        use_wet_mass? "Center of Mass" : "Center of Dry Mass",
				                        CoM_Icon);
				Engines.ForEach(e => e.part.SetHighlightDefault());
				ActiveEngines.Balanced.ForEach(highlight_engine);
				ActiveEngines.Main.ForEach(highlight_engine);
				parts_highlighted = true;
			}
			else if(parts_highlighted)
			{
				reset_highlightig();
				parts_highlighted = false;
			}
		}
	}
}