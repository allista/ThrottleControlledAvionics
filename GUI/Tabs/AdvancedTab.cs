//   AdvancedTab.cs
//
//  Author:
//       Allis Tauri <allista@gmail.com>
//
//  Copyright (c) 2017 Allis Tauri

using System;
using System.Linq;
using UnityEngine;
using AT_Utils;

namespace ThrottleControlledAvionics
{
    public class AdvancedTab : ControlTab
    {
        public AdvancedTab(ModuleTCA tca) : base(tca) { }

        public override bool Valid { get { return true; } }

        ThrottleControl THR;
        VTOLControl VTOL;
        VTOLAssist VLA;
        FlightStabilizer STB;
        TranslationControl TRA;
        private HorizontalSpeedControl HSC;
        CollisionPreventionSystem CPS;

        public bool SelectingKey;

        NamedConfig selected_config;
        string config_name = string.Empty;
        readonly DropDownList named_configs = new DropDownList();

        private readonly FloatField MinHorizontalAccel = new FloatField(min:0);

        public override void Init()
        {
            base.Init();
            MinHorizontalAccel.Value = CFG.MinHorizontalAccel;
        }

        #region Configs Selector
        public void UpdateNamedConfigs()
        {
            var configs = TCAScenario.NamedConfigs.Keys.ToList();
            var first = named_configs.Items.Count == 0;
            configs.Add(string.Empty); named_configs.Items = configs;
            if(first) named_configs.SelectItem(configs.Count - 1);
        }

        void SelectConfig_start()
        {
            named_configs.styleListBox = Styles.list_box;
            named_configs.styleListItem = Styles.list_item;
            named_configs.windowRect = UI.WindowPos;
            if(TCAScenario.NamedConfigs.Count > 0)
                named_configs.DrawBlockingSelector();
        }

        void SelectConfig()
        {
            if(TCAScenario.NamedConfigs.Count == 0)
                GUILayout.Label("", Styles.white, GUILayout.ExpandWidth(true));
            else
            {
                named_configs.DrawButton();
                var new_config = TCAScenario.GetConfig(named_configs.SelectedIndex);
                if(new_config != selected_config)
                {
                    selected_config = new_config;
                    config_name = selected_config != null ? selected_config.Name : string.Empty;
                }
            }
        }

        void SelectConfig_end()
        {
            if(TCAScenario.NamedConfigs.Count == 0) return;
            named_configs.DrawDropDown();
            named_configs.CloseOnOutsideClick();
        }
        #endregion

        public void Toggles()
        {
            GUILayout.BeginHorizontal();
            if(VTOL != null)
            {
                if(Utils.ButtonSwitch("VTOL Mode", CFG.CTRL[ControlMode.VTOL],
                                      "Keyboard controls thrust direction instead of torque", GUILayout.ExpandWidth(true)))
                    CFG.CTRL.XToggle(ControlMode.VTOL);
            }
            if(VLA != null)
                Utils.ButtonSwitch("VTOL Assist", ref CFG.VTOLAssistON,
                                   "Assist with vertical takeoff and landing", GUILayout.ExpandWidth(true));
            if(STB != null)
                Utils.ButtonSwitch("Stabilizer", ref CFG.StabilizeFlight,
                                   "Try to stabilize flight if spinning uncontrollably", GUILayout.ExpandWidth(true));
            if(CPS != null)
                Utils.ButtonSwitch("CPS", ref CFG.UseCPS,
                                   "Enable Collision Prevention System", GUILayout.ExpandWidth(true));
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
            if(GUILayout.Button(new GUIContent("Modules", "Show TCA modules installed on this ship"),
                                Styles.active_button, GUILayout.ExpandWidth(true)))
                UI.ModulesGraph.Toggle();
            GUILayout.EndHorizontal();
            GUILayout.BeginHorizontal();
            if(HSC != null)
            {
                Utils.ButtonSwitch("Hor. Thrust",
                    ref CFG.UseHorizontalThrust,
                    "Use maneuver engines to provide thrust for horizontal flight",
                    GUILayout.ExpandWidth(true));
                if(MinHorizontalAccel.Draw("kN/t",
                    field_width: 50,
                    suffix_tooltip:
                    "Maneuver engines will be used as horizontal thrusters only if they produce more thrust than this.")
                )
                    CFG.MinHorizontalAccel = MinHorizontalAccel;
                if(TRA != null)
                    Utils.ButtonSwitch("RCS Translation",
                        ref CFG.CorrectWithTranslation,
                        "Use RCS to correct horizontal velocity",
                        GUILayout.ExpandWidth(true));
            }
            Utils.ButtonSwitch("RCS Rotation", ref CFG.RotateWithRCS,
                "Use RCS for attitude control", GUILayout.ExpandWidth(true));
            GUILayout.EndHorizontal();
        }

        void ConfigsGUI()
        {
            GUILayout.BeginVertical();
            GUILayout.Label("Manage named configurations", Styles.label, GUILayout.ExpandWidth(true));
            GUILayout.BeginHorizontal();
            GUILayout.Label("Name:", GUILayout.ExpandWidth(false));
            config_name = GUILayout.TextField(config_name, GUILayout.ExpandWidth(true), GUILayout.MinWidth(50));
            if(TCAScenario.NamedConfigs.ContainsKey(config_name))
            {
                if(GUILayout.Button(new GUIContent("Overwrite", "Overwrite selected configuration with the current one"),
                                    Styles.danger_button, GUILayout.ExpandWidth(false)))
                    TCAScenario.SaveNamedConfig(config_name, CFG, true);
            }
            else if(GUILayout.Button(new GUIContent("Add", "Save current configuration"),
                                     Styles.open_button, GUILayout.ExpandWidth(false))
                    && config_name != string.Empty)
            {
                TCAScenario.SaveNamedConfig(config_name, CFG);
                UpdateNamedConfigs();
                named_configs.SelectItem(TCAScenario.NamedConfigs.IndexOfKey(config_name));
            }
            SelectConfig();
            if(GUILayout.Button(new GUIContent("Load", "Load selected configuration"),
                                Styles.active_button, GUILayout.ExpandWidth(false))
               && selected_config != null)
                CFG.Copy(selected_config);
            if(GUILayout.Button(new GUIContent("Delete", "Delete selected configuration"),
                                Styles.danger_button, GUILayout.ExpandWidth(false))
               && selected_config != null)
            {
                TCAScenario.NamedConfigs.Remove(selected_config.Name);
                named_configs.SelectItem(named_configs.SelectedIndex - 1);
                UpdateNamedConfigs();
                selected_config = null;
            }
            GUILayout.EndHorizontal();
            GUILayout.EndVertical();
        }

        void ControllerProperties()
        {
            Utils.ButtonSwitch("Autotune engines' controller parameters", ref CFG.AutoTune, "", GUILayout.ExpandWidth(true));
            if(CFG.AutoTune) return;
            //steering modifiers
            GUILayout.BeginHorizontal();
            CFG.SteeringGain = Utils.FloatSlider("Steering Gain", CFG.SteeringGain, 0, 1, "P1");
            CFG.PitchYawLinked = GUILayout.Toggle(CFG.PitchYawLinked, "Link Pitch&Yaw", GUILayout.ExpandWidth(false));
            GUILayout.EndHorizontal();
            GUILayout.BeginHorizontal();
            if(CFG.PitchYawLinked && !CFG.AutoTune)
            {
                CFG.SteeringModifier.x = Utils.FloatSlider("Pitch&Yaw", CFG.SteeringModifier.x, 0, 1, "P1");
                CFG.SteeringModifier.z = CFG.SteeringModifier.x;
            }
            else
            {
                CFG.SteeringModifier.x = Utils.FloatSlider("Pitch", CFG.SteeringModifier.x, 0, 1, "P1");
                CFG.SteeringModifier.z = Utils.FloatSlider("Yaw", CFG.SteeringModifier.z, 0, 1, "P1");
            }
            CFG.SteeringModifier.y = Utils.FloatSlider("Roll", CFG.SteeringModifier.y, 0, 1, "P1");
            GUILayout.EndHorizontal();
            //engines
            CFG.Engines.DrawControls("Engines Controller", EngineOptimizer.C.MaxP, EngineOptimizer.C.MaxI);
        }

        public override void Draw()
        {
            GUILayout.BeginHorizontal();
            GUILayout.Label(UI.Title, Styles.label, GUILayout.ExpandWidth(true));
            if(GUILayout.Button(new GUIContent("Reload", "Reload TCA settings from file"),
                                Styles.active_button, GUILayout.ExpandWidth(true)))
            {
                Globals.Load();
                TCA.OnReloadGlobals();
            }
            GUILayout.EndHorizontal();
            GUILayout.BeginHorizontal();
            //change key binding
            if(GUILayout.Button(SelectingKey ? new GUIContent("HotKey: ?", "Choose new TCA hotkey") :
                                new GUIContent(string.Format("HotKey: {0}", UI.TCA_Key), "Select TCA Hotkey"),
                                SelectingKey ? Styles.enabled_button : Styles.active_button,
                                GUILayout.ExpandWidth(true)))
            {
                SelectingKey = true;
                Utils.Message("Press a key that will toggle TCA.\n" +
                              "Press BACKSPACE to remove TCA hotkey.\n" +
                              "Press ESCAPE to cancel.");
            }
            if(Utils.ButtonSwitch("AutoSave", ref Globals.Instance.AutosaveBeforeLanding,
                                  "Automatically save the game before executing complex autopilot programs",
                                  GUILayout.ExpandWidth(true)))
                Globals.Save(":AutosaveBeforeLanding");
            if(Utils.ButtonSwitch(Globals.Instance.UseStockAppLauncher ? "Launcher" : "Toolbar",
                                  ref Globals.Instance.UseStockAppLauncher,
                                  "Use stock AppLauncher or Toolbar plugin?",
                                  GUILayout.ExpandWidth(true)))
            {
                TCAAppToolbar.Init();
                Globals.Save(":UseStockAppLauncher");
            }
            Utils.ButtonSwitch("AutoShow", ref UI.ShowOnHover,
                               "Show collapsed TCA window when mouse hovers over it", 
                               GUILayout.ExpandWidth(true));
            if(GUILayout.Button(new GUIContent("InfoPanel", 
                "Show test info message to change the position of the info panel"),
                GUILayout.ExpandWidth(true)) 
               && string.IsNullOrEmpty(TCAGui.StatusMessage))
                Status(InfoPanel.TEST_MSG);
            GUILayout.EndHorizontal();
            Toggles();
            if(THR != null)
            {
                GUILayout.BeginHorizontal();
                CFG.ControlSensitivity = Utils.FloatSlider("Keyboard sensitivity", CFG.ControlSensitivity, 0.001f, 0.05f, "P2");
                GUILayout.EndHorizontal();
            }
            ControllerProperties();
            ConfigsGUI();
        }

        public override void Update()
        {
            var e = Event.current;
            if(e.isKey)
            {
                if(e.keyCode == KeyCode.Escape)
                {
                    Message("TCA: hotkey selection canceled.");
                }
                else if(e.keyCode == KeyCode.Backspace || e.character == '\b')
                {
                    UI.TCA_Key = KeyCode.None;
                    Message("TCA: hotkey removed!");
                }
                else
                {
                    //try to get the keycode if the Unity provided us only with the character
                    if(e.keyCode == KeyCode.None && char.IsLetterOrDigit(e.character))
                    {
                        var ec = new string(e.character, 1).ToUpper();
                        if(char.IsDigit(e.character)) ec = "Alpha" + ec;
                        try { e.keyCode = (KeyCode)Enum.Parse(typeof(KeyCode), ec); }
                        catch(Exception ex)
                        { Utils.Log("TCA GUI: exception caught while trying to set hotkey:\n{}", ex); }
                    }
                    if(e.keyCode == KeyCode.None)
                        Utils.Message("Unable to convert '{0}' to keycode.\n" +
                                      "Please, try an alphabet or numeric character.",
                                      e.character);
                    else
                        UI.TCA_Key = e.keyCode;
                    Utils.Log("TCA: new key slected: {}", UI.TCA_Key);
                }
                SelectingKey = false;
            }
            if(!CFG.MinHorizontalAccel.Equals(MinHorizontalAccel))
                MinHorizontalAccel.Value = CFG.MinHorizontalAccel;
        }
    }
}

