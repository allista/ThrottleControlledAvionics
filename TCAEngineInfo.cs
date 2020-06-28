//  Author:
//       Allis Tauri <allista@gmail.com>
//
//  Copyright (c) 2015 Allis Tauri
//
// This work is licensed under the Creative Commons Attribution-ShareAlike 4.0 International License. 
// To view a copy of this license, visit http://creativecommons.org/licenses/by-sa/4.0/ 
// or send a letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.

using System;
using AT_Utils;

namespace ThrottleControlledAvionics
{
    public enum TCARole
    {
        MAIN,
        MANEUVER,
        MANUAL,
        BALANCE,
        UNBALANCE
    }

    [Flags]
    public enum ManeuverMode { ALL = TORQUE | TRANSLATION, TORQUE = 1, TRANSLATION = 1 << 1 }

    public class TCAEngineInfo : PartModule
    {
        public static readonly OrderedDict<TCARole, string> Roles =
            new OrderedDict<TCARole, string>
            {
                { TCARole.MAIN, "Thrust & Maneuver" },
                { TCARole.BALANCE, "Thrust" },
                { TCARole.MANEUVER, "Maneuver" },
                { TCARole.UNBALANCE, "UnBalanced Thrust" },
                { TCARole.MANUAL, "Manual Control" }
            };

        public static readonly OrderedDict<ManeuverMode, string> Modes =
            new OrderedDict<ManeuverMode, string>
            {
                { ManeuverMode.ALL, "Rotation & Translation" },
                { ManeuverMode.TORQUE, "Rotation" },
                { ManeuverMode.TRANSLATION, "Translation" }
            };

        [KSPField(isPersistant = true)] public TCARole Role = TCARole.MAIN;

        [KSPField(isPersistant = true)] public ManeuverMode Mode = ManeuverMode.ALL;

        [UI_ChooseOption]
        [KSPField(isPersistant = true,
            guiActive = true,
            guiActiveEditor = true,
            guiName = "Engine Group:",
            groupName = "TCAEngineInfo",
            groupDisplayName = "TCA")]
        public int group;

        public override void OnLoad(ConfigNode node)
        {
            base.OnLoad(node);
            // legacy config conversion
            if(node.HasValue("role")
               && int.TryParse(node.GetValue("role"), out var role))
                Role = (TCARole)role;
            update_events();
        }

        private static void setup_groups(UI_ChooseOption chooser)
        {
            if(chooser == null)
                return;
            chooser.options = new string[Globals.Instance.MaxManualGroups];
            chooser.options[0] = "OFF";
            for(var i = 1; i < Globals.Instance.MaxManualGroups; i++)
                chooser.options[i] = $"G{i:D}";
        }

        public override void OnStart(StartState state)
        {
            var f = Fields[nameof(group)];
            setup_groups(f.uiControlEditor as UI_ChooseOption);
            setup_groups(f.uiControlFlight as UI_ChooseOption);
            update_events();
        }

        public void SetGroup(int newGroup)
        {
            group = newGroup >= 0 ? newGroup : 0;
        }

        public void SetRole(TCARole role)
        {
            Role = role;
            update_events();
        }

        public void SetMode(ManeuverMode mode)
        {
            Mode = mode;
            update_events();
        }

        public void SetRoleAndMode(TCARole role, ManeuverMode mode)
        {
            Role = role;
            Mode = mode;
            update_events();
        }

        [KSPEvent(guiActive = true,
            guiActiveEditor = true,
            guiName = "TCA Role",
            groupName = "TCAEngineInfo",
            groupDisplayName = "TCA",
            active = true)]
        public void SwitchRole()
        {
            if(!HighLogic.LoadedSceneIsEditor && group > 0)
            {
                Utils.Message("Cannot change the role of an engine belonging to a group.\n"
                              + "Use in-flight group controls instead.");
                return;
            }
            Role = Roles.Next(Role);
            update_events();
            applyToCounterparts(e => e.SetRole(Role));
        }

        [KSPEvent(guiActive = true,
            guiActiveEditor = true,
            guiName = "TCA Maneuver Mode",
            groupName = "TCAEngineInfo",
            groupDisplayName = "TCA",
            active = false)]
        public void SwitchMode()
        {
            if(!HighLogic.LoadedSceneIsEditor && group > 0)
            {
                Utils.Message("Cannot change the mode of an engine belonging to a group.\n"
                              + "Use in-flight group controls instead.");
                return;
            }
            Mode = Modes.Next(Mode);
            update_events();
            applyToCounterparts(e => e.SetMode(Mode));
        }

        private void applyToCounterparts(Action<TCAEngineInfo> action)
        {
            if(!Globals.Instance.RoleSymmetryInFlight
               && HighLogic.LoadedSceneIsFlight)
                return;
            foreach(var cp in part.symmetryCounterparts)
            {
                var info = cp.Modules.GetModule<TCAEngineInfo>();
                if(info != null
                   && (HighLogic.LoadedSceneIsEditor
                       || info.group == 0))
                    action(info);
            }
        }

        private void update_events()
        {
            Events[nameof(SwitchRole)].guiName = $"Role: {Roles[Role]}";
            Events[nameof(SwitchMode)].guiName = $"Mode: {Modes[Mode]}";
            Events[nameof(SwitchMode)].active = Role == TCARole.MANEUVER;
            if(HighLogic.LoadedSceneIsEditor)
                GameEvents.onEditorShipModified.Fire(EditorLogic.fetch.ship);
        }
    }
}
