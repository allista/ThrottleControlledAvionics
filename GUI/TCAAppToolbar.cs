//  Author:
//       Allis Tauri <allista@gmail.com>
//
//  Copyright (c) 2015 Allis Tauri
//
// This work is licensed under the Creative Commons Attribution-ShareAlike 4.0 International License. 
// To view a copy of this license, visit http://creativecommons.org/licenses/by-sa/4.0/ 
// or send a letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.

using UnityEngine;
using KSP.UI.Screens;
using AT_Utils;

namespace ThrottleControlledAvionics
{
    [KSPAddon(KSPAddon.Startup.MainMenu, true)]
    public class TCAAppToolbar : AppToolbar<TCAAppToolbar>
    {
        ModuleTCA TCA;

        const string AL_ICON_ON = "ThrottleControlledAvionics/Icons/applauncher-icon_on";
        const string AL_ICON_OFF = "ThrottleControlledAvionics/Icons/applauncher-icon_off";
        const string AL_ICON_NC = "ThrottleControlledAvionics/Icons/applauncher-icon_noCharge";
        const string AL_ICON_MAN = "ThrottleControlledAvionics/Icons/applauncher-icon_MAN";

        const string TB_ICON_ON = "ThrottleControlledAvionics/Icons/toolbar-icon_on";
        const string TB_ICON_OFF = "ThrottleControlledAvionics/Icons/toolbar-icon_off";
        const string TB_ICON_NC = "ThrottleControlledAvionics/Icons/toolbar-icon_noCharge";
        const string TB_ICON_MAN = "ThrottleControlledAvionics/Icons/toolbar-icon_MAN";

        Texture textureOn;
        Texture textureOff;
        Texture textureNoCharge;
        Texture textureMan;

        protected override string TB_ICON => TB_ICON_MAN;
        protected override string AL_ICON => AL_ICON_MAN;

        protected override ApplicationLauncher.AppScenes AL_SCENES =>
        ApplicationLauncher.AppScenes.FLIGHT | ApplicationLauncher.AppScenes.MAPVIEW |
            ApplicationLauncher.AppScenes.SPH | ApplicationLauncher.AppScenes.VAB;

        protected override GameScenes[] TB_SCENES =>
        new[] { GameScenes.FLIGHT, GameScenes.EDITOR };

        protected override string button_tooltip => "Throttle Controlled Avionics";

        protected override bool ForceAppLauncher => Globals.Instance.UseStockAppLauncher;

        protected override void on_app_launcher_init()
        {
            base.on_app_launcher_init();
            textureOn = TextureCache.GetTexture(AL_ICON_ON);
            textureOff = TextureCache.GetTexture(AL_ICON_OFF);
            textureNoCharge = TextureCache.GetTexture(AL_ICON_NC);
            textureMan = TextureCache.GetTexture(AL_ICON_MAN);
        }

        protected override void onLeftClick()
        {
            if(TCA != null) TCAGui.ToggleWithButton(ALButton);
            else if(HighLogic.LoadedSceneIsEditor && TCAGuiEditor.Available)
                TCAGuiEditor.ToggleWithButton(ALButton);
            else TCAManual.ToggleWithButton(ALButton);
        }

        void attach_TCA(ModuleTCA tca)
        {
            TCA = tca;
            if(TCA == null) 
                set_default_button();
        }
        public static void AttachTCA(ModuleTCA tca) => Instance?.attach_TCA(tca);

        void set_default_button()
        {
            if(TBButton != null) TBButton.TexturePath = TB_ICON_MAN;
            if(ALButton != null) ALButton.SetTexture(textureMan);
        }

        void Update()
        {
            if(TCA != null)
            {
                if(TBButton != null)
                {
                    if(TCA.IsStateSet(TCAState.Enabled))
                        TBButton.TexturePath = TCA.State != TCAState.NoEC ? TB_ICON_ON : TB_ICON_NC;
                    else TBButton.TexturePath = TB_ICON_OFF;
                }
                if(ALButton != null)
                {
                    if(TCA.IsStateSet(TCAState.Enabled))
                        ALButton.SetTexture(TCA.State != TCAState.NoEC ? textureOn : textureNoCharge);
                    else ALButton.SetTexture(textureOff);
                }
            }
            else if(HighLogic.LoadedSceneIsEditor && TCAGuiEditor.Available)
            {
                if(TBButton != null) TBButton.TexturePath = TB_ICON_ON;
                if(ALButton != null) ALButton.SetTexture(textureOn);
            }
            else set_default_button();
        }
    }
}

