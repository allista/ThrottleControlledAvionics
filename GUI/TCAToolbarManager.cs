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
    /// <summary>
    /// TCA toolbar manager. It is needed becaus in KSP-1.0+ the ApplicationLauncher
    /// works differently: it only fires OnReady event at MainMenu and the first 
    /// time the Spacecenter is loaded. Thus we need to register the AppButton only 
    /// once and then just hide and show it using VisibleScenes, not removing it.
    /// IMHO, this is a bug in the RemoveModApplication method, cause if you use
    /// Add/RemoveModApp repeatedly, the buttons are duplicated each time.
    /// </summary>
    [KSPAddon(KSPAddon.Startup.EveryScene, false)]
    public class TCAToolbarManager : MonoBehaviour
    {
        static TCAToolbarManager Instance;
        //icons
        const string AL_ICON_ON  = "ThrottleControlledAvionics/Icons/applauncher-icon_on";
        const string AL_ICON_OFF = "ThrottleControlledAvionics/Icons/applauncher-icon_off";
        const string AL_ICON_NC  = "ThrottleControlledAvionics/Icons/applauncher-icon_noCharge";
        const string AL_ICON_MAN = "ThrottleControlledAvionics/Icons/applauncher-icon_MAN";

        const string TB_ICON_ON  = "ThrottleControlledAvionics/Icons/toolbar-icon_on";
        const string TB_ICON_OFF = "ThrottleControlledAvionics/Icons/toolbar-icon_off";
        const string TB_ICON_NC  = "ThrottleControlledAvionics/Icons/toolbar-icon_noCharge";
        const string TB_ICON_MAN = "ThrottleControlledAvionics/Icons/toolbar-icon_MAN";

        static Texture textureOn;
        static Texture textureOff;
        static Texture textureNoCharge;
        static Texture textureMan;
        //buttons
        const ApplicationLauncher.AppScenes SCENES = ApplicationLauncher.AppScenes.FLIGHT|
            ApplicationLauncher.AppScenes.VAB|ApplicationLauncher.AppScenes.SPH|
            ApplicationLauncher.AppScenes.SPACECENTER|ApplicationLauncher.AppScenes.MAPVIEW;
        static IButton TCAToolbarButton;
        static ApplicationLauncherButton TCAButton;
        //TCA isntance
        static ModuleTCA TCA;

        void Awake()
        {
            if(Instance != null) 
            {
                Destroy(this);
                return;
            }
            DontDestroyOnLoad(this);
            Instance = this;
            init();
        }

        void init()
        {
            //setup toolbar/applauncher button
            if(ToolbarManager.ToolbarAvailable && !Globals.Instance.UseStockAppLauncher)
            { 
                Utils.Log("Using Blizzy's toolbar");
                if(TCAToolbarButton == null) AddToolbarButton(); 
                if(TCAButton != null) TCAButton.VisibleInScenes = ApplicationLauncher.AppScenes.NEVER;
            }
            else 
            {
                Utils.Log("Using stock AppLauncher");
                if(TCAButton == null)
                {
                    if(HighLogic.CurrentGame != null && ApplicationLauncher.Ready) AddAppLauncherButton();
                    else GameEvents.onGUIApplicationLauncherReady.Add(AddAppLauncherButton);
                }
                else TCAButton.VisibleInScenes = SCENES;
                if(TCAToolbarButton != null)
                {
                    TCAToolbarButton.Destroy();
                    TCAToolbarButton = null;
                }
            }
        }
        public static void Init() { if(Instance != null) Instance.init(); }

        //need to be instance method for Event.Add to work
        void AddAppLauncherButton()
        {
            if(!ApplicationLauncher.Ready || TCAButton != null) return;
            Utils.Log("Adding AppLauncher button");
            textureOn       = TextureCache.GetTexture(AL_ICON_ON);
            textureOff      = TextureCache.GetTexture(AL_ICON_OFF);
            textureNoCharge = TextureCache.GetTexture(AL_ICON_NC);
            textureMan      = TextureCache.GetTexture(AL_ICON_MAN);
            TCAButton = ApplicationLauncher.Instance.AddModApplication(
                onAppLaunchToggleOn,
                onAppLaunchToggleOff,
                null, null, null, null,
                SCENES,
                textureMan);
        }

        static void AddToolbarButton()
        {
            TCAToolbarButton = ToolbarManager.Instance.add("ThrottleControlledAvionics", "ThrottleControlledAvionicsButton");
            TCAToolbarButton.TexturePath = TB_ICON_MAN;
            TCAToolbarButton.ToolTip     = "Throttle Controlled Avionics";
            TCAToolbarButton.Visibility  = new GameScenesVisibility(GameScenes.FLIGHT, GameScenes.EDITOR, GameScenes.SPACECENTER);
            TCAToolbarButton.Visible     = true;
            TCAToolbarButton.OnClick    += onToolbarToggle;
        }

        static void onToolbarToggle(ClickEvent e) 
        { 
            if(TCA != null) TCAGui.ToggleWithButton(TCAButton);
            else if(HighLogic.LoadedSceneIsEditor && TCAGuiEditor.Available)
                TCAGuiEditor.ToggleWithButton(TCAButton);
            else TCAManual.ToggleWithButton(TCAButton);
        }
        static void onAppLaunchToggleOn() { onToolbarToggle(null); }
        static void onAppLaunchToggleOff() { onToolbarToggle(null); }

        public static void AttachTCA(ModuleTCA tca) 
        { 
            TCA = tca;
            if(TCA == null) SetDefaultButton();
        }

        public static void SetDefaultButton()
        {
            if(TCAToolbarButton != null) TCAToolbarButton.TexturePath = TB_ICON_MAN;
            if(TCAButton != null) TCAButton.SetTexture(textureMan);
        }

        public void Update()
        { 
            if(TCA != null)
            {
                if(TCAToolbarButton != null)
                {
                    if(TCA.IsStateSet(TCAState.Enabled))
                        TCAToolbarButton.TexturePath = TCA.State != TCAState.NoEC? TB_ICON_ON : TB_ICON_NC;
                    else TCAToolbarButton.TexturePath = TB_ICON_OFF;
                }
                if(TCAButton != null) 
                {
                    if(TCA.IsStateSet(TCAState.Enabled))
                        TCAButton.SetTexture(TCA.State != TCAState.NoEC? textureOn : textureNoCharge);
                    else TCAButton.SetTexture(textureOff);
                }
            }
            else if(HighLogic.LoadedSceneIsEditor && TCAGuiEditor.Available)
            {
                if(TCAToolbarButton != null) TCAToolbarButton.TexturePath = TB_ICON_ON;
                if(TCAButton != null) TCAButton.SetTexture(textureOn);
            }
            else SetDefaultButton();
        }
    }
}

