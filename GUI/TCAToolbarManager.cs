/* Name: Throttle Controlled Avionics, Fork by Allis Tauri
 *
 * Authors: Quinten Feys & Willem van Vliet & Allis Tauri
 * License: BY: Attribution-ShareAlike 3.0 Unported (CC BY-SA 3.0): 
 * http://creativecommons.org/licenses/by-sa/3.0/
 * 
 */

using UnityEngine;

namespace ThrottleControlledAvionics
{
	/// <summary>
	/// TCA toolbar manager. It is needed as since KSP-1.0 the ApplicationLauncher
	/// works differently: it only fires OnReady event at MainMenu and the first 
	/// time the Spacecenter is loaded. Thus we need to register the AppButton only 
	/// once and then just hide and show it using VisibleScenes, not removing int.
	/// IMHO, this is a bug in the RemoveModApplication method, cause if you use
	/// Add/RemoveModApp repeatedly, the buttons are duplicated each time.
	/// </summary>
	[KSPAddon(KSPAddon.Startup.MainMenu, true)]
	public class TCAToolbarManager : MonoBehaviour
	{
		//icons
		const string ICON_ON  = "ThrottleControlledAvionics/Icons/icon_button_on";
		const string ICON_OFF = "ThrottleControlledAvionics/Icons/icon_button_off";
		const string ICON_NC  = "ThrottleControlledAvionics/Icons/icon_button_noCharge";
		//buttons
		const ApplicationLauncher.AppScenes SCENES = ApplicationLauncher.AppScenes.FLIGHT|ApplicationLauncher.AppScenes.VAB|ApplicationLauncher.AppScenes.SPH;
		static IButton TCAToolbarButton;
		static ApplicationLauncherButton TCAButton;
		static Texture textureOn;
		static Texture textureOff;
		static Texture textureNoCharge;
		//TCA isntance
		static ModuleTCA TCA;

		void Awake()
		{
			//setup toolbar/applauncher button
			if(TCAToolbarButton == null &&
				ToolbarManager.ToolbarAvailable && 
			   !TCAScenario.Globals.UseStockAppLauncher)
			{
				Utils.Log("Using Blizzy's toolbar");
				TCAToolbarButton = ToolbarManager.Instance.add("ThrottleControlledAvionics", "ThrottleControlledAvionicsButton");
				TCAToolbarButton.TexturePath = ICON_OFF;
				TCAToolbarButton.ToolTip     = "Throttle Controlled Avionics";
				TCAToolbarButton.Visibility  = new GameScenesVisibility(GameScenes.FLIGHT, GameScenes.EDITOR);
				TCAToolbarButton.Visible     = false;
				TCAToolbarButton.OnClick    += onToolbarToggle;
			}
			else 
			{
				Utils.Log("Using stock AppLauncher");
				textureOn = GameDatabase.Instance.GetTexture(ICON_ON, false);
				textureOff = GameDatabase.Instance.GetTexture(ICON_OFF, false);
				textureNoCharge = GameDatabase.Instance.GetTexture(ICON_NC, false);
				GameEvents.onGUIApplicationLauncherReady.Add(AddAppLauncherButton);
//				GameEvents.onGUIApplicationLauncherDestroyed.Add(RemoveAppLauncherButton);
			}
		}

		//left here in case the RemoveModApp will be fixed
//		public void OnDestroy() 
//		{ 
//			Utils.Log("Removing AppLauncher|Toolbar button");
//			GameEvents.onGUIApplicationLauncherReady.Remove(AddAppLauncherButton);
//			GameEvents.onGUIApplicationLauncherDestroyed.Remove(RemoveAppLauncherButton);
//			if(TCAToolbarButton != null) TCAToolbarButton.Destroy();
//		}
//
//		void RemoveAppLauncherButton()
//		{
//			Utils.Log("Removing AppLauncher button");//debug
//			if(TCAButton != null)
//				ApplicationLauncher.Instance.RemoveModApplication(TCAButton);
//		}

		void AddAppLauncherButton()
		{
			if(!ApplicationLauncher.Ready) return;
			if(TCAButton == null)
			{
				Utils.Log("Adding AppLauncher button");
				TCAButton = ApplicationLauncher.Instance.AddModApplication(
					onAppLaunchToggleOn,
					onAppLaunchToggleOff,
					null, null, null, null,
					ApplicationLauncher.AppScenes.NEVER,
					textureOff);
			}
		}

		void onToolbarToggle(ClickEvent e) 
		{ 
			if(TCA != null && TCA.Controllable) 
				TCA.CFG.GUIVisible = !TCA.CFG.GUIVisible; 
			EnginesProfileEditor.GUIVisible = !EnginesProfileEditor.GUIVisible; 
		}

		void onAppLaunchToggleOn() 
		{ 
			if(TCA != null && TCA.Controllable) TCA.CFG.GUIVisible = true; 
			EnginesProfileEditor.GUIVisible = true;
		}

		void onAppLaunchToggleOff() 
		{ 
			if(TCA != null && TCA.Controllable) TCA.CFG.GUIVisible = false; 
			EnginesProfileEditor.GUIVisible = false;
		}

		public static void AttachTCA(ModuleTCA tca)
		{
			TCA = tca;
			ShowButton(TCA != null);
		}

		public static void ShowButton(bool show = true)
		{
			if(show)
			{
				if(TCAButton != null) TCAButton.VisibleInScenes = SCENES;
				if(TCAToolbarButton != null) TCAToolbarButton.Visible = true;
			}
			else
			{
				if(TCAButton != null) TCAButton.VisibleInScenes = ApplicationLauncher.AppScenes.NEVER;
				if(TCAToolbarButton != null) TCAToolbarButton.Visible = false;
			}
		}

		public static void SetDefaultButton()
		{
			if(TCAToolbarButton != null) TCAToolbarButton.TexturePath = ICON_OFF;
			if(TCAButton != null) TCAButton.SetTexture(textureOff);
		}

		public static void UpdateToolbarButton()
		{ 
			if(TCA == null) return;
			if(TCAToolbarButton != null)
			{
				if(TCA.IsStateSet(TCAState.Enabled))
					TCAToolbarButton.TexturePath = TCA.State != TCAState.NoEC? ICON_ON : ICON_NC;
				else TCAToolbarButton.TexturePath = ICON_OFF;
			}
			if(TCAButton != null) 
			{
				if(TCA.IsStateSet(TCAState.Enabled))
					TCAButton.SetTexture(TCA.State != TCAState.NoEC? textureOn : textureNoCharge);
				else TCAButton.SetTexture(textureOff);
			}
		}
	}
}

