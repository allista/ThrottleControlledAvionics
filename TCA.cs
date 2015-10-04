/* The GUI for ThrottleControlledAvionics.
 * Authors: Quinten Feys, Willem van Vliet, Allis Tauri
 * License: BY: Attribution-ShareAlike 3.0 Unported (CC BY-SA 3.0): http://creativecommons.org/licenses/by-sa/3.0/
 */

using System;
using System.Linq;
using UnityEngine;

namespace ThrottleControlledAvionics
{
	[KSPAddon(KSPAddon.Startup.Flight, false)]
	public partial class ThrottleControlledAvionics : AddonWindowBase<ThrottleControlledAvionics>
	{
		static Vessel vessel;
		static Part part;
		static ModuleTCA TCA;
		static VesselWrapper VSL { get { return TCA.VSL; } }
		static VesselConfig CFG { get { return TCA.CFG; } }
		static ActionDamper UpDamper = new ActionDamper(0.1);
		static ActionDamper DownDamper = new ActionDamper(0.1);

		public override void LoadConfig()
		{
			base.LoadConfig ();
			HelpWindow = GUI_CFG.GetValue<Rect>(Utils.PropertyName(new {HelpWindow}), HelpWindow);
			TCA_Key = GUI_CFG.GetValue<KeyCode>(Utils.PropertyName(new {TCA_Key}), TCA_Key);
			UpDamper.Period = GLB.KeyRepeatTime;
			DownDamper.Period = GLB.KeyRepeatTime;
			updateConfigs();
		}

		public override void SaveConfig(ConfigNode node = null)
		{
			GUI_CFG.SetValue(Utils.PropertyName(new {HelpWindow}), HelpWindow);
			GUI_CFG.SetValue(Utils.PropertyName(new {TCA_Key}), TCA_Key);
			base.SaveConfig(node);
		}

		public override void Awake()
		{
			base.Awake();
			GameEvents.onGameStateSave.Add(SaveConfig);
			GameEvents.onVesselChange.Add(onVesselChange);
			WayPointMarker = GameDatabase.Instance.GetTexture(WPM_ICON, false);
			PathNodeMarker = GameDatabase.Instance.GetTexture(PN_ICON, false);
			RenderingManager.AddToPostDrawQueue(1, WaypointOverlay);
			#if DEBUG
			CheatOptions.InfiniteRCS  = true;
			CheatOptions.InfiniteFuel = true;
			#endif
		}

		public override void OnDestroy ()
		{
			base.OnDestroy();
			TCAToolbarManager.AttachTCA(null);
			GameEvents.onGameStateSave.Remove(SaveConfig);
			GameEvents.onVesselChange.Remove(onVesselChange);
			RenderingManager.RemoveFromPostDrawQueue(1, WaypointOverlay);
			#if DEBUG
			ModuleTCA.prof.TreeReport();
			#endif
		}

		void onVesselChange(Vessel vsl)
		{
			if(vsl == null || vsl.parts == null) return;
			vessel = vsl; init();
		}

		public static void AttachTCA(ModuleTCA tca) { if(tca.vessel == vessel) init(); }

		static bool init()
		{
			TCA = null; TCAToolbarManager.AttachTCA(null); 
			part = vessel.parts.FirstOrDefault(p => p.HasModule<ModuleTCA>());
			if(part == null) return false;
			TCA = part.Modules.OfType<ModuleTCA>().FirstOrDefault();
			if(TCA == null || !TCA.Available) { TCA = null; return false; }
			TCAToolbarManager.AttachTCA(TCA);
			updateConfigs();
			UpDamper.Reset();
			DownDamper.Reset();
			return true;
		}

		public void Update()
		{
			if(TCA == null) return;
			if(!TCA.Available && !init()) return;
			TCAToolbarManager.UpdateToolbarButton();
			if(!TCA.Controllable) return;
			if(selecting_key)
			{ 
				var e = Event.current;
				if(e.isKey)
				{
					if(e.keyCode != KeyCode.Escape)
					{
						//try to get the keycode if the Unity provided us only with the character
						if(e.keyCode == KeyCode.None && e.character >= 'a' && e.character <= 'z')
						{
							var ec = new string(e.character, 1).ToUpper();
							try { e.keyCode = (KeyCode)Enum.Parse(typeof(KeyCode), ec); }
							catch {}
						}
						if(e.keyCode == KeyCode.None) 
							ScreenMessages
								.PostScreenMessage(string.Format("Unable to convert '{0}' to keycode.\nPlease, try an alphabet character.", e.character), 
							    	                             5, ScreenMessageStyle.UPPER_CENTER);
						else TCA_Key = e.keyCode;
						Utils.Log("TCA: new key slected: {0}", TCA_Key);
					}
					selecting_key = false;
				}
			}
			else if(Input.GetKeyDown(TCA_Key)) TCA.ToggleTCA();
			if(CFG.Enabled && CFG.BlockThrottle)
			{
				if(CFG.VF[VFlight.AltitudeControl])
				{
					var old_altitude = CFG.DesiredAltitude;
					if(GameSettings.THROTTLE_UP.GetKey())
						CFG.DesiredAltitude = Mathf.Lerp(CFG.DesiredAltitude, 
						                                 CFG.DesiredAltitude+10, 
						                                 CFG.VSControlSensitivity);
					else if(GameSettings.THROTTLE_DOWN.GetKey())
						CFG.DesiredAltitude = Mathf.Lerp(CFG.DesiredAltitude,
						                                 CFG.DesiredAltitude-10, 
						                                 CFG.VSControlSensitivity);
					else if(GameSettings.THROTTLE_FULL.GetKey())
						UpDamper.Run(() => CFG.DesiredAltitude = CFG.DesiredAltitude+10);
					else if(GameSettings.THROTTLE_CUTOFF.GetKey())
						DownDamper.Run(() => CFG.DesiredAltitude = CFG.DesiredAltitude-10);
					if(!old_altitude.Equals(CFG.DesiredAltitude))
						apply_cfg(cfg => cfg.DesiredAltitude = CFG.DesiredAltitude);
				}
				else
				{
					var old_cutoff = CFG.VerticalCutoff;
					if(GameSettings.THROTTLE_UP.GetKey())
						CFG.VerticalCutoff = Mathf.Lerp(CFG.VerticalCutoff, 
						                                GLB.VSC.MaxSpeed, 
						                                CFG.VSControlSensitivity);
					else if(GameSettings.THROTTLE_DOWN.GetKey())
						CFG.VerticalCutoff = Mathf.Lerp(CFG.VerticalCutoff, 
						                                -GLB.VSC.MaxSpeed, 
						                                CFG.VSControlSensitivity);
					else if(GameSettings.THROTTLE_FULL.GetKeyDown())
						CFG.VerticalCutoff = GLB.VSC.MaxSpeed;
					else if(GameSettings.THROTTLE_CUTOFF.GetKeyDown())
						CFG.VerticalCutoff = -GLB.VSC.MaxSpeed;
					if(!old_cutoff.Equals(CFG.VerticalCutoff))
						apply_cfg(cfg => cfg.VerticalCutoff = CFG.VerticalCutoff);
				}
			}
		}
	}
}
