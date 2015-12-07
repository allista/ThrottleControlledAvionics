/* The GUI for ThrottleControlledAvionics.
 * Authors: Quinten Feys, Willem van Vliet, Allis Tauri
 * License: BY: Attribution-ShareAlike 3.0 Unported (CC BY-SA 3.0): http://creativecommons.org/licenses/by-sa/3.0/
 */

using System;
using UnityEngine;

namespace ThrottleControlledAvionics
{
	[KSPAddon(KSPAddon.Startup.Flight, false)]
	public partial class ThrottleControlledAvionics : AddonWindowBase<ThrottleControlledAvionics>
	{
		static Vessel vessel;
		public static ModuleTCA TCA { get; private set; }
		public static VesselWrapper VSL { get { return TCA.VSL; } }
		public static VesselConfig CFG { get { return TCA.CFG; } }
		static ActionDamper UpDamper = new ActionDamper(0.1);
		static ActionDamper DownDamper = new ActionDamper(0.1);

		public override void LoadConfig()
		{
			base.LoadConfig ();
			TCA_Key = GUI_CFG.GetValue<KeyCode>(Utils.PropertyName(new {TCA_Key}), TCA_Key);
			UpDamper.Period = GLB.KeyRepeatTime;
			DownDamper.Period = GLB.KeyRepeatTime;
			updateConfigs();
		}

		public override void SaveConfig(ConfigNode node = null)
		{
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
			TCA = ModuleTCA.AvailableTCA(vessel);
			if(TCA == null) return false;
			TCAToolbarManager.AttachTCA(TCA);
			updateConfigs();
			UpDamper.Reset();
			DownDamper.Reset();
			return true;
		}

		#region Sqad Mode
		static bool squad_mode;

		static void apply_to_others(Action<ModuleTCA> action)
		{
			if(TCA.CFG.Squad == 0 || !squad_mode) return;
			for(int i = 0, num_vessels = FlightGlobals.Vessels.Count; i < num_vessels; i++)
			{
				var v = FlightGlobals.Vessels[i];
				if(v == null || v == VSL.vessel || !v.loaded) continue;
				var tca = ModuleTCA.EnabledTCA(v);
				if(tca == null || !tca.Controllable || 
				   tca.CFG.Squad == 0 || tca.CFG.Squad != TCA.CFG.Squad) continue;
				//try to reach packed vessels
				if(v.packed) 
				{
					var dist = (v.transform.position-vessel.transform.position).magnitude;
					var sit = v.vesselRanges.GetSituationRanges(v.situation);
					sit.pack = dist*1.5f;
					sit.unpack = dist*1.2f;
					v.GoOffRails();
				}
				action(tca);
			}
			ScreenMessages.PostScreenMessage("Squad Action Executed", 5, ScreenMessageStyle.UPPER_CENTER);
		}

		public static void Apply(Action<ModuleTCA> action)
		{
			if(TCA == null || action == null) return;
			action(TCA);
			apply_to_others(action);
		}

		static void apply_cfg(Action<VesselConfig> action)
		{
			if(TCA == null || action == null) return;
			action(TCA.CFG);
			apply_to_others(tca => action(tca.CFG));
		}

		static void sync_cfg(Action<VesselConfig> action)
		{
			if(TCA == null || action == null) return;
			apply_to_others(tca => action(tca.CFG));
		}

		static void follow_me()
		{
			Apply(tca => 
			{
				if(tca == TCA) return;
				tca.vessel.targetObject = TCA.vessel;
				tca.CFG.Nav.XOn(Navigation.FollowTarget);
			});
		}

		static void update_altitude()
		{
			if(s_altitude == null || !altitude.Equals(CFG.DesiredAltitude))
				s_altitude = CFG.DesiredAltitude.ToString("F1");
			altitude = CFG.DesiredAltitude;
		}

		static void set_altitude()
		{
			apply_cfg(cfg =>
			{
				cfg.DesiredAltitude = altitude;
				s_altitude = altitude.ToString("F1");
				cfg.BlockThrottle |= CFG.VSCIsActive;
			});
		}

		static void set_vspeed(float vspeed)
		{
			apply_cfg(cfg =>
			{
				cfg.VerticalCutoff = vspeed;
				cfg.BlockThrottle |= CFG.VSCIsActive;
			});
		}
		#endregion

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
							catch(Exception ex) { Utils.Log("TCA GUI: exception caught while trying to set hotkey:\n{0}", ex); }
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
					update_altitude();
					if(GameSettings.THROTTLE_UP.GetKey())
						altitude = Mathf.Lerp(CFG.DesiredAltitude, 
						                      CFG.DesiredAltitude+10, 
						                      CFG.VSControlSensitivity);
					else if(GameSettings.THROTTLE_DOWN.GetKey())
						altitude = Mathf.Lerp(CFG.DesiredAltitude,
						                      CFG.DesiredAltitude-10, 
						                      CFG.VSControlSensitivity);
					else if(GameSettings.THROTTLE_FULL.GetKey())
						UpDamper.Run(() => altitude = altitude+10);
					else if(GameSettings.THROTTLE_CUTOFF.GetKey())
						DownDamper.Run(() => altitude = altitude-10);
					if(!altitude.Equals(CFG.DesiredAltitude)) set_altitude();
				}
				else
				{
					var cutoff = CFG.VerticalCutoff;
					if(GameSettings.THROTTLE_UP.GetKey())
						cutoff = Mathf.Lerp(CFG.VerticalCutoff, 
						                    GLB.VSC.MaxSpeed, 
						                    CFG.VSControlSensitivity);
					else if(GameSettings.THROTTLE_DOWN.GetKey())
						cutoff = Mathf.Lerp(CFG.VerticalCutoff, 
						                    -GLB.VSC.MaxSpeed, 
						                    CFG.VSControlSensitivity);
					else if(GameSettings.THROTTLE_FULL.GetKeyDown())
						cutoff = GLB.VSC.MaxSpeed;
					else if(GameSettings.THROTTLE_CUTOFF.GetKeyDown())
						cutoff = -GLB.VSC.MaxSpeed;
					if(!cutoff.Equals(CFG.VerticalCutoff)) set_vspeed(cutoff);
				}
			}
		}
	}
}
