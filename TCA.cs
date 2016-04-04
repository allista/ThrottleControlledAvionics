/* The GUI for ThrottleControlledAvionics.
 * Authors: Quinten Feys, Willem van Vliet, Allis Tauri
 * License: BY: Attribution-ShareAlike 3.0 Unported (CC BY-SA 3.0): http://creativecommons.org/licenses/by-sa/3.0/
 */

using System;
using System.Linq;
using System.Reflection;
using System.Collections.Generic;
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

		static SquadControl SQD;
		static AltitudeControl ALT;
		static VerticalSpeedControl VSC;
		static ThrottleControl THR;
		static VTOLAssist VLA;
		static FlightStabilizer STB;

		static List<FieldInfo> ModuleFields = typeof(ThrottleControlledAvionics)
			.GetFields(BindingFlags.Static|BindingFlags.NonPublic)
			.Where(fi => fi.FieldType.IsSubclassOf(typeof(TCAModule))).ToList();

		public override void LoadConfig()
		{
			base.LoadConfig ();
			TCA_Key = GUI_CFG.GetValue<KeyCode>(Utils.PropertyName(new {TCA_Key}), TCA_Key);
			update_configs();
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
			NavigationPanel.OnAwake();
			#if DEBUG
//			CheatOptions.InfiniteRCS  = true;
//			CheatOptions.InfiniteFuel = true;
			#endif
		}

		public override void OnDestroy()
		{
			base.OnDestroy();
			clear_fields();
			TCAToolbarManager.AttachTCA(null);
			GameEvents.onGameStateSave.Remove(SaveConfig);
			GameEvents.onVesselChange.Remove(onVesselChange);
			#if DEBUG
			ModuleTCA.prof.TreeReport();
			#endif
		}

		void onVesselChange(Vessel vsl)
		{
			if(vsl == null || vsl.parts == null) return;
			vessel = vsl; StartCoroutine(init_on_load());
		}

		public static void AttachTCA(ModuleTCA tca) 
		{ 
			if(tca.vessel != vessel || instance == null) return;
			instance.StartCoroutine(init_on_load()); 
		}

		static IEnumerator<YieldInstruction> init_on_load()
		{
			do {
				if(vessel == null) yield break;
				yield return null;
			} while(!vessel.loaded || vessel.packed);	
			init();
		}

		static void create_fields()
		{
			AllPanels.Clear();
			foreach(var fi in AllPanelFields)
			{
				var panel = TCA.CreateComponent(fi.FieldType) as ControlPanel;
				if(panel != null) AllPanels.Add(panel);
				fi.SetValue(null, panel);
			}
			ModuleFields.ForEach(fi => fi.SetValue(null, TCA.GetModule(fi.FieldType)));
		}

		static void clear_fields()
		{
			TCA = null;
			parts = null;
			AllPanels.ForEach(p => p.Reset());
			AllPanelFields.ForEach(fi => fi.SetValue(null, null));
			ModuleFields.ForEach(fi => fi.SetValue(null, null));
			AllPanels.Clear();
		}

		static bool init()
		{
			clear_fields();
			TCAToolbarManager.AttachTCA(null);
			TCA = ModuleTCA.AvailableTCA(vessel);
			if(TCA == null) return false;
			TCAToolbarManager.AttachTCA(TCA);
			create_fields();
			update_configs();
			parts = TCAModulesDatabase.GetPurchasedParts();
			return true;
		}

		public void Update()
		{
			if(TCA == null) return;
			if(!TCA.Available && !init()) return;
			if(!TCA.Controllable) return;
			AllPanels.ForEach(p => p.Update());
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
			if(CFG.Enabled && CFG.BlockThrottle && THR != null)
			{
				if(CFG.VF[VFlight.AltitudeControl]) 
				{ if(ALT != null) ALT.ProcessKeys(); }
				else if(VSC != null) VSC.ProcessKeys();
			}
		}
	}
}
