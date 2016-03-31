//  Author:
//       Allis Tauri <allista@gmail.com>
//
//  Copyright (c) 2016 Allis Tauri
//
// This work is licensed under the Creative Commons Attribution-ShareAlike 4.0 International License. 
// To view a copy of this license, visit http://creativecommons.org/licenses/by-sa/4.0/ 
// or send a letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.
//
using System;
using UnityEngine;

namespace ThrottleControlledAvionics
{
	public class OnPlanetPanel : ControlPanel
	{
		VerticalSpeedControl VSC;
		HorizontalSpeedControl HSC;
		AltitudeControl ALT;
		ThrottleControl THR;
		Anchor ANC;
		AutoLander LND;
		CruiseControl CC;
		Radar RAD;

		public OnPlanetPanel(ModuleTCA tca) : base(tca) {}

		public override void Draw()
		{
			if(!VSL.OnPlanet) return;
			GUILayout.BeginHorizontal();
			if(ALT != null && CFG.VF[VFlight.AltitudeControl]) ALT.Draw();
			else if(VSC != null) VSC.Draw();
			if(THR != null) THR.Draw();
			GUILayout.EndHorizontal();
			GUILayout.BeginHorizontal();
			if(HSC != null)
			{
				if(GUILayout.Button(new GUIContent("Stop", "Kill horizontal velocity"), 
				                    CFG.HF[HFlight.Stop]? Styles.green_button : Styles.yellow_button,
				                    GUILayout.Width(50)))
				{
					if(CFG.HF[HFlight.Stop]) apply_cfg(cfg => cfg.HF.OffIfOn(HFlight.Stop));
					else apply_cfg(cfg => { cfg.HF.XOn(HFlight.Stop); cfg.StopMacro(); });
				}
				if(ANC != null && 
				   GUILayout.Button(new GUIContent("Anchor", "Hold current position"), 
				                    CFG.Nav.Any(Navigation.AnchorHere, Navigation.Anchor)? 
				                    Styles.green_button : Styles.yellow_button,
				                    GUILayout.Width(60)))
					apply_cfg(cfg => cfg.Nav.XToggle(Navigation.AnchorHere));
				if(GUILayout.Button(new GUIContent("Level", "Point thrust vertically"), 
				                    CFG.HF[HFlight.Level]? 
				                    Styles.green_button : Styles.yellow_button,
				                    GUILayout.Width(50)))
					apply_cfg(cfg => cfg.HF.XToggle(HFlight.Level));
			}
			if(LND != null)
			{
				if(GUILayout.Button(new GUIContent("Land", "Try to land on a nearest flat surface"), 
				                    CFG.AP1[Autopilot1.Land]? Styles.green_button : Styles.yellow_button,
				                    GUILayout.Width(50)))
				{
					var state = !CFG.AP1[Autopilot1.Land];
					if(state) { follow_me(); CFG.AP1.XOn(Autopilot1.Land); }
					else apply_cfg(cfg => cfg.AP1.XOffIfOn(Autopilot1.Land));
				}
			}
			if(CC != null)
			{
				if(GUILayout.Button(new GUIContent("Cruise", "Maintain course and speed"), 
				                    CFG.HF[HFlight.CruiseControl]? Styles.green_button : Styles.yellow_button,
				                    GUILayout.Width(60)))
				{
					CFG.HF.XToggle(HFlight.CruiseControl);
					if(CFG.HF[HFlight.CruiseControl]) follow_me();
				}
			}
			if(ALT != null)
			{
				if(GUILayout.Button(new GUIContent("Hover", "Maintain altitude"), 
				                    CFG.VF[VFlight.AltitudeControl]? Styles.green_button : Styles.yellow_button,
				                    GUILayout.Width(60)))
					apply_cfg(cfg => cfg.VF.XToggle(VFlight.AltitudeControl));
				if(RAD != null)
				{
					if(Utils.ButtonSwitch("Follow Terrain", ref CFG.AltitudeAboveTerrain, 
					                      "Keep altitude above the ground", GUILayout.ExpandWidth(false)))
							apply(tca => 
						{
							var alt = tca.GetModule<AltitudeControl>();
							if(alt != null) alt.SetAltitudeAboveTerrain(CFG.AltitudeAboveTerrain);
						});
				}
			}
			GUILayout.EndHorizontal();
		}
	}
}

