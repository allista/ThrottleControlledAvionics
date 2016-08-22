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
using AT_Utils;

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
		BearingControl BRC;
		Radar RAD;

		public OnPlanetPanel(ModuleTCA tca) : base(tca) {}

		public override void Draw()
		{
			if(!VSL.OnPlanet) return;
			GUILayout.BeginHorizontal();
			GUILayout.Label(new GUIContent(
				string.Format("{0} ▲{1} ►{2}", 
				              Utils.formatBigValue(VSL.Altitude.Current, "m"), 
				              Utils.formatBigValue(VSL.VerticalSpeed.Display, "m/s", "+0.0;-0.0;+0.0"), 
				              Utils.formatBigValue(VSL.HorizontalSpeed.Absolute, "m/s")), 
				"Altitude, Vertical speed, Horizontal speed."),
			                Styles.boxed_label, GUILayout.Width(240));
			if(ALT != null && CFG.VF[VFlight.AltitudeControl]) ALT.Draw();
			else if(VSC != null) VSC.Draw();
			if(THR != null) THR.Draw();
			GUILayout.EndHorizontal();
			GUILayout.BeginHorizontal();
			if(HSC != null)
			{
				if(Utils.ButtonSwitch("Stop", CFG.HF[HFlight.Stop], "Kill horizontal velocity", GUILayout.Width(50)))
				{
					if(CFG.HF[HFlight.Stop]) apply_cfg(cfg => cfg.HF.OffIfOn(HFlight.Stop));
					else apply_cfg(cfg => { cfg.HF.XOn(HFlight.Stop); cfg.StopMacro(); });
				}
				if(ANC != null && 
				   Utils.ButtonSwitch("Anchor", CFG.Nav.Any(Navigation.AnchorHere, Navigation.Anchor), 
				                      "Hold current position", GUILayout.Width(60)))
					apply_cfg(cfg => cfg.Nav.XToggle(Navigation.AnchorHere));
				if(Utils.ButtonSwitch("Level", CFG.HF[HFlight.Level], "Point thrust vertically", GUILayout.Width(50)))
					apply_cfg(cfg => cfg.HF.XToggle(HFlight.Level));
			}
			if(LND != null)
			{
//				LND.RadarBeam();//debug
				if(Utils.ButtonSwitch("Land", CFG.AP1[Autopilot1.Land], "Try to land on a nearest flat surface", GUILayout.Width(50)))
				{
					var state = !CFG.AP1[Autopilot1.Land];
					if(state) { follow_me(); CFG.AP1.XOn(Autopilot1.Land); }
					else apply_cfg(cfg => cfg.AP1.XOffIfOn(Autopilot1.Land));
				}
			}
			if(ALT != null)
			{
				if(Utils.ButtonSwitch("Hover", CFG.VF[VFlight.AltitudeControl], "Maintain altitude", GUILayout.Width(60)))
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
					#if DEBUG
//					RAD.DrawDebugLines();
					#endif
				}
			}
			if(CC != null)
			{
				if(Utils.ButtonSwitch("Cruise", CFG.HF[HFlight.CruiseControl], "Maintain course and speed", GUILayout.Width(60)))
				{
					CFG.HF.XToggle(HFlight.CruiseControl);
					if(CFG.HF[HFlight.CruiseControl]) follow_me();
				}
			}
			if(BRC != null) BRC.Draw();
			GUILayout.EndHorizontal();
		}
	}
}

