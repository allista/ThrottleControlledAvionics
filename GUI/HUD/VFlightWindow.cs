//  Author:
//       Allis Tauri <allista@gmail.com>
//
//  Copyright (c) 2016 Allis Tauri
//
// This work is licensed under the Creative Commons Attribution-ShareAlike 4.0 International License. 
// To view a copy of this license, visit http://creativecommons.org/licenses/by-sa/4.0/ 
// or send a letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.
//

using AT_Utils;
using UnityEngine;

namespace ThrottleControlledAvionics
{
    public class VFlightWindow : ControlWindow
    {
        VerticalSpeedControl VSC;
        AltitudeControl ALT;
        ThrottleControl THR;
        Radar RAD;

        public VFlightWindow() { Anchor = AnchorPosition.TopLeft; }

        protected override bool can_draw() { return base.can_draw() && VSL.OnPlanet && AllModules.Count > 0; }

        protected override void DrawContent()
        {
            GUILayout.BeginVertical();
            GUILayout.Label(CFG.Enabled?
                            new GUIContent(
                                string.Format("{0} {1} ►{2}", 
                                              Utils.formatBigValue(VSL.Altitude.Current, "m"), 
                                              Utils.formatBigValue(VSL.VerticalSpeed.Display, "m/s", "▲ 0.0;▼ 0.0;▲ 0.0"), 
                                              Utils.formatBigValue(VSL.HorizontalSpeed.Absolute, "m/s")), 
                                "Altitude, Vertical speed, Horizontal speed.") : new GUIContent(""),
                            Styles.boxed_label, GUILayout.MinWidth(240), GUILayout.ExpandWidth(true));
            GUILayout.BeginHorizontal();
            if(ALT != null && CFG.VF[VFlight.AltitudeControl]) ALT.Draw();
            else if(VSC != null) VSC.Draw();
            GUILayout.EndHorizontal();
            GUILayout.BeginHorizontal();
            if(ALT != null)
            {
                if(Utils.ButtonSwitch("Hover", CFG.VF[VFlight.AltitudeControl], "Maintain altitude", GUILayout.ExpandWidth(true)))
                    TCA.SquadConfigAction(cfg => cfg.VF.XToggle(VFlight.AltitudeControl));
                if(RAD != null)
                {
                    if(Utils.ButtonSwitch("Follow Terrain", ref CFG.AltitudeAboveTerrain, 
                        "Keep altitude above the ground", GUILayout.ExpandWidth(true)))
                        TCA.SquadAction(tca => 
                            {
                                var alt = tca.GetModule<AltitudeControl>();
                                if(alt != null) alt.SetAltitudeAboveTerrain(CFG.AltitudeAboveTerrain);
                            });
//                    RAD.DrawDebugLines();//debug
                }
            }
            if(THR != null) THR.Draw();
            GUILayout.EndHorizontal();
            GUILayout.EndVertical();
        }
    }
}
