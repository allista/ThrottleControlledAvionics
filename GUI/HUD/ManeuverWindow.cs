//  Author:
//       Allis Tauri <allista@gmail.com>
//
//  Copyright (c) 2016 Allis Tauri
//
// This work is licensed under the Creative Commons Attribution-ShareAlike 4.0 International License. 
// To view a copy of this license, visit http://creativecommons.org/licenses/by-sa/4.0/ 
// or send a letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.

using UnityEngine;
using AT_Utils;

namespace ThrottleControlledAvionics
{
    public class ManeuverWindow : ControlWindow
    {
        class InfoWindow : ControlSubwindow
        {
            protected override bool can_draw()
            { return VSL.Info.Countdown >= 0 || VSL.Info.TTB >= 0; }

            protected override void MainWindow(int windowID)
            { 
                GUILayout.BeginVertical(Styles.white_on_black);
                VSL.Info.Draw();
                GUILayout.EndVertical();
                TooltipManager.GetTooltip();
            }
        }

        TimeWarpControl WRP;
        ManeuverAutopilot MAN;

        public ManeuverWindow() { Anchor = AnchorPosition.BottomLeft; }

        protected override bool can_draw () { return base.can_draw() && AllModules.Count > 0; }

        protected override void DrawContent()
        {
            GUILayout.BeginHorizontal();
            if(WRP != null) WRP.Draw();
            if(MAN != null && VSL.HasManeuverNode) MAN.Draw();
            if(VSL.Info.TTB >= 0 || VSL.Info.Countdown >= 0)
                VSL.Info.Draw();
            GUILayout.EndHorizontal();
        }
    }
}
