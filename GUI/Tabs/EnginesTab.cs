//   EnginesTab.cs
//
//  Author:
//       Allis Tauri <allista@gmail.com>
//
//  Copyright (c) 2017 Allis Tauri

using UnityEngine;
using AT_Utils;

namespace ThrottleControlledAvionics
{
    public class EnginesTab : ControlTab
    {
        public EnginesTab(ModuleTCA tca) : base(tca) {}

        public override bool Valid { get { return true; } }

        const int profiles_height = TCAGui.ControlsHeight-TCAGui.LineHeight*2;

        public override void Draw()
        {
            if(CFG.ActiveProfile.NumManual > 0)
            {
                if(GUILayout.Button(CFG.ShowManualLimits? "Hide Manual Engines" : "Show Manual Engines", 
                                    Styles.active_button,
                                    GUILayout.ExpandWidth(true)))
                    CFG.ShowManualLimits = !CFG.ShowManualLimits;
                if(CFG.ShowManualLimits) 
                    CFG.EnginesProfiles.DrawManual(Utils.ClampH(TCAGui.LineHeight*CFG.ActiveProfile.NumManual*2, profiles_height));
            }
            GUILayout.BeginHorizontal();
            if(Utils.ButtonSwitch("Smart Engines", ref CFG.UseSmartEngines, 
                                  "Group engines by thrust direction and automatically use appropriate group for a meneuver", GUILayout.ExpandWidth(true)))
            { if(CFG.UseSmartEngines && !CFG.SmartEngines) CFG.SmartEngines.XOn(SmartEnginesMode.Best); }
            if(Utils.ButtonSwitch("Rotation", CFG.UseSmartEngines && CFG.SmartEngines[SmartEnginesMode.Closest], 
                                  "Minimize amount of rotation needed for maneuvering", GUILayout.ExpandWidth(true)))
            { if(CFG.UseSmartEngines) CFG.SmartEngines.XOnIfNot(SmartEnginesMode.Closest); }
            if(Utils.ButtonSwitch("Time", CFG.UseSmartEngines && CFG.SmartEngines[SmartEnginesMode.Fastest], 
                                  "Minimize both rotation and thrusting time", GUILayout.ExpandWidth(true)))
            { if(CFG.UseSmartEngines) CFG.SmartEngines.XOnIfNot(SmartEnginesMode.Fastest); }
            if(Utils.ButtonSwitch("Efficiency", CFG.UseSmartEngines && CFG.SmartEngines[SmartEnginesMode.Best], 
                                  "Minimize rotation, thrusting time and needed fuel", GUILayout.ExpandWidth(true)))
            { if(CFG.UseSmartEngines) CFG.SmartEngines.XOnIfNot(SmartEnginesMode.Best); }
            GUILayout.EndHorizontal();
            CFG.EnginesProfiles.Draw(profiles_height);
        }
    }
}

