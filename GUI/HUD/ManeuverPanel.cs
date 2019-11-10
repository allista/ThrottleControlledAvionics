//   ManeuverPanel.cs
//
//  Author:
//       Allis Tauri <allista@gmail.com>
//
//  Copyright (c) 2019 Allis Tauri
using AT_Utils.UI;
using TCA.UI;
namespace ThrottleControlledAvionics
{
    public class ManeuverPanel : ControlPanel<ManeuverUI>
    {
        TimeWarpControl WRP;
        ManeuverAutopilot MAN;

        protected override void init_controller()
        {
            if(WRP != null)
            {
                Controller.WarpToggle.gameObject.SetActive(true);
                Controller.WarpToggle.onValueChanged.AddListener(onWarpChanged);
            }
            else
                Controller.WarpToggle.gameObject.SetActive(false);
            if(MAN != null)
            {
                Controller.ManeuverSwitch.SetActive(true);
                Controller.ManeuverSwitch.Button.onClick.AddListener(onManeuverSwitch);
            }
            else
                Controller.ManeuverSwitch.SetActive(false);
            base.init_controller();
        }

        void onWarpChanged(bool state)
        {
            CFG.WarpToNode = state;
            if(!CFG.WarpToNode)
                WRP.AbortWarp();
        }

        void onManeuverSwitch()
        {
            CFG.AP1.XToggle(Autopilot1.Maneuver);
        }

        protected override void OnLateUpdate()
        {
            base.OnLateUpdate();
            if(WRP != null)
                Controller.WarpToggle.SetIsOnAndColorWithoutNotify(CFG.WarpToNode);
            if(MAN != null)
            {
                if(MAN.ControlsActive)
                {
                    Controller.ManeuverSwitch.SetManeuverActive(CFG.AP1[Autopilot1.Maneuver]);
                    Controller.ManeuverSwitch.SetActive(true);
                }
                else
                    Controller.ManeuverSwitch.SetActive(false);

            }
            if(VSL.Info.TTB >= 0 || VSL.Info.Countdown >= 0)
            {
                Controller.ManeuverInfo.SetActive(true);
                Controller.ManeuverInfo.UpdateInfo((float)VSL.Info.Countdown, VSL.Info.TTB);
            }
            else
                Controller.ManeuverInfo.SetActive(false);
        }
    }
}
