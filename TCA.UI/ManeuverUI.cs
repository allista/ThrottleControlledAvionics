//   ManeuverUI.cs
//
//  Author:
//       Allis Tauri <allista@gmail.com>
//
//  Copyright (c) 2019 Allis Tauri
using AT_Utils.UI;
using UnityEngine;
using UnityEngine.UI;

namespace TCA.UI
{
    public class ManeuverUI : ScreenBoundRect
    {
        public Toggle WarpToggle;
        public ManeuverInfo ManeuverInfo;
        public ManeuverSwitch ManeuverSwitch;
    }

    public class ManeuverInfo : PanelledUI
    {
        public Text Countdown, ThrustDuration;

        public void UpdateInfo(float countdown, float ttb)
        {
            Countdown.text = countdown >= 0 ? string.Format("{0:F1}s", countdown) : "";
            Countdown.color = countdown > 10 ? Colors.Neutral : Colors.Danger;
            if(ttb >= 0 && ttb < float.MaxValue)
            {
                ThrustDuration.gameObject.SetActive(true);
                ThrustDuration.text = string.Format("{0:F1}s", ttb);
            }
            else
                ThrustDuration.gameObject.SetActive(false);
        }
    }

    public class ManeuverSwitch : PanelledUI
    {
        public Button Button;
        public Text ButtonText;
        bool maneuverActive;

        void Awake()
        {
            SetManeuverActive(false, true);
        }

        void onButtonColorChange(Color color)
        {
            ButtonText.color = color;
        }

        public void SetManeuverActive(bool active, bool force = false)
        {
            if(force || maneuverActive != active)
            {
                maneuverActive = active;
                if(active)
                {
                    ButtonText.text = "Abort Maneuver";
                    Colors.Active.removeOnColorChangeListner(onButtonColorChange);
                    Colors.Danger.addOnColorChangeListner(onButtonColorChange);
                    onButtonColorChange(Colors.Danger);
                }
                else
                {
                    ButtonText.text = "Execute Node";
                    Colors.Danger.removeOnColorChangeListner(onButtonColorChange);
                    Colors.Active.addOnColorChangeListner(onButtonColorChange);
                    onButtonColorChange(Colors.Active);
                }
            }
        }
    }
}
