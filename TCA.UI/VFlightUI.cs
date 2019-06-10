//   VFlightUI.cs
//
//  Author:
//       Allis Tauri <allista@gmail.com>
//
//  Copyright (c) 2019 Allis Tauri

using UnityEngine;
using UnityEngine.UI;

using AT_Utils.UI;

namespace TCA.UI
{
    public class VFlightUI : ScreenBoundRect
    {
        public VSC_UI VSC;
        public ALT_UI ALT;

        public Toggle hoverButton;
        public Toggle followTerrainButton;
        public Toggle autoThrottleButton;

        public Text Altitude, vSpeed, vSpeedLabel, hSpeed;

        protected override void Awake()
        {
            base.Awake();
            hoverButton.onValueChanged.AddListener(EnableALT);
            EnableALT(false);
        }

        void OnDestroy()
        {
            hoverButton.onValueChanged.RemoveListener(EnableALT);
        }

        public void EnableALT(bool state)
        {
            ALT.SetActive(state);
            VSC.SetActive(!state);
        }

        public void UpdateInfo(float altitude, float v_speed, float h_speed)
        {
            Altitude.text = FormatUtils.formatBigValue(altitude, "m");
            vSpeed.text = FormatUtils.formatBigValue(v_speed, "m/s", "0.0;0.0;0.0");
            vSpeedLabel.text = v_speed > 0 ? "▲" : "▼";
            hSpeed.text = FormatUtils.formatBigValue(h_speed, "m/s");
        }
    }

    public class VSC_UI : BoundedFloatValueUI
    {
        public Slider slider;
        public Text display;

        public float min;
        public override float Min { get => min; set { min = value; slider.minValue = min; } }

        public float max;
        public override float Max { get => max; set { max = value; slider.maxValue = max; } }

        readonly FloatEvent _onValueChange = new FloatEvent();
        public override FloatEvent onValueChanged => _onValueChange;

        void update_display()
        {
            slider.value = value;
            if(value < max)
                display.text = FormatUtils.formatBigValue(value, "m/s", "+0.0;-0.0; 0.0");
            else
                display.text = "OFF";
        }

        void Awake()
        {
            value = max;
            slider.minValue = min;
            slider.maxValue = max;
            update_display();
            slider.onValueChanged.AddListener(changeValueAndNotify);
        }

        void OnDestroy()
        {
            slider.onValueChanged.RemoveListener(changeValueAndNotify);
        }

        public override bool SetValueWithoutNotify(float newValue)
        {
            if(!value.Equals(newValue))
            {
                if(Mathf.Abs(newValue) < 0.1f)
                    newValue = 0;
                value = newValue;
                update_display();
                return true;
            }
            return false;
        }
    }

    public class ALT_UI : FloatValueUI
    {
        public FloatController Altitude;
        public TooltipTrigger tooltip;
        bool altitudeAboveGround;

        public override FloatEvent onValueChanged => Altitude.onValueChanged;

        void Awake()
        {
            SetAltitudeAboveGround(!altitudeAboveGround);
        }

        public void SetAltitudeAboveGround(bool above)
        {
            if(altitudeAboveGround == above) return;
            if(above)
            {
                tooltip.text = "Desired altitude is above the ground";
                Colors.Danger.removeOnColorChangeListner(onInputColorChange);
                Colors.Good.addOnColorChangeListner(onInputColorChange);
                onInputColorChange(Colors.Good);
            }
            else
            {
                tooltip.text = "Warning! Desired altitude is below the ground";
                Colors.Good.removeOnColorChangeListner(onInputColorChange);
                Colors.Danger.addOnColorChangeListner(onInputColorChange);
                onInputColorChange(Colors.Danger);
            }
            altitudeAboveGround = above;
        }

        void onInputColorChange(Color color) => Altitude.input.textComponent.color = color;

        public override bool SetValueWithoutNotify(float newValue) => Altitude.SetValueWithoutNotify(newValue);
    }
}
