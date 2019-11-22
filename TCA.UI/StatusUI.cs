using AT_Utils.UI;
using UnityEngine;
using UnityEngine.UI;

namespace TCA.UI
{
    public class StatusUI : ScreenBoundRect
    {
        public Toggle soundToggle;
        public RectTransform indicatorsCommon;
        public RectTransform indicatorsOnPlanet;
        public RectTransform indicatorsInOrbit;

        public Indicator Ascending,
            LoosingAltitude,
            TerrainCollision,
            VesselCollision,
            LowControlAuthority,
            EnginesUnoptimized,
            ALT,
            VSC,
            VTOLMode,
            VTOLAssist,
            Stabilizing,
            NoEngines,
            NoEC;

        public void EnableSound(bool isOn)
        {
            if(Indicator.soundEnabled == isOn)
                return;
            Indicator.soundEnabled = isOn;
            foreach(var indicator in gameObject.GetComponentsInChildren<Indicator>())
                indicator.EnableSound(isOn);
        }

        private static void togglePanel(Component panel, bool enable)
        {
            if(enable)
                panel.gameObject.SetActive(true);
            else if(panel.gameObject.activeSelf)
            {
                foreach(var indicator in panel.gameObject.GetComponentsInChildren<Indicator>())
                    indicator.isOn = false;
                panel.gameObject.SetActive(false);
            }
        }

        public void ToggleOnPlanet(bool enable) => togglePanel(indicatorsOnPlanet, enable);
        public void ToggleInOrbit(bool enable) => togglePanel(indicatorsInOrbit, enable);

        protected override void Awake()
        {
            base.Awake();
            soundToggle.SetIsOnWithoutNotify(Indicator.soundEnabled);
            soundToggle.onValueChanged.AddListener(EnableSound);
        }

        private void OnDestroy()
        {
            soundToggle.onValueChanged.RemoveListener(EnableSound);
        }
    }
}
