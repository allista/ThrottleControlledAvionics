using AT_Utils.UI;
using UnityEngine.UI;

namespace TCA.UI
{
    public class StatusUI : ScreenBoundRect
    {
        public Toggle soundToggle;
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

        public ClickableLabel message;

        public void EnableSound(bool isOn)
        {
            if(Indicator.soundEnabled == isOn)
                return;
            Indicator.soundEnabled = isOn;
            foreach(var indicator in gameObject.GetComponentsInChildren<Indicator>())
                indicator.EnableSound(isOn);
        }

        public void ClearMessage()
        {
            message.text.text = "";
            message.gameObject.SetActive(false);
        }

        public void SetMessage(string text)
        {
            message.text.text = text;
            message.gameObject.SetActive(true);
        }

        protected override void Awake()
        {
            base.Awake();
            soundToggle.SetIsOnWithoutNotify(Indicator.soundEnabled);
            soundToggle.onValueChanged.AddListener(EnableSound);
            ClearMessage();
        }

        private void OnDestroy()
        {
            soundToggle.onValueChanged.RemoveListener(EnableSound);
        }
    }
}
