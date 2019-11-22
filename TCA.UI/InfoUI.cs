using AT_Utils.UI;

namespace TCA.UI
{
    public class InfoUI : ScreenBoundRect
    {
        public ClickableLabel message;

        protected override void Start()
        {
            base.Start();
            gameObject.SetActive(!string.IsNullOrEmpty(message.text.text));
        }

        public void ClearMessage()
        {
            message.text.text = "";
            gameObject.SetActive(false);
        }

        public void SetMessage(string text)
        {
            message.text.text = text;
            gameObject.SetActive(true);
        }
    }
}
