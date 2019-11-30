using TCA.UI;

namespace ThrottleControlledAvionics
{
    public class InfoPanel : ControlPanel<InfoUI>
    {
        public const string TEST_MSG = "Drag this to set the location of TCA status panel";
        void clearGUIStatus() => TCAGui.ClearStatus();

        protected override void init_controller()
        {
            Controller.message.onLabelClicked.AddListener(clearGUIStatus);
            if(!initialized)
                Controller.SetMessage(TEST_MSG);
            base.init_controller();
        }

        public override void Reset()
        {
            if(Controller != null)
                Controller.message.onLabelClicked.RemoveListener(clearGUIStatus);
            base.Reset();
        }

        public void ClearMessage()
        {
            if(Controller != null)
                Controller.ClearMessage();
        }

        protected override void OnLateUpdate()
        {
            base.OnLateUpdate();
            if(!string.IsNullOrEmpty(TCAGui.StatusMessage))
                Controller.SetMessage(TCAGui.StatusMessage);
        }
    }
}
