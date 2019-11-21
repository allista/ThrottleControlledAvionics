using AT_Utils;
using TCA.UI;
using UnityEngine;

namespace ThrottleControlledAvionics
{
    public class StatusPanel : ControlPanel<StatusUI>
    {
        [ConfigOption] private Vector3 messagePos = Vector3.zero;

        private VerticalSpeedControl VSC;
        private AltitudeControl ALT;
        private VTOLAssist VTOL_assist;
        private VTOLControl VTOL_control;
        private FlightStabilizer Stabilizer;
        private CollisionPreventionSystem CPS;
        private Radar RAD;

        public override void SyncState()
        {
            base.SyncState();
            if(Controller != null)
                messagePos = Controller.messagePanel.anchoredPosition;
        }

        protected override void init_controller()
        {
            if(VSC == null)
                Controller.VSC.Show(false);
            if(ALT == null)
                Controller.ALT.Show(false);
            if(VTOL_assist == null)
                Controller.VTOLAssist.Show(false);
            if(VTOL_control == null)
                Controller.VTOLMode.Show(false);
            if(Stabilizer == null)
                Controller.Stabilizing.Show(false);
            if(CPS == null)
                Controller.VesselCollision.Show(false);
            if(RAD == null)
                Controller.TerrainCollision.Show(false);
            Controller.message.onLabelClicked.AddListener(clearGUIStatus);
            if(initialized)
                Controller.messagePanel.anchoredPosition = messagePos;
            base.init_controller();
        }

        protected override void onGamePause()
        {
            base.onGamePause();
            if(Controller != null)
                Controller.EnableSound(false);
        }

        protected override void onGameUnpause()
        {
            base.onGameUnpause();
            if(Controller != null)
                Controller.EnableSound(true);
        }

        public override void Reset()
        {
            if(Controller != null)
                Controller.message.onLabelClicked.RemoveListener(clearGUIStatus);
            base.Reset();
        }

        void clearGUIStatus() => TCAGui.ClearStatus();

        public void ClearMessage()
        {
            if(Controller != null)
                Controller.ClearMessage();
        }

        protected override void OnLateUpdate()
        {
            base.OnLateUpdate();
            // disable sub-panels depending on situation
            Controller.ToggleOnPlanet(VSL.OnPlanet);
            Controller.ToggleInOrbit(VSL.InOrbit);
            // set states of the indicators
            Controller.Ascending.isOn = TCA.IsStateSet(TCAState.Ascending);
            Controller.LoosingAltitude.isOn = TCA.IsStateSet(TCAState.LoosingAltitude);
            Controller.TerrainCollision.isOn = TCA.IsStateSet(TCAState.GroundCollision);
            Controller.VesselCollision.isOn = TCA.IsStateSet(TCAState.VesselCollision);
            Controller.LowControlAuthority.isOn = !VSL.Controls.HaveControlAuthority;
            Controller.EnginesUnoptimized.isOn = TCA.IsStateSet(TCAState.Unoptimized);
            Controller.VSC.isOn = TCAModule.ExistsAndActive(VSC);
            Controller.ALT.isOn = TCAModule.ExistsAndActive(ALT);
            Controller.VTOLMode.isOn = TCAModule.ExistsAndActive(VTOL_control);
            Controller.VTOLAssist.isOn = TCA.IsStateSet(TCAState.VTOLAssist);
            Controller.Stabilizing.isOn = TCA.IsStateSet(TCAState.StabilizeFlight);
            Controller.NoEngines.isOn = TCA.IsStateSet(TCAState.HaveEC)
                                        && !TCA.IsStateSet(TCAState.HaveActiveEngines);
            Controller.NoEC.isOn = TCA.IsStateSet(TCAState.Enabled)
                                   && !TCA.IsStateSet(TCAState.HaveEC);
            // fade out irrelevant indicators
            Controller.TerrainCollision.SetActive(TCAModule.ExistsAndActive(RAD));
            Controller.VesselCollision.SetActive(CFG.UseCPS);
            // set status message
            if(!string.IsNullOrEmpty(TCAGui.StatusMessage))
                Controller.SetMessage(TCAGui.StatusMessage);
        }
    }
}
