using TCA.UI;

namespace ThrottleControlledAvionics
{
    public class StatusPanel : ControlPanel<StatusUI>
    {
        private VerticalSpeedControl VSC;
        private AltitudeControl ALT;
        private VTOLAssist VTOL_assist;
        private VTOLControl VTOL_control;
        private FlightStabilizer Stabilizer;
        private CollisionPreventionSystem CPS;
        private Radar RAD;

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
            base.init_controller();
        }

        protected override void OnLateUpdate()
        {
            base.OnLateUpdate();
            Controller.Ascending.isOn = TCA.IsStateSet(TCAState.Ascending);
            Controller.LoosingAltitude.isOn = TCA.IsStateSet(TCAState.LoosingAltitude);
            Controller.TerrainCollision.isOn = TCA.IsStateSet(TCAState.GroundCollision);
            Controller.VesselCollision.isOn = TCA.IsStateSet(TCAState.VesselCollision);
            Controller.LowControlAuthority.isOn = !VSL.Controls.HaveControlAuthority;
            Controller.EnginesUnoptimized.isOn = TCA.IsStateSet(TCAState.Unoptimized);
            Controller.VSC.isOn = TCA.IsStateSet(TCAState.VerticalSpeedControl);
            Controller.ALT.isOn = TCA.IsStateSet(TCAState.AltitudeControl);
            Controller.VTOLMode.isOn = CFG.CTRL[ControlMode.VTOL];
            Controller.VTOLAssist.isOn = TCA.IsStateSet(TCAState.VTOLAssist);
            Controller.Stabilizing.isOn = TCA.IsStateSet(TCAState.StabilizeFlight);
            Controller.NoEngines.isOn = TCA.IsStateSet(TCAState.HaveEC)
                                        && !TCA.IsStateSet(TCAState.HaveActiveEngines);
            Controller.NoEC.isOn = TCA.IsStateSet(TCAState.Enabled)
                                   && !TCA.IsStateSet(TCAState.HaveEC);
            // fade out irrelevant indicators
            Controller.VesselCollision.SetActive(CFG.UseCPS);
        }
    }
}
