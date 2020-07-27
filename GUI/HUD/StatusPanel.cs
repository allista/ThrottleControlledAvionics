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
        private HorizontalSpeedControl HSC;
        private PointNavigator NAV;
        private Anchor anchor;

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
            if(NAV == null)
                Controller.Navigation.Show(false);
            if(HSC == null)
                Controller.Stop.Show(false);
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

        protected override void OnLateUpdate()
        {
            base.OnLateUpdate();
            if(!IsShown)
                return;
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
            Controller.SmartEngines.isOn = CFG.UseSmartEngines;
            Controller.Stop.isOn = CFG.HF[HFlight.Stop] 
                                   || TCAModule.ExistsAndActive(anchor);
            Controller.Navigation.isOn = CFG.Nav.Any(Navigation.GoToTarget,
                Navigation.FollowPath,
                Navigation.FollowTarget);
            // fade out irrelevant indicators
            Controller.TerrainCollision.SetActive(TCAModule.ExistsAndActive(RAD));
            Controller.VesselCollision.SetActive(CFG.UseCPS);
            Controller.VTOLAssist.SetActive(CFG.VTOLAssistON);
            Controller.Stabilizing.SetActive(CFG.StabilizeFlight);
            Controller.SmartEngines.SetActive(VSL.Engines.Clusters.Multi);
        }
    }
}
