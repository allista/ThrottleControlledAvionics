//   VFlightPanel.cs
//
//  Author:
//       Allis Tauri <allista@gmail.com>
//
//  Copyright (c) 2019 Allis Tauri

using AT_Utils.UI;
using TCA.UI;

namespace ThrottleControlledAvionics
{
    public class VFlightPanel : ControlPanel<VFlightUI>
    {
        VerticalSpeedControl VSC;
        AltitudeControl ALT;
        ThrottleControl THR;
        Radar RAD;
        private BearingControl BRC;

        protected override bool shouldShow => base.shouldShow && VSL.OnPlanet && AllModules.Count > 0;

        protected override void init_controller()
        {
            Controller.hoverButton.onValueChanged.RemoveListener(onHover);
            Controller.followTerrainButton.onValueChanged.RemoveListener(onFollowTerrain);
            Controller.autoThrottleButton.onValueChanged.RemoveListener(onAutoThrottle);
            Controller.VSC.onValueChanged.RemoveListener(onVSC);
            Controller.ALT.onValueChanged.RemoveListener(onALT);
            if(ALT != null)
            {
                Controller.ALT.onValueChanged.AddListener(onALT);
                Controller.hoverButton.onValueChanged.AddListener(onHover);
                if(RAD != null)
                    Controller.followTerrainButton.onValueChanged.AddListener(onFollowTerrain);
                else
                    Controller.followTerrainButton.gameObject.SetActive(false);
            }
            else
            {
                Controller.hoverButton.gameObject.SetActive(false);
                Controller.followTerrainButton.gameObject.SetActive(false);
                Controller.ALT.SetActive(false);
            }
            if(THR != null && (ALT != null || VSC != null))
                Controller.autoThrottleButton.onValueChanged.AddListener(onAutoThrottle);
            else
                Controller.autoThrottleButton.gameObject.SetActive(false);
            if(VSC != null)
                Controller.VSC.onValueChanged.AddListener(onVSC);
            else
                Controller.VSC.SetActive(false);
            base.init_controller();
        }

        void onHover(bool hover)
        {
            if(hover)
                TCA.SquadConfigAction(cfg => { 
                    cfg.VF.XOnIfNot(VFlight.AltitudeControl); 
                    cfg.BlockThrottle = true; 
                });
            else
                TCA.SquadConfigAction(cfg => cfg.VF.XOffIfOn(VFlight.AltitudeControl));
        }

        void onFollowTerrain(bool follow_terrain)
        {
            CFG.AltitudeAboveTerrain = follow_terrain;
            TCA.SquadAction(tca =>
            {
                var alt = tca.GetModule<AltitudeControl>();
                if(alt != null)
                    alt.SetAltitudeAboveTerrain(CFG.AltitudeAboveTerrain);
            });
        }

        void onAutoThrottle(bool auto_throttle)
        {
            THR.BlockThrottle(auto_throttle);
        }

        void onVSC(float vertical_speed)
        {
            VSC.SetVerticalCutoff(vertical_speed);
        }

        void onALT(float altitude)
        {
            ALT.SetDesiredAltitude(altitude);
        }

        protected override void OnLateUpdate()
        {
            base.OnLateUpdate();
            Controller.UpdateInfo(VSL.Altitude.Current, 
                                  VSL.VerticalSpeed.Display, 
                                  VSL.HorizontalSpeed.Absolute);
            if(ALT != null)
            {
                Controller.EnableALT(CFG.VF[VFlight.AltitudeControl]);
                Controller.ALT.SetValueWithoutNotify(CFG.DesiredAltitude);
                Controller.ALT.SetAltitudeAboveGround(VSL.Altitude.AboveGround);
                Controller.hoverButton.SetIsOnAndColorWithoutNotify(CFG.VF[VFlight.AltitudeControl]);
                if(RAD != null)
                    Controller.followTerrainButton.SetIsOnAndColorWithoutNotify(CFG.AltitudeAboveTerrain);
            }
            if(VSC != null)
                Controller.VSC.SetValueWithoutNotify(CFG.VerticalCutoff);
            if(THR != null)
                Controller.autoThrottleButton.SetIsOnAndColorWithoutNotify(CFG.BlockThrottle);
        }

        protected override void OnRender()
        {
            base.OnRender();
            BRC?.DrawForwardDirection();
        }
    }
}
