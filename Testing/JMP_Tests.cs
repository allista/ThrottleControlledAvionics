//   DEO_Tests.cs
//
//  Author:
//       Allis Tauri <allista@gmail.com>
//
//  Copyright (c) 2017 Allis Tauri
#if DEBUG
using AT_Utils;

namespace ThrottleControlledAvionics
{
    public abstract class JMP_Test_Base : LND_Test_Base<BallisticJump>
    {

        protected JMP_Test_Base()
        {
            program = Autopilot2.BallisticJump;
        }

        protected override void OnLand()
        {
            MOD.UseBrakes = MOD.UseChutes = false;
            if(!TrajectoryCalculator.setp_by_step_computation)
            {
                if(VSL.LandedOrSplashed)
                    CFG.BlockThrottle = true;
                if(MOD.stage == BallisticJump.Stage.Wait)
                    TCAGui.ClearStatus();
                if(delay.TimePassed && MapView.MapIsEnabled)
                    MapView.ExitMapView();
                if(CFG.Target)
                {
                    var target = CFG.Target.GetTransform();
                    FlightCameraOverride.Target(FlightCameraOverride.Mode.LookFromTo, 
                                                VSL.vessel.vesselTransform, target, 10, 
                                                target != FlightCameraOverride.target);
                }
            }
        }
    }

    public class JMP_Test_Mun : JMP_Test_Base
    {
        public JMP_Test_Mun()
        {
            Save = "Jump Test";
            latSpread = 90;
        }
    }
}
#endif