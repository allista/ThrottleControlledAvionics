//   ORB_Tests.cs
//
//  Author:
//       Allis Tauri <allista@gmail.com>
//
//  Copyright (c) 2018 Allis Tauri
#if DEBUG
using UnityEngine;
using AT_Utils;

namespace ThrottleControlledAvionics
{
    public abstract class ORB_Test : TCA_Test
    {
        protected string Save = "";
        protected ToOrbitAutopilot ORB;
        protected RealTimer delay = new RealTimer(5);

        bool level_loaded, orbit_set_up;
        float ApA, inclination, abs_inclination;
        void onLevelWasLoaded(GameScenes scene) { level_loaded = true; }

        enum Stage { LOAD, WAIT_FOR_LEVEL, SETUP, TO_ORBIT, FINISH }
        Stage stage;

        public override string Setup()
        {
            GameEvents.onLevelWasLoadedGUIReady.Add(onLevelWasLoaded);
            stage = Stage.LOAD;
            return null;
        }

        protected bool GetModule()
        {
            if(TCA == null && !GetTCA()) return false;
            ORB = TCA.GetModule<ToOrbitAutopilot>();
            return ORB != null;
        }

        public override bool Update(System.Random RND)
        {
            Status = stage.ToString().Replace("_", " ");
            switch(stage)
            {
            case Stage.LOAD:
                level_loaded = false;
                ScenarioTester.LoadGame(Save);
                stage = Stage.WAIT_FOR_LEVEL;
                delay.Reset();
                break;
            case Stage.WAIT_FOR_LEVEL:
                if(level_loaded && 
                   FlightGlobals.ready && 
                   FlightGlobals.ActiveVessel != null &&
                   !FlightGlobals.ActiveVessel.packed)
                {
                    if(!delay.TimePassed) break;
                    if(!GetTCA()) 
                    {
                        Utils.Message("TCA is not installed/enabled on the ship");
                        return false;
                    }
                    if(!GetModule()) 
                    {
                        Utils.Message("{0} module is not installed on the ship", typeof(ToOrbitAutopilot).Name);
                        return false;
                    }
                    VSL.SetTarget(null);
                    Debug.ClearDeveloperConsole();
                    orbit_set_up = false;
                    stage = Stage.SETUP;
                }
                break;
            case Stage.SETUP:
                if(!orbit_set_up)
                {
                    ResetFlightCamera();
                    TCAGui.ShowInstance(true);
                    TCAGui.Instance.ActiveTab = TCAGui.Instance.ORB;
                    ORB.TargetOrbit.RetrogradeOrbit = RND.NextDouble() > 0.5? true : false;
                    ORB.TargetOrbit.DescendingNode = RND.NextDouble() > 0.5? true : false;
                    ORB.TargetOrbit.UpdateValues();
                    ORB.TargetOrbit.Inclination.Value = (float)RND.NextDouble() * 90;
                    ORB.TargetOrbit.Inclination.ClampValue();
                    inclination = ORB.TargetOrbit.Inclination;
                    abs_inclination = (float)ORB.TargetOrbit.TargetInclination;
                    ORB.TargetOrbit.ApA.Value = (float)(ORB.MinR+1000+RND.NextDouble() * 500000 - VSL.Body.Radius)/1000;
                    ORB.TargetOrbit.ApA.ClampValue();
                    ApA = ORB.TargetOrbit.ApA*1000;
                    ORB.ShowOptions = true;
                    VSL.vessel.ActionGroups.SetGroup(KSPActionGroup.RCS, true);
                    CFG.AP2.XOn(Autopilot2.ToOrbit);
                    orbit_set_up = true;
                    Log("TargetOrbit: {}", ORB.TargetOrbit);
                    Log("TargetInclination: {}", ORB.TargetOrbit.TargetInclination);
                    break;
                }
                if(CFG.AP2[Autopilot2.ToOrbit])
                {
                    delay.Reset();
                    stage = Stage.TO_ORBIT;
                }
                break;
            case Stage.TO_ORBIT:
                CFG.WarpToNode = true;
                if(VSL == null || VSL.vessel == null || 
                   VSL.vessel.state == Vessel.State.DEAD)
                {
                    LogFlightLog("Vessel was destroyed:");
                    stage = Stage.FINISH;
                    delay.Reset();
                    break;
                }
                if(CFG.AP2[Autopilot2.ToOrbit]) 
                {
                    if(ORB.stage >= ToOrbitAutopilot.Stage.ChangeApA)
                    {
                        if(!MapView.MapIsEnabled)
                            MapView.EnterMapView();
                        RotateMapView();
                    }
                    else
                        FlightCameraOverride.AnchorForSeconds(FlightCameraOverride.Mode.OrbitAround, VSL.vessel.transform, 1);
                    break;
                }
                Log("Achived Orbit: {}", VSL.vessel.orbit);
                var dApA = VSL.vessel.orbit.ApA-ApA;
                Log("ApA Error: {} m {} %", dApA, dApA/ApA*100);
                var dInc = VSL.vessel.orbit.inclination-abs_inclination;
                Log("Inclination Error: {} deg {} %", dInc, dInc/inclination*100);
                CFG.AP2.XOff();
                stage = Stage.FINISH;
                delay.Reset();
                break;
            case Stage.FINISH:
                if(!delay.TimePassed) break;
                Log("Done.");
                Cleanup();
                Setup();
                break;
            }
            return true;
        }

        public override void Cleanup()
        {
            if(TCA != null && CFG != null) CFG.AP2.XOff();
            GameEvents.onLevelWasLoadedGUIReady.Remove(onLevelWasLoaded);
        }

        public override void Draw()
        {
            GUILayout.BeginHorizontal();
            {
                GUILayout.Label("Savegame:");
                Save = GUILayout.TextField(Save, GUILayout.ExpandWidth(true));
            }
            GUILayout.EndHorizontal();
        }

        public override bool NeedsFixedUpdate { get { return false; } }
        public override bool NeedsUpdate { get { return true; } }
    }
}
#endif