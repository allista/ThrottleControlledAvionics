//   REN_Tests.cs
//
//  Author:
//       Allis Tauri <allista@gmail.com>
//
//  Copyright (c) 2017 Allis Tauri
#if DEBUG
using System;
using UnityEngine;
using AT_Utils;

namespace ThrottleControlledAvionics
{
    public abstract class REN_Test_Base : TCA_Test
    {
        protected RendezvousAutopilot REN;
        protected ProtoVessel target;
        protected RealTimer delay = new RealTimer(5);
        protected RendezvousAutopilot.Stage last_stage;

        protected abstract Orbit CreateRandomOrbit(Orbit baseOrbit);

        protected bool CreateAsteriod(System.Random RND)
        {
            var obt = CreateRandomOrbit(TCA.vessel.orbit);
            var seed = (uint)RND.Next();
            target = DiscoverableObjectsUtil.SpawnAsteroid("REN Test "+seed, obt, seed, UntrackedObjectClass.C, 5e5, 1e6);
            if(target.vesselRef == null) 
            {
                Utils.Message("Unable to create test asteroid");
                return false;
            }
            target.vesselRef.DiscoveryInfo.SetLevel(DiscoveryLevels.Owned);
            VSL.SetTarget(REN, new WayPoint(target.vesselRef));
            REN.MaxDist.Value = (float)(obt.semiMajorAxis-obt.referenceBody.Radius)/10;
            return true;
        }

        protected bool GetREN()
        {
            if(TCA == null && !GetTCA()) return false;
            REN = TCA.GetModule<RendezvousAutopilot>();
            return REN != null;
        }

        protected void CleanupTarget()
        {
            if(target != null)
            {
                if(target.vesselRef != null)
                    target.vesselRef.Die();
                target = null;
            }
        }

        public override void Cleanup()
        {
            CleanupTarget();
            if(TCA != null) CFG.AP2.XOff();
        }

        protected void LogStageChange()
        {
            if(REN != null && last_stage != REN.stage)
            {
                Log("Stage changed: {} -> {}", last_stage, REN.stage);
                if(REN.Valid && CFG.Target != null && REN.TargetOrbit != null)
                {
                    var ApA_UT = REN.VSL.Physics.UT+REN.VesselOrbit.timeToAp;
                    Log("Distance to target: {}\n" +
                        "Arc to target: {}\n" +
                        "Arc@ApA: {}\n" +
                        "Rel.vel: {}\n" +
                        "Trajectory: {}",
                        REN.RelPos.magnitude,
                        -Utils.ProjectionAngle(REN.TargetOrbit.pos, 
                                               REN.VesselOrbit.pos, 
                                               REN.TargetOrbit.vel),
                        -Utils.ProjectionAngle(REN.TargetOrbit.getRelativePositionAtUT(ApA_UT), 
                                               REN.VesselOrbit.getRelativePositionAtUT(ApA_UT), 
                                               REN.TargetOrbit.getOrbitalVelocityAtUT(ApA_UT)),
                        REN.RelVel.magnitude,
                        REN.CurrentTrajectory);
                }
                last_stage = REN.stage;
            }
        }
    }

    public class REN_Test_InOrbit : REN_Test_Base
    {
        protected override Orbit CreateRandomOrbit(Orbit baseOrbit)
        {
            Orbit orbit = null;
            var minR = baseOrbit.MinPeR();
            while(orbit == null || orbit.PeR < minR)
            {
                orbit = new Orbit();
                orbit.eccentricity = (double)UnityEngine.Random.Range(0.001f, (float)Math.Min(baseOrbit.eccentricity+0.2, 0.999));
                orbit.semiMajorAxis = Math.Max(baseOrbit.semiMajorAxis * (double)UnityEngine.Random.Range(0.9f, 1.1f), minR+1000);
                orbit.inclination = baseOrbit.inclination + (double)UnityEngine.Random.Range(-20f, 20f);
                orbit.LAN = baseOrbit.LAN * (double)UnityEngine.Random.Range(0.5f, 1.5f);
                orbit.argumentOfPeriapsis = baseOrbit.argumentOfPeriapsis * (double)UnityEngine.Random.Range(0.5f, 1.5f);
                orbit.meanAnomalyAtEpoch = baseOrbit.meanAnomalyAtEpoch * (double)UnityEngine.Random.Range(0.5f, 1.5f);
                orbit.epoch = baseOrbit.epoch;
                orbit.referenceBody = baseOrbit.referenceBody;
                orbit.Init();
            }
            return orbit;
        }

        public override string Setup()
        { 
            if(!GetREN()) return "No Rendezvous Autopilot installed on active vessel"; 
            if(!VSL.InOrbit) return "Vessel should be in orbit for this test";
            last_stage = REN.stage;
            return null;
        }

        public override bool Update(System.Random RND)
        {
            CFG.WarpToNode = true;
            LogStageChange();
            if(target == null)
            {
                Status = "New Target";
                if(!CreateAsteriod(RND)) return false;
                CheatOptions.InfinitePropellant = true;
                CFG.AP2.XOn(Autopilot2.Rendezvous);
                MapView.EnterMapView();
                Log("Target: {}", target.vesselName);
                return true;
            }
            if(VSL == null || VSL.vessel == null ||
               VSL.vessel.state == Vessel.State.DEAD)
            {
                if(!delay.TimePassed) return true;
                LogFlightLog("Vessel was destroyed:");
                Log("Done.");
                return false;
            }
            if(CFG.AP2[Autopilot2.Rendezvous]) 
            {
                Status = "Working...";
                if(REN.stage >= RendezvousAutopilot.Stage.MatchOrbits && MapView.MapIsEnabled)
                    MapView.ExitMapView();
                if(target.vesselRef != null && target.vesselRef.loaded)
                    FlightCameraOverride.Target(FlightCameraOverride.Mode.LookFromTo, VSL.vessel.transform, target.vesselRef.transform, 10);
                delay.Reset();
                return true;
            }
            if(CFG.AP2)
            {
                FlightCameraOverride.AnchorForSeconds(FlightCameraOverride.Mode.OrbitAround, VSL.vessel.transform, delay.Period);
                CFG.AP2.XOff();
            }
            Status = "Waiting for next iteration";
            if(delay.TimePassed)
            {
                FlightCameraOverride.Deactivate();
                CleanupTarget();
                Log("Done.");
            }
            return true;
        }

        public override bool NeedsFixedUpdate  { get { return false; } }
        public override bool NeedsUpdate { get { return true; } }
    }


    public class REN_Test_ToOrbit : REN_Test_Base
    {
        protected string Saves;
        string[] saves;
        string save;
        int save_i;

        protected override Orbit CreateRandomOrbit(Orbit baseOrbit)
        {
            Orbit orbit = null;
            var minR = baseOrbit.MinPeR();
            while(orbit == null || orbit.PeR < minR)
            {
                orbit = new Orbit();
                orbit.eccentricity = (double)UnityEngine.Random.Range(0.001f, 0.999f);
                orbit.semiMajorAxis = UnityEngine.Random.Range((float)minR+1000, (float)Math.Min(minR*5, 0.9*baseOrbit.referenceBody.sphereOfInfluence));
                orbit.inclination = (double)UnityEngine.Random.Range(-90f, 90f);
                orbit.LAN = (double)UnityEngine.Random.Range(0f, 360);
                orbit.argumentOfPeriapsis = (double)UnityEngine.Random.Range(0f, 2*Mathf.PI);
                orbit.meanAnomalyAtEpoch = (double)UnityEngine.Random.Range(0f, 2*Mathf.PI);
                orbit.epoch = baseOrbit.epoch;
                orbit.referenceBody = baseOrbit.referenceBody;
                orbit.Init();
            }
            return orbit;
        }

        enum Stage { LOAD, WAIT_FOR_LEVEL, CREATE_TARGET, RENDEZVOUS, FINISH }
        Stage stage;

        bool level_loaded;
        void onLevelWasLoaded(GameScenes scene) { level_loaded = true; }

        public override string Setup()
        { 
            stage = Stage.LOAD;
            GameEvents.onLevelWasLoadedGUIReady.Add(onLevelWasLoaded);
            saves = Saves.Split(Utils.Delimiters, StringSplitOptions.RemoveEmptyEntries);
            save_i = 0;
            save = saves[save_i];
            return saves.Length == 0 ? "Savegames cannot be empty" : null;
        }

        public override bool Update(System.Random RND)
        {
            Status = string.Format("{0}: {1}", save, stage.ToString().Replace("_", " "));
            LogStageChange();
            switch(stage)
            {
            case Stage.LOAD:
                level_loaded = false;
                save = saves[save_i];
                if(!ScenarioTester.LoadGame(save))
                {
                    Utils.Message("Unable to load savegame: {0}", save);
                    return false;
                }
                save_i = (save_i+1) % saves.Length;
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
                    if(GetREN()) stage = Stage.CREATE_TARGET;
                    else 
                    {
                        Utils.Message("No Rendezvous Autopilot installed on the active vessel");
                        return false;
                    }
                    TCAGui.ShowInstance(true);
                    TCAGui.Instance.ActiveTab = TCAGui.Instance.ORB;
                    ResetFlightCamera();
                }
                break;
            case Stage.CREATE_TARGET:
                if(target == null)
                {
                    if(!CreateAsteriod(RND)) return false;
                    CheatOptions.InfinitePropellant = false;
                    CheatOptions.InfiniteElectricity = true;
                    CFG.AP2.XOn(Autopilot2.Rendezvous);
                    break;
                }
                if(CFG.AP2[Autopilot2.Rendezvous])
                {
                    MapView.EnterMapView();
                    VSL.vessel.ActionGroups.SetGroup(KSPActionGroup.RCS, true);
                    REN.mode = RendezvousAutopilot.Mode.TimeToTarget;
                    stage = Stage.RENDEZVOUS;
                }
                break;
            case Stage.RENDEZVOUS:
                CFG.WarpToNode = true;
                if(CFG.AP2[Autopilot2.Rendezvous]) 
                {
                    if(VSL.Engines.NoActiveEngines && !VSL.Engines.HaveNextStageEngines)
                    {
                        Utils.Message("Out of fuel");
                        CFG.AP2.XOff();
                        stage = Stage.FINISH;
                        delay.Reset();
                        break;
                    }
                    if(target.vesselRef != null && target.vesselRef.loaded)
                        FlightCameraOverride.Target(FlightCameraOverride.Mode.LookFromTo, VSL.vessel.transform, target.vesselRef.transform, 10);
                    else if(VSL.LandedOrSplashed && VSL.Info.Countdown > 5 ||
                            REN.stage == RendezvousAutopilot.Stage.ToOrbit &&
                            REN.VesselOrbit.ApR > REN.VesselOrbit.MinPeR() ||
                            REN.stage >= RendezvousAutopilot.Stage.ComputeRendezvou &&
                            REN.stage < RendezvousAutopilot.Stage.MatchOrbits)
                    {
                        if(!TrajectoryCalculator.setp_by_step_computation)
                        {
                            if(!MapView.MapIsEnabled)
                                MapView.EnterMapView();
                            RotateMapView();
                        }
                    }
                    else
                    {
                        if(MapView.MapIsEnabled)
                        {
                            MapView.ExitMapView();
                            ResetFlightCamera();
                        }
                        FlightCameraOverride.AnchorForSeconds(FlightCameraOverride.Mode.OrbitAround, VSL.vessel.transform, 1);
                    }
                    break;
                }
                CFG.AP2.XOff();
                stage = Stage.FINISH;
                delay.Reset();
                break;
            case Stage.FINISH:
                if(!delay.TimePassed) break;
                Log("Done.");
                CleanupTarget();
                stage = Stage.LOAD;
                break;
            }
            return true;
        }

        public override void Cleanup()
        {
            base.Cleanup();
            GameEvents.onLevelWasLoadedGUIReady.Remove(onLevelWasLoaded);
        }

        public override void Draw()
        {
            GUILayout.BeginHorizontal();
            {
                GUILayout.Label(new GUIContent("Savegames:", "Delimited by commas or spaces"),
                                GUILayout.MinWidth(100));
                Saves = GUILayout.TextArea(Saves, GUILayout.ExpandWidth(true));
            }
            GUILayout.EndHorizontal();
        }

        public override bool NeedsFixedUpdate  { get { return false; } }
        public override bool NeedsUpdate { get { return true; } }
    }

    public class REN_Test_ToOrbit_Low : REN_Test_ToOrbit
    {
        protected override Orbit CreateRandomOrbit(Orbit baseOrbit)
        {
            Orbit orbit = null;
            var minR = baseOrbit.MinPeR();
            while(orbit == null || orbit.PeR < minR)
            {
                orbit = new Orbit();
                orbit.eccentricity = (double)UnityEngine.Random.Range(0.001f, 0.1f);
                orbit.semiMajorAxis = UnityEngine.Random.Range((float)minR+1000, (float)minR+ToOrbitAutopilot.C.RadiusOffset);
                orbit.inclination = (double)UnityEngine.Random.Range(1f, 5f);
                orbit.LAN = (double)UnityEngine.Random.Range(0f, 360);
                orbit.argumentOfPeriapsis = (double)UnityEngine.Random.Range(0f, 2*Mathf.PI);
                orbit.meanAnomalyAtEpoch = (double)UnityEngine.Random.Range(0f, 2*Mathf.PI);
                orbit.epoch = baseOrbit.epoch;
                orbit.referenceBody = baseOrbit.referenceBody;
                orbit.Init();
            }
            return orbit;
        }
    }
}
#endif
