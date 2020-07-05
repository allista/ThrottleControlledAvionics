//  Author:
//       Allis Tauri <allista@gmail.com>
//
//  Copyright (c) 2015 Allis Tauri
//
// This work is licensed under the Creative Commons Attribution-ShareAlike 4.0 International License. 
// To view a copy of this license, visit http://creativecommons.org/licenses/by-sa/4.0/ 
// or send a letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.

using System;
using System.Diagnostics.CodeAnalysis;
using AT_Utils;
using AT_Utils.UI;
using JetBrains.Annotations;

namespace ThrottleControlledAvionics
{
    [CareerPart]
    [RequireModules(typeof(AttitudeControl),
        typeof(ThrottleControl),
        typeof(TranslationControl),
        typeof(TimeWarpControl))]
    public class ManeuverAutopilot : TCAModule
    {
        [SuppressMessage("ReSharper", "FieldCanBeMadeReadOnly.Global"),
         SuppressMessage("ReSharper", "ConvertToConstant.Global")]
        public class Config : ComponentConfig<Config>
        {
            [Persistent] public float WrapThreshold = 600f; //s

            //controls best engine cluster calculation
            [Persistent] public float ClosestCluster = 5f; //s
            [Persistent] public float EfficientCluster = 0.1f; //fraction of vessel mass
            [Persistent] public float EfficiencyWeight = 10; //how much the fuel mass will affect cluster selection
        }

        public static Config C => Config.INST;

        public enum Stage { WAITING, IN_PROGRESS, FINISHED }

        public ManeuverAutopilot(ModuleTCA tca) : base(tca) { }

        private ThrottleControl THR;

        private ManeuverNode Node;
        private PatchedConicSolver Solver => VSL.vessel.patchedConicSolver;

        public double NodeUT => Node?.UT ?? -1;
        public Vector3d NodeDeltaV { get; private set; }
        public CelestialBody NodeCB { get; private set; }
        public Orbit TargetOrbit { get; private set; }
        public Stage ManeuverStage { get; private set; }

        private ManeuverExecutor Executor;
        public float MinDeltaV = 1;

        public void AddCourseCorrection(Vector3d dV) => Executor.AddCourseCorrection(dV);
        public bool ThrustWhenAligned = true;

        public void UpdateNode()
        {
            if(VSL.HasManeuverNode
               && !update_maneuver_node())
                Disable();
        }

        private bool update_maneuver_node()
        {
            Node = null;
            NodeCB = null;
            TargetOrbit = null;
            if(Solver != null)
            {
                if(Solver.maneuverNodes.Count <= 0)
                    return false;
                Node = Solver.maneuverNodes[0];
            }
            else
            {
                if(VSL.vessel.flightPlanNode.nodes.Count <= 0)
                    return false;
                var node = VSL.vessel.flightPlanNode.nodes[0];
                Node = new ManeuverNode();
                Node.Load(node);
                Node.patch = new Orbit(VSL.orbit);
                Node.nextPatch =
                    TrajectoryCalculator.NewOrbit(VSL.orbit, Utils.Node2OrbitalDeltaV(Node), Node.UT);
                VSL.vessel.flightPlanNode.RemoveNode(node);
            }
            NodeCB = Node.patch.referenceBody;
            TargetOrbit = Node.nextPatch;
            update_node_deltaV();
            if(VSL.Engines.MaxDeltaV < Node.DeltaV.magnitude)
                Status(Colors.Warning,
                    "WARNING: there may be not enough propellant for the maneuver");
            return true;
        }

        private void remove_maneuver_node()
        {
            if(Node.solver != null)
                Node.RemoveSelf();
            else if(Solver != null
                    && Solver.maneuverNodes.Count > 0
                    && Math.Abs(Solver.maneuverNodes[0].UT - Node.UT) < 1e-6)
                Solver.maneuverNodes[0].RemoveSelf();
            Node = null;
        }

        private void update_node_deltaV() =>
            NodeDeltaV = (TargetOrbit.GetFrameVelAtUT(NodeUT) - VSL.orbit.GetFrameVelAtUT(NodeUT)).xzy;

        public override void Init()
        {
            base.Init();
            MinDeltaV = ThrottleControl.C.MinDeltaV;
            CFG.AP1.AddHandler(this, Autopilot1.Maneuver);
            Executor = new ManeuverExecutor(TCA) { ThrustWhenAligned = ThrustWhenAligned, StopAtMinimum = true };
        }

        public override void Disable()
        {
            CFG.AP1.OffIfOn(Autopilot1.Maneuver);
        }

        protected override void UpdateState()
        {
            base.UpdateState();
            IsActive &= Node != null;
            ControlsActive &= IsActive
                              || TCAScenario.HavePatchedConics && VSL.Engines.HaveThrusters && VSL.HasManeuverNode;
        }

        [UsedImplicitly]
        public void ManeuverCallback(Multiplexer.Command cmd)
        {
            switch(cmd)
            {
                case Multiplexer.Command.Resume:
                case Multiplexer.Command.On:
                    ManeuverStage = Stage.WAITING;
                    if(!TCAScenario.HavePatchedConics)
                    {
                        Status(Colors.Warning,
                            "WARNING: maneuver nodes are not yet available. Upgrade the Tracking Station.");
                        CFG.AP1.Off();
                        return;
                    }
                    if(!VSL.HasManeuverNode || !update_maneuver_node())
                    {
                        CFG.AP1.Off();
                        return;
                    }
                    VSL.Controls.StopWarp();
                    CFG.AT.On(Attitude.ManeuverNode);
                    THR.Throttle = 0;
                    CFG.DisableVSC();
                    break;

                case Multiplexer.Command.Off:
                    VSL.Controls.StopWarp();
                    if(!CFG.WarpToNode && TimeWarp.CurrentRateIndex > 0)
                        TimeWarp.SetRate(0, false);
                    CFG.AT.On(Attitude.KillRotation);
                    Reset();
                    break;
                default:
                    Log($"Unknown Multiplexer.Command: {cmd}");
                    break;
            }
        }

        protected override void Reset()
        {
            base.Reset();
            if(Working)
                THR.Throttle = 0;
            if(CFG.AT[Attitude.ManeuverNode])
                CFG.AT.On(Attitude.KillRotation);
            Executor.Reset();
            NodeDeltaV = Vector3d.zero;
            NodeCB = null;
            MinDeltaV = ThrottleControl.C.MinDeltaV;
            VSL.Info.Countdown = 0;
            VSL.Info.TTB = 0;
            Working = false;
            Node = null;
        }

        public static void AddNode(VesselWrapper VSL, Vector3d dV, double UT) => Utils.AddNode(VSL.vessel, dV, UT);

        public static void AddNodeRaw(VesselWrapper VSL, Vector3d NodeV, double UT) =>
            Utils.AddNodeRaw(VSL.vessel, NodeV, UT);

        private bool StartCondition(float dV)
        {
            if(Working)
                return true;
            var ttb = VSL.Engines.TTB_Precise(dV);
            if(float.IsNaN(ttb))
            {
                Log("WARNING: TTB is NaN: dV {}", dV);
                return false;
            }
            VSL.Info.TTB = ttb;
            var burn = Node.UT - VSL.Info.TTB / 2f;
            if(CFG.WarpToNode && VSL.Controls.WarpToTime < 0)
            {
                if((burn - VSL.Physics.UT) / dV > C.WrapThreshold
                   || burn - VSL.Physics.UT > 180 + TimeWarpControl.C.DewarpTime)
                {
                    VSL.Controls.NoDewarpOffset = true;
                    VSL.Controls.WarpToTime = burn - 180;
                }
                else if(VSL.Controls.CanWarp)
                    VSL.Controls.WarpToTime = burn - VSL.Controls.MinAlignmentTime;
            }
            VSL.Info.Countdown = burn - VSL.Physics.UT;
            //emergency dewarping
            if(!CFG.WarpToNode && TimeWarp.CurrentRate > 1 && VSL.Info.Countdown < TimeWarpControl.C.DewarpTime)
                VSL.Controls.AbortWarp(true);
            if(VSL.Info.Countdown > 0)
                return false;
//            Log("burn {}, countdown {}", burn, VSL.Info.Countdown);//debug
            VSL.Info.Countdown = 0;
            ManeuverStage = Stage.IN_PROGRESS;
            Working = true;
            return true;
        }

        private bool checkManeuverNode()
        {
            // ReSharper disable once InvertIf
            if(Solver != null && Node.solver == Solver)
            {
                if(Solver.maneuverNodes.Count <= 0
                   || Node != Solver.maneuverNodes[0])
                {
                    Message("Maneuver node was changed.");
                    return false;
                }
                // ReSharper disable once InvertIf
                if(NodeCB != Node.patch.referenceBody)
                {
                    Message("Maneuver node changed SoI.");
                    return false;
                }
            }
            return true;
        }

        protected override void Update()
        {
            if(!checkManeuverNode())
            {
                Disable();
                return;
            }
            if(!VSL.Engines.HaveThrusters && !VSL.Engines.HaveNextStageEngines)
            {
                Message("Out of fuel");
                Disable();
                return;
            }
            //update the node
            update_node_deltaV();
            Executor.ThrustWhenAligned = ThrustWhenAligned;
            ThrustWhenAligned = true;
            if(Executor.Execute(NodeDeltaV, MinDeltaV, StartCondition))
                return;
            ManeuverStage = Stage.FINISHED;
            remove_maneuver_node();
            Disable();
        }
    }
}
