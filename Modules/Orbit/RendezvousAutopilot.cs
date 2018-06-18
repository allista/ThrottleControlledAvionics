//  Author:
//       Allis Tauri <allista@gmail.com>
//
//  Copyright (c) 2016 Allis Tauri
//
// This work is licensed under the Creative Commons Attribution-ShareAlike 4.0 International License. 
// To view a copy of this license, visit http://creativecommons.org/licenses/by-sa/4.0/ 
// or send a letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.
//

using System;
using System.Collections.Generic;
using UnityEngine;
using AT_Utils;

namespace ThrottleControlledAvionics
{
    [CareerPart]
    [RequireModules(typeof(ManeuverAutopilot),
                    typeof(MatchVelocityAutopilot),
                    typeof(AttitudeControl),
                    typeof(ThrottleControl),
                    typeof(TranslationControl),
                    typeof(BearingControl),
                    typeof(TimeWarpControl))]
    public partial class RendezvousAutopilot : TargetedTrajectoryCalculator<RendezvousTrajectory>
    {
        public new class Config : ComponentConfig<Config>
        {
            [Persistent] public float Dtol = 100f; //m
            [Persistent] public float MaxTTR = 3f; //VesselOrbit.periods
            [Persistent] public float MaxDeltaV = 100f; //m/s
            [Persistent] public float CorrectionStart = 10000f; //m
            [Persistent] public float CorrectionTimer = 10f;    //s
            [Persistent] public float ApproachThreshold = 500f; //m
            [Persistent] public float MaxApproachV = 20f;    //parts
            [Persistent] public float MaxInclinationDelta = 30; //deg
            [Persistent] public float MaxInclinationDeltaToOrbit = 5; //deg
            [Persistent] public float MaxDaysToLaunch = 5; //celectial body rotations
            [Persistent] public MinMax TargetArc = new MinMax(15, 90, 60); //deg
        }
        public static new Config C => Config.INST;

        public RendezvousAutopilot(ModuleTCA tca) : base(tca) { }

        ThrottleControl THR;
        MatchVelocityAutopilot MVA;

        public override RendezvousTrajectory CurrentTrajectory
        { get { return new RendezvousTrajectory(VSL, Vector3d.zero, VSL.Physics.UT, CFG.Target); } }

        public enum Stage
        {
            None, Start, PreLaunch, Launch, ToOrbit, StartOrbit,
            ComputeSuborbitRendezvous, ComputeRendezvou, ComputeCorrection,
            Rendezvou, Coast, MatchOrbits,
            Approach, Brake
        }
        [Persistent] public Stage stage;
        [Persistent] public TargetedToOrbitExecutor ToOrbit = new TargetedToOrbitExecutor();
        [Persistent] public FloatField Steepness = new FloatField(format: "F1", min: 0, max: 100);
        [Persistent] public FloatField IncDelta = new FloatField(min: 1, max: 90);
        [Persistent] public FloatField MaxDist = new FloatField(min: 1, max: 100);
        [Persistent] public FloatField MaxDays = new FloatField(min: 1, max: 365);
        [Persistent] public bool StartInPlane = false;
        [Persistent] public Mode mode;
        [Persistent] public bool ShowOptions;

        public enum Mode { DeltaV, TimeToTarget, Manual }
        static int NumModes = Enum.GetValues(typeof(Mode)).Length;
        static string[] ModeNames = { "Min. dV", "Fast Transfer", "Manual" };
        static string[] ModeDesc =
        {
            "Use the most fuel-efficient transfer",
            "Prefer transfers that take less overall time",
            "Manually choose transfer if several are available"
        };
        CDOS_Optimizer2D optimizer;

        MinimumD MinDist = new MinimumD();
        Vector3d ToOrbitIniApV;
        double CurrentDistance = -1;
        bool CorrectingManeuver;
        LowPassFilterVd Correction = new LowPassFilterVd();
        AtmoSim sim;

        public override void Init()
        {
            base.Init();
            CorrectionTimer.Period = C.CorrectionTimer;
            CFG.AP2.AddHandler(this, Autopilot2.Rendezvous);
            Steepness.Value = Mathf.InverseLerp(C.TargetArc.Max, C.TargetArc.Min, C.TargetArc.Val) * 100;
            IncDelta.Value = C.MaxInclinationDeltaToOrbit;
            ToOrbit.AttachTCA(TCA);
            MaxDist.Value = 5;
            MaxDays.Value = C.MaxDaysToLaunch;
            Correction.Tau = 0.5f;
            sim = new AtmoSim(TCA);
        }

        public void RendezvousCallback(Multiplexer.Command cmd)
        {
            switch(cmd)
            {
            case Multiplexer.Command.Resume:
                if(!check_patched_conics())
                {
                    Disable();
                    break;
                }
                ShowOptions = true;
                UseTarget();
                NeedCPSWhenMooving();
                switch(stage)
                {
                case Stage.None:
                case Stage.ComputeRendezvou:
                    stage = Stage.Start;
                    break;
                case Stage.Launch:
                    if(VSL.LandedOrSplashed) stage = Stage.Start;
                    break;
                case Stage.ToOrbit:
                    break;
                }
                if(VSL.HasManeuverNode)
                    CFG.AP1.OnIfNot(Autopilot1.Maneuver);
                break;

            case Multiplexer.Command.On:
                Reset();
                if(setup())
                    goto case Multiplexer.Command.Resume;
                Disable();
                break;

            case Multiplexer.Command.Off:
                CFG.AT.On(Attitude.KillRotation);
                ReleaseCPS();
                StopUsingTarget();
                ShowOptions = false;
                Reset();
                break;
            }
        }

        protected override bool check_target()
        {
            if(!base.check_target()) return false;
            if(TargetVessel == null)
            {
                Status("yellow", "Target should be a vessel or an asteroid");
                return false;
            }
            if(TargetVessel.LandedOrSplashed)
            {
                Status("yellow", "Target is landed");
                return false;
            }
            if(!VSL.OnPlanet &&
               VesselOrbit.patchEndTransition == Orbit.PatchTransitionType.FINAL &&
               TargetOrbit.patchEndTransition == Orbit.PatchTransitionType.FINAL &&
               VesselOrbit.referenceBody == TargetOrbit.referenceBody)
            {
                var dInc = Math.Abs(TargetOrbit.inclination - VesselOrbit.inclination);
                if(dInc > 90)
                {
                    Status("yellow", "Target orbits in the oposite direction.\n" +
                           "You need to change orbit direction before the rendezvou maneuver.");
                    return false;
                }
                else if(dInc > C.MaxInclinationDelta)
                {
                    Status("yellow", "Target orbit plane is tilted more than {0:F}° with respect to ours.\n" +
                           "You need to change orbit plane before the rendezvou maneuver.", C.MaxInclinationDelta);
                    return false;
                }
            }
            return true;
        }

        RendezvousTrajectory new_trajectory(double StartUT, double transfer_time)
        {
            var endUT = StartUT + transfer_time;
            var obt = NextOrbit(StartUT);
            solver.Init(obt, RelativePosAtUT(obt.referenceBody, TargetOrbit, endUT), StartUT);
            if(solver.NotElliptic(transfer_time))
                transfer_time = solver.ParabolicTime + 1;
            var dV = solver.dV4Transfer(transfer_time);
//            Log("transfer time {}\ncV {}\ndV {}\norbit {}\ntOrbit {}", 
//                transfer_time, VesselOrbit.getOrbitalVelocityAtUT(StartUT), dV, 
//                NextOrbit(endUT), NextOrbit(TargetOrbit, endUT));//debug
            return new RendezvousTrajectory(VSL, dV, StartUT, CFG.Target, transfer_time);
        }

        void compute_rendezvou_trajectory()
        {
            VSL.Controls.StopWarp();
            var minStartUT = VSL.Physics.UT + ManeuverOffset + TrajectoryCalculator.C.CorrectionOffset;
            var transfer = (TargetOrbit.period + VesselOrbit.period) / 4;
            var softMaxStart = true;
            double maxStartUT, maxEndUT = -1;
            var lastOrbitV = LastOrbit(VesselOrbit);
            var lastOrbitT = LastOrbit(TargetOrbit);
            if(DiscontiniousOrbit(lastOrbitV))
            {
                maxStartUT = lastOrbitV.EndUT;
                softMaxStart = false;
            }
            else
            {
                var period = Math.Max(double.IsInfinity(lastOrbitV.period) ? 0 : lastOrbitV.period,
                                      double.IsInfinity(lastOrbitT.period) ? 0 : lastOrbitT.period);
                maxStartUT = VSL.Physics.UT + period * (C.MaxTTR + 1);
            }
            if(DiscontiniousOrbit(lastOrbitT))
            {
                maxEndUT = lastOrbitT.EndUT;
                maxStartUT = Math.Min(maxStartUT, maxEndUT);
                softMaxStart = false;
            }
            optimizer = new CDOS_Optimizer2D(this,
                                             minStartUT,
                                             maxStartUT,
                                             maxEndUT,
                                             softMaxStart,
                                             minStartUT + 1,
                                             transfer,
                                             Utils.Clamp(transfer / 10, 10, 1000));
            ComputeTrajectory(optimizer);
            stage = Stage.ComputeRendezvou;
            trajectory = null;
        }

        protected override void fine_tune_approach()
        {
            update_trajectory();
            var startUT = VSL.Physics.UT + CorrectionOffset +
                             TimeWarp.fixedDeltaTime * Math.Max(GameSettings.FRAMERATE_LIMIT, 60);
            var transfer = trajectory.AtTargetUT - startUT;
            var dT = trajectory.TransferTime / 10;
            if(transfer <= 0)
            {
                transfer = trajectory.OrigOrbit.GetEndUT() - startUT;
                dT = transfer / 10;
            }
            ComputeTrajectory(new StartTimeOptimizer(this, startUT, transfer, dT));
            stage = Stage.ComputeCorrection;
            trajectory = null;
        }

        protected void compute_approach_orbit(double StartUT)
        {
            VSL.Controls.StopWarp();
            var transfer_time = (VesselOrbit.period + TargetOrbit.period) / 4;
            double maxEndUT;
            var lastOrbitT = LastOrbit(TargetOrbit);
            if(DiscontiniousOrbit(lastOrbitT))
                maxEndUT = lastOrbitT.EndUT;
            else if(!double.IsInfinity(lastOrbitT.period))
                maxEndUT = VSL.Physics.UT + lastOrbitT.period * (C.MaxTTR + 2);
            else
                maxEndUT = NearestRadiusUT(lastOrbitT, lastOrbitT.referenceBody.sphereOfInfluence, VSL.Physics.UT, false);
            ComputeTrajectory(new TransferOptimizer(this,
                                                    maxEndUT,
                                                    StartUT,
                                                    transfer_time,
                                                    transfer_time / 2));
            stage = Stage.ComputeSuborbitRendezvous;
            trajectory = null;
        }

        void start_orbit()
        {
            ToOrbit.Reset();
            THR.Throttle = 0;
            CFG.BR.OffIfOn(BearingMode.Auto);
            update_trajectory();
            if(VesselOrbit.PeR < MinR)
            {
                var StartUT = Math.Min(trajectory.AtTargetUT, VSL.Physics.UT + (VesselOrbit.ApAhead() ? VesselOrbit.timeToAp : CorrectionOffset));
                //approach is close enough to directly match orbits
                if(trajectory.DistanceToTarget < C.ApproachThreshold * 2 &&
                   StartUT.Equals(trajectory.AtTargetUT))
                    match_orbits();
                else compute_approach_orbit(StartUT);
            }
            else next_stage();
        }

        void circularize()
        {
            update_trajectory();
            //start from circular orbit and proceed to TTR fitting
            var StartUT = VesselOrbit.ApAhead() ? VSL.Physics.UT + VesselOrbit.timeToAp : VSL.Physics.UT + CorrectionOffset;
            var dV = dV4C(VesselOrbit, hV(StartUT), StartUT);
            //compute orbit with desired TTR and activate maneuver autopilot
            dV += dV4TTR(NewOrbit(VesselOrbit, dV, StartUT), TargetOrbit, C.MaxTTR, C.MaxDeltaV, ToOrbit.MinApR, StartUT);
            if(!dV.IsZero())
            {
                add_node_abs(dV, StartUT);
                CFG.AP1.On(Autopilot1.Maneuver);
            }
            stage = Stage.StartOrbit;
        }

        class Launch
        {
            public double UT;
            public double Transfer;
            public double ApR;
            public double ApAArc;
            public Vector3d ApV, StartPos;
            public double Dist;
            public Vector3d hVdir, Norm;
            readonly RendezvousAutopilot REN;

            public Launch(RendezvousAutopilot ren, double startUT, double transfer, double minApR, double maxApR, double ApAArc)
            {
                REN = ren;
                this.ApAArc = ApAArc;
                var tN = ren.TargetOrbit.GetOrbitNormal();
                var ApAUT = transfer < 0 ? startUT : startUT + transfer;
                StartPos = BodyRotationAtdT(ren.Body, startUT - ren.VSL.Physics.UT) * ren.VesselOrbit.pos;
                hVdir = Vector3d.Cross(StartPos, tN).normalized;
                Norm = Vector3d.Cross(hVdir, StartPos).normalized;
                UT = startUT;
                ApR = Utils.Clamp(ren.TargetOrbit.getRelativePositionAtUT(ApAUT).magnitude, minApR, maxApR);
                ApV = QuaternionD.AngleAxis(ApAArc * Mathf.Rad2Deg, Norm) * StartPos.normalized * ApR;
                Transfer = -1;
                Dist = double.MaxValue;
            }

            public IEnumerable<double> CalculateTransfer()
            {
                var sim = REN.sim.FromSurfaceTTA(REN.ManeuverOffset, ApR - REN.Body.Radius,
                                                 ApAArc, REN.ToOrbit.MaxG, 
                                                 -(float)Vector3d.Dot(REN.Body.zUpAngularVelocity, Norm));
                foreach(var t in sim)
                {
                    if(t > 0)
                    {
                        Transfer = t;
                        break;
                    }
                    yield return -1;
                    continue;
                }
                Dist = (ApV - REN.TargetOrbit.getRelativePositionAtUT(UT + Transfer)).magnitude;
                yield return Transfer;
            }

            public static bool operator <(Launch a, Launch b) => a.Dist < b.Dist;
            public static bool operator >(Launch a, Launch b) => a.Dist > b.Dist;
            public static implicit operator bool(Launch l) => l.Transfer < 0;

            public override string ToString()
            {
                return Utils.Format("StartT {}, Transfer {}, Dist {}, ApR {}",
                                    UT-REN.VSL.Physics.UT, Transfer, Dist, ApR);
            }
        }

        double inclinationDelta(double UT)
        {
            var tN = TargetOrbit.GetOrbitNormal();
            var startPos = BodyRotationAtdT(Body, UT - VSL.Physics.UT) * VesselOrbit.pos;
            return Math.Abs(Utils.Angle2(tN, startPos) - 90);
        }

        double maxInclinationDelta(double startUT, double dT)
        {
            var maxIncUT = startUT;
            var maxInc = 0.0;
            while(Math.Abs(dT) > 0.01)
            {
                var dInc = inclinationDelta(startUT);
                if(dInc > maxInc)
                {
                    maxInc = dInc;
                    maxIncUT = startUT;
                }
                startUT = startUT + dT;
                if(!maxInc.Equals(dInc))
                {
                    dT /= -2.1;
                    startUT = maxIncUT + dT;
                }
            }
            return maxInc;
        }

        double findInPlaneUT(double startUT, double dT)
        {
            var inPlaneUT = startUT;
            var bestInc = double.MaxValue;
            while(Math.Abs(dT) > 0.01)
            {
                var dInc = inclinationDelta(startUT);
                if(dInc < bestInc)
                {
                    bestInc = dInc;
                    inPlaneUT = startUT;
                    if(bestInc < 1e-4) break;
                }
                startUT = startUT + dT;
                if(!bestInc.Equals(dInc))
                {
                    dT /= -2.1;
                    startUT = inPlaneUT + dT;
                }
            }
            return inPlaneUT >= VSL.Physics.UT ? inPlaneUT :
                findInPlaneUT(inPlaneUT + Body.rotationPeriod / 2, 60);
        }

        ComputationBalancer.Task launch_window_calculator;
        IEnumerator<int> calculate_launch_window()
        {
            ToOrbit.Reset();
            sim.Init();
            Launch best = null;
            var minApR = ToOrbit.MinApR;
            var maxApR = ToOrbit.MaxApR;
            var maxT = 3600*MaxDays*(GameSettings.KERBIN_TIME? 6 : 24);
            var ApAArc = Mathf.Lerp(C.TargetArc.Max, C.TargetArc.Min, Steepness.Value / 100) * Mathf.Deg2Rad;
            if(!StartInPlane)
            {
                var startUT = VSL.Physics.UT;
                var endUT = startUT + maxT;
                var tEndUT = TargetOrbit.GetEndUT();
                if(DiscontiniousOrbit(LastOrbit(TargetOrbit)))
                    endUT = tEndUT;
                else if(!double.IsInfinity(tEndUT))
                    endUT = Math.Max(endUT, tEndUT);
                var dT = Math.Min((endUT - startUT) / 10, Body.rotationPeriod / 10);
                //search for the nearest approach start UT
                //first scan startUT for possible minima
                var minDis = new MinimumD();
                var minima = new List<double>();
                Launch cur = null;
                while(startUT < endUT)
                {
                    Status("{0} searching for possible launch windows:{1:F0}", 
                           ProgressIndicator.Get, (endUT - startUT) / dT);
#if DEBUG
                    if(setp_by_step_computation &&
                       !string.IsNullOrEmpty(TCAGui.StatusMessage))
                    {
                        yield return 0;
                        continue;
                    }
#endif
                    cur = new Launch(this, startUT, cur == null ? -1 : cur.Transfer, minApR, maxApR, ApAArc);
                    foreach(var t in cur.CalculateTransfer()) yield return 0;
                    minDis.Update(cur.Dist);
                    if(minDis) minima.Add(startUT);
                    startUT += dT;
#if DEBUG
                    if(setp_by_step_computation)
                        Status("Push to proceed");
#endif
                    yield return 0;
                }
                //then find each of the minima exactly and choose the best
                for(int i = 0, minimaCount = minima.Count; i < minimaCount; i++)
                {
                    Launch min = null;
                    var m = minima[i];
                    startUT = m;
                    dT = 100;
                    while(Math.Abs(dT) > 0.01)
                    {
                        Status("{0} checking possible launch windows: {1}/{2}", 
                               ProgressIndicator.Get, i+1, minimaCount);
#if DEBUG
                        if(setp_by_step_computation &&
                           !string.IsNullOrEmpty(TCAGui.StatusMessage))
                        {
                            yield return 0;
                            continue;
                        }
#endif
                        cur = new Launch(this, startUT, cur == null ? -1 : cur.Transfer, minApR, maxApR, ApAArc);
                        foreach(var t in cur.CalculateTransfer()) yield return 0;
                        if(min == null || cur < min) min = cur;
                        startUT += dT;
                        if(startUT < VSL.Physics.UT || startUT > endUT || cur != min)
                        {
                            dT /= -2.1;
                            startUT = Utils.Clamp(min.UT + dT, VSL.Physics.UT, endUT);
                        }
#if DEBUG
                        if(setp_by_step_computation)
                            Status("Push to proceed");
#endif
                        yield return 0;
                    }
                    //Log("Min.launch: {}\nBest.launch: {}\nincl.delta: {} < {}, max dist: {}\nmin better: {} && {} && {}", 
                        //min, best, inclinationDelta(min.UT), IncDelta.Value, MaxDist.Value,
                        //min.Dist < MaxDist*1000,
                        //inclinationDelta(min.UT) < IncDelta, 
                        //(best == null || min < best));//debug
                    if(min.Dist < MaxDist*1000 &&
                       inclinationDelta(min.UT) < IncDelta && 
                       (best == null || min < best))
                        best = min;
                }
            }
            //if the closest approach is too far away or too out of plane with the target,
            //start when in plane
            if(best == null)
            {
                Utils.Message("No launch window was found for direct rendezvous.\n\n" +
                              "Launching in plane with the target...");
                var first_in_plane_UT = findInPlaneUT(VSL.Physics.UT, Body.rotationPeriod / 10);
                var in_plane_UT = first_in_plane_UT;
                var proj_anlgle = -Utils.ProjectionAngle(
                    TargetOrbit.getRelativePositionAtUT(in_plane_UT),
                    BodyRotationAtdT(Body, in_plane_UT-VSL.Physics.UT)*VesselOrbit.pos,
                    TargetOrbit.getOrbitalVelocityAtUT(in_plane_UT)
                );
                while(Math.Abs(proj_anlgle) > 30 && in_plane_UT-VSL.Physics.UT < maxT)
                {
                    //Log("proj_angle {}, time2launch {}", 
                        //proj_anlgle, in_plane_UT-VSL.Physics.UT);//debug
                    Status("{0} choosing optimal in-plane launch window: {1:P0}", 
                           ProgressIndicator.Get, (in_plane_UT-VSL.Physics.UT) / maxT);
                    yield return 0;
                    in_plane_UT = findInPlaneUT(in_plane_UT+Body.rotationPeriod/2, Body.rotationPeriod / 10);
                    proj_anlgle = -Utils.ProjectionAngle(
                        TargetOrbit.getRelativePositionAtUT(in_plane_UT),
                        BodyRotationAtdT(Body, in_plane_UT-VSL.Physics.UT)*VesselOrbit.pos,
                        TargetOrbit.getOrbitalVelocityAtUT(in_plane_UT)
                    );
                }
                //Log("proj_angle {}, time2launch {}", 
                    //proj_anlgle, in_plane_UT-VSL.Physics.UT);//debug
                var ApR = minApR;
                if(proj_anlgle < 0)
                {
                    if(proj_anlgle < -30)
                        in_plane_UT = first_in_plane_UT;
                    ApR = maxApR;
                }
                best = new Launch(this, in_plane_UT, -1, ApR, ApR, ApAArc);
                ToOrbit.InPlane = true;
                ToOrbit.CorrectOnlyAltitude = true;

            }
            else
                ToOrbit.ApAUT = best.UT + best.Transfer;
            ToOrbit.LaunchUT = best.UT;
            ToOrbit.Target = best.ApV;
        }

        void to_orbit()
        {
            #if DEBUG
            startUT = -1;
            #endif
            //setup launch
            CFG.DisableVSC();
            if(VSL.LandedOrSplashed)
            {
                launch_window_calculator = ComputationBalancer.AddTask(calculate_launch_window());
                stage = Stage.PreLaunch;
            }
            else
            {
                //calculate target vector
                var proj_anlgle = Utils.ProjectionAngle(VesselOrbit.pos, TargetOrbit.pos, TargetOrbit.vel);
                var ApR = proj_anlgle > 0? Math.Max(ToOrbit.MinApR, VesselOrbit.ApR) : ToOrbit.MaxApR;
                var hVdir = Vector3d.Cross(VesselOrbit.pos, TargetOrbit.GetOrbitNormal()).normalized;
                var ascO = AscendingOrbit(ApR, hVdir, ToOrbitAutopilot.C.LaunchSlope);
                MinDist.Reset();
                ToOrbit.Reset();
                ToOrbit.LaunchUT = VSL.Physics.UT;
                ToOrbit.Target = ToOrbitIniApV = ascO.getRelativePositionAtUT(VSL.Physics.UT+ascO.timeToAp);
                ToOrbit.InPlane = true;
                ToOrbit.CorrectOnlyAltitude = true;
                ToOrbit.StartGravityTurn();
                stage = Stage.ToOrbit;
            }
        }

        void match_orbits()
        {
            VSL.Controls.StopWarp();
            SetTarget(CFG.Target);
            update_trajectory();
            clear_nodes();
            add_target_node();
            CFG.AP1.On(Autopilot1.Maneuver);
            stage = Stage.MatchOrbits;
            MAN.MinDeltaV = 0.5f;
        }

        void approach()
        {
            CFG.AT.On(Attitude.TargetCorrected);
            stage = Stage.Approach;
        }

        void approach_or_brake()
        {
            if(RelPos.magnitude - VSL.Geometry.MinDistance > C.Dtol)
                approach();
            else
                brake();
        }

        void brake()
        {
            SetTarget(CFG.Target);
            stage = Stage.Brake;
            var dV = -RelVel;
            var dVm = dV.magnitude;
            var dist = TargetVessel.CurrentCoM - VSL.Physics.wCoM;
            var distm = dist.magnitude - VSL.Geometry.MinDistance;
            if(distm < C.Dtol && dVm < ThrottleControl.C.MinDeltaV * 2) return;
            if(distm > C.Dtol && Vector3.Dot(dist, dV.xzy) > 0)
                CFG.AP1.On(Autopilot1.MatchVelNear);
            else CFG.AP1.On(Autopilot1.MatchVel);
        }

        void next_stage()
        {
            update_trajectory();
            var rel_dist = RelPos.magnitude;
            var direct_approach = new RendezvousTrajectory(VSL,
                                                           RelVel +
                                                           RelPos/rel_dist * C.MaxApproachV,
                                                           VSL.Physics.UT, CFG.Target);
            rel_dist -= VSL.Geometry.MinDistance;
            if(rel_dist < C.Dtol)
                brake();
            else if(CurrentDistance < C.Dtol*Utils.Clamp(VSL.Torque.NoEngines.TurnTime, 2, 10) &&
                    (!TargetLoaded || trajectory.BrakeDeltaV.magnitude > 1))
                match_orbits();
            else if(TargetLoaded)
            {
                //Log("cur.dist {} > rel.dist/2 {}, direct approach: {}", 
                    //CurrentDistance, rel_dist/2, direct_approach);//debug
                if(direct_approach.FullBrake &&
                   CurrentDistance > rel_dist/2)
                    approach_or_brake();
                else
                {
                    var time_ratio = direct_approach.TimeToTarget/trajectory.TimeToTarget;
                    var dV_ratio = trajectory.GetTotalDeltaV()/direct_approach.GetTotalDeltaV();
                    //Log("time ratio {}, dV ratio {}",  time_ratio, dV_ratio);//debug
                    if(time_ratio < dV_ratio)
                        approach_or_brake();
                    else
                        match_orbits();
                }
            }
            else
            {
                if(mode != Mode.DeltaV &&
                   direct_approach.FullBrake &&
                   direct_approach.DistanceToTarget < C.Dtol &&
                   direct_approach.ManeuverDeltaV.magnitude < C.MaxApproachV * 2)
                    approach_or_brake();
                else if(CurrentDistance / VesselOrbit.semiMajorAxis > C.CorrectionStart)
                    compute_rendezvou_trajectory();
                else
                    fine_tune_approach();
            }
        }

        protected override void update_trajectory()
        {
            base.update_trajectory();
            CurrentDistance = trajectory.DistanceToTarget;
        }

        protected override void Reset()
        {
            base.Reset();
            stage = Stage.None;
            CorrectingManeuver = false;
            CurrentDistance = -1;
            optimizer = null;
            CFG.AP1.Off();
            ToOrbit.Reset();
        }

        protected override void UpdateState()
        {
            base.UpdateState();
            IsActive &= CFG.AP2[Autopilot2.Rendezvous] && TargetOrbit != null;
            ControlsActive &= IsActive ||
                VSL.TargetVessel != null;
        }

#if DEBUG
        double startUT = -1;
        Vector3d y = Vector3d.zero;
        Vector3d x = Vector3d.zero;
        void log_flight()
        {
            var r = VesselOrbit.pos;
            var r1 = ToOrbit.Target;
            if(startUT < 0) 
                startUT = VSL.Physics.UT;
            if(y.IsZero()) 
            {
                y = r.normalized;
                //pos x [fwd x pos] = fwd(pos*pos) - pos(fwd*pos)
                x = (r1*y.magnitude-y*Vector3d.Dot(r1, y)).normalized;
            }
            CSV(VSL.Physics.UT - startUT, 
                new Vector2d(Vector3d.Dot(r,x), Vector3d.Dot(r,y)));
        }

        struct OptRes
        {
            public double dV;
            public double dist;
            public double endUT;
            public void Update(RendezvousTrajectory t)
            {
                dV = t.GetTotalDeltaV();
                dist = t.DistanceToTarget;
                endUT = t.AtTargetUT;
            }
            public override string ToString()
            {
                return string.Format("dV {0} m/s, dist {1} m", dV, dist);
            }
        }
#endif

        protected override void Update()
        {
            switch(stage)
            {
            case Stage.Start:
                if(VSL.InOrbit &&
                   VesselOrbit.radius > MinR)
                    start_orbit();
                else to_orbit();
                break;
            case Stage.PreLaunch:
                if(launch_window_calculator != null)
                {
                    if(!launch_window_calculator.finished) break;
                    launch_window_calculator = null;
                }
                stage = Stage.Launch;
                break;
            case Stage.Launch:
                if(ToOrbit.LaunchUT > VSL.Physics.UT)
                {
                    TmpStatus("Waiting for launch window...");
                    ToOrbit.UpdateTargetPosition();
                    VSL.Info.Countdown = ToOrbit.LaunchUT - VSL.Physics.UT;
                    VSL.Controls.WarpToTime = ToOrbit.LaunchUT;
                    break;
                }
                //log_flight();//debug
                if(ToOrbit.Liftoff()) break;
                stage = Stage.ToOrbit;
                MinDist.Reset();
                break;
            case Stage.ToOrbit:
                //log_flight();//debug
                if(ToOrbit.InPlane ?
                   ToOrbit.GravityTurn(C.Dtol) :
                   ToOrbit.TargetedGravityTurn(C.Dtol))
                {
                    if(!ToOrbit.InPlane && VesselOrbit.ApR > ToOrbit.MinApR)
                    {
                        update_trajectory();
                        if(CurrentDistance < MaxDist)
                        {
                            MinDist.Update(CurrentDistance);
                            if(MinDist) 
                            {
                                start_orbit();
                                break;
                            }
                        }
                        //var arcError = Utils.ProjectionAngle(trajectory.AtTargetPos, 
                        //                                     trajectory.TargetPos,
                        //                                     trajectory.AtTargetVel);
                        ////Log("Target angle correction: {}", -arcError);//debug
                        //ToOrbit.Target = QuaternionD.AngleAxis(-arcError * TimeWarp.fixedDeltaTime,
                            //                                   VesselOrbit.GetOrbitNormal())
                            //* ToOrbit.Target;
                    }
                }
                else start_orbit();
                break;
            case Stage.StartOrbit:
                TmpStatus("Achiving orbit...");
                if(CFG.AP1[Autopilot1.Maneuver]) break;
                next_stage();
                break;
            case Stage.ComputeSuborbitRendezvous:
                if(!trajectory_computed()) break;
                optimizer = null;
                var EndApR = trajectory.EndOrbit.ApR;
                if(!trajectory.KillerOrbit &&
                   trajectory.RelDistanceToTarget < C.CorrectionStart &&
                   Math.Abs(trajectory.AtTargetPos.magnitude - EndApR) / EndApR < 0.1)
                {
                    clear_nodes();
                    add_trajectory_node_rel();
                    if(VSL.Physics.UT < trajectory.StartUT-ManeuverOffset &&
                       VesselOrbit.radius < Body.Radius + Body.atmosphereDepth)
                    {
                        TmpStatus("Coasting...");
                        CFG.AT.OnIfNot(Attitude.Prograde);
                        break;
                    }
                    CFG.AP1.On(Autopilot1.Maneuver);
                    stage = Stage.Rendezvou;
                }
                else circularize();
                break;
            case Stage.ComputeRendezvou:
                if(!trajectory_computed()) break;
                optimizer = null;
                if(!trajectory.KillerOrbit &&
                    trajectory.RelDistanceToTarget < C.CorrectionStart)
                {
                    clear_nodes();
                    add_trajectory_node_rel();
                    CFG.AP1.On(Autopilot1.Maneuver);
                    stage = Stage.Rendezvou;
                }
                else
                {
                    Status("red", "Failed to compute rendezvou trajectory.\nPlease, try again.");
                    Disable();
                }
                break;
            case Stage.ComputeCorrection:
                if(TimeWarp.CurrentRateIndex == 0 && TimeWarp.CurrentRate > 1)
                {
                    TmpStatus("Waiting for Time Warp to end...");
                    break;
                }
                if(!trajectory_computed()) break;
                if(!trajectory.KillerOrbit &&
                   trajectory.RelDistanceToTarget < C.CorrectionStart &&
                   trajectory.DistanceToTarget < CurrentDistance &&
                   trajectory.TimeToTarget > trajectory.BrakeDuration + trajectory.ManeuverDuration + ManeuverOffset + TimeWarp.CurrentRate)
                {
                    if(trajectory.ManeuverDeltaV.sqrMagnitude > 0.8)
                    {
                        VSL.Controls.StopWarp();
                        if(TimeWarp.CurrentRate > 1 ||
                           trajectory.TimeToStart < trajectory.ManeuverDuration / 2 + VSL.Torque.NoEngines.TurnTime)
                        {
                            update_trajectory();
                            fine_tune_approach();
                        }
                        else
                        {
                            clear_nodes();
                            add_trajectory_node_rel();
                            CFG.AP1.On(Autopilot1.Maneuver);
                            stage = Stage.Rendezvou;
                        }
                    }
                    else
                    {
                        update_trajectory();
                        CorrectionTimer.Period = Math.Max(trajectory.TimeToTarget / 100, 10);
                        CorrectionTimer.Reset();
                        stage = Stage.Coast;
                    }
                }
                else if(CurrentDistance / VesselOrbit.semiMajorAxis < C.CorrectionStart)
                    match_orbits();
                else
                    compute_rendezvou_trajectory();
                break;
            case Stage.Rendezvou:
                TmpStatus("Executing rendezvous maneuver...");
                if(CFG.AP1[Autopilot1.Maneuver]) break;
                next_stage();
                break;
            case Stage.Coast:
                TmpStatus("Coasting...");
                if(!CorrectionTimer.TimePassed)
                {
                    if(VSL.Controls.CanWarp)
                        VSL.Controls.WarpToTime = VSL.Physics.UT + (trajectory.TimeToTarget - trajectory.BrakeDuration + ManeuverOffset) / 2;
                    break;
                }
                CorrectionTimer.Reset();
                update_trajectory();
                fine_tune_approach();
                break;
            case Stage.MatchOrbits:
                TmpStatus("Matching orbits at nearest approach...");
                update_trajectory();
                if(CFG.AP1[Autopilot1.Maneuver])
                {
                    if(trajectory.TimeToTarget > 0 &&
                       (CorrectingManeuver || trajectory.DirectHit && TargetLoaded))
                    {
                        var threshold = VSL.Geometry.MinDistance * 2 - trajectory.AtTargetRelPos.magnitude;
#if DEBUG
                        if(!CorrectingManeuver && threshold > 0)
                        {
                            Emailer.SendLocalSelf("allista@gmail.com", "TCA.REN: Proximity Alert!",
                                                  "ETA: {} s\ndV: {} m/s\nDist {} m\nEnd Dist: {} m",
                                                  trajectory.TimeToTarget,
                                                  RelVel.magnitude,
                                                  RelPos.magnitude,
                                                  trajectory.AtTargetRelPos.magnitude);
                        }
#endif
                        VSL.Controls.StopWarp();
                        if(threshold > 0)
                        {
                            TmpStatus("Matching orbits at nearest approach...\n" +
                                      "<color=yellow><b>PROXIMITY ALERT!</b></color> Clearence {0:F1} m",
                                      trajectory.DistanceToTarget);
                            var correction = Vector3d.Exclude(RelPos, -trajectory.AtTargetRelPos).normalized *
                                threshold / Utils.Clamp(trajectory.TimeToTarget / 2, 0.1, 10);
                            if(CorrectingManeuver)
                                Correction.Update(correction);
                            else
                                Correction.Set(correction);
                            MAN.AddCourseCorrection(Correction);
                            MAN.ThrustWhenAligned = false;
                            CorrectingManeuver = true;
                        }
                        else
                        {
                            CorrectingManeuver = false;
                            Correction.Reset();
                            clear_nodes();
                            add_target_node();
                            MAN.UpdateNode();
                        }
                    }
                    break;
                }
                if(MAN.ManeuverStage == ManeuverAutopilot.Stage.IN_PROGRESS)
                {
                    //Log("Resuming IN_PROGRESS maneuver");//debug
                    clear_nodes();
                    add_node_abs(RelVel, VSL.Physics.UT);
                    CFG.AP1.On(Autopilot1.Maneuver);
                    break;
                }
                next_stage();
                break;
            case Stage.Approach:
                TmpStatus("Approaching...");
                THR.DeltaV = 0;
                var rel_pos = RelPos;
                var dist = rel_pos.magnitude;
                if(dist - VSL.Geometry.MinDistance < C.Dtol)
                { brake(); break; }
                var throttle = VSL.Controls.Aligned? VSL.Controls.AlignmentFactor : 0;
                if(throttle.Equals(0)) break;
                var dVr = Vector3d.Dot(-RelVel, rel_pos / dist);
                var nVm = Utils.Clamp((dist - VSL.Geometry.MinDistance) / 
                                      (TrajectoryCalculator.C.ManeuverOffset+
                                       VSL.Torque.NoEngines.RotationTime3Phase(180, 0.05f)),
                                      1, C.MaxApproachV);
                if(dVr < nVm)
                {
                    VSL.Engines.ActivateEngines();
                    THR.DeltaV = (float)Math.Max(nVm - dVr, 1) * throttle;
                }
                else
                {
                    MVA.MinDeltaV = 0.5f;
                    brake();
                }
                break;
            case Stage.Brake:
                if(CFG.AP1[Autopilot1.MatchVelNear]) 
                {
                    TmpStatus("Braking near target...");
                    break;
                }
                if(CFG.AP1[Autopilot1.MatchVel])
                {
                    TmpStatus("Braking...");
                    if((RelVel).magnitude > MAN.MinDeltaV*1.1f) break;
                    CFG.AP1.Off();
                    THR.Throttle = 0;
                }
                if((VSL.Physics.wCoM - TargetVessel.CurrentCoM).magnitude - VSL.Geometry.MinDistance > C.Dtol)
                { approach(); break; }
                if(RelVel.magnitude > ThrottleControl.C.MinDeltaV*2)
                { brake(); break; }
                Disable();
                CFG.AT.OnIfNot(Attitude.KillRotation);
                ClearStatus();
                break;
            }
        }

        static GUIContent button_content = new GUIContent("Rendezvous", "Compute and perform a rendezvous maneuver, then brake near the target.");
        public override void Draw()
        {
#if DEBUG
            if(CFG.Target && Body != null && TargetOrbit != null)
            {
                Utils.GLVec(Body.position, ToOrbit.Target.xzy, Color.green);
                Utils.GLVec(Body.position, Vector3d.Cross(VesselOrbit.pos, ToOrbit.Target).normalized.xzy * Body.Radius * 1.1, Color.red);
                Utils.GLVec(Body.position, TargetOrbit.getRelativePositionAtUT(ToOrbit.ApAUT).xzy, Color.yellow);
                Utils.GLVec(Body.position, TargetOrbit.getRelativePositionAtUT(VSL.Physics.UT + VesselOrbit.timeToAp).xzy, new Color(0, 0.3f, 1));
                Utils.GLVec(Body.position, VesselOrbit.getRelativePositionAtUT(VSL.Physics.UT + VesselOrbit.timeToAp).xzy, Color.magenta);
                Utils.GLVec(Body.position, VesselOrbit.GetOrbitNormal().normalized.xzy * Body.Radius * 1.1, Color.cyan);
                Utils.GLVec(Body.position, TargetOrbit.GetOrbitNormal().normalized.xzy * Body.Radius * 1.1, new Color(1, 0.3f, 0));
                //Utils.GLVec(VSL.vessel.transform.position, SurfaceVel.xzy, new Color(0, 0.3f, 1));
            }
#endif
            if(ControlsActive)
            {
                if(CFG.AP2[Autopilot2.Rendezvous])
                    GUILayout.Label(button_content, Styles.enabled_button, GUILayout.ExpandWidth(true));
                else if(GUILayout.Button(button_content, Styles.active_button, GUILayout.ExpandWidth(true)))
                    ShowOptions = !ShowOptions;
            }
            else GUILayout.Label(button_content, Styles.inactive_button, GUILayout.ExpandWidth(true));
        }

        public void DrawOptions()
        {
            GUILayout.BeginHorizontal();
            GUILayout.BeginVertical();
            if(VSL.LandedOrSplashed && !CFG.AP2)
            {
                var in_plane = StartInPlane;
                GUILayout.BeginHorizontal();
                if(Utils.ButtonSwitch("Start In Plane", in_plane,
                                      "Launch in plane with the target, then rendezvous from orbit.",
                                      GUILayout.ExpandWidth(true)))
                    in_plane = true;
                if(Utils.ButtonSwitch("Attempt direct rendezvous", !in_plane,
                                      "Try to find a launch window to rendezvous with the target " +
                                      "directly.",
                                      GUILayout.ExpandWidth(true)))
                    in_plane = false;
                GUILayout.EndHorizontal();
                GUILayout.BeginHorizontal();
                GUILayout.Label(new GUIContent("Max. Days to Launch:",
                                               "Maximum allowed days untill launch. If no suitable " +
                                               "launch window is found within that period, launch when " +
                                               "in plane with the target."),
                                GUILayout.ExpandWidth(true));
                MaxDays.Draw("d", 5, "F0", suffix_width: 25);
                GUILayout.EndHorizontal();
                if(StartInPlane)
                    ToOrbit.DrawOptions();
                else
                {
                    GUILayout.BeginHorizontal();
                    GUILayout.BeginVertical();
                    GUILayout.Label(new GUIContent("Gravity Turn Sharpness:",
                                                   "How sharp the gravity turn will be. " +
                                                   "Used only when direct rendezvous is possible."),
                                    GUILayout.ExpandWidth(true));
                    GUILayout.Label(new GUIContent("Max. Distance:",
                                                   "Maximum allowed distance to the target at apoapsis."),
                                    GUILayout.ExpandWidth(true));
                    GUILayout.Label(new GUIContent("Max. Inclination Delta:",
                                                   "Maximum allowed difference between initial " +
                                                   "vessel orbit and target orbit. If that requirement " +
                                                   "is not met, launch in plane with the target orbit."),
                                    GUILayout.ExpandWidth(true));
                    GUILayout.EndVertical();
                    GUILayout.BeginVertical();
                    Steepness.Draw("%", 5, "F0", suffix_width: 25);
                    MaxDist.Draw("km", 5, "F0", suffix_width: 25);
                    IncDelta.Draw("°", 1, "F0", suffix_width: 25);
                    GUILayout.EndVertical();
                    GUILayout.EndHorizontal();
                }
                StartInPlane = in_plane;
            }
            GUILayout.BeginHorizontal();
            if(computing)
                GUILayout.Label(new GUIContent("Search Mode: " + ModeNames[(int)mode], ModeDesc[(int)mode]),
                                Styles.grey, GUILayout.ExpandWidth(true));
            else
            {
                GUILayout.Label("Search Mode:", GUILayout.ExpandWidth(false));
                var choice = Utils.LeftRightChooser(ModeNames[(int)mode], ModeDesc[(int)mode]);
                if(choice > 0) mode = (Mode)(((int)mode + 1) % NumModes);
                if(choice < 0) mode = (Mode)(mode > 0 ? (int)mode - 1 : NumModes - 1);
            }
            GUILayout.EndHorizontal();
            if(stage == Stage.ToOrbit)
                ToOrbit.DrawInfo();
            GUILayout.EndVertical();
            GUILayout.FlexibleSpace();
            if(CFG.AP2[Autopilot2.Rendezvous])
            {
                if(GUILayout.Button(new GUIContent("Abort", "Abort Rendezvous Autopilot"),
                                    Styles.danger_button, GUILayout.Width(60)))
                    CFG.AP2.XOff();
            }
            else
            {
                if(GUILayout.Button(new GUIContent("Start", "Start Rendezvous Autopilot"),
                                    Styles.enabled_button, GUILayout.Width(60)))
                    CFG.AP2.XOn(Autopilot2.Rendezvous);
            }
            GUILayout.EndHorizontal();
        }

        public void DrawBestTrajectories()
        {
            if(optimizer != null && mode == Mode.Manual)
                optimizer.DrawBestTrajecotries();
        }
    }
}
