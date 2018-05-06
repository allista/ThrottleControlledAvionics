//   RendezvousTrajectoryOptimizers.cs
//
//  Author:
//       Allis Tauri <allista@gmail.com>
//
//  Copyright (c) 2018 Allis Tauri
using System;
using System.Collections.Generic;
using UnityEngine;
using AT_Utils;

namespace ThrottleControlledAvionics
{
    public partial class RendezvousAutopilot
    {
        #region optimizers
        abstract class RendezvousTrajectoryOptimizer : TrajectoryOptimizer
        {
            protected RendezvousAutopilot ren;
            protected double minStartUT, maxStartUT, maxEndUT, dT;
            protected bool softMaxStart;

            protected RendezvousTrajectoryOptimizer(RendezvousAutopilot ren,
                                                    double minStartUT, double maxStartUT,
                                                    double maxEndUT, bool softMaxStart, double dT)
            {
                this.minStartUT = minStartUT;
                this.maxStartUT = maxStartUT;
                this.maxEndUT = maxEndUT;
                this.softMaxStart = softMaxStart;
                this.dT = dT;
                this.ren = ren;
            }

            public RendezvousTrajectory Best { get; protected set; }

            public abstract IEnumerator<RendezvousTrajectory> GetEnumerator();

            System.Collections.IEnumerator System.Collections.IEnumerable.GetEnumerator()
            { return GetEnumerator(); }

            protected bool trajectory_is_better(RendezvousTrajectory t)
            {
                //always accept the first trajecotry
                if(Best == null) return true;
                //between two killers choose the one with greater PeR
                if(t.KillerOrbit && Best.KillerOrbit)
                    return t.Orbit.PeR > Best.Orbit.PeR;
                //if current is a killer, it's no better
                if(t.KillerOrbit) return false;
                //if best is still a killer, any non-killer is better
                if(Best.KillerOrbit) return true;
                //if best is has negligable maneuver dV, or current misses the target,
                //compare qualities
                var bestDeltaV = Best.ManeuverDeltaV.sqrMagnitude;
                if(bestDeltaV > 1 || t.DistanceToTarget > C.Dtol)
                    return t.Quality < Best.Quality;
                //otherwise select the one with larger maneuver dV
                return t.ManeuverDeltaV.sqrMagnitude > bestDeltaV;
            }

            protected string BestDesc
            {
                get
                {
                    var tts = Best.TimeToStart;
                    return string.Format("T- {0}  ETA <color=lime>{1}</color>  dV: <color=yellow><b>{2:F1}</b> m/s</color>",
                                         Utils.formatTimeDelta(tts),
                                         Utils.formatTimeDelta(tts + Best.TransferTime),
                                         Best.GetTotalDeltaV());
                }
            }

            public virtual string Status
            {
                get
                {
                    var s = ProgressIndicator.Get + " searching for the best trajectory";
                    if(Best == null) return s + "...";
                    return s + "\n" + BestDesc;
                }
            }
        }

        class TransferOptimizer : RendezvousTrajectoryOptimizer
        {
            readonly double startUT, transferT;

            public TransferOptimizer(RendezvousAutopilot ren,
                                     double maxEndUT, double startUT, double transfer, double dT)
                : base(ren, ren.VSL.Physics.UT + ren.CorrectionOffset, maxEndUT, maxEndUT, false, dT)
            {
                this.startUT = startUT;
                this.transferT = transfer;
            }

            public override IEnumerator<RendezvousTrajectory> GetEnumerator()
            {
                var transfer = transferT;
                var start = startUT;
                var dt = dT;
                var scanned = false;
                while(Math.Abs(dt) > 1e-5)
                {
                    var t = ren.new_trajectory(start, transfer);
                    var is_better = trajectory_is_better(t);
                    if(is_better) Best = t;
                    minStartUT = ren.VSL.Physics.UT + ren.CorrectionOffset;
                    if(start - minStartUT < t.ManeuverDuration / 2)
                        start = minStartUT + t.ManeuverDuration / 2 +
                                              TimeWarp.fixedDeltaTime * Math.Max(GameSettings.FRAMERATE_LIMIT, 60);
                    transfer = Math.Max(t.TransferTime + dt, 0);
                    if(scanned && (start + transfer > maxEndUT || transfer < 1 || t.KillerOrbit ||
                                   !t.KillerOrbit && !Best.KillerOrbit && !is_better) ||
                       !scanned && start + transfer > maxEndUT)
                    {
                        dt /= -2.1;
                        transfer = Math.Max(Best.TransferTime + dt, 1);
                        scanned = true;
                    }
                    //ren.Log("startT {}, transfer {}, Trajectory {}",
                            //start-ren.VSL.Physics.UT, transfer, t);//debug
                    yield return t;
                }
            }
        }

        class StartTimeOptimizer : RendezvousTrajectoryOptimizer
        {
            readonly double startUT;

            public StartTimeOptimizer(RendezvousAutopilot ren,
                                      double startUT, double transfer, double dT)

                : base(ren, ren.VSL.Physics.UT + ren.CorrectionOffset, startUT + transfer, -1, false, dT)
            {
                this.startUT = startUT;
            }

            public override IEnumerator<RendezvousTrajectory> GetEnumerator()
            {
                var start = startUT;
                var dt = dT;
                var scanned = false;
                while(Math.Abs(dt) > 1e-5)
                {
                    var t = ren.new_trajectory(start, maxStartUT - start);
                    var is_better = trajectory_is_better(t);
                    if(is_better) Best = t;
                    minStartUT = ren.VSL.Physics.UT + ren.CorrectionOffset;
                    start = t.StartUT + dt;
                    if(scanned && (start >= maxStartUT || start <= minStartUT ||
                                   !t.KillerOrbit && !Best.KillerOrbit && !is_better) ||
                       !scanned && start >= maxStartUT)
                    {
                        dt /= -2.1;
                        start = Utils.Clamp(Best.StartUT + dt, minStartUT, maxStartUT);
                        scanned = true;
                    }
                    yield return t;
                }
            }
        }


        /// <summary>
        /// 2D optimizer for transfer trajectory based on 
        /// Conjugate Directions with Orthogonal Shift algorithm:
        /// https://arxiv.org/abs/1102.1347
        /// </summary>
        class CDOS_Optimizer2D : RendezvousTrajectoryOptimizer
        {
            struct Point : IEquatable<Point>
            {
                CDOS_Optimizer2D opt;
                public double start, transfer, distance;
                public RendezvousTrajectory trajectory;

                public Point(double s, double t, CDOS_Optimizer2D optimizer)
                {
                    start = s;
                    transfer = t;
                    distance = double.MaxValue;
                    trajectory = null;
                    opt = optimizer;
                }

                public void UpdateTrajectory(bool with_distance = false)
                {
                    trajectory = opt.ren.new_trajectory(start, transfer);
                    transfer = trajectory.TransferTime;
                    if(with_distance) UpdateDist();
                }

                public void UpdateDist()
                {
                    distance = trajectory.Quality;
                }

                public void Shift(double s, double t, bool with_distance = false)
                {
                    start += s;
                    transfer += t;
                    if(with_distance)
                        UpdateTrajectory(with_distance);
                }

                public static Point operator +(Point p, Vector2d step)
                {
                    var newP = p;
                    newP.Shift(step.x, step.y);
                    return newP;
                }

                public static Point operator +(Point p, Vector3d step)
                {
                    var newP = p;
                    newP.Shift(step.x, step.y);
                    return newP;
                }

                public Point Shifted(double s, double t, bool with_distance = false)
                {
                    var newP = this;
                    newP.Shift(s, t, with_distance);
                    return newP;
                }

                public static bool operator <(Point a, Point b)
                { return a.distance < b.distance; }

                public static bool operator >(Point a, Point b)
                { return a.distance < b.distance; }

                public static Vector2d Delta(Point a, Point b)
                { return new Vector2d(b.start - a.start, b.transfer - a.transfer); }

                public static double DistK(Point a, Point b)
                { return 1 - Math.Abs(a.distance - b.distance) / Math.Max(a.distance, b.distance); }

                public static bool Close(Point a, Point b)
                { return Math.Abs(a.start - b.start) < 10 && Math.Abs(a.transfer - b.transfer) < 10; }

                public bool Better(Point b)
                {
                    if(opt.ren.mode == Mode.TimeToTarget)
                        return
                            distance + (transfer + start - opt.minStartUT) / 100 <
                            b.distance + (b.transfer + b.start - opt.minStartUT) / 100;
                    return distance < b.distance;
                }

                public override string ToString()
                {
                    return Utils.Format("startUT {}, transfer {}, distance {}, trajectory {}",
                                        start, transfer, distance, trajectory);
                }

                public bool Draw(bool selected)
                {
                    GUILayout.BeginHorizontal();
                    var tts = trajectory.TimeToStart;
                    var label = string.Format("ETA:  <color=lime>{0}</color>\n" +
                                              "Node: <color={1}>{2}</color>",
                                              Utils.formatTimeDelta(tts + transfer),
                                              tts > opt.ren.ManeuverOffset ? "white" : "red",
                                              Utils.formatTimeDelta(tts));
                    var sel = GUILayout.Button(new GUIContent(label, "Press to select this transfer"),
                                               Styles.rich_label, GUILayout.ExpandWidth(false));
                    GUILayout.FlexibleSpace();
                    GUILayout.Label(string.Format("dV: <color=yellow><b>{0:F1}</b> m/s</color>", trajectory.GetTotalDeltaV()),
                                    Styles.rich_label, GUILayout.ExpandWidth(false));
                    GUILayout.FlexibleSpace();
                    if(selected) GUILayout.Label("<color=lime><b>●</b></color>",
                                                 Styles.rich_label, GUILayout.ExpandWidth(false));
                    GUILayout.EndHorizontal();
                    return sel;
                }

                #region IEquatable implementation
                public bool Equals(Point other)
                { return start.Equals(other.start) && transfer.Equals(other.transfer); }
                #endregion
            }

            Vector2d dir;
            Point P0, P;
            double dDist = -1;
            List<Point> start_points = new List<Point>();
            List<Point> best_points = new List<Point>();

            #if DEBUG
            void LogP(Point p, string tag = "")
            {
                //                Utils.Log("{}, startT {}, transfer {}, dist {}, dir.x {}, dir.y {}, dT {}, dDist {}",
                //                          tag, p.x-minUT, p.y, p.z, dir.x, dir.y, dT, dDist);
                DebugUtils.CSV("CDOS_test.csv", tag, p.start, p.transfer, p.distance, dir.x, dir.y, dDist, feasible_point(p), DateTime.Now.ToShortTimeString());
            }

            void LogP(string tag = "") { LogP(P, tag); }
            #endif

            public CDOS_Optimizer2D(RendezvousAutopilot ren,
                                    double minStartUT, double maxStartUT,
                                    double maxEndUT, bool softMaxStart,
                                    double startUT, double startTransfer, double dT)
                : base(ren, minStartUT, maxStartUT, maxEndUT, softMaxStart, dT)
            {
                P0 = P = new Point(Math.Max(startUT, minStartUT + 1), startTransfer, this);
                set_dir(new Vector2d(0, 1));
            }

            void set_dir(Vector2d new_dir)
            {
                if(new_dir.x.Equals(0) && new_dir.y.Equals(0))
                    new_dir[Math.Abs(dir.x) > Math.Abs(dir.y) ? 1 : 0] = 1;
                dir = new_dir;
            }

            Vector2d anti_grad()
            {
                var d0 = P.distance;
                var Px = P.Shifted(10, 0, true);
                var Py = P.Shifted(0, 10, true);
                return new Vector2d(d0 - Px.distance, d0 - Py.distance).normalized;
            }

            bool feasible_point(Point p)
            {
                var eobt = p.trajectory.EndOrbit;
                return !p.trajectory.KillerOrbit &&
                         p.start > minStartUT && p.transfer > 1 &&
                         (softMaxStart || p.start < maxStartUT) &&
                         (maxEndUT < 0 || p.start + p.transfer < maxEndUT) &&
                         (eobt.patchEndTransition == Orbit.PatchTransitionType.FINAL ||
                     eobt.EndUT - p.trajectory.AtTargetUT > p.trajectory.BrakeDuration + 1) &&
                         p.trajectory.ManeuverDeltaV.sqrMagnitude > 1;
            }

            IEnumerable<RendezvousTrajectory> find_first_point()
            {
                while(true)
                {
                    P.UpdateTrajectory();
                    yield return P.trajectory;
                    if(feasible_point(P)) break;
                    P.transfer += dT;
                }
                P.UpdateDist();
            }

            void add_end_point()
            {
                if(ren.TargetOrbit.patchEndTransition != Orbit.PatchTransitionType.FINAL)
                {
                    var p = P;
                    p.start = minStartUT + 1;
                    p.transfer = Math.Max(ren.TargetOrbit.EndUT - p.start - ren.ManeuverOffset, 2);
                    p.UpdateTrajectory(true);
                    if(feasible_point(p))
                        start_points.Add(p);
                }
            }

            IEnumerable<RendezvousTrajectory> scan_start_time()
            {
                var cur = P;
                var prev = P;
                var bestP = cur;
                var bestOK = feasible_point(bestP);
                var endUT = Math.Max(P.start + P.transfer * 2, maxStartUT);
                var maxTT = maxStartUT - minStartUT;
                var minZ = new MinimumD();
                while(cur.start < endUT)
                {
                    var requestedTT = cur.transfer;
                    cur.UpdateTrajectory(true);
                    var tOK = feasible_point(cur);
                    if((bestOK && tOK && cur < bestP) ||
                       !bestOK && (tOK || cur.trajectory.Orbit.PeR > bestP.trajectory.Orbit.PeR))
                    {
                        //always add the first feasable point
                        if(tOK && !bestOK) start_points.Add(cur);
                        bestP = cur;
                        bestOK = tOK;
                    }
                    if(bestOK)
                    {
                        minZ.Update(cur.distance);
                        if(minZ.True && feasible_point(prev))
                            start_points.Add(prev);
                        var step_k = Point.DistK(prev, cur);
                        if(step_k.Equals(0)) step_k = 1;
                        prev = cur;
                        cur.start += dT * step_k;
                    }
                    else
                    {
                        cur.transfer += dT;
                        if(cur.transfer > maxTT || requestedTT - cur.transfer > 1)
                        {
                            cur.transfer = 1;
                            cur.start += dT / 10;
                        }
                    }
                    yield return cur.trajectory;
                }
                if(!bestOK)
                {
                    P = bestP;
                    foreach(var t in find_first_point()) yield return t;
                    bestP = P;
                }
                if(!start_points.Contains(bestP))
                    start_points.Add(bestP);
                set_dir(new Vector2d(1, 0));
                for(int i = 0, minimaCount = start_points.Count; i < minimaCount; i++)
                {
                    P = start_points[i];
                    foreach(var t in find_minimum(dT, true)) yield return t;
                    start_points[i] = P;
                }
            }

            IEnumerable<RendezvousTrajectory> find_minimum(double dt, bool no_scan = false)
            {
                var bestP = P;
                var prev = P;
                var cur = P;
                var scan_finished = no_scan;
                var path = 0.0;
                const int endL = 50;
                dt = Math.Min(dt, 1000);
                while(Math.Abs(dt) > 1E-5)
                {
                    var step_k = Point.DistK(prev, cur);
                    prev = cur;
                    path += step_k;
                    cur += dir * dt * step_k;
                    var requestedTT = cur.transfer;
                    cur.UpdateTrajectory(true);
                    if(feasible_point(cur) && cur < bestP)
                    {
                        var dD = bestP.distance - cur.distance;
                        bestP = cur;
                        if(no_scan || !scan_finished || dD > 0.1 || dt > 0.1)
                        {
                            path = 0.0;
                            if(dir.x.Equals(0) && Math.Abs(cur.transfer - requestedTT) > 1) dt /= 2;
                            else if(scan_finished) dt *= 1.4;
                        }
                        else break;
                    }
                    else if(scan_finished)
                    {
                        if(cur.trajectory.ManeuverDuration > 0)
                        {
                            dt /= -2.1;
                            cur = bestP;
                        }
                    }
                    else
                        scan_finished |= cur.start < minStartUT || cur.transfer < 1 || path > endL;
                    yield return cur.trajectory;
                }
                P = bestP;
            }


            IEnumerable<RendezvousTrajectory> orto_shift(double shift_delta)
            {
                var shift_dir = new Vector2d(-dir.y, dir.x);
                shift_delta = Math.Min(shift_delta, 100);
                Point P1;
                while(Math.Abs(shift_delta) > 1e-5)
                {
                    P1 = P + shift_dir * shift_delta;
                    P1.UpdateTrajectory();
                    yield return P1.trajectory;
                    if(feasible_point(P1))
                    {
                        P1.UpdateDist();
                        P = P1;
                        break;
                    }
                    shift_delta /= -2;
                }
            }

            IEnumerable<RendezvousTrajectory> shift_and_find(double dt, double stride = 1)
            {
                P0 = P;
                foreach(var t in orto_shift(0.62 * dt)) yield return t;
                foreach(var t in find_minimum(dt, true)) yield return t;
                if(P0 < P)
                {
                    set_dir(Point.Delta(P, P0).normalized);
                    P = P0;
                }
                else set_dir(Point.Delta(P0, P).normalized);
                if(P.start < minStartUT + 1 && dir.x < 0)
                {
                    dir.x = 0.1;
                    dir.y = Math.Sign(dir.y);
                }
                foreach(var t in find_minimum(stride * dt)) yield return t;
                dDist = P.distance.Equals(double.MaxValue) || P0.distance.Equals(double.MaxValue) ?
                         -1 : Math.Abs(P.distance - P0.distance);
                if(P0 < P)
                {
                    P = P0;
                    yield return P.trajectory;
                }
            }

            IEnumerable<RendezvousTrajectory> build_conjugate_set(double dt)
            {
                set_dir(new Vector2d(0, 1));
                foreach(var t in find_minimum(dt)) yield return t;
                foreach(var t in shift_and_find(dt)) yield return t;
            }

            IEnumerable<RendezvousTrajectory> full_search(double dt)
            {
                foreach(var t in build_conjugate_set(dt))
                    yield return t;
                while(dt > 0.1 || dDist > 0.1 || dDist < 0)
                {
                    if(need_to_stop) yield break;
                    foreach(var t in shift_and_find(dt, 3)) yield return t;
                    dt = 0.3 * Point.Delta(P0, P).magnitude + 0.1 * dt;
                    if(dt.Equals(0)) dt = 1;
                }
            }

            bool need_to_stop
            {
                get
                {
                    return ren.mode == Mode.Manual && manual_stop ||
                              Best != null && Best.TimeToStart < ren.CorrectionOffset;
                }
            }

            void add_best_node()
            {
                ren.clear_nodes();
                ManeuverAutopilot.AddNodeRaw(ren.VSL, Best.NodeDeltaV, Best.StartUT);
            }

            float progress;
            bool manual_stop;
            public override IEnumerator<RendezvousTrajectory> GetEnumerator()
            {
                progress = 0;
                manual_stop = false;
                best_points.Clear();
                start_points.Clear();
                P.UpdateTrajectory(true);
                add_end_point();
                foreach(var t in scan_start_time()) yield return t;
                var bestP = P;
                Best = bestP.trajectory;
                if(P.trajectory.DistanceToTarget < C.Dtol) best_points.Add(P);
                for(int i = 0, count = start_points.Count; i < count; i++)
                {
                    if(need_to_stop) yield break;
                    progress = (i + 1f) / (count + 1);
                    P = start_points[i];
                    foreach(var t in full_search(dT)) yield return t;
                    if(!manual_stop &&
                       (ren.mode != Mode.Manual || best_points.Count == 0) &&
                       P.Better(bestP))
                    {
                        bestP = P;
                        Best = bestP.trajectory;
                    }
                    if(ren.mode == Mode.Manual &&
                       P.trajectory.DistanceToTarget < C.Dtol)
                    {
                        var j = best_points.FindIndex(p => Point.Close(p, P));
                        if(j < 0)
                            best_points.Add(P);
                        else if(P < best_points[j])
                            best_points[j] = P;
                        sort_best_points();
                    }
                }
                progress = 1;
                if(ren.mode == Mode.Manual && best_points.Count > 1)
                {
                    add_best_node();
                    while(!manual_stop) yield return null;
                }
            }

            void sort_best_points()
            {
                switch(sort_order)
                {
                case 0:
                    best_points.Sort((a, b) => a.distance.CompareTo(b.distance));
                    break;
                case 1:
                    best_points.Sort((a, b) => (a.start + a.transfer).CompareTo(b.start + b.transfer));
                    break;
                case 2:
                    best_points.Sort((a, b) => a.start.CompareTo(b.start));
                    break;
                }
            }

            int sort_order = 0;
            static string[] sorting = { "dV", "ETA", "Start" };
            Vector2 scroll = Vector2.zero;
            public void DrawBestTrajecotries()
            {
                if(manual_stop) return;
                GUILayout.BeginVertical();
                GUILayout.BeginHorizontal();
                GUILayout.Label("Sort by:", GUILayout.ExpandWidth(false));
                var old_order = sort_order;
                sort_order = GUILayout.Toolbar(sort_order, sorting, GUILayout.ExpandWidth(true));
                if(sort_order != old_order) sort_best_points();
                GUILayout.EndHorizontal();
                scroll = GUILayout.BeginScrollView(scroll, Styles.white, GUILayout.Height(75));
                foreach(var p in best_points)
                {
                    if(p.trajectory.TimeToStart > 0 &&
                       p.Draw(p.trajectory == Best))
                    {
                        Best = p.trajectory;
                        add_best_node();
                    }
                }
                GUILayout.EndScrollView();
                if(Best != null && GUILayout.Button(new GUIContent("Continue", "Continue using selected transfer"),
                                                    Styles.enabled_button, GUILayout.ExpandWidth(true)))
                    manual_stop = true;
                GUILayout.EndVertical();
            }

            public override string Status
            {
                get
                {
                    if(ren.mode == Mode.Manual && Best != null && progress.Equals(1))
                        return "Selected transfer:\n" + BestDesc;
                    var s = ProgressIndicator.Get +
                                             (ren.mode == Mode.Manual ?
                         " searching for transfers" : " searching for the best transfer");
                    if(Best == null) return s + "...";
                    return s + string.Format(" {0:P0}\n", progress) + BestDesc;
                }
            }
        }
        #endregion

        #if DEBUG
        static void log_patches(Orbit o, string tag)
        { Utils.Log(Utils.formatPatches(o, tag)); }

        public static bool _CalculatePatch(Orbit p, Orbit nextPatch, double startEpoch, PatchedConics.SolverParameters pars, CelestialBody targetBody)
        {
            p.activePatch = true;
            p.nextPatch = nextPatch;
            p.patchEndTransition = Orbit.PatchTransitionType.FINAL;
            p.closestEncounterLevel = Orbit.EncounterSolutionLevel.NONE;
            p.numClosePoints = 0;
            log_patches(p, "Patch 0");
            int count = Planetarium.Orbits.Count;
            for(int i = 0; i < count; i++)
            {
                OrbitDriver orbitDriver = Planetarium.Orbits[i];
                if(orbitDriver.orbit == p) continue;
                if(orbitDriver.celestialBody)
                {
                    if(targetBody == null)
                    {
                        goto IL_B6;
                    }
                    if(orbitDriver.celestialBody == targetBody)
                    {
                        goto IL_B6;
                    }
                    IL_C5:
                    if(orbitDriver.referenceBody == p.referenceBody)
                    {
                        var enc = PatchedConics.CheckEncounter(p, nextPatch, startEpoch, orbitDriver, targetBody, pars);
                        Utils.Log("Encounter with {}: {}", orbitDriver.celestialBody.bodyName, enc);
                        log_patches(p, "Patch");
                        goto IL_FA;
                    }
                    goto IL_FA;
                    IL_B6:
                    p.closestTgtApprUT = 0.0;
                    goto IL_C5;
                }
                IL_FA:;
            }
            log_patches(p, "Patch 1");
            if(p.patchEndTransition == Orbit.PatchTransitionType.FINAL)
            {
                if(!pars.debug_disableEscapeCheck)
                {
                    if(p.ApR <= p.referenceBody.sphereOfInfluence)
                    {
                        if(p.eccentricity < 1.0)
                        {
                            p.UTsoi = -1.0;
                            p.StartUT = startEpoch;
                            p.EndUT = startEpoch + p.period;
                            p.patchEndTransition = Orbit.PatchTransitionType.FINAL;
                            goto IL_2C0;
                        }
                    }
                    if(double.IsInfinity(p.referenceBody.sphereOfInfluence))
                    {
                        p.FEVp = Math.Acos(-(1.0 / p.eccentricity));
                        p.SEVp = -p.FEVp;
                        p.StartUT = startEpoch;
                        p.EndUT = double.PositiveInfinity;
                        p.UTsoi = double.PositiveInfinity;
                        p.patchEndTransition = Orbit.PatchTransitionType.FINAL;
                    }
                    else
                    {
                        p.FEVp = p.TrueAnomalyAtRadius(p.referenceBody.sphereOfInfluence);
                        p.SEVp = -p.FEVp;
                        p.timeToTransition1 = p.GetDTforTrueAnomaly(p.FEVp, 0.0);
                        p.timeToTransition2 = p.GetDTforTrueAnomaly(p.SEVp, 0.0);
                        p.UTsoi = startEpoch + p.timeToTransition1;
                        nextPatch.UpdateFromOrbitAtUT(p, p.UTsoi, p.referenceBody.referenceBody);
                        p.StartUT = startEpoch;
                        p.EndUT = p.UTsoi;
                        p.patchEndTransition = Orbit.PatchTransitionType.ESCAPE;
                    }
                }
            }
            IL_2C0:
            nextPatch.StartUT = p.EndUT;
            double arg_2FD_1;
            if(nextPatch.eccentricity < 1.0)
            {
                arg_2FD_1 = nextPatch.StartUT + nextPatch.period;
            }
            else
            {
                arg_2FD_1 = nextPatch.period;
            }
            nextPatch.EndUT = arg_2FD_1;
            nextPatch.patchStartTransition = p.patchEndTransition;
            nextPatch.previousPatch = p;
            log_patches(p, "Patch 2");
            return p.patchEndTransition != Orbit.PatchTransitionType.FINAL;
        }
        #endif
    }
}
