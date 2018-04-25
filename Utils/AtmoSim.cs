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
    public class AtmoSim : TCAComponent
    {
        public class Config : ComponentConfig<Config>
        {
            [Persistent] public float DeltaTime = 0.5f;
            [Persistent] public float DragK = 0.0008f;
            [Persistent] public float RotMiddlePhase = 0.6f;
        }
        public static Config C => Config.INST;

        static double _Cd = -1;
        public static double Cd
        {
            get
            {
                if(_Cd < 0)
                    //0.0005 converts dynamic pressure to kPa and divides area by 2: Drag = dP * Cd * S/2.
                    _Cd = 0.0005 * C.DragK * PhysicsGlobals.DragCubeMultiplier * PhysicsGlobals.DragMultiplier;
                return _Cd;
            }
        }

        CelestialBody Body { get { return VSL.Body; } }

        public AtmoSim(ModuleTCA tca) : base(tca) { }

        double drag(double s, double h, double v)
        {
            if(h > Body.atmosphereDepth) return 0;
            var atm = Body.AtmoParamsAtAltitude(h);
            var v2 = v * v;
            var dP = atm.Rho * v2;
            var mach = v / atm.Mach1;
            var d = Cd *
                PhysicsGlobals.DragCurveMultiplier.Evaluate((float)mach) *
                PhysicsGlobals.DragCurvePseudoReynolds.Evaluate((float)(atm.Rho * Math.Abs(v)));
            return dP * d * s;
        }

        double StG(double h)
        {
            var r = Body.Radius + h;
            return Body.gMagnitudeAtCenter / r / r;
        }

        Vector2d getApV(double m, double s, Vector2d r, Vector2d v, double dt, out double t)
        {
            t = 0;
            while(Vector2d.Dot(r, v) > 0)
            {
                r += v * dt;
                var R = r.magnitude;
                var V = v.magnitude;
                v -= (r * Body.gMagnitudeAtCenter / R / R / R + v / V * drag(s, R - Body.Radius, V) / m) * dt;
                t += dt;
            }
            return r;
        }

        float max_G_throttle(float T, double m, float maxG) =>
        maxG / Utils.ClampL(T / (float)m / VSL.Physics.StG - 1, maxG);

        public struct PV
        {
            public Vector3d pos, vel;
            public double time, alt;
        }

        public double FreeFallTime(Orbit orb, double startUT,
                                   Vector3d vel, Vector3d pos, double end_altitude, double dt,
                                   out Vector3d terminal_velocity, out Vector3d end_position)
        {
            var R = Body.Radius + end_altitude;
            var p = new PV { pos = pos.xzy, vel = vel.xzy, alt = pos.magnitude - R };
            var m = (double)VSL.Physics.M;
            var s = (double)VSL.Geometry.BoundsSideAreas.MinComponentF();
            var started = false;
            var UT = startUT;
            var endUT = startUT + orb.timeToPe;
            while(p.alt > 0 && UT < endUT)
            {
                var sv = p.vel - Vector3d.Cross(Body.angularVelocity, p.pos);
                var svm = sv.magnitude;
                var drag_dv = drag(s, p.alt, svm) / m * dt;
                started |= drag_dv > 0.1;
                UT = startUT + p.time;
                if(started)
                {
                    var r = p.pos.magnitude;
                    p.vel -= p.pos * Body.gMagnitudeAtCenter / r / r / r * dt + sv / svm * Math.Min(drag_dv, svm);
                    p.pos += p.vel * dt;
                }
                else
                {
                    p.vel = orb.getOrbitalVelocityAtUT(UT).xzy;
                    p.pos = orb.getRelativePositionAtUT(UT).xzy;
                }
                p.alt = p.pos.magnitude - R;
                p.time += dt;
                if(dt > 0.01 && p.alt / svm < dt)
                    dt = p.alt / svm * 0.9;
                //                Utils.Log("h {}, t {}, dt {}, pos {}, vel {}", 
                //                          p.alt, p.time, dt, p.pos, p.vel);//debug
            }
            terminal_velocity = p.vel.xzy;
            end_position = p.pos.xzy;
            return p.time;
        }

        public double FreeFallTime(double end_altitude, out double terminal_velocity)
        {
            var t = 0.0;
            var v = (double)VSL.VerticalSpeed.Absolute;
            var h = (double)VSL.Altitude.Relative;
            var th = (double)VSL.Altitude.TerrainAltitude;
            var m = (double)VSL.Physics.M;
            var s = VSL.Geometry.AreaInDirection(VSL.Physics.UpL);
            var dt = v < 0 ? Math.Abs(h / v / 10) : 1;
            if(dt > C.DeltaTime) dt = C.DeltaTime;
            while(h > end_altitude)
            {
                h += v * dt;
                var ah = h + th;
                v = Utils.ClampH(v - (StG(ah) - drag(s, ah, v) / m) * dt, -0.1);
                t += dt;
                dt = Math.Max(Math.Min(dt, (end_altitude - h) / v * 0.9), 0.01);
                //                Utils.Log("h {}, v {}, t {}", h, v, t);//debug
            }
            terminal_velocity = Math.Abs(v);
            return t;
        }

        public double FreeFallTime(out double terminal_velocity)
        { return FreeFallTime(VSL.Geometry.H, out terminal_velocity); }

        public IEnumerable<double> FromSurfaceTTA(float ApA_offset, double ApA, double alpha, float maxG)
        {
            var t = 0.0;
            var BR = Body.Radius;
            var ApR = BR + ApA;
            var v = new Vector2d(1e-3, 1e-3);
            var r = new Vector2d(0, VSL.orbit.radius);
            var r1n = new Vector2d(Math.Sin(alpha), Math.Cos(alpha));
            var r1 = r1n * ApR;
            var T = new Vector2d(0, 1);
            var m = (double)VSL.Physics.M;
            var eStats = VSL.Engines.NoActiveEngines ?
                VSL.Engines.GetNearestEnginedStageStats() :
                VSL.Engines.GetEnginesStats(VSL.Engines.Active);
            var mT = eStats.MaxThrust;
            var mTm = mT.magnitude;
            var mflow = eStats.MaxMassFlow;
            var s = VSL.Geometry.AreaInDirection(mT);
            var pitch = new PIDf_Controller3();
            var throttle = new PIDf_Controller3();
            var throttle_correction = new PIDf_Controller3();
            pitch.setPID(TargetedToOrbitExecutor.C.TargetPitchPID);
            pitch.Min = -AttitudeControlBase.C.MaxAttitudeError;
            throttle.setPID(ToOrbitExecutor.C.ThrottlePID);
            throttle.setClamp(0.5f);
            throttle_correction.setPID(TargetedToOrbitExecutor.C.ThrottleCorrectionPID);
            var thrust = true;
            var R = r.magnitude;
            var prev_r = r;
            var maxR = ApR*2;
            double turn_start = VSL.Altitude.Absolute;
            while(R > BR && R < maxR &&
                  Vector2d.Dot(r / R - r1n, r1n - prev_r / R) < 0)
            {
                yield return -1;
                prev_r = r;
                R = r.magnitude;
                var h = R - BR;
                double time2ApA;
                var ApV = getApV(m, s, r, v, C.DeltaTime * 4, out time2ApA);
                thrust &= Vector2d.Dot(ApV - r1, r1 - r) < 0;
                var srf_dir = -r.Rotate90().normalized;
                var thr = thrust ? max_G_throttle(mTm, m, maxG) : 0;
                var nV = v;
                if(thrust &&
                   Vector2d.Dot(r.normalized, v) / VSL.Physics.StG > ToOrbitExecutor.Config.INST.MinClimbTime)
                {
                    var rr1 = r1 - r;
                    var slv = new LambertSolver2D(Body, r, v, r1);
                    var minT = slv.ParabolicTime;
                    var maxT = slv.MinEnergyTime;
                    nV = slv.dV4TransferME();
                    while(maxT - minT > 0.1)
                    {
                        var curT = (maxT + minT) / 2;
                        nV = slv.dV4Transfer(curT);
                        var obt = TrajectoryCalculator.NewOrbit(Body, r, v + nV, VSL.Physics.UT);
                        if(obt.timeToAp > curT) minT = curT;
                        else maxT = curT;
                    }
                    var neededAoA = Utils.ClampH(Utils.Angle2(rr1, r) / 2, 45);
                    var angle2Hor = 90 - Utils.Angle2(nV, r);
                    var AoA = Utils.Angle2(r, v);
                    pitch.Max = AoA < neededAoA ? 0 : (float)AoA;
                    pitch.Update((float)angle2Hor);
                    if(AoA < neededAoA && pitch.Action.Equals(pitch.Max))
                        pitch.Action = (float)Utils.ClampL(AoA - neededAoA, -AttitudeControlBase.C.MaxAttitudeError);
                    if(h < Body.atmosphereDepth) 
                    {
                        var atm = Body.AtmoParamsAtAltitude(h);
                        if(atm.Rho > ToOrbitExecutor.C.AtmDensityOffset)
                            turn_start = h;
                    }
                    var startF = Utils.Clamp((h - turn_start) / ToOrbitExecutor.C.GTurnOffset, 0, 1);
                    var nT = v.Rotate(pitch.Action * startF);
                    var atErr = Utils.Angle2Rad(r, T) - Utils.Angle2Rad(r, nT);
                    T = T.RotateRad(atErr /
                                    Math.Max(C.DeltaTime, 
                                             eStats.TorqueInfo.RotationTime3Phase((float)Math.Abs(atErr*Mathf.Rad2Deg), C.RotMiddlePhase)) *
                                    C.DeltaTime)
                         .normalized;
                    if(Vector2d.Dot(T, r) < 0)
                        T = srf_dir;
                    throttle_correction.Update((float)angle2Hor);
                    throttle.Update(ApA_offset + throttle_correction.Action - (float)time2ApA);
                    thr = Utils.ClampH(0.5f + throttle, thr);
                }
                if(thrust && thr > 0)
                {
                    if(!CheatOptions.InfinitePropellant)
                    {
                        var dm = mflow * thr * C.DeltaTime;
                        if(m < dm) thrust = false;
                        else m -= dm;
                    }
                    if(thrust)
                        v += T * mTm / m * thr * C.DeltaTime;
                }
                v -= r * Body.gMagnitudeAtCenter / R / R / R * C.DeltaTime;
                if(h < Body.atmosphereDepth)
                {
                    var vm = v.magnitude;
                    if(vm > 0)
                        v -= v / vm * drag(s, h, vm) / m * C.DeltaTime;
                }
                r += v * C.DeltaTime;
                t += C.DeltaTime;
                DebugUtils.CSV("ToOrbitSim.csv", t, r, r1);//debug
                //DebugUtils.CSV("ToOrbitSim.csv", t, r, v, rel_v, T*mTm/m*thr, h, m, thr, r1, nV);//debug
            }
            yield return t - C.DeltaTime / 2;
        }

        public static IEnumerable<double> FromSurfaceTTA(ModuleTCA tca, float ApA_offset, double ApA, double alpha, float maxG)
        {
            var sim = new AtmoSim(tca);
            return sim.FromSurfaceTTA(ApA_offset, ApA, alpha, maxG);
        }
    }
}

