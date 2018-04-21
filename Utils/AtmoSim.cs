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
using UnityEngine;
using AT_Utils;

namespace ThrottleControlledAvionics
{
    public class AtmoSim
    {
        const double DeltaTime = 0.5;
        static Globals GLB { get { return Globals.Instance; } }

        static double _Cd = -1;
        public static double Cd
        {
            get 
            {
                if(_Cd < 0)
                    //0.0005 converts dynamic pressure to kPa and divides area by 2: Drag = dP * Cd * S/2.
                    _Cd = 0.0005 * ToOrbitAutopilot.C.DragK * PhysicsGlobals.DragCubeMultiplier * PhysicsGlobals.DragMultiplier;
                return _Cd;
            }
        }

        readonly VesselWrapper VSL;
        CelestialBody Body { get { return VSL.Body; } }

        /// <summary>
        /// Initializes a new instance of the <see cref="ThrottleControlledAvionics.AtmoSim"/> class.
        /// </summary>
        /// <param name="vsl">VesselWrapper.</param>
        public AtmoSim(VesselWrapper vsl)
        {
            VSL = vsl;
        }

        double drag(double s, double h, double v)
        { 
            if(h > Body.atmosphereDepth) return 0;
            var atm = Body.AtmoParamsAtAltitude(h);
            var v2 = v*v;
            var dP = atm.Rho * v2;
            var mach = v/atm.Mach1;
            var Cd = AtmoSim.Cd *
                PhysicsGlobals.DragCurveMultiplier.Evaluate((float)mach) *
                PhysicsGlobals.DragCurvePseudoReynolds.Evaluate((float)(atm.Rho*Math.Abs(v)));
            return dP * Cd * s;
        }

        double StG(double h) 
        { 
            var r = Body.Radius+h;
            return Body.gMagnitudeAtCenter/r/r; 
        }

        Vector2d getApV(double m, double s, Vector2d r, Vector2d v, double dt, out double t)
        {
            t = 0;
            while(Vector2d.Dot(r, v) > 0)
            {
                r += v*dt;
                var R = r.magnitude;
                var V = v.magnitude;
                v -= (r*Body.gMagnitudeAtCenter/R/R/R + v/V*drag(s, R-Body.Radius, V)/m)*dt;
                t += dt;
            }
            return r;
        }

        float max_G_throttle(float T, double m,  float maxG) =>
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
            var R = Body.Radius+end_altitude;
            var p = new PV{pos=pos.xzy, vel=vel.xzy, alt=pos.magnitude-R};
            var m = (double)VSL.Physics.M;
            var s = (double)VSL.Geometry.BoundsSideAreas.MinComponentF();
            var started = false;
            var UT = startUT;
            var endUT = startUT+orb.timeToPe;
            while(p.alt > 0 && UT < endUT)
            {
                var sv = p.vel-Vector3d.Cross(Body.angularVelocity, p.pos);
                var svm = sv.magnitude;
                var drag_dv = drag(s, p.alt, svm)/m*dt;
                started |= drag_dv > 0.1;
                UT = startUT+p.time;
                if(started)
                {
                    var r = p.pos.magnitude;
                    p.vel -= p.pos*Body.gMagnitudeAtCenter/r/r/r*dt + sv/svm*Math.Min(drag_dv, svm);
                    p.pos += p.vel*dt;
                }
                else
                {
                    p.vel = orb.getOrbitalVelocityAtUT(UT).xzy;
                    p.pos = orb.getRelativePositionAtUT(UT).xzy;
                }
                p.alt = p.pos.magnitude-R;
                p.time += dt;
                if(dt > 0.01 && p.alt/svm < dt) 
                    dt = p.alt/svm*0.9;
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
            var dt = v < 0? Math.Abs(h/v/10) : 1;
            if(dt > DeltaTime) dt = DeltaTime;
            while(h > end_altitude)
            {
                h += v*dt;
                var ah = h+th;
                v = Utils.ClampH(v-(StG(ah) - drag(s, ah, v)/m)*dt, -0.1);
                t += dt;
                dt = Math.Max(Math.Min(dt, (end_altitude-h)/v*0.9), 0.01);
//                Utils.Log("h {}, v {}, t {}", h, v, t);//debug
            }
            terminal_velocity = Math.Abs(v);
            return t;
        }

        public double FreeFallTime(out double terminal_velocity)
        { return FreeFallTime(VSL.Geometry.H, out terminal_velocity); }

        public double FromSurfaceTTA(float ApA_offset, double ApA, double alpha, double surface_vel, float maxG)
        {
            var t = 0.0;
            var BR = Body.Radius;
            var ApR = BR+ApA;
            var v = new Vector2d(surface_vel, 1e-3);
            var r = new Vector2d(0, VSL.orbit.radius);
            var r1n = new Vector2d(Math.Cos(alpha), Math.Sin(alpha));
            var r1 = r1n*ApR;
            var T = new Vector2d(0, 1);
            var m = (double)VSL.Physics.M;
            var eStats = VSL.Engines.NoActiveEngines? 
                VSL.Engines.GetNearestEnginedStageStats() :
                VSL.Engines.GetEnginesStats(VSL.Engines.Active);
            var mT = eStats.MaxThrust;
            var mTm = mT.magnitude;
            var mflow = eStats.MaxMassFlow;
            var s = VSL.Geometry.AreaInDirection(mT);
            var pitch = new PIDf_Controller3();
            var throttle = new PIDf_Controller3();
            var throttle_correction = new PIDf_Controller3();
            pitch.setPID(ToOrbitAutopilot.C.TargetPitchPID);
            pitch.Min = -AttitudeControlBase.C.MaxAttitudeError;
            throttle.setPID(ToOrbitAutopilot.C.ThrottlePID);
            throttle.setClamp(0.5f);
            throttle_correction.setPID(ToOrbitAutopilot.C.ThrottleCorrectionPID);
            var thrust = true;
			var R = r.magnitude;
            var prev_r = r;
            while(Vector2d.Dot(r, v) >= 0 && R > BR &&
                  Vector2d.Dot(r/R-r1n, r1n-prev_r/R) < 0)
            {
                prev_r = r;
                R = r.magnitude;
                var h = R-BR;
                double time2ApA;
                var ApV = getApV(m, s, r, v, DeltaTime*4, out time2ApA);
                thrust &= Vector2d.Dot(ApV-r1, r1-r) < 0;
                var srf_dir = -r.Rotate90().normalized;
                var rel_v = v-srf_dir*surface_vel;
                var thr = thrust? max_G_throttle(mTm, m, maxG) : 0;
                var nV = v;
                if(thrust && 
                   Vector2d.Dot(r.normalized, v) / VSL.Physics.StG > ToOrbitExecutor.Config.INST.MinClimbTime)
                {
                    var rr1 = r1-r;
                    var slv = new LambertSolver2D(Body, r, v, r1);
                    var minT = slv.ParabolicTime;
                    var maxT = slv.MinEnergyTime;
                    nV = slv.dV4TransferME();
                    while(maxT-minT > 0.1)
                    {
                        var curT = (maxT+minT)/2;
                        nV = slv.dV4Transfer(curT);
                        var obt = TrajectoryCalculator.NewOrbit(Body, r, v+nV, VSL.Physics.UT);
                        if(obt.timeToAp > curT) minT = curT;
                        else maxT = curT;
                    }
                    //nV = slv.dV4Transfer((slv.MinEnergyTime+slv.ParabolicTime)/2);
                    var neededAoA = Utils.ClampH(Utils.Angle2(rr1, r) / 2, 45);
					var angle2Hor = 90-Utils.Angle2(nV, r);
                    var AoA = Utils.Angle2(r, rel_v);
                    pitch.Max = AoA < neededAoA ? 0 : (float)AoA;
                    pitch.Update((float)angle2Hor);
                    if(AoA < neededAoA && pitch.Action.Equals(pitch.Max))
                        pitch.Action = (float)Utils.ClampL(AoA - neededAoA, -AttitudeControlBase.C.MaxAttitudeError);
                    var startF = Utils.Clamp((h - VSL.Altitude.Absolute) / ToOrbitAutopilot.C.GTurnOffset, 0, 1);
                    T = rel_v.Rotate(pitch.Action * startF).normalized;
                    if(Vector2d.Dot(T, r) < 0)
                        T = srf_dir;
					throttle_correction.Update((float)angle2Hor);
					throttle.Update(ApA_offset + throttle_correction.Action - (float)time2ApA);
					thr = Utils.ClampH(0.5f + throttle, thr);
                }
                if(thrust && thr > 0)
                {
                    v += T*mTm/m*thr*DeltaTime;
                    if(!CheatOptions.InfinitePropellant)
                    {
                        var dm = mflow*thr*DeltaTime;
                        if(m < dm) { thrust = false; continue; }
                        m -= dm;
                    }
                }
                v -= r*Body.gMagnitudeAtCenter/R/R/R*DeltaTime;
                if(h < Body.atmosphereDepth)
                {
                    var rvm = rel_v.magnitude;
                    if(rvm > 0)
                        v -= rel_v/rvm*drag(s, h, rvm)/m*DeltaTime;
                }
                r += v*DeltaTime;
                t += DeltaTime;
                //DebugUtils.CSV("ToOrbitSim.csv", t, r, v, rel_v, T*mTm/m*thr, h, m, thr, r1, nV);//debug
            }
            return t-DeltaTime/2;
        }

        public static double FromSurfaceTTA(VesselWrapper VSL, float ApA_offset, double ApA, double alpha, double surface_vel, float maxG)
        {
            var sim = new AtmoSim(VSL);
            return sim.FromSurfaceTTA(ApA_offset, ApA, alpha, surface_vel, maxG);
        }

        //public double FromSurfaceTTA2(double ApA, double alpha, double gturn_curve, double surface_vel)
        //{
        //    var t = 0.0;
        //    var v = new Vector3d(0, surface_vel);
        //    var h = (double)VSL.Altitude.Absolute;
        //    var m = (double)VSL.Physics.M;
        //    var eStats = VSL.Engines.NoActiveEngines? 
        //                    VSL.Engines.GetNearestEnginedStageStats() :
        //                    VSL.Engines.GetEnginesStats(VSL.Engines.Active);
        //    var mT = eStats.MaxThrust;
        //    var mflow = eStats.MaxMassFlow;
        //    var mTm = mT.magnitude*Mathfx.Lerp(0.6, 0.95, Utils.ClampH(
        //        VSL.Torque.AngularAcceleration(eStats.TorqueInfo.Torque+VSL.Torque.RCSLimits.Max+VSL.Torque.WheelsLimits.Max).magnitude, 1));
        //    var s = VSL.Geometry.AreaInDirection(mT);
        //    var R = Body.Radius;
        //    var thrust = true;
        //    var throttle = 1.0;
        //    //            var alpha0 = alpha; //debug
        //    while(v.x >= 0)
        //    {
        //        var atmF = Utils.Clamp(h/Body.atmosphereDepth, 0, 1);
        //        if(thrust)
        //        {
        //            double apaT;
        //            var apa = getApA(m, s, h, v.x, v.y, DeltaTime*4, out apaT);
        //            var dapa = (ApA-apa)*gturn_curve;
        //            var arc = (alpha-v.y*apaT/(R+(h+apa)/2))*(R+apa);
        //            var vv = Utils.ClampL(dapa, 0);
        //            var hv = Utils.ClampL(arc-dapa, 0)*Utils.Clamp((h-VSL.Altitude.Absolute)/ToOrbitAutopilot.ORB.GTurnOffset, 0, 1);
        //            if(h < Body.atmosphereDepth) hv *= Math.Sqrt(atmF);
        //            var angle = Math.Atan2(vv, hv);
        //            throttle = ThrottleControl.NextThrottle((float)(Math.Sqrt(vv*vv+hv*hv)*Globals.Instance.ORB.Dist2VelF*VSL.Physics.StG/Utils.G0), 
        //                                                    (float)throttle, (float)m, (float)mTm, 0);
        //            var v_throttle = Math.Sin(angle);
        //            v.x += (mTm*v_throttle*throttle/m-G(h, v.y))*DeltaTime;
        //            var h_throttle = 0.0;
        //            if(arc > 0)
        //            {
        //                h_throttle = Math.Cos(angle) ;
        //                v.y += (mTm*h_throttle*throttle/m)*DeltaTime;
        //            }
        //            thrust = ApA-apa > ToOrbitAutopilot.ORB.Dtol && arc > ToOrbitAutopilot.ORB.Dtol;
        //            if(!CheatOptions.InfinitePropellant)
        //            {
        //                var dm = mflow*(h_throttle+v_throttle)*throttle*DeltaTime;
        //                if(m < dm) { thrust = false; continue; }
        //                m -= dm;
        //            }
        //            //                    Utils.Log("apaT {}, dapa {}, arc {}, throttle {}, h-thr {}, v-thr {}", apaT, dapa, arc, throttle);//debug
        //        }
        //        else v.x -= G(h, v.y)*DeltaTime;
        //        if(h < Body.atmosphereDepth)
        //        {
        //            var y = v.y-surface_vel*(1-atmF);
        //            var vm = Math.Sqrt(v.x*v.x+y*y);
        //            var D = drag(s, h, vm)/m*DeltaTime/vm;
        //            v.x -= v.x*D;
        //            v.y -= y*D;
        //        }
        //        alpha -= v.y*DeltaTime/Body.Radius;
        //        h += v.x*DeltaTime;
        //        t += DeltaTime;
        //        //                Utils.Log("v.v {}, v.h {}, drag {}, h {}, hmove {}", v.x, v.y, drag(s, h, vm)/m, h, hmove);//debug
        //        //                DebugUtils.CSV("LambertSolver", t, v.x, v.y, (alpha0-alpha)*(R+h), h, m, mTm*throttle, throttle, Math.Atan2(v.x, v.y)*Mathf.Rad2Deg);//debug
        //    }
        //    return t-DeltaTime/2;
        //}
        //
        //public static double FromSurfaceTTA(VesselWrapper VSL, double ApA, double alpha, double gturn_curve, double surface_vel)
        //{
        //    var sim = new AtmoSim(VSL);
        //    return sim.FromSurfaceTTA(ApA, alpha, gturn_curve, surface_vel);
        //}

    }
}

