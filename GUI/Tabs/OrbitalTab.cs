//   OrbitalTab.cs
//
//  Author:
//       Allis Tauri <allista@gmail.com>
//
//  Copyright (c) 2017 Allis Tauri

using System;
using UnityEngine;
using AT_Utils;

#if DEBUG
using System.Linq;
using System.Collections.Generic;
#endif

namespace ThrottleControlledAvionics
{
	public class OrbitalTab : ControlTab
	{
        public OrbitalTab(ModuleTCA tca) : base(tca) {}

		MatchVelocityAutopilot MVA;
		DeorbitAutopilot DEO;
		RendezvousAutopilot REN;
		ToOrbitAutopilot ORB;
		[InternalModule]
		PointNavigator PN;

		public override void Draw()
		{
			GUILayout.BeginHorizontal();
			if(MVA != null) MVA.Draw();
			GUILayout.EndHorizontal();
			if(PN  != null && UI.NAV != null) 
				UI.NAV.TargetUI();
			GUILayout.BeginHorizontal();
			if(ORB != null) ORB.Draw();
			if(REN != null) REN.Draw();
			if(DEO != null) DEO.Draw();
			GUILayout.EndHorizontal();
            if(ORB != null && ORB.ShowEditor)
                ORB.DrawOrbitEditor();
            if(REN != null && REN.ShowOptions)
            {
                REN.DrawOptions();
                REN.DrawBestTrajectories();
            }
			if(DEO != null && CFG.AP2[Autopilot2.Deorbit])
				DEO.DrawDeorbitSettings();
			#if DEBUG
			if(Utils.ButtonSwitch("DBG", ref TrajectoryCalculator.setp_by_step_computation, 
			                      "Toggles step-by-step trajectory computation", GUILayout.ExpandWidth(true)) &&
			   TrajectoryCalculator.setp_by_step_computation)
				MapView.EnterMapView();
//            if(MapView.MapIsEnabled)
//                Utils.GLLines(FreeFallSim(VSL.orbit.vel, VSL.orbit.pos, 0, 5).ToArray(), Color.magenta);
			#endif
		}

        #if DEBUG
        double drag(double s, double h, double v)
        { 
            if(h > VSL.Body.atmosphereDepth) return 0;
            var atm = VSL.Body.AtmoParamsAtAltitude(h);
            var v2 = v*v;
            var dP = atm.Rho * v2;
            var mach = v/atm.Mach1;
            var Cd = AtmoSim.Cd *
                PhysicsGlobals.DragCurveMultiplier.Evaluate((float)mach) *
                PhysicsGlobals.DragCurvePseudoReynolds.Evaluate((float)(atm.Rho*Math.Abs(v)));
            return dP * Cd * s;
        }

        IEnumerable<Vector3> FreeFallSim(Vector3d vel, Vector3d pos, double end_altitude, double dt)
        {
            var t = 0.0;
            var r = pos.xzy;
            var v = vel.xzy;
            var m = (double)VSL.Physics.M;
            var s = (double)VSL.Geometry.BoundsSideAreas.MinComponentF();
            var h = r.magnitude-VSL.Body.Radius-end_altitude;
            var started = false;
            var UT = VSL.Physics.UT;
            var endUT = VSL.Physics.UT+VSL.orbit.timeToPe;
            yield return (Vector3)(VSL.Body.position+r);
            while(h > 0 && UT < endUT)
            {
                var sv = v-Vector3d.Cross(VSL.Body.angularVelocity, r);
                var svm = sv.magnitude;
                var drag_dv = drag(s, h, svm)/m*dt;
                started |= drag_dv > 0.1;
                UT = VSL.Physics.UT+t;
                if(started)
                {
                    var rm = r.magnitude;
                    v -= r*VSL.Body.gMagnitudeAtCenter/rm/rm/rm*dt + sv/svm*Math.Min(drag_dv, svm);
                    r += v*dt;
                }
                else
                {
                    v = VSL.orbit.getOrbitalVelocityAtUT(UT).xzy;
                    r = VSL.orbit.getRelativePositionAtUT(UT).xzy;
                }
                h = r.magnitude-VSL.Body.Radius-end_altitude;
                t += dt;
                if(dt > 0.01 && h/svm < dt) dt = h/svm*0.9;
                yield return (Vector3)(VSL.Body.position+
                                       QuaternionD.AngleAxis(-(t/VSL.Body.rotationPeriod*360 % 360.0), 
                                                             VSL.Body.angularVelocity.normalized)*r);
//                Utils.Log("h {} <= {}, t {}, dt {}, r {}, v {}", 
//                          h, prevH, t, dt, r, v);//debug
            }
        }
        #endif
	}
}