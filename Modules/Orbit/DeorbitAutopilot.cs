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
	[RequireModules(typeof(AttitudeControl),
	                typeof(BearingControl), 
	                typeof(ThrottleControl),
	                typeof(ManeuverAutopilot), 
	                typeof(AutoLander))]
	public class DeorbitAutopilot : LandingTrajectoryAutopilot
	{
		public new class Config : TCAModule.ModuleConfig
		{
			[Persistent] public float MinLandingAngle    = 20f;
            [Persistent] public float MaxLandingAngle    = 50f;
			[Persistent] public float MaxDynPressure     = 7f;
			[Persistent] public int   EccSteps           = 10;
		}
		static Config DEO { get { return Globals.Instance.DEO; } }

		public DeorbitAutopilot(ModuleTCA tca) : base(tca) {}

		public override void Init()
		{
			base.Init();
			CFG.AP2.AddHandler(this, Autopilot2.Deorbit);
		}

		public enum Stage { None, Precalculate, Compute, Deorbit, Correct, Coast, Wait }
		[Persistent] public Stage stage;
		double currentEcc;
		double dEcc;


		Vector3d PlaneCorrection(TargetedTrajectoryBase old)
		{
			var angle = old.DeltaFi;
			angle *= Math.Sin(old.TransferTime/old.Orbit.period*2*Math.PI);
            angle *= Math.Sign(Utils.ProjectionAngle(old.StartPos, old.AtTargetPos, old.AfterStartVel));
			var rot = QuaternionD.AngleAxis(angle, old.StartPos);
            return Orbit2NodeDeltaV((rot*old.AfterStartVel)-old.AfterStartVel, old.StartUT);
		}

        double ProgradeCorrection(LandingTrajectory old)
        {
            return old.DeltaR *
                Utils.ClampH(old.Orbit.period/old.TimeToTarget*Body.GeeASL, 1);
        }

        abstract class DeorbitOptimizerBase : LandingSiteOptimizerBase
        {
            protected readonly DeorbitAutopilot m;
            protected double targetAlt;
            protected float inclination;

            protected DeorbitOptimizerBase(DeorbitAutopilot module, float dtol) : base(dtol) 
            { m = module; }

            protected LandingTrajectory newT(double startUT, float I, Vector3d dV)
            {
                var pos = m.VesselOrbit.getRelativePositionAtUT(startUT);
                var vel = m.VesselOrbit.getOrbitalVelocityAtUT(startUT);
                var vel1 = dV+vel;
                return new LandingTrajectory(m.VSL, 
                                             Quaternion.AngleAxis(I, pos)*vel1 - vel,
                                             startUT, m.CFG.Target, targetAlt);
            }

            protected IEnumerable<LandingTrajectory> optimize_inclination(float dI, Vector3d dV)
            {
                LandingTrajectory cur = null;
                var startUT = Best.StartUT;
                var I = inclination;
                while(Math.Abs(dI) > 1e-5)
                {
                    cur = cur == null? Best : newT(startUT, I, dV);
//                    m.Log("startUT {}, opt.I {}, dI {}, dFi {}, dist {}", startUT, I, dI, cur.DeltaFi, cur.DistanceToTarget);//debug
                    var turn = cur.DeltaFi*Best.DeltaFi < 0;
                    if(Math.Abs(cur.DeltaFi) < Math.Abs(Best.DeltaFi))
                    {
                        Best = cur;
                        inclination = I;
                    }
                    I += dI;
                    if(I < -90 || I > 90 || turn || Best != cur)
                    {
                        dI /= -2.1f;
                        I = Utils.Clamp(inclination+dI, -90, 90);
                    }
                    yield return cur;
                }
            }
        }

        class DeorbitTrajectoryOptimizer : DeorbitOptimizerBase
        {
            public DeorbitTrajectoryOptimizer(DeorbitAutopilot module, float dtol) 
                : base(module, dtol) {}

            Vector3d deorbit(double startUT)
            { return dV4Ecc(m.VesselOrbit, m.currentEcc, startUT, m.Body.Radius+targetAlt-10); }

            IEnumerable<LandingTrajectory> scan_startUT()
            {
                var startUT = m.VSL.Physics.UT+m.ManeuverOffset;
                var endUT = startUT + m.VesselOrbit.period*(1 + m.VesselOrbit.period/m.Body.rotationPeriod);
                var dT = (endUT-startUT)/10;
                startUT += dT;
                while(startUT < endUT)
                {
                    var cur = newT(startUT, inclination, deorbit(startUT));
//                    m.Log("scan.startUT {}, I {}, dT {}, dist {}", startUT, inclination, dT, cur.DistanceToTarget);//debug
                    if(Best == null || cur.DistanceToTarget < Best.DistanceToTarget) Best = cur;
                    startUT += dT;
                    yield return cur;
                }
            }

            IEnumerable<LandingTrajectory> optimize_startUT(double dT)
            {
                LandingTrajectory cur = null;
                var startUT = Best.StartUT;
                while(Math.Abs(dT) > 0.1)
                {
                    cur = cur == null? Best : newT(startUT, inclination, deorbit(startUT));
//                    m.Log("opt.startUT {}, I {}, dT {}, dist {}", startUT, inclination, dT, cur.DistanceToTarget);//debug
                    if(cur.DistanceToTarget < Best.DistanceToTarget) Best = cur;
                    startUT += dT;
                    var minUT = m.VSL.Physics.UT+m.ManeuverOffset;
                    if(startUT < minUT || Best != cur)
                    {
                        dT /= -2.1;
                        startUT = Math.Max(Best.StartUT+dT, minUT);
                    }
                    yield return cur;
                }
            }

            IEnumerable<LandingTrajectory> optimize_startUT_inc(double dT, float dI)
            {
                LandingTrajectory cur = null;
                var startUT = Best.StartUT;
                var I = inclination;
                while(Math.Abs(dT) > 0.1 && Math.Abs(dI) > 1e-5)
                {
                    cur = cur == null? Best : newT(startUT, I, deorbit(startUT));
//                    m.Log("opt.startUT {}, opt.I {}, dT {}, dI {}, dist {}", startUT, I, dT, dI, cur.DistanceToTarget);//debug
                    if(cur.DistanceToTarget < Best.DistanceToTarget) 
                    {
                        Best = cur;
                        inclination = I;
                    }
                    startUT += dT;
                    I += dI;
                    var minUT = m.VSL.Physics.UT+m.ManeuverOffset;
                    if(startUT < minUT || I < -90 || I > 90 || Best != cur)
                    {
                        dT /= -2.1;
                        dI /= -2.1f;
                        startUT = Math.Max(Best.StartUT+dT, minUT);
                        I = Utils.Clamp(inclination+dI, -90, 90);
                    }
                    yield return cur;
                }
            }

            IEnumerable<LandingTrajectory> optimize_ecc(double ddV)
            {
                LandingTrajectory cur = null;
                var startUT = Best.StartUT;
                var dir = deorbit(startUT);
                var dVm = dir.magnitude;
                dir /= dVm;
                var bestDeltaV = dVm;
                while(Math.Abs(ddV) > TRJ.dVtol)
                {
                    cur = cur == null? Best : newT(startUT, inclination, dir*dVm);
//                    m.Log("startUT {}, I {}, opt.dV {}, dist {}", startUT, inclination, dVm, cur.DistanceToTarget);//debug
                    if(cur.DistanceToTarget < Best.DistanceToTarget) 
                    {
                        Best = cur;
                        bestDeltaV = dVm;
                    }
                    dVm += ddV;
                    if(dVm <= 0 || Best != cur)
                    {
                        ddV /= -2.1;
                        dVm = Math.Max(bestDeltaV+ddV, 0);
                    }
                    yield return cur;
                }
                m.currentEcc = Best.Orbit.eccentricity;
            }

            public override IEnumerator<LandingTrajectory> GetEnumerator()
            {
                inclination = 0f;
                targetAlt = m.TargetAltitude;
                foreach(var t in scan_startUT()) yield return t;
                while(Best.DistanceToTarget > dtol)
                {
                    var dR = Math.Abs(Best.DeltaR);
                    var I = inclination;
                    var startUT = Best.StartUT;
                    var dT = Mathf.Abs((float)Best.DeltaR)/360*m.VesselOrbit.period;
                    foreach(var t in optimize_startUT(dT)) yield return t;
                    var dI = Utils.Clamp((float)Best.DeltaFi, -10, 10);
                    foreach(var t in optimize_inclination(dI, deorbit(Best.StartUT))) yield return t;
                    foreach(var t in optimize_startUT_inc((Best.StartUT-startUT)/10, (inclination-I)/10)) yield return t;
                    if(Math.Abs(Math.Abs(Best.DeltaR)-dR) < 1)
                    { 
                        var ddV = Utils.ClampSignedL(Best.DeltaR, 1);
                        foreach(var t in optimize_ecc(ddV)) yield return t; 
                    }
                }
            }
        }

        class DeorbitTrajectoryCorrector : DeorbitOptimizerBase
        {
            double prograde_dV;

            public DeorbitTrajectoryCorrector(DeorbitAutopilot module, float dtol) 
                : base(module, dtol) {}

            IEnumerable<LandingTrajectory> optimize_prograde(double ddV)
            {
                LandingTrajectory cur = null;
                var startUT = Best.StartUT;
                var dir = m.hV(startUT).normalized;
                var pg = prograde_dV;
                while(Math.Abs(ddV) > TRJ.dVtol)
                {
                    cur = cur == null? Best : newT(startUT, inclination, dir*pg);
//                    m.Log("startUT {}, I {}, opt.dV {}, ddV {}, dist {}", startUT, inclination, pg, ddV, cur.DistanceToTarget);//debug
                    if(cur.DistanceToTarget < Best.DistanceToTarget) 
                    {
                        Best = cur;
                        prograde_dV = pg;
                    }
                    pg += ddV;
                    if(Best != cur)
                    {
                        ddV /= -2.1;
                        pg = prograde_dV+ddV;
                    }
                    yield return cur;
                }
            }

            public override IEnumerator<LandingTrajectory> GetEnumerator()
            {
                inclination = 0f;
                prograde_dV = 0;
                targetAlt = m.TargetAltitude;
                Best = newT(m.VSL.Physics.UT+m.CorrectionOffset+1, 0, Vector3d.zero);
                while(Best.DistanceToTarget > dtol)
                {
                    var ddV = Utils.ClampSignedL(Best.DeltaR, 1);
                    foreach(var t in optimize_prograde(ddV)) yield return t;
                    var dI = Utils.Clamp((float)Best.DeltaFi, -10, 10);
                    var dV = m.hV(Best.StartUT).normalized*prograde_dV;
                    foreach(var t in optimize_inclination(dI, dV)) yield return t;
                }
            }
        }


//        class LandingSiteOptimizer : LandingSiteOptimizerBase
//        {
//            readonly DeorbitAutopilot m;
//
//            public LandingSiteOptimizer(DeorbitAutopilot module, float dtol) : base(dtol) 
//            { m = module; }
//
//            public override IEnumerator<LandingTrajectory> GetEnumerator()
//            {
//                Vector3d nodeDeltaV = Vector3d.zero;
//                var startUT = m.VSL.Physics.UT+m.ManeuverOffset;
//                var targetAlt = m.TargetAltitude;
//                LandingTrajectory prev = null, cur = null;
//                var inv_target_lat = 90-Math.Abs(Vector3d.Angle(m.VesselOrbit.GetOrbitNormal(), m.CFG.Target.RelOrbPos(m.Body))-90);
//                var target_at_pole = inv_target_lat < 5;
//                var angleK = inv_target_lat/90;
//                while(continue_calculation(prev, cur))
//                {                   
//                    prev = cur;
//                    cur = new LandingTrajectory(m.VSL, 
//                                                dV4Ecc(m.VesselOrbit, m.currentEcc, startUT, m.Body.Radius+targetAlt-10)+
//                                                m.Node2OrbitDeltaV(nodeDeltaV, startUT), 
//                                                startUT, m.CFG.Target, targetAlt);
//                    if(Best == null || cur.DistanceToTarget < Best.DistanceToTarget) Best = cur;
//                    targetAlt = cur.TargetAltitude;
//                    var deltaAngle = Math.Abs(90-m.VesselOrbit.inclination) > 45 ? cur.DeltaLon : cur.DeltaLat;
//                    deltaAngle *= angleK;
//                    m.Log("at pole {}, startUT {}, deltaAngle {}, node dV {}, current trajectory {}", 
//                          target_at_pole, startUT, deltaAngle, nodeDeltaV, cur);//debug
//                    //if target is at a pole, we can start the maneuver right now
//                    if(inv_target_lat < 1)
//                        startUT = m.VSL.Physics.UT+m.ManeuverOffset;
//                    //otherwise set startUT to minimize the arc distance between the node and the target
//                    else if(target_at_pole)
//                    {
//                        startUT = AngleDelta(m.VesselOrbit, m.CFG.Target.RelOrbPos(m.Body))/360*m.VesselOrbit.period-m.ManeuverOffset;
//                        if(startUT < 0) startUT += m.VesselOrbit.period;
//                        startUT = m.VSL.Physics.UT + startUT;
//                    }
//                    //otherwise correct start UT according to lat-lon delta between target and landing site
//                    else startUT = m.AngleDelta2StartUT(cur, deltaAngle, m.ManeuverOffset, m.VesselOrbit.period, m.VesselOrbit.period);
//                    //if start UT is good enough, correct orbital plane and landing site
//                    if(target_at_pole || Math.Abs(deltaAngle) < 1)
//                    {
//                        nodeDeltaV += m.PlaneCorrection(cur);
//                        //correct fly-over altitude if needed
//                        if(cur.BrakeEndDeltaAlt < LTRJ.FlyOverAlt)
//                            nodeDeltaV += new Vector3d((LTRJ.FlyOverAlt-cur.BrakeEndDeltaAlt)*m.Body.GeeASL/100, 0, 0);
//                        //if close enough, start tuning deltaR
//                        if(Math.Abs(cur.DeltaFi) < 1)
//                            nodeDeltaV += m.Orbit2NodeDeltaV(cur.AfterStartVel.normalized * m.ProgradeCorrection(cur), cur.StartUT);
//                    }
//                    yield return cur;
//                }
//            }
//        }
//
//        class LandingSiteCorrector : LandingSiteOptimizerBase
//        {
//            readonly DeorbitAutopilot m;
//
//            public LandingSiteCorrector(DeorbitAutopilot module, float dtol) : base(dtol) 
//            { m = module; }
//
//            public override IEnumerator<LandingTrajectory> GetEnumerator()
//            {
//                Vector3d nodeDeltaV = Vector3d.zero;
//                var start_offset = Math.Max(m.CorrectionOffset, m.VSL.Torque.NoEngines.TurnTime);
//                LandingTrajectory prev = null, cur = null;
//                while(continue_calculation(prev, cur))
//                {                   
//                    prev = cur;
//                    var startUT = m.VSL.Physics.UT+start_offset;
//                    cur = new LandingTrajectory(m.VSL, m.Node2OrbitDeltaV(nodeDeltaV, startUT), 
//                                                startUT, m.CFG.Target, cur == null? m.TargetAltitude : cur.TargetAltitude);
//                    if(Best == null || cur.DistanceToTarget < Best.DistanceToTarget) Best = cur;
//                    if(Math.Abs(cur.DeltaR) > Math.Abs(cur.DeltaFi)) 
//                        nodeDeltaV += new Vector3d(0, 0, m.ProgradeCorrection(cur));
//                    else 
//                        nodeDeltaV += m.PlaneCorrection(cur);
//                    yield return cur;
//                }
//            }
//        }

		void compute_landing_trajectory()
		{
			MAN.MinDeltaV = 1;
            ComputeTrajectory(new DeorbitTrajectoryOptimizer(this, LTRJ.Dtol));
			stage = Stage.Compute;
            trajectory = null;
		}

		protected override void fine_tune_approach()
		{
			CorrectionTimer.Reset();
            ComputeTrajectory(new DeorbitTrajectoryCorrector(this, LTRJ.Dtol/2));
			stage = Stage.Correct;
            trajectory = null;
		}

		protected override void reset()
		{
			base.reset();
			stage = Stage.None;
			CFG.AP1.Off();
		}

		IEnumerator<YieldInstruction> eccentricity_calculator = null;
		IEnumerator<YieldInstruction> compute_initial_eccentricity()
		{
			var tPos = CFG.Target.RelOrbPos(Body);
			var UT = VSL.Physics.UT +
				AngleDelta(VesselOrbit, tPos, VSL.Physics.UT)/360*VesselOrbit.period;
			var vPos = VesselOrbit.getRelativePositionAtUT(UT);
			var vVel = VesselOrbit.getOrbitalVelocityAtUT(UT);
            var incl = Math.Abs(90-Vector3.Angle(tPos, VesselOrbit.GetOrbitNormal()));
            var ini_dV = dV4Pe(VesselOrbit, (Body.Radius+TargetAltitude)*0.999, UT);
            ini_dV = Quaternion.AngleAxis(incl, vPos)*(ini_dV+vVel);
            var dir = -ini_dV.normalized;
            ini_dV -=  vVel;
			var maxV = vVel.magnitude;
			var minV = 0.0;
			var dV = 0.0;
			var in_plane = Math.Abs(90-Vector3.Angle(tPos, VesselOrbit.GetOrbitNormal())) < 5;
            var maxDynP = DEO.MaxDynPressure*VSL.Torque.MaxPossible.AngularDragResistance;
            var trj = new LandingTrajectory(VSL, ini_dV, UT, CFG.Target, TargetAltitude);
			yield return null;
			while(maxV-minV > 1)
			{
				dV = (maxV+minV)/2;
				trj = new LandingTrajectory(VSL, ini_dV+dir*dV, UT, CFG.Target, trj.TargetAltitude);
				var atmo_curve = trj.GetAtmosphericCurve(5);
                if(trj.FullManeuver && (in_plane || trj.DeltaR < 1) && trj.LandingAngle < DEO.MaxLandingAngle && trj.BrakeDuration > 3 &&
                   (!Body.atmosphere && trj.LandingAngle < DEO.MinLandingAngle || 
                    atmo_curve != null && (atmo_curve[atmo_curve.Count-1].DynamicPressure > maxDynP || will_overheat(atmo_curve))))
					minV = dV;
				else maxV = dV;
				yield return null;
			}
			currentEcc = trj.Orbit.eccentricity;
			dEcc = currentEcc/DEO.EccSteps;
		}

		public void DeorbitCallback(Multiplexer.Command cmd)
		{
			switch(cmd)
			{
			case Multiplexer.Command.Resume:
				if(!check_patched_conics()) return;
				UseTarget();
				NeedCPSWhenMooving();
				if(trajectory == null) update_trajectory();
				if(stage == Stage.None && !landing) 
					goto case Multiplexer.Command.On;
				if(VSL.HasManeuverNode)
					CFG.AP1.OnIfNot(Autopilot1.Maneuver);
				break;

			case Multiplexer.Command.On:
				reset();
				if(!check_patched_conics()) return;
				if(!setup()) { CFG.AP2.Off(); return; }
				if(VesselOrbit.PeR < Body.Radius)
				{
					Status("red", "Already deorbiting. Trying to correct course and land.");
					fine_tune_approach();
				}
				else 
				{
					eccentricity_calculator = compute_initial_eccentricity();
					stage = Stage.Precalculate;
				}
				goto case Multiplexer.Command.Resume;

			case Multiplexer.Command.Off:
				ReleaseCPS();
                StopUsingTarget();
				reset();
				break;
			}
		}

		void deorbit()
		{
            SaveGame("before_landing");
			CorrectionTimer.Reset();
			clear_nodes(); add_trajectory_node_rel();
			CFG.AP1.On(Autopilot1.Maneuver); 
			stage = Stage.Deorbit; 
		}

		protected override void UpdateState()
		{
			base.UpdateState();
			IsActive &= CFG.AP2[Autopilot2.Deorbit];
			ControlsActive &= IsActive || 
				!VSL.LandedOrSplashed && (VSL.TargetIsWayPoint || VSL.TargetVessel != null && VSL.TargetVessel.LandedOrSplashed);
		}

		protected override void Update()
		{
			if(!IsActive) { CFG.AP2.OffIfOn(Autopilot2.Deorbit); return; }
			if(landing) { do_land(); return; }
			switch(stage)
			{
			case Stage.Precalculate:
				Status("Computing trajectory...");
				if(eccentricity_calculator != null &&
				   eccentricity_calculator.MoveNext()) break;
				eccentricity_calculator = null;
				compute_landing_trajectory();
				break;
			case Stage.Compute:
				if(!trajectory_computed()) break;
				if(trajectory.DistanceToTarget < LTRJ.Dtol || currentEcc < 1e-10)
				{
					if(check_initial_trajectory()) deorbit();
					else stage = Stage.Wait;
				}
				else 
				{
                    currentEcc -= dEcc;
                    if(currentEcc > 1e-10 && trajectory.Orbit.PeR < Body.Radius) 
						compute_landing_trajectory();
				}
				break;
			case Stage.Wait:
				VSL.Info.CustomMarkersWP.Add(trajectory.SurfacePoint);
				if(!string.IsNullOrEmpty(TCAGui.StatusMessage)) break;
				deorbit();
				break;
			case Stage.Deorbit:
				Status("Executing deorbit burn...");
				VSL.Info.CustomMarkersWP.Add(trajectory.SurfacePoint);
				if(CFG.AP1[Autopilot1.Maneuver]) break;
				fine_tune_approach();
				break;
			case Stage.Correct:
				if(!trajectory_computed()) break;
				if(!will_overheat(trajectory.GetAtmosphericCurve(5)))
					add_correction_node_if_needed();
				stage = Stage.Coast; 
				break;
			case Stage.Coast:
				update_trajectory();
				VSL.Info.Countdown = trajectory.BrakeStartUT-VSL.Physics.UT-ManeuverOffset;
				if(CFG.AP1[Autopilot1.Maneuver]) 
				{ 
					if(VSL.Info.Countdown > 0 ||
					   trajectory.BrakeStartUT-Math.Max(MAN.NodeUT, VSL.Physics.UT)-VSL.Info.TTB -
                       VSL.Torque.NoEngines.RotationTime2Phase(Vector3.Angle(VesselOrbit.vel, MAN.NodeDeltaV)) > CorrectionOffset)
					{
						Status("Correcting trajectory..."); 
						break; 
					}
				}
				Status("Coasting...");
				WRP.NoDewarpOffset = true;
				if(VSL.Info.Countdown > 0 && !correct_trajectory()) break;
				CFG.AP1.OffIfOn(Autopilot1.Maneuver);
				stage = Stage.None;
				start_landing();
				break;
			}
		}

		public override void Draw()
		{
			#if DEBUG
			DrawDebugLines();
			#endif
			if(ControlsActive)
			{
				if(computing) 
				{
					if(GUILayout.Button(new GUIContent("Land", "Computing trajectory. Push to cancel."), 
					                    Styles.inactive_button, GUILayout.ExpandWidth(true)))
						CFG.AP2.XOff();
				}
				else if(Utils.ButtonSwitch("Land", CFG.AP2[Autopilot2.Deorbit],
				                           "Compute and perform a deorbit maneuver, then land near the target.", 
				                           GUILayout.ExpandWidth(true)))
                    VSL.XToggleWithEngines(CFG.AP2, Autopilot2.Deorbit);
			}
			else GUILayout.Label(new GUIContent("Land", "Compute and perform a deorbit maneuver, then land near the target."), 
			                     Styles.inactive_button, GUILayout.ExpandWidth(true));
		}

        #if DEBUG
        RealTimer delay = new RealTimer(5);
        public void Test(System.Random rnd = null)
        {
            VSL.Info.AddCustopWaypoint(new Coordinates(0,0,0),   "Zero");
            VSL.Info.AddCustopWaypoint(new Coordinates(90,0,0),  "North");
            VSL.Info.AddCustopWaypoint(new Coordinates(-90,0,0), "South");
            VSL.Info.AddCustopWaypoint(new Coordinates(0,90,0),  "90 deg");
            VSL.Info.AddCustopWaypoint(new Coordinates(0,180,0), "180 deg");
            VSL.Info.AddCustopWaypoint(new Coordinates(0,270,0), "270 deg");
            if(CFG.Target == null)
            {
                MapView.EnterMapView();
                CheatOptions.InfinitePropellant = true;
                CheatOptions.InfiniteElectricity = true;
                var c = Coordinates.SurfacePoint(rnd.NextDouble()*180-90, //30+60, //5+85, 
                                                 rnd.NextDouble()*360, 
                                                 Body);
                SetTarget(new WayPoint(c));
                CFG.AP2.XOnIfNot(Autopilot2.Deorbit);
            }
            if(CFG.AP2[Autopilot2.Deorbit] &&
               !CFG.AP1[Autopilot1.Maneuver])
            {
                delay.Reset();
                return;
            }
            if(trajectory != null)
                Log("Landing site error: {}m, {}", trajectory.DistanceToTarget, CFG.Target);
            CFG.AP2.XOff();
            if(!delay.TimePassed) return;
            CFG.Target = null;
        }
        #endif
	}
}

