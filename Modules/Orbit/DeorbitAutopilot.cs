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
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using AT_Utils;
using LTComparer = AT_Utils.HierarchicalComparer<ThrottleControlledAvionics.LandingTrajectory>;

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

        public enum Stage { None, Precalculate, Compute, Deorbit, Correct, Coast, Wait }
        [Persistent] public Stage stage;
        double initialEcc;

		public override void Init()
		{
			base.Init();
			CFG.AP2.AddHandler(this, Autopilot2.Deorbit);
		}

        protected override Autopilot2 program
        {
            get { return Autopilot2.Deorbit; }
        }

        protected override string program_name
        {
            get { return "Landing Autopilot"; }
        }

		Vector3d PlaneCorrection(TargetedTrajectoryBase old)
		{
			var angle = old.DeltaFi;
			angle *= Math.Sin(old.TransferTime/old.Orbit.period*2*Math.PI);
            angle *= Math.Sign(Utils.ProjectionAngle(old.StartPos, old.AtTargetPos, old.AfterStartVel));
			var rot = QuaternionD.AngleAxis(angle, old.StartPos);
            return ManeuverAutopilot.Orbital2NodeDeltaV(VesselOrbit, (rot*old.AfterStartVel)-old.AfterStartVel, old.StartUT);
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

            protected DeorbitOptimizerBase(DeorbitAutopilot module, float dtol) 
                : base(module, dtol) 
            { 
                m = module;
                inclination = 0f;
                targetAlt = m.TargetAltitude;
            }

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
                var startUT = Best.StartUT;
                var I = Utils.Clamp(inclination+dI, -90, 90);
                while(Math.Abs(dI) > 1e-5)
                {
                    var cur = newT(startUT, I, dV);
//                    m.Log("startUT {}, opt.I {}, dI {}, dFi {}, dist {}", startUT, I, dI, cur.DeltaFi, cur.DistanceToTarget);//debug
                    if(cur.FullManeuver &&
                       cur.DistanceToTarget < Best.DistanceToTarget)
                    {
                        Best = cur;
                        inclination = I;
                    }
                    I += dI;
                    if(I < -90 || I > 90 || Best != cur)
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
            protected double Ecc, dEcc;

            public DeorbitTrajectoryOptimizer(DeorbitAutopilot module, float dtol) 
                : base(module, dtol) 
            {
                Ecc = m.initialEcc;
                dEcc = Math.Max(Ecc/DEO.EccSteps, 0.05);
            }

            Vector3d deorbit(double startUT)
            { return dV4Ecc(m.VesselOrbit, Ecc, startUT, m.Body.Radius+targetAlt-10); }

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
                    if(cur.DistanceToTarget < Best.DistanceToTarget) 
                        Best = cur;
                    startUT += dT;
                    yield return cur;
                }
            }

            IEnumerable<LandingTrajectory> optimize_startUT(double dT)
            {
                var startUT = Math.Max(Best.StartUT+dT, m.VSL.Physics.UT+m.ManeuverOffset);
                while(Math.Abs(dT) > 0.1)
                {
                    var cur = newT(startUT, inclination, deorbit(startUT));
//                    m.Log("opt.startUT {}, I {}, dT {}, dist {}", startUT, inclination, dT, cur.DistanceToTarget);//debug
                    if(cur.DistanceToTarget < Best.DistanceToTarget) 
                        Best = cur;
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

            IEnumerable<LandingTrajectory> optimize_ecc(double ddV)
            {
                var startUT = Best.StartUT;
                var dir = deorbit(startUT);
                var dVm = dir.magnitude;
                dir /= dVm;
                var bestDeltaV = dVm;
                dVm = Math.Max(dVm+ddV, 0);
                while(Math.Abs(ddV) > TRJ.dVtol)
                {
                    var cur = newT(startUT, inclination, dir*dVm);
//                    m.Log("startUT {}, I {}, opt.dV {}, ddV {}, dist {}", startUT, inclination, dVm, ddV, cur.DistanceToTarget);//debug
                    if(cur.FullManeuver &&
                       cur.DistanceToTarget < Best.DistanceToTarget) 
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
                Ecc = Best.Orbit.eccentricity;
            }

            public override IEnumerator<LandingTrajectory> GetEnumerator()
            {
                var startUT = m.VSL.Physics.UT+m.ManeuverOffset;
                Best = newT(startUT, inclination, deorbit(startUT));
                foreach(var t in scan_startUT()) yield return t;
                LandingTrajectory prev = null;
                while(continue_calculation(prev, Best))
                {
                    prev = Best;
                    startUT = Best.StartUT;
                    var dR = Math.Abs(Best.DeltaR);
                    var dT = (float)Math.Max(Math.Abs(Best.DeltaR)/360*m.VesselOrbit.period, 1);
                    foreach(var t in optimize_startUT(dT)) yield return t;
                    var dI = (float)Utils.Clamp(Utils.ClampSignedL(Best.DeltaFi, 0.1), -10, 10);
                    foreach(var t in optimize_inclination(dI, deorbit(Best.StartUT))) yield return t;
                    var dR1 = Math.Abs(Best.DeltaR);
                    if(!Best.FullBrake || dR1 < 1 || Math.Abs(dR-dR1) < 1)
                    { 
                        var ddV = -Utils.ClampSignedL(dR2dV(Best.DeltaR), 1);
                        foreach(var t in optimize_ecc(ddV)) yield return t; 
                    }
//                    if(Math.Abs(dR-Math.Abs(Best.DeltaR)) < 1)
//                        Ecc = Math.Max(Ecc-dEcc, 0);
                }
            }
        }

        class DeorbitTrajectoryCorrector : DeorbitOptimizerBase
        {
            double prograde_dV;

            public DeorbitTrajectoryCorrector(DeorbitAutopilot module, float dtol) 
                : base(module, dtol) 
            {
                prograde_dV = 0;
            }

            IEnumerable<LandingTrajectory> optimize_prograde(double ddV)
            {
                var startUT = Best.StartUT;
                var dir = m.hV(startUT).normalized;
                var pg = prograde_dV+ddV;
                while(Math.Abs(ddV) > TRJ.dVtol)
                {
                    var cur = newT(startUT, inclination, dir*pg);
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
                Best = newT(m.VSL.Physics.UT+m.CorrectionOffset+1, 0, Vector3d.zero);
                LandingTrajectory cur = null;
                while(continue_calculation(cur, Best))
                {
                    cur = Best;
                    var ddV = Utils.ClampSignedL(dR2dV(Best.DeltaR), 1);
                    foreach(var t in optimize_prograde(ddV)) yield return t;
                    var dI = Utils.Clamp((float)Best.DeltaFi, -10, 10);
                    var dV = m.hV(Best.StartUT).normalized*prograde_dV;
                    foreach(var t in optimize_inclination(dI, dV)) yield return t;
                }
            }
        }

        class EccentricityOptimizer : TrajectoryOptimizer
        {
            DeorbitAutopilot m;
            LTComparer comparer;
            public LandingTrajectory Best { get; private set; }

            static LTComparer.Condition fuel, maneuver, dR, overheat, steepness;
            static EccentricityOptimizer()
            {
                fuel = LTComparer.MakeCondition(null, (x, y) => x.GetTotalFuel() < y.GetTotalFuel());
                maneuver = LTComparer.MakeCondition(x => x.FullManeuver, (x, y) => x.ManeuverFuel < y.ManeuverFuel);
                dR = LTComparer.MakeCondition(x => x.DeltaR < -1, (x, y) => x.DeltaR < y.DeltaR);
                overheat = LTComparer.MakeCondition(x => !x.WillOverheat, (x, y) => x.MaxShipTemperature < y.MaxShipTemperature);
                steepness = LTComparer.MakeCondition(x => x.LandingSteepness > DEO.MinLandingAngle, (x, y) => x.LandingSteepness > y.LandingSteepness);
            }

            public EccentricityOptimizer(DeorbitAutopilot module)
            {
                m = module;
                comparer = new LTComparer(maneuver, dR);
                if(m.Body.atmosphere)
                {
                    var maxDynP = DEO.MaxDynPressure*m.VSL.Torque.MaxPossible.AngularDragResistance;
                    comparer.AddConditions(overheat, steepness);
                    comparer.AddCondition(x => x.MaxDynamicPressure < maxDynP, (x, y) => x.MaxDynamicPressure < y.MaxDynamicPressure);
                }
                else 
                    comparer.AddCondition(steepness);
                comparer.AddCondition(fuel);
            }

            public IEnumerator<LandingTrajectory> GetEnumerator()
            {
//                m.Log("Calculating initial orbit eccentricity...");//debug
                var tPos = m.CFG.Target.OrbPos(m.Body);
                var UT = m.VSL.Physics.UT +
                    TrajectoryCalculator.AngleDelta(m.VesselOrbit, tPos, m.VSL.Physics.UT)/360*m.VesselOrbit.period;
                if(UT < m.VSL.Physics.UT) UT += m.VesselOrbit.period;
                var vPos = m.VesselOrbit.getRelativePositionAtUT(UT);
                var vVel = m.VesselOrbit.getOrbitalVelocityAtUT(UT);
                var incl = Math.Abs(90-Utils.Angle2(tPos, m.VesselOrbit.GetOrbitNormal()));
                var ini_dV = TrajectoryCalculator.dV4Pe(m.VesselOrbit, (m.Body.Radius+m.TargetAltitude*0.9), UT);
                ini_dV = QuaternionD.AngleAxis(incl, vPos)*(ini_dV+vVel);
                var dir = -ini_dV.normalized;
                var maxV = ini_dV.magnitude;
                ini_dV -=  vVel;
                Best = new LandingTrajectory(m.VSL, ini_dV, UT, m.CFG.Target, m.TargetAltitude);
                yield return Best;
                //if there's not enough fuel, just go with the smallest maneuver possible
                if(Best.FullManeuver)
                {
                    //search for the best trajectory using current comparer
                    var start = Best;
                    var bestV = 0.0;
                    var dV = maxV/10;
                    var V = dV;
                    while(Math.Abs(dV) > TRJ.dVtol)
                    {
                        var cur = new LandingTrajectory(m.VSL, start.ManeuverDeltaV+dir*V, start.StartUT, m.CFG.Target, start.TargetAltitude);
//                        m.Log("V {}, dV {}, is better {}\ncur {}\nbest {}", 
//                              V, dV, comparer.isBetter(cur, Best), cur, Best);//debug
                        if(comparer.isBetter(cur, Best))
                        {
                            Best = cur;
                            bestV = V;
                        }
                        V += dV;
                        if(V < 0 || V > maxV || cur != Best)
                        {
                            dV /= -2.1;
                            V = bestV;
                        }
                        yield return cur;
                    }
                }
                m.initialEcc = Best.Orbit.eccentricity;
//                m.Log("initialEcc: {}", m.initialEcc);//debug
            }

            IEnumerator IEnumerable.GetEnumerator()
            { return GetEnumerator(); }

            public string Status
            { get { return "Computing optimal deorbit parameters..."; } }
        }

        void compute_initial_eccentricity()
        {
            ComputeTrajectory(new EccentricityOptimizer(this));
            stage = Stage.Precalculate;
            trajectory = null;
        }

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

		protected override void Reset()
		{
			base.Reset();
			stage = Stage.None;
            initialEcc = 0;
			CFG.AP1.Off();
		}

		public void DeorbitCallback(Multiplexer.Command cmd)
		{
			switch(cmd)
			{
			case Multiplexer.Command.Resume:
                if(stage == Stage.None && !landing) 
                    goto case Multiplexer.Command.On;
                if(!check_patched_conics()) return;
				UseTarget();
				NeedCPSWhenMooving();
				if(trajectory == null) update_trajectory();
				if(VSL.HasManeuverNode)
					CFG.AP1.OnIfNot(Autopilot1.Maneuver);
				break;

			case Multiplexer.Command.On:
				Reset();
				if(!check_patched_conics()) return;
                if(!setup()) { Disable(); return; }
                UseTarget();
                NeedCPSWhenMooving();
				if(VesselOrbit.PeR < Body.Radius)
				{
					Status("red", "Already deorbiting. Trying to correct course and land.");
					fine_tune_approach();
				}
				else 
                    compute_initial_eccentricity();
                break;

			case Multiplexer.Command.Off:
				ReleaseCPS();
                StopUsingTarget();
				Reset();
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
				!VSL.LandedOrSplashed && 
                (VSL.HasTarget || CFG.Target);
		}

		protected override void Update()
		{
			if(landing) { do_land(); return; }
			switch(stage)
			{
			case Stage.Precalculate:
                if(trajectory_computed())
                    compute_landing_trajectory();
				break;
			case Stage.Compute:
				if(!trajectory_computed()) break;
                if(check_initial_trajectory()) deorbit();
                else stage = Stage.Wait;
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
//                update_trajectory();//debug
//                Log("Trajectory after deorbit: {}", trajectory);//debug
				fine_tune_approach();
				break;
			case Stage.Correct:
				if(!trajectory_computed()) break;
                if(!trajectory.WillOverheat)
					add_correction_node_if_needed();
                else 
                    update_landing_trajecotry();
				stage = Stage.Coast; 
				break;
			case Stage.Coast:
                if(!coast_to_start())
                    stage = Stage.None;
				break;
			}
		}

        static readonly GUIContent button_content = new GUIContent("Land", "Compute and perform a deorbit maneuver, then land near the target.");
		public override void Draw()
		{
            if(ControlsActive)
            {
                if(CFG.AP2[program])
                    GUILayout.Label(button_content, Styles.enabled_button, GUILayout.ExpandWidth(true));
                else if(GUILayout.Button(button_content, Styles.active_button, GUILayout.ExpandWidth(true)))
                    ShowOptions = !ShowOptions;
            }
            else GUILayout.Label(button_content, Styles.inactive_button, GUILayout.ExpandWidth(true));
		}
	}
}

