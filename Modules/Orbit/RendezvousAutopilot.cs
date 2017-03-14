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
	public class RendezvousAutopilot : TargetedTrajectoryCalculator<RendezvousTrajectory>
	{
		public new class Config : TCAModule.ModuleConfig
		{
			[Persistent] public float Dtol                = 100f;   //m
			[Persistent] public float MaxTTR              = 3f;     //VesselOrbit.periods
			[Persistent] public float MaxDeltaV           = 100f;   //m/s
			[Persistent] public float CorrectionStart     = 10000f; //m
			[Persistent] public float CorrectionTimer     = 10f;    //s
			[Persistent] public float ApproachThreshold   = 500f;   //m
			[Persistent] public float MaxApproachV        = 20f;    //parts
			[Persistent] public float ApproachVelF        = 0.01f;  //parts
			[Persistent] public float MaxInclinationDelta = 30;     //deg
		}
		static Config REN { get { return Globals.Instance.REN; } }

		public RendezvousAutopilot(ModuleTCA tca) : base(tca) {}

		ThrottleControl THR;

		protected override RendezvousTrajectory CurrentTrajectory
		{ get { return new RendezvousTrajectory(VSL, Vector3d.zero, VSL.Physics.UT, CFG.Target, MinPeR); } }

		public enum Stage { None, Start, Launch, ToOrbit, StartOrbit, ComputeRendezvou, Rendezvou, Coast, MatchOrbits, Approach, Brake }
		[Persistent] public Stage stage;
		[Persistent] public Vector3 ToOrbitTarget;

		MinimumD MinDist = new MinimumD();
		ToOrbitExecutor ToOrbit;
		Vector3d ToOrbitIniApV;

		double CurrentDistance = -1;
        double DirectDistance = -1;
		Orbit TargetOrbit { get { return CFG.Target.GetOrbit(); } }
		Vessel TargetVessel { get { return CFG.Target.GetVessel(); } }
        bool TargetLoaded { get { return TargetVessel != null && TargetVessel.loaded; } }

		public override void Init()
		{
			base.Init();
			Dtol = REN.Dtol;
			CorrectionTimer.Period = REN.CorrectionTimer;
			CFG.AP2.AddHandler(this, Autopilot2.Rendezvous);
            UnityEngine.Random.InitState(42);
		}

		public override void Save(ConfigNode node)
		{
			if(ToOrbit != null) ToOrbitTarget = ToOrbit.Target;
			base.Save(node);
		}

		void resume_to_orbit()
		{
			ToOrbit = new ToOrbitExecutor(TCA);
			ToOrbit.Target = ToOrbitTarget;
		}

		public void RendezvousCallback(Multiplexer.Command cmd)
		{
			switch(cmd)
			{
			case Multiplexer.Command.Resume:
				if(!check_patched_conics()) 
				{
					CFG.AP2.Off();
					break;
				}
				UseTarget();
				NeedRadarWhenMooving();
				switch(stage)
				{
				case Stage.None:
				case Stage.ComputeRendezvou:
					stage = Stage.Start;
                    startUT = VSL.Physics.UT;
                    init_optimizer();//debug
					break;
				case Stage.Launch:
					if(VSL.LandedOrSplashed) stage = Stage.Start;
					else resume_to_orbit();
					break;
				case Stage.ToOrbit:
					resume_to_orbit();
					break;
				}
				if(VSL.HasManeuverNode) 
					CFG.AP1.OnIfNot(Autopilot1.Maneuver);
				break;

			case Multiplexer.Command.On:
				reset();
				if(setup()) 
					goto case Multiplexer.Command.Resume;
				CFG.AP2.Off();
				break;

			case Multiplexer.Command.Off:
                CFG.AT.On(Attitude.KillRotation);
                UnregisterFrom<Radar>();
                StopUsingTarget();
				reset();
				break;
			}
		}

		protected override bool check_target()
		{
			if(!base.check_target()) return false;
			if(TargetVessel == null) 
			{
				Status("yellow", "Target should be a vessel");
				return false;
			}
			if(TargetVessel.LandedOrSplashed)
			{
				Status("yellow", "Target vessel is landed");
				return false;
			}
            var dInc = Math.Abs(TargetOrbit.inclination-VesselOrbit.inclination);
            if(dInc > 90)
            {
                Status("yellow", "Target orbits in the oposite direction.\n" +
                       "You need to change orbit direction before the rendezvou maneuver.");
                return false;
            }
			else if(dInc > REN.MaxInclinationDelta)
			{
				Status("yellow", "Target orbit plane is tilted more than {0:F}° with respect to ours.\n" +
				       "You need to change orbit plane before the rendezvou maneuver.", REN.MaxInclinationDelta);
				return false;
			}
			return true;
		}

		

		RendezvousTrajectory new_trajectory(double StartUT, double transfer_time)
		{
			var endUT = StartUT+transfer_time;
			var solver = new LambertSolver(NextOrbit(endUT), NextOrbit(TargetOrbit, endUT).getRelativePositionAtUT(endUT), StartUT);
            if(solver.NotElliptic(transfer_time)) transfer_time = solver.ParabolicTime+1;
			var dV = solver.dV4Transfer(transfer_time);
//            Log("transfer time {}\ncV {}\ndV {}\norbit {}\ntOrbit {}", 
//                transfer_time, VesselOrbit.getOrbitalVelocityAtUT(StartUT), dV, 
//                NextOrbit(endUT), NextOrbit(TargetOrbit, endUT));//debug
			return new RendezvousTrajectory(VSL, dV, StartUT, CFG.Target, MinPeR, transfer_time);
		}

        protected RendezvousTrajectory orbit_correction(RendezvousTrajectory old, RendezvousTrajectory best, 
                                                        double ini_transfer_time, ref double dT)
        { return orbit_correction(old, best, VSL.Physics.UT + CorrectionOffset, ini_transfer_time, ref dT); }

        protected RendezvousTrajectory orbit_correction(RendezvousTrajectory old, RendezvousTrajectory best, 
                                                        double StartUT, double ini_transfer_time, ref double dT)
		{
			double transfer_time;
			if(VSL.Physics.UT+CorrectionOffset > StartUT)
				StartUT = VSL.Physics.UT+CorrectionOffset+TimeWarp.fixedDeltaTime;
			if(old != null) 
			{
                #if DEBUG
                if(best.KillerOrbit && !setp_by_step_computation)
                {
                    setp_by_step_computation = true;
                    PauseMenu.Display();
                }
                #endif
				if(StartUT-VSL.Physics.UT < old.ManeuverDuration/2)
					StartUT = VSL.Physics.UT+old.ManeuverDuration/2+TimeWarp.fixedDeltaTime;
				if(old.ManeuverDuration.Equals(0) && best.ManeuverDuration.Equals(0))
					transfer_time = Math.Max(old.TransferTime+dT, old.TransferTime*2);
				else
				{
					transfer_time = old.TransferTime+dT;
					if(transfer_time > Math.Max(VesselOrbit.period, TargetOrbit.period) ||
						transfer_time < ManeuverOffset ||
						!old.KillerOrbit && !best.KillerOrbit &&
						!trajectory_is_better(old, old, best))
					{
						dT /= -2.1;
						transfer_time = best.TransferTime+dT;
					}
				}
			}
			else transfer_time = ini_transfer_time;
//            Log("StartT {}, transfer_time {}, dT {}", StartUT-VSL.Physics.UT, transfer_time, dT);//debug
            DebugUtils.CSV("CDOS_test.csv", "old scan transfer", StartUT, transfer_time, -1, 0, 0, dT, -1, DateTime.Now.ToShortTimeString());
			return new_trajectory(StartUT, Math.Max(transfer_time, ManeuverOffset));
		}

        protected RendezvousTrajectory orbit_correction(RendezvousTrajectory old, RendezvousTrajectory best, 
                                                        double StartUT, double EndUT, 
                                                        ref double transfer_time, ref double UT, ref double dT)
        {
            if(old == null)
                return new_trajectory(StartUT, transfer_time);
            if(UT > 0)
            {
                if(best.KillerOrbit) 
                    transfer_time += dT/10;
                else UT += dT;
                if(UT < EndUT) 
                {
//                    Log("T {}, TT {}, dT", UT-StartUT, transfer_time, dT);//debug
                    var trj = new_trajectory(UT, transfer_time);
                    transfer_time = trj.TransferTime;
                    DebugUtils.CSV("CDOS_test.csv", "old scan start", UT, transfer_time, -1, 0, 0, dT, -1, DateTime.Now.ToShortTimeString());
                    return trj;
                }
                dT = best.TransferTime/10;
                UT = -1;
                return orbit_correction(null, best, best.StartUT, ManeuverOffset, ref dT);
            }
            return orbit_correction(old, best, best.StartUT, ManeuverOffset, ref dT);
        }

        protected override bool continue_calculation(RendezvousTrajectory old, RendezvousTrajectory cur, RendezvousTrajectory best)
        {
//            Log("cur == best {}, best.Dist {}, cur.ManT {}, dTransT {}, dDeltaV {}",
//                cur == best,
//                best.DistanceToTarget,
//                cur.ManeuverDuration,
//                Math.Abs(cur.TransferTime-best.TransferTime),
//                Math.Abs(cur.ManeuverDeltaV.sqrMagnitude-best.ManeuverDeltaV.sqrMagnitude));//debug
            return 
                cur == best ||
                best.DistanceToTarget > Dtol ||
                cur.ManeuverDuration.Equals(0) ||
                !cur.StartUT.Equals(best.StartUT) ||
                Math.Abs(cur.TransferTime-best.TransferTime) > 1e-5 &&
                Math.Abs(cur.ManeuverDeltaV.sqrMagnitude-best.ManeuverDeltaV.sqrMagnitude) > 1;
        }

        protected override bool trajectory_is_better(RendezvousTrajectory old, RendezvousTrajectory cur, RendezvousTrajectory best)
        { 
//            Log("cur.Trans {}, cur.Killer {}, best.Killer {}\ncur.Dist {}, best.Dist {}\ncur.Time {}+{}, best.Time {}+{}",
//                cur.TransferTime,
//                cur.KillerOrbit,
//                best.KillerOrbit,
//                cur.DistanceToTarget,
//                best.DistanceToTarget,
//                cur.ManeuverDuration, cur.BrakeDuration,
//                best.ManeuverDuration, best.BrakeDuration);//debug
            if(cur.KillerOrbit && best.KillerOrbit) 
                return cur.Orbit.PeR > best.Orbit.PeR;
            return 
                !cur.KillerOrbit && 
                (best.KillerOrbit || base.trajectory_is_better(old, cur, best)); 
        }

		void compute_rendezvou_trajectory()
		{
			trajectory = null;
			stage = Stage.ComputeRendezvou;
            var StartUT = VSL.Physics.UT+ManeuverOffset;
            //compute approach UT estimate
            double approachUT;
            var transfer_time = VesselOrbit.period/2;
            var ttr = TimeToResonance(VesselOrbit, TargetOrbit, StartUT);
            if(ttr < REN.MaxTTR) 
            {
                approachUT = VSL.Physics.UT+ttr*VesselOrbit.period;
                StartUT = Math.Max(approachUT-transfer_time, StartUT);
            }
            else 
                approachUT = VSL.Physics.UT+VesselOrbit.period*REN.MaxTTR;
//                ClosestApproach(VesselOrbit, TargetOrbit, StartUT, VSL.Geometry.MinDistance, out approachUT);
            if(approachUT <= StartUT) 
                approachUT += VesselOrbit.period;
            var EndUT = approachUT+transfer_time;
            var dT = (approachUT-StartUT)/10;
            var UT = StartUT;
            DebugUtils.CSV("CDOS_test.csv", "initial point", UT, transfer_time, -1, 0, 0, dT, -1, DateTime.Now.ToShortTimeString());
            Log("StartT {}, approachT {}, EndT {}, transfer_time {}, dT {}", 
                StartUT-VSL.Physics.UT, approachUT-VSL.Physics.UT, EndUT-VSL.Physics.UT, transfer_time, dT);//debug
            setup_calculation((o, b) => orbit_correction(o, b, StartUT, EndUT, ref transfer_time, ref UT, ref dT));
		}

		protected override void fine_tune_approach()
		{
            update_trajectory();
			stage = Stage.ComputeRendezvou;
            var dT = trajectory.TimeToTarget/4;
            var transfer_time = trajectory.TimeToTarget;
            DebugUtils.CSV("CDOS_test.csv", "initial point", VSL.Physics.UT, transfer_time, -1, 0, 0, dT, -1, DateTime.Now.ToShortTimeString());
            setup_calculation((o, b) => orbit_correction(o, b, transfer_time, ref dT));
            trajectory = null;
		}

		protected void compute_start_orbit(double StartUT)
		{
			trajectory = null;
			stage = Stage.ComputeRendezvou;
            var transfer_time = (VesselOrbit.period+TargetOrbit.period)/8;
            var dT = transfer_time/2;
            DebugUtils.CSV("CDOS_test.csv", "initial point", StartUT, ManeuverOffset, -1, 0, 0, dT, -1, DateTime.Now.ToShortTimeString());
            setup_calculation((o, b) => orbit_correction(o, b, StartUT, ManeuverOffset, ref dT));
		}

		void start_orbit()
		{
			ToOrbit = null;
			var dV = Vector3d.zero;
			var old = VesselOrbit;
			var StartUT = VSL.Physics.UT+CorrectionOffset;
			CFG.BR.OffIfOn(BearingMode.Auto);
            update_trajectory();
            ttr_tuning = 0;
			if(VesselOrbit.PeR < MinPeR) 
			{
                StartUT = Math.Min(trajectory.AtTargetUT, VSL.Physics.UT+(VesselOrbit.ApAhead()? VesselOrbit.timeToAp : CorrectionOffset));
				if(trajectory.DistanceToTarget < REN.ApproachThreshold*2 && StartUT.Equals(trajectory.AtTargetUT)) 
				{ //approach is close enough to directly match orbits
					match_orbits(); 
					return; 
				}
				var transfer_time = Utils.ClampL(TargetOrbit.period*(0.25-AngleDelta(VesselOrbit, TargetOrbit, StartUT)/360), 1);
				var solver = new LambertSolver(VesselOrbit, TargetOrbit.getRelativePositionAtUT(StartUT+transfer_time), StartUT);
				dV = solver.dV4Transfer(transfer_time);
				var trj = new RendezvousTrajectory(VSL, dV, StartUT, CFG.Target, MinPeR, transfer_time);
				if(!dV.IsZero() && !trj.KillerOrbit)
				{ //approach orbit is possible
					compute_start_orbit(StartUT);
					return;
				}
				//starting from circular orbit and proceeding to TTR fitting...
				StartUT = VesselOrbit.ApAhead()? VSL.Physics.UT+VesselOrbit.timeToAp : VSL.Physics.UT+CorrectionOffset;
				dV = dV4C(old, hV(StartUT), StartUT);
				old = NewOrbit(old, dV, StartUT);
			}
            else if(trajectory.RelDistanceToTarget < REN.CorrectionStart || TargetLoaded)
            {
                if(trajectory.RelDistanceToTarget > REN.CorrectionStart/4 && !TargetLoaded) 
                    fine_tune_approach();
                else match_orbits();
                return;
            }
            else
            {
                double alpha;
                var TTR = TimeToResonance(VesselOrbit, TargetOrbit, VSL.Physics.UT, out res_before, out alpha);
                Log("old ttr: {}", TTR);//debug
                ttr_before = TTR;//debug
                periodT = TargetOrbit.period;
                periodV_before = VesselOrbit.period;
                eccT = TargetOrbit.eccentricity;
                eccV_before = VesselOrbit.eccentricity;
                enT = TargetOrbit.orbitalEnergy;
                enV_before = VesselOrbit.orbitalEnergy;
                var tPe = TargetOrbit.getRelativePositionFromMeanAnomaly(0);//debug
                var vPe = VesselOrbit.getRelativePositionFromMeanAnomaly(0);//debug
                PeA_angle_before = Math.Abs(Utils.ProjectionAngle(vPe, tPe, VesselOrbit.getOrbitalVelocityAtObT(0)));//debug
//                if(false)
                if(TTR > REN.MaxTTR && alpha*res_before > 0) 
                {
                    var up = Vector3d.Dot(dV4Resonance(VesselOrbit, TargetOrbit, Math.Max(REN.MaxTTR/2, 0.75), alpha, VSL.Physics.UT), VesselOrbit.vel) > 0;
                    ttr_up = up;//debug
                    tPe = TargetOrbit.getRelativePositionFromMeanAnomaly(0);
                    var tAp = TargetOrbit.getRelativePositionFromMeanAnomaly(Math.PI);
                    var v2tPe = AngleDelta(VesselOrbit, tPe)*Mathf.Deg2Rad;
                    var v2tAp = AngleDelta(VesselOrbit, tAp)*Mathf.Deg2Rad;
                    var vtPe = VesselOrbit.getRelativePositionFromTrueAnomaly(VesselOrbit.trueAnomaly+v2tPe);
                    var vtAp = VesselOrbit.getRelativePositionFromTrueAnomaly(VesselOrbit.trueAnomaly+v2tAp);
                    var dPe = tPe.sqrMagnitude-vtPe.sqrMagnitude;
                    var dAp = tAp.sqrMagnitude-vtAp.sqrMagnitude;
                    double dTA;
                    if(dPe*dAp < 0)
                    {
                        ttr_async = true;//debug
                        if(dPe < 0) dTA = up? v2tPe : v2tAp;
                        else dTA = up? v2tAp : v2tPe;
                    }
                    else
                    {
                        ttr_async = false;//debug
                        if(dPe < 0) 
                        {
                            if(dPe < dAp) dTA = up? v2tPe : v2tAp;
                            else dTA = up? v2tAp : v2tPe;
                        }
                        else dTA = dPe < dAp ? v2tPe : v2tAp;
                    }
                    StartUT = VesselOrbit.GetDTforTrueAnomaly(VesselOrbit.trueAnomaly+dTA, 0);
                    Log("up {}, dPe {}, dAp {}, v2tPe {}, v2tAp {}, dTA {}, StartT {}\n" +
                        "tPe  {}\n" +
                        "vtPe {}\n" +
                        "tAp  {}\n" +
                        "vtAp {}", 
                        up, dPe, dAp, v2tPe, v2tAp, dTA, StartUT, tPe, vtPe, tAp, vtAp);//debug
                    StartUT = VSL.Physics.UT+Math.Max(StartUT, CorrectionOffset);
                }
            }
            //compute orbit with desired TTR and activate maneuver autopilot
			dV += dV4TTR(old, TargetOrbit, REN.MaxTTR, REN.MaxDeltaV, MinPeR, StartUT);
			if(!dV.IsZero())
			{
                var new_orb = NewOrbit(old, dV, StartUT);
                var tPe = TargetOrbit.getRelativePositionFromMeanAnomaly(0);//debug
                var vPe = new_orb.getRelativePositionFromMeanAnomaly(0);//debug
                PeA_angle_after = Math.Abs(Utils.ProjectionAngle(vPe, tPe, new_orb.getOrbitalVelocityAtObT(0)));//debug
                periodV_after = new_orb.period;
                eccV_after = new_orb.eccentricity;
                enV_after = new_orb.orbitalEnergy;
                double alpha;
                var TTR = TimeToResonance(new_orb, TargetOrbit, StartUT, out res_after, out alpha);
                signed_ttr_after = alpha*res_after;
                Log("new ttr: {}", TTR);//debug
                //PCA divider
//                # incl, PeA_angle_after, eccV_after
//                # +0.64, -0.56, -0.52
//                # -0.07, +0.63, -0.77
//                # -0.76, -0.53, -0.36
//                #
//                # divider: PC1 = 0.5
//                #
//                # center:
//                # incl: 8.474319
//                # PeA_angle_after: 20.285899
//                # eccV_after: -0.076491
//                #
//                # std:
//                # incl: 6.042648
//                # PeA_angle_after: 18.780472
//                # eccV_after: 0.074847
//                var incl = Math.Abs(VesselOrbit.inclination-TargetOrbit.inclination);
//                var PeA_angle = Math.Abs(PeA_angle_after-90)-Math.Abs(PeA_angle_before-90);
//                var eccV = eccV_after-eccT;
//                var PC1 = 
//                    (incl-8.5)/6*0.64 -
//                    (PeA_angle-20.3)/18.8*0.56 -
//                    (eccV+0.077)/0.075*0.52;
//                Log("incl {}, PeA angle {}, eccV {}: PC1 {}", incl, PeA_angle, eccV, PC1);//debug
//                if(TTR < REN.MaxTTR*2)
//                if(false)
//                {
                    ttr_tuning += dV.magnitude;//debug
                    ttrUT = StartUT;
    				add_node(dV, StartUT);
    				CFG.AP1.On(Autopilot1.Maneuver);
//                }
			}
			stage = Stage.StartOrbit;
		}

        //TODO: need to handle high target orbits differenly 
		void to_orbit()
		{
			if(!LiftoffPossible) return;
			//calculate target vector
            var ApR = Utils.Clamp((TargetOrbit.PeR+TargetOrbit.ApR)/2, MinPeR, MinPeR+GLB.ORB.RadiusOffset);
			var hVdir = Vector3d.Cross(TargetOrbit.GetOrbitNormal(), VesselOrbit.pos).normalized;
			var ascO = AscendingOrbit(ApR, hVdir, GLB.ORB.LaunchSlope);
			ToOrbit = new ToOrbitExecutor(TCA);
			ToOrbit.LaunchUT = VSL.Physics.UT;
			ToOrbit.ApAUT    = VSL.Physics.UT+ascO.timeToAp;
			ToOrbit.Target = ToOrbitIniApV = ascO.getRelativePositionAtUT(ToOrbit.ApAUT);
            //tune target vector
			if(VSL.LandedOrSplashed)
			{
				double TTR;
				do { TTR = correct_launch(); } 
				while(Math.Abs(TTR) > 1);
			}
			//setup launch
			CFG.DisableVSC();
            if(VSL.LandedOrSplashed)
                stage = Stage.Launch;
            else
            {
                MinDist.Reset();
                ToOrbit.StartGravityTurn();
                stage = Stage.ToOrbit;
            }
		}

		double correct_launch(bool allow_wait = true)
		{
			var TTR = AngleDelta(TargetOrbit, ToOrbit.Target, ToOrbit.ApAUT)/360*TargetOrbit.period;
			ToOrbit.LaunchUT += TTR;
			if(allow_wait && ToOrbit.LaunchUT-VSL.Physics.UT <= 0) ToOrbit.LaunchUT += TargetOrbit.period;
			ToOrbit.ApAUT = ToOrbit.LaunchUT+
				AtmoSim.FromSurfaceTTA(VSL, ToOrbit.TargetR-Body.Radius, 
				                       ToOrbit.ArcDistance, GLB.ORB.GTurnCurve, 
				                       Vector3d.Dot(SurfaceVel, Vector3d.Exclude(VesselOrbit.pos, ToOrbit.Target-VesselOrbit.pos).normalized));
			ToOrbit.Target = QuaternionD.AngleAxis((VSL.Physics.UT-ToOrbit.LaunchUT)/Body.rotationPeriod*360, Body.angularVelocity.xzy)*
				ToOrbitIniApV.normalized*TargetOrbit.getRelativePositionAtUT(ToOrbit.ApAUT).magnitude;
//			LogF("TTR: {}, LaunchT {}, ApAT {}", TTR, ToOrbit.LaunchUT-VSL.Physics.UT, ToOrbit.ApAUT-ToOrbit.LaunchUT);//debug
			return TTR;
		}

		void match_orbits()
		{
			SetTarget(CFG.Target);
			update_trajectory();
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
            if(DirectDistance > REN.Dtol) 
                approach();
            else 
                brake();
        }

		void brake()
		{
			SetTarget(CFG.Target);
			stage = Stage.Brake;
			var dV = VesselOrbit.vel-TargetOrbit.vel;
			var dVm = dV.magnitude;
			var dist = TargetVessel.CurrentCoM-VSL.Physics.wCoM;
			var distm = dist.magnitude-VSL.Geometry.MinDistance;
			if(distm < REN.Dtol && dVm < GLB.THR.MinDeltaV*2) return;
			if(distm > REN.Dtol && Vector3.Dot(dist, dV.xzy) > 0)
				CFG.AP1.On(Autopilot1.MatchVelNear);
			else CFG.AP1.On(Autopilot1.MatchVel);
		}

        void update_direct_distance()
        {
            DirectDistance = Utils.ClampL((TargetOrbit.pos-VesselOrbit.pos).magnitude-VSL.Geometry.MinDistance, 0);
        }

		protected override void update_trajectory()
		{
			base.update_trajectory();
			CurrentDistance = trajectory.DistanceToTarget;
		}

		protected override void reset()
		{
			base.reset();
			stage = Stage.None;
			CurrentDistance = -1;
            DirectDistance = -1;
			CFG.AP1.Off();
		}

		protected override void UpdateState()
		{
			base.UpdateState();
            IsActive &= CFG.AP2[Autopilot2.Rendezvous] && TargetOrbit != null;
			ControlsActive &= IsActive || VSL.TargetVessel != null;
		}

//		double startAlpha; //debug
//		double startUT = -1; //debug
//		void log_flight() //debug
//		{
//			var target = ToOrbit != null? ToOrbit.Target : VesselOrbit.getRelativePositionAtUT(VSL.Physics.UT+VesselOrbit.timeToAp);
//			var alpha = Utils.ProjectionAngle(VesselOrbit.pos, target, target-VesselOrbit.pos) * Mathf.Deg2Rad;
//			var arc = (startAlpha-alpha)*VesselOrbit.radius;
//			CSV(VSL.Physics.UT-startUT, VSL.VerticalSpeed.Absolute, Vector3d.Exclude(VesselOrbit.pos, VesselOrbit.vel).magnitude, 
//			    arc, VesselOrbit.radius-Body.Radius, 
//			    VSL.Engines.Thrust.magnitude,
//			    VSL.Physics.M, 90-Vector3d.Angle(VesselOrbit.vel.xzy, VSL.Physics.Up));//debug
//		}

        struct OptRes//debug
        {
            public double dV;
            public double dist;
            public double endUT;
            public void Update(RendezvousTrajectory t)
            {
                dV = t.ManeuverDeltaV.magnitude+t.BrakeDeltaV.magnitude;
                dist = t.DistanceToTarget;
                endUT = t.AtTargetUT;
            }
            public override string ToString()
            {
                return string.Format("dV {0} m/s, dist {1} m", dV, dist);
            }
        }

        bool ttr_up, ttr_async;//debug
        double ttr_tuning, 
        PeA_angle_before, PeA_angle_after, 
        ttr_before, ttr_after, 
        res_before, res_after, signed_ttr_after,
        periodV_before, periodV_after, periodT, 
        eccV_before, eccV_after, eccT,
        enV_before, enV_after, enT,
        ttrUT,
        startUT;//debug

        OptRes cdos, ttr_cdos, old_search;//debug
        CDOS_Optimizer2D optimizer = new CDOS_Optimizer2D();//debug
        void init_optimizer()//debug
        {
            //compute approach UT estimate
            var mean_orbit = (TargetOrbit.period+VesselOrbit.period)/2;
            var minUT = VSL.Physics.UT+ManeuverOffset+TRJ.CorrectionOffset;
            double maxUT;
            var lastOrbitV = LastOrbit(VesselOrbit);
            var lastOrbitT = LastOrbit(TargetOrbit);
            if(DiscontiniousOrbit(lastOrbitV))
            {
                maxUT = DiscontiniousOrbit(lastOrbitT) ? 
                    Math.Min(lastOrbitV.EndUT, lastOrbitT.EndUT) : lastOrbitV.EndUT;
            }
            else if(DiscontiniousOrbit(lastOrbitT)) 
                maxUT = lastOrbitT.EndUT;
            else 
                maxUT = VSL.Physics.UT+Math.Max(lastOrbitT.period, lastOrbitT.period)*(REN.MaxTTR+1);
//            var approachUT = 0.0;
//            var ttr = TimeToResonance(VesselOrbit, TargetOrbit, VSL.Physics.UT+ManeuverOffset);
//            if(ttr < REN.MaxTTR) 
//            {
//                approachUT = VSL.Physics.UT+ttr*VesselOrbit.period;
//                StartUT = Math.Max(approachUT-mean_orbit/2, minUT+1);
//            }
//            else 
//                approachUT = VSL.Physics.UT+VesselOrbit.period*REN.MaxTTR;
//            if(approachUT <= minUT) approachUT += VesselOrbit.period;
            var StartUT = minUT+1;
            var transfer = mean_orbit/2;
            var dT = Utils.Clamp(transfer/10, 10, 1000);

            log_patches(VesselOrbit, "Vessel Orbit");
            log_patches(TargetOrbit, "Target Orbit");

//            Log("CDOS: ttr {}, StartUT {}, approachT {}, transfer_time {}, dT {}", 
//                ttr, StartUT-VSL.Physics.UT, approachUT-VSL.Physics.UT, transfer, dT);//debug
            optimizer.Init(new_trajectory, 
                           minUT, 
                           maxUT,
                           StartUT, 
                           transfer, 
                           dT);
        }

        void log_patches(Orbit o, string tag)
        {
            if(string.IsNullOrEmpty(tag))
                Log("{}", o);
            else Log("===================== {} : {} =======================\n{}", tag, VSL.Physics.UT, o);
            if(o.nextPatch != null &&
               o.nextPatch.referenceBody != null)
                log_patches(o.nextPatch, "");
            if(!string.IsNullOrEmpty(tag))
                Log("===================================================================");
        }

		protected override void Update()
		{
			if(!IsActive) { CFG.AP2.OffIfOn(Autopilot2.Rendezvous); return; }
			switch(stage)
			{
			case Stage.Start:
                if(!setp_by_step_computation ||
                   string.IsNullOrEmpty(TCAGui.StatusMessage))
                {
                    RendezvousTrajectory t = null;
                    for(int i = 0; i < 10; i++)
                    {
                        t = optimizer.NextTrajectory();
                        if(t != null)
                        {
                            trajectory = t;
                            clear_nodes();
                            add_trajectory_node();
                            if(setp_by_step_computation)
                                Status("Push to continue...");
                            continue;
                        }
                        trajectory = optimizer.Best;
                        clear_nodes();
                        add_trajectory_node();
                        break;
                    }
                    if(t != null) break;
                    if(cdos.dV > 0) 
                        ttr_cdos.Update(trajectory);
                    else 
                        cdos.Update(trajectory);
                    Log("stage {}, first {}, second {}, third {}", 
                        stage, cdos, ttr_cdos, old_search);//debug
                    if(ttr_cdos.dV > 0) 
                    {
                        CurrentDistance = -1;
                        compute_rendezvou_trajectory();
                    }
                    else
                    {
                        clear_nodes();
                        if(VSL.InOrbit && 
                           VesselOrbit.ApR > MinPeR &&
                           VesselOrbit.radius > MinPeR)
                            start_orbit();
                        else to_orbit();
                    }
                }
                break;

				if(VSL.InOrbit && 
				   VesselOrbit.ApR > MinPeR &&
				   VesselOrbit.radius > MinPeR)
					start_orbit();
				else to_orbit();
				break;
			case Stage.Launch:
				if(ToOrbit.LaunchUT > VSL.Physics.UT) 
				{
					Status("Waiting for launch window...");
					correct_launch(false); //FIXME: sometimes AtmoSim returns inconsistent TTA which results in jumps of the Countdown.
					VSL.Info.Countdown = ToOrbit.LaunchUT-VSL.Physics.UT;
					VSL.Controls.WarpToTime = ToOrbit.LaunchUT;
					break;
				}
//				if(startUT < 0) //debug
//				{
//					startAlpha = Utils.ProjectionAngle(VesselOrbit.pos, ToOrbit.Target, ToOrbit.Target-VesselOrbit.pos) * Mathf.Deg2Rad;
//					startUT = VSL.Physics.UT;
//				}
//				log_flight();//debug
				if(ToOrbit.Liftoff()) break;
				stage = Stage.ToOrbit;
				MinDist.Reset();
				break;
			case Stage.ToOrbit:
//				log_flight();//debug
				if(ToOrbit.GravityTurn(ManeuverOffset, GLB.ORB.GTurnCurve, GLB.ORB.Dist2VelF, REN.Dtol))
				{
					if(ToOrbit.dApA < REN.Dtol)
					{
						update_trajectory();
						MinDist.Update(CurrentDistance);
						if(MinDist) start_orbit(); 
					}
				} else start_orbit();
				break;
			case Stage.StartOrbit:
//				log_flight();//debug
				Status("Achiving starting orbit...");
                CFG.WarpToNode = true;
				if(CFG.AP1[Autopilot1.Maneuver]) break;
				update_trajectory();
                ttr_after = TimeToResonance(VesselOrbit, TargetOrbit, VSL.Physics.UT);

                init_optimizer();
                stage = Stage.Start;
                break;

				CurrentDistance = -1;
				if(trajectory.RelDistanceToTarget < REN.CorrectionStart ||
				   trajectory.TimeToTarget/VesselOrbit.period > REN.MaxTTR) 
					stage = Stage.Rendezvou;
				else compute_rendezvou_trajectory();
				break;
			case Stage.ComputeRendezvou:
//				log_flight();//debug
				if(TimeWarp.CurrentRateIndex == 0 && TimeWarp.CurrentRate > 1) 
				{
					Status("Waiting for Time Warp to end...");
					break;
				}
				if(!trajectory_computed()) break;

                clear_nodes();
                add_trajectory_node();
                old_search.Update(trajectory);
                Log("stage {}, first: {}; second {}; third {};", 
                    stage, cdos, ttr_cdos, old_search);//debug
                if(ttr_cdos.dV.Equals(0))
                    DebugUtils.CSV("CDOS_test.csv", "initial point", VSL.Physics.UT, ManeuverOffset, -1, 0, 0, -1, -1, DateTime.Now.ToShortTimeString());
                DebugUtils.CSV("CDOS_dV.csv", 
                               cdos.dV, cdos.dist, cdos.endUT-startUT,
                               ttr_tuning, ttrUT-startUT,
                               ttr_cdos.dV, ttr_cdos.dV+ttr_tuning, ttr_cdos.dist, ttr_cdos.endUT-startUT,
                               old_search.dV, old_search.dV+ttr_tuning, old_search.dist, old_search.endUT-startUT,
                               ttr_cdos.dV+ttr_tuning-cdos.dV, old_search.dV-ttr_cdos.dV,
                               Math.Abs(VesselOrbit.inclination-TargetOrbit.inclination),
                               VesselOrbit.PeR-MinPeR, VesselOrbit.ApR-MinPeR, 
                               PeA_angle_before, PeA_angle_after, 
                               ttr_before, ttr_after, 
                               res_before, res_after,
                               signed_ttr_after,
                               periodT, periodV_before, periodV_after,
                               eccV_before, eccV_after, eccT,
                               enV_before, enV_after, enT,
                               ttr_up? 1 : 0, ttr_async? 1 : 0,
                               DateTime.Now.ToShortTimeString());
                cdos = ttr_cdos = old_search = new OptRes();
                PeA_angle_before = PeA_angle_after = -1;
                ttr_after = -1;
                ttr_up = ttr_async = false;
                CFG.AP2.Off();
                break;

                #if DEBUG
                if(trajectory.KillerOrbit ||
                   trajectory.Orbit.eccentricity > 0.6)
                    PauseMenu.Display();
                #endif
                if(!trajectory.KillerOrbit &&
                    trajectory.RelDistanceToTarget < REN.CorrectionStart &&
                   (CurrentDistance < 0 || trajectory.DistanceToTarget < CurrentDistance))
				{
					CorrectionTimer.Reset();
					if(trajectory.ManeuverDeltaV.magnitude > 1)
					{
						VSL.Controls.StopWarp();
						if(TimeWarp.CurrentRate > 1 ||
						   trajectory.TimeToStart < trajectory.ManeuverDuration/2)
						{
							update_trajectory();
							fine_tune_approach();
						}
						else
						{
							CurrentDistance = trajectory.DistanceToTarget;
							add_trajectory_node();
							CFG.AP1.On(Autopilot1.Maneuver);
							stage = Stage.Rendezvou;
						}
						break;
					}
					update_trajectory();
                    if(CurrentDistance > REN.Dtol && 
                       trajectory.TimeToTarget > trajectory.BrakeDuration+ManeuverOffset+TimeWarp.CurrentRate)
					{
						stage = Stage.Coast;
						break;
					}
				}
				if(CurrentDistance > 0 && 
				   CurrentDistance/VesselOrbit.semiMajorAxis < REN.CorrectionStart) 
				{
					VSL.Controls.StopWarp();
					match_orbits();
					break;
				}
				Status("red", "Failed to compute rendezvou trajectory.\nPlease, try again.");
				CFG.AP2.Off();
                #if DEBUG
                PauseMenu.Display();
                #endif
				break;
			case Stage.Rendezvou:
//				log_flight();//debug
				Status("Correcting trajectory...");
				if(CFG.AP1[Autopilot1.Maneuver]) break;
				update_trajectory();
				if(CurrentDistance < REN.ApproachThreshold) match_orbits();
				else stage = Stage.Coast;
				break;
			case Stage.Coast:
				Status("Coasting...");
				if(!CorrectionTimer.TimePassed)
				{
					if(VSL.Controls.CanWarp)
						VSL.Controls.WarpToTime = trajectory.AtTargetUT-trajectory.BrakeDuration+ManeuverOffset;
					break;
				}
				CorrectionTimer.Reset();
				update_trajectory();
				fine_tune_approach();
				break;
			case Stage.MatchOrbits:
//				log_flight();//debug
				Status("Matching orbits at nearest approach...");
				if(CFG.AP1[Autopilot1.Maneuver]) break;
                update_trajectory();
                update_direct_distance();
                if(TargetLoaded) 
                    approach_or_brake();
                else
                {
                    var relD = DirectDistance/VesselOrbit.semiMajorAxis;
    				if(relD > REN.CorrectionStart) 
                        start_orbit();
    				else if(relD > REN.CorrectionStart/4) 
                        fine_tune_approach();
                    else 
                        approach_or_brake();
                }
				break;
			case Stage.Approach:
				Status("Approaching...");
                THR.DeltaV = 0;
				var dP = TargetOrbit.pos-VesselOrbit.pos;
				var dPm = dP.magnitude;
                if(dPm - VSL.Geometry.MinDistance < REN.Dtol) 
				{ brake(); break; }
                var throttle = VSL.Controls.OffsetAlignmentFactor();
                if(throttle.Equals(0)) break;
				var dV = Vector3d.Dot(VesselOrbit.vel-TargetOrbit.vel, dP/dPm);
				var nV = Utils.Clamp(dPm*REN.ApproachVelF, 1, REN.MaxApproachV);
				if(dV+GLB.THR.MinDeltaV < nV) 
				{
					VSL.Engines.ActivateEngines();
                    THR.DeltaV = (float)(nV-dV)*throttle*throttle;
				}
				else brake();
				break;
			case Stage.Brake:
				Status("Braking near target...");
				if(CFG.AP1[Autopilot1.MatchVelNear]) break;
				if(CFG.AP1[Autopilot1.MatchVel])
				{
					if((TargetOrbit.vel-VesselOrbit.vel).magnitude > GLB.THR.MinDeltaV) break;
					CFG.AP1.Off();
					THR.Throttle = 0;
				}
				if((VSL.Physics.wCoM-TargetVessel.CurrentCoM).magnitude-VSL.Geometry.R-TargetVessel.Radius() > REN.Dtol)
				{ approach(); break; }
				CFG.AP2.Off();
				CFG.AT.OnIfNot(Attitude.KillRotation);
				ClearStatus();
				break;
			}
		}

		public override void Draw()
		{
			#if DEBUG
//			if(CFG.Target != null && Body != null && TargetOrbit != null)
//			{
//				if(ToOrbit != null)
//				{
//					Utils.GLVec(Body.position, ToOrbit.Target.xzy, Color.green);
//					Utils.GLVec(Body.position, Vector3d.Cross(VesselOrbit.pos, ToOrbit.Target).normalized.xzy*Body.Radius*1.1, Color.red);
//					Utils.GLVec(Body.position, TargetOrbit.getRelativePositionAtUT(ToOrbit.ApAUT).xzy, Color.yellow);
//				}
//				else Utils.GLVec(Body.position, TargetOrbit.getRelativePositionAtUT(VSL.Physics.UT+VesselOrbit.timeToAp).xzy, Color.yellow);
//				Utils.GLVec(Body.position, VesselOrbit.getRelativePositionAtUT(VSL.Physics.UT+VesselOrbit.timeToAp).xzy, Color.magenta);
//				Utils.GLVec(Body.position, VesselOrbit.GetOrbitNormal().normalized.xzy*Body.Radius*1.1, Color.cyan);
//			}
			#endif
			if(ControlsActive)
			{
				if(computing) 
				{
					if(GUILayout.Button(new GUIContent("Rendezvous", "Computing trajectory. Push to cancel."), 
					                    Styles.inactive_button, GUILayout.ExpandWidth(true)))
						CFG.AP2.Off();
//					#if DEBUG
//					if(current != null)
//					{
//						Utils.GLVec(Body.position, current.AtTargetPos.xzy, Color.green);//debug
//						Utils.GLVec(Body.position, current.TargetPos.xzy, Color.magenta);//debug
//						Utils.GLVec(Body.position+current.StartPos.xzy, current.ManeuverDeltaV.normalized.xzy*Body.Radius/4, Color.yellow);//debug
//						Utils.GLLine(Body.position+current.StartPos.xzy, Body.position+current.TargetPos.xzy, Color.cyan);//debug
//					}
//					#endif
				}
				else if(Utils.ButtonSwitch("Rendezvous", CFG.AP2[Autopilot2.Rendezvous],
				                           "Compute and perform a rendezvous maneuver, then brake near the target.", 
				                           GUILayout.ExpandWidth(true)))
					CFG.AP2.XToggle(Autopilot2.Rendezvous);
			}
			else GUILayout.Label(new GUIContent("Rendezvous", "Compute and perform a rendezvous maneuver, then brake near the target."), 
			                     Styles.inactive_button, GUILayout.ExpandWidth(true));
		}

        #if DEBUG
        Orbit CreateRandomOrbitNearby(Orbit baseOrbit)
        {
            Orbit orbit = null;
            while(orbit == null || orbit.PeR < MinPeR)
            {
                orbit = new Orbit();
                orbit.eccentricity = baseOrbit.eccentricity + (double)UnityEngine.Random.Range(0.001f, 0.2f);
                orbit.semiMajorAxis = Math.Max(baseOrbit.semiMajorAxis * (double)UnityEngine.Random.Range(0.9f, 1.1f), MinPeR+1000);
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

        ProtoVessel tgt_ast;
        RealTimer delay = new RealTimer(5);
        public void Test(System.Random rnd)
        {
            if(tgt_ast == null)
            {
                var obt = CreateRandomOrbitNearby(VesselOrbit);
                var seed = (uint)rnd.Next();
                tgt_ast = DiscoverableObjectsUtil.SpawnAsteroid("REN Test "+seed, obt, seed, UntrackedObjectClass.C, 5e5, 1e6);
                if(tgt_ast.vesselRef != null) 
                {
                    tgt_ast.vesselRef.DiscoveryInfo.SetLevel(DiscoveryLevels.Owned);
                    CheatOptions.InfinitePropellant = true;
                    SetTarget(tgt_ast.vesselRef);
                    CFG.AP2.XOn(Autopilot2.Rendezvous);
                }
                return;
            }
            if(CFG.AP2[Autopilot2.Rendezvous]) 
            {
                delay.Reset();
                return;
            }
            CFG.AP2.XOff();
            if(!delay.TimePassed) return;
            if(tgt_ast != null)
            {
                if(tgt_ast.vesselRef != null)
                    tgt_ast.vesselRef.Die();
                tgt_ast = null;
            }
        }
        #endif


        /// <summary>
        /// 2D optimizer for transfer trajectory based on 
        /// Conjugate Directions with Orthogonal Shift algorithm:
        /// https://arxiv.org/abs/1102.1347
        /// </summary>
        class CDOS_Optimizer2D
        {
            struct Point : IEquatable<Point>
            {
                Func<double,double,RendezvousTrajectory> newT;
                public double start, transfer, distance;
                public RendezvousTrajectory trajectory;

                public Point(double s, double t, Func<double,double,RendezvousTrajectory> new_trajectory = null)
                {
                    start = s;
                    transfer = t;
                    distance = double.MaxValue;
                    trajectory = null;
                    newT = new_trajectory;
                }

                public void UpdateTrajectory(bool with_distance = false)
                {
                    trajectory = newT(start, transfer);
                    transfer = trajectory.TransferTime;
                    if(with_distance) UpdateDist();
                }

                public void UpdateDist()
                {
                    distance = trajectory.DistanceToTarget*trajectory.DistanceToTarget +
                        trajectory.ManeuverDeltaV.sqrMagnitude+trajectory.BrakeDeltaV.sqrMagnitude;
                }

                public void Shift(double s, double t, bool with_distance = false)
                { 
                    start += s; 
                    transfer += t; 
                    if(with_distance)
                        UpdateTrajectory(with_distance);
                }

                public static Point operator+(Point p, Vector2d step)
                { 
                    var newP = p;
                    newP.Shift(step.x, step.y);
                    return newP;
                }

                public static Point operator+(Point p, Vector3d step)
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

                public static bool operator<(Point a, Point b)
                { return a.distance < b.distance; }

                public static bool operator>(Point a, Point b)
                { return a.distance < b.distance; }

                public static Vector2d Delta(Point a, Point b)
                { return new Vector2d(b.start-a.start, b.transfer-a.transfer); }

                public static double DistK(Point a, Point b)
                { return 1-Math.Abs(a.distance-b.distance)/Math.Max(a.distance, b.distance); }

                public override string ToString()
                { return Utils.Format("startUT {}, transfer {}, distance {}", start, transfer, distance); }

                #region IEquatable implementation
                public bool Equals(Point other)
                { return start.Equals(other.start) && transfer.Equals(other.transfer); }
                #endregion
            }

            double minUT;
            double maxUT;
//            Func<double,double,RendezvousTrajectory> T;

            double dT;
            Vector2d dir;
            Point P0, P;
            double dDist = -1;
            List<Point> start_points = new List<Point>();
            IEnumerator<RendezvousTrajectory> optimization;
            public RendezvousTrajectory Best { get; private set; }

            #if DEBUG
            void LogP(Point p, string tag = "")
            {
//                Utils.Log("{}, startT {}, transfer {}, dist {}, dir.x {}, dir.y {}, dT {}, dDist {}",
//                          tag, p.x-minUT, p.y, p.z, dir.x, dir.y, dT, dDist);
                DebugUtils.CSV("CDOS_test.csv", tag, p.start, p.transfer, p.distance, dir.x, dir.y, dDist, feasible_point(p), DateTime.Now.ToShortTimeString());
            }

            void LogP(string tag = "") { LogP(P, tag); }
            #endif

            public void Init(Func<double,double,RendezvousTrajectory> new_trajectory, 
                         double minUT, double maxUT, double startUT, double startTransfer, double dT)
            {
                optimization = null;
                start_points.Clear();
                P0 = P = new Point(Math.Max(startUT, minUT+1), startTransfer, new_trajectory);
                set_dir(new Vector2d(0,1));
                Best = null;
                this.maxUT = maxUT;
                this.minUT = minUT;
                this.dT = dT;
                dDist = -1;
            }

            void set_dir(Vector2d new_dir)
            {
                if(new_dir.x.Equals(0) && new_dir.y.Equals(0)) 
                    new_dir[Math.Abs(dir.x) > Math.Abs(dir.y)? 1 : 0] = 1;
                dir = new_dir;
            }

            Vector2d anti_grad()
            {
                var d0 = P.distance;
                var Px = P.Shifted(10,0, true);
                var Py = P.Shifted(0,10, true);
                return new Vector2d(d0-Px.distance, d0-Py.distance).normalized;
            }

            bool feasible_point(Point p)
            {
                return !p.trajectory.KillerOrbit && 
                    p.start > minUT && p.transfer > 1 && 
                    p.trajectory.ManeuverDuration > 0;
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
                LogP("first point");
            }

            IEnumerable<RendezvousTrajectory> scan_start_time()
            {
                var cur  = P;
                var prev = P;
                var bestP = cur;
                var bestOK = feasible_point(bestP);
                var endUT = Math.Max(P.start+P.transfer*2, maxUT);
                var maxTT = maxUT-minUT;
                var minZ = new MinimumD();
                while(cur.start < endUT)
                {
                    var requestedTT = cur.transfer;
                    cur.UpdateTrajectory(true);
                    var tOK = feasible_point(cur);
                    LogP(cur, "scan start");
//                    Utils.Log("scan start: StartT {}, TT {}/{}, maxTT-TT {}, dist {}/{}, OK {}\n" +
//                              "BestT {}, BestTT {}, dist {}/{}, OK {}", 
//                              P1.x, P1.y, requestedTT, maxTT-P1.y, trajectory.DistanceToTarget, P1.z, tOK, 
//                              bestP.x, bestP.y, bestT.DistanceToTarget,bestP.z, bestOK);//debug
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
                        cur.start += dT*step_k;
                    }
                    else 
                    {
                        cur.transfer += dT;
                        if(cur.transfer > maxTT || requestedTT-cur.transfer > 1)
                        {
                            cur.transfer = 1;
                            cur.start += dT/10;
                        }
                    }
                    yield return cur.trajectory;
                }
                if(!bestOK)
                {
                    P = bestP;
                    foreach(var t in find_first_point()) yield return t;
                    LogP("first feasable point");
                    bestP = P;
                }
                if(!start_points.Contains(bestP))
                    start_points.Add(bestP);
                set_dir(new Vector2d(1,0));
                for(int i = 0, minimaCount = start_points.Count; i < minimaCount; i++)
                {
                    P = start_points[i];
                    LogP("scan minimum 1");
                    foreach(var t in find_minimum(dT, true)) yield return t;
                    LogP("scan minimum 2");
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
//                Utils.Log("find min: start {}, TT {}, dT {}", P.start-minUT, P.transfer, dt);//debug
                while(Math.Abs(dt) > 1E-5)
                {
                    var step_k = Point.DistK(prev,cur);
                    prev = cur;
                    path += step_k;
                    cur += dir*dt*step_k;
                    var requestedTT = cur.transfer;
                    cur.UpdateTrajectory(true);
//                    Utils.Log("find min: StartT {}, TT {}/{}, dist {}/{}, OK {}, dT {}, scan finished {}\n" +
//                              "BestT {}, BestTT {}, dist {}/{}, OK {}", 
//                              cur.start, cur.transfer, cur.transfer-requestedTT, cur.trajectory.DistanceToTarget, cur.distance, feasable_point(cur), dt, scan_finished,
//                              bestP.start, bestP.transfer, bestP.trajectory.DistanceToTarget, bestP.distance, feasable_point(bestP));//debug
                    if(feasible_point(cur) && cur < bestP)
                    {
//                        LogP(cur, "find minimum");
                        var dD = bestP.distance-cur.distance;
                        bestP = cur;
                        if(no_scan || !scan_finished || dD > 0.1 || dt > 0.1)
                        {
                            path = 0.0;
                            if(dir.x.Equals(0) && Math.Abs(cur.transfer-requestedTT) > 1) dt /= 2;
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
                    {
                        scan_finished |= cur.start < minUT || cur.transfer < 1 || path > endL;
//                        LogP(cur, "scanning");
                    }
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
                    P1 = P+shift_dir*shift_delta;
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
                foreach(var t in orto_shift(0.62*dt)) yield return t;
                foreach(var t in find_minimum(dt, true)) yield return t;
//                LogP("parallel minimum");
                if(P0 < P) 
                {
                    set_dir(Point.Delta(P, P0).normalized);
                    P = P0;
                }
                else set_dir(Point.Delta(P0, P).normalized);
                if(P.start < minUT+1 && dir.x < 0) 
                { 
                    dir.x = 0.1;
                    dir.y = Math.Sign(dir.y);
                }
                foreach(var t in find_minimum(stride*dt)) yield return t;
                dDist = P.distance.Equals(double.MaxValue) || P0.distance.Equals(double.MaxValue)? 
                    -1 : Math.Abs(P.distance-P0.distance);
                if(P0 < P) 
                {
                    P = P0;
                    yield return P.trajectory;
                }
//                Utils.Log("best so far: StartT {}, TT {}, dist {}/{}, OK {}", 
//                          P.x-minUT, P.y, trajectory.DistanceToTarget, P.z, feasable_point(trajectory));//debug
                LogP("partial minimum");
            }

            IEnumerable<RendezvousTrajectory> build_conjugate_set(double dt)
            {
                set_dir(new Vector2d(0,1));
                foreach(var t in find_minimum(dt)) yield return t;
                LogP("minimum transfer");
                foreach(var t in shift_and_find(dt)) yield return t;
            }

            IEnumerable<RendezvousTrajectory> full_search()
            {
                P.UpdateTrajectory(true);
                LogP("initial point");
                foreach(var t in scan_start_time()) yield return t;
                var bestP = P;
                foreach(var p in start_points)
                {
                    P = p;
                    var dt = dT;
                    foreach(var t in build_conjugate_set(dt)) yield return t;
                    while(dt > 0.1 || dDist > 0.1 || dDist < 0)
                    {
                        foreach(var t in shift_and_find(dt, 3)) yield return t;
                        dt = 0.3*Point.Delta(P0, P).magnitude+0.1*dt;
                        if(dt.Equals(0)) dt = 1;
                    }
                    if(P < bestP) bestP = P;
                }
                P = bestP;
                Best = P.trajectory;
                yield return Best;
            }

            public RendezvousTrajectory NextTrajectory()
            {
                if(optimization == null)
                    optimization = full_search().GetEnumerator();
                if(optimization.MoveNext()) return optimization.Current;
                optimization = null;
                return null;
            }
        }
	}
}
