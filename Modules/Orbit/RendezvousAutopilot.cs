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
        MatchVelocityAutopilot MVA;

		protected override RendezvousTrajectory CurrentTrajectory
		{ get { return new RendezvousTrajectory(VSL, Vector3d.zero, VSL.Physics.UT, CFG.Target); } }

		public enum Stage 
        { 
            None, Start, PreLaunch, Launch, ToOrbit, StartOrbit, 
            ComputeSuborbitRendezvous, ComputeRendezvou, ComputeCorrection, 
            Rendezvou, Coast, MatchOrbits, 
            Approach, Brake 
        }
		[Persistent] public Stage stage;
		[Persistent] public Vector3 ToOrbitTarget;
        [Persistent] public FloatField GTurnCurve = new FloatField(format: "F1", min: 0.1f, max: 10);
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
		ToOrbitExecutor ToOrbit;
		Vector3d ToOrbitIniApV;

		double CurrentDistance = -1;
        bool CorrectingManeuver;

		public override void Init()
		{
			base.Init();
			CorrectionTimer.Period = REN.CorrectionTimer;
			CFG.AP2.AddHandler(this, Autopilot2.Rendezvous);
            UnityEngine.Random.InitState(42);
            GTurnCurve.Value = GLB.ORB.GTurnCurve;
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
            }
			return true;
		}

		RendezvousTrajectory new_trajectory(double StartUT, double transfer_time)
		{
			var endUT = StartUT+transfer_time;
            var obt = NextOrbit(StartUT);
            var solver = new LambertSolver(obt, RelativePosAtUT(obt.referenceBody, TargetOrbit, endUT), StartUT);
            if(solver.NotElliptic(transfer_time)) 
                transfer_time = solver.ParabolicTime+1;
			var dV = solver.dV4Transfer(transfer_time);
//            Log("transfer time {}\ncV {}\ndV {}\norbit {}\ntOrbit {}", 
//                transfer_time, VesselOrbit.getOrbitalVelocityAtUT(StartUT), dV, 
//                NextOrbit(endUT), NextOrbit(TargetOrbit, endUT));//debug
			return new RendezvousTrajectory(VSL, dV, StartUT, CFG.Target, transfer_time);
		}

		void compute_rendezvou_trajectory()
		{
            VSL.Controls.StopWarp();
            var minStartUT = VSL.Physics.UT+ManeuverOffset+TRJ.CorrectionOffset;
            var transfer = (TargetOrbit.period+VesselOrbit.period)/4;
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
                var period = Math.Max(double.IsInfinity(lastOrbitV.period)? 0 : lastOrbitV.period, 
                                      double.IsInfinity(lastOrbitT.period)? 0 : lastOrbitT.period);
                maxStartUT = VSL.Physics.UT+period*(REN.MaxTTR+1);
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
                                             minStartUT+1, 
                                             transfer, 
                                             Utils.Clamp(transfer/10, 10, 1000));
            ComputeTrajectory(optimizer);
            stage = Stage.ComputeRendezvou;
            trajectory = null;
		}

		protected override void fine_tune_approach()
		{
            update_trajectory();
            var startUT = VSL.Physics.UT+CorrectionOffset+TimeWarp.fixedDeltaTime*TRJ.PerFrameIterations*10;
            var transfer = trajectory.AtTargetUT-startUT;
            var dT = trajectory.TransferTime/10;
            if(transfer <= 0)
            {
                transfer = trajectory.OrigOrbit.GetEndUT()-startUT;
                dT = transfer/10;
            }
            ComputeTrajectory(new StartTimeOptimizer(this, startUT, transfer, dT));
            stage = Stage.ComputeCorrection;
            trajectory = null;
		}

		protected void compute_approach_orbit(double StartUT)
		{
            VSL.Controls.StopWarp();
            var transfer_time = (VesselOrbit.period+TargetOrbit.period)/4;
            double maxEndUT;
            var lastOrbitT = LastOrbit(TargetOrbit);
            if(DiscontiniousOrbit(lastOrbitT))
                maxEndUT = lastOrbitT.EndUT;
            else if(!double.IsInfinity(lastOrbitT.period))
                maxEndUT = VSL.Physics.UT+lastOrbitT.period*(REN.MaxTTR+2);
            else 
                maxEndUT = NearestRadiusUT(lastOrbitT, lastOrbitT.referenceBody.sphereOfInfluence, VSL.Physics.UT, false);
            ComputeTrajectory(new TransferOptimizer(this,
                                                    maxEndUT,
                                                    StartUT,
                                                    transfer_time,
                                                    transfer_time/2));
            stage = Stage.ComputeSuborbitRendezvous;
            trajectory = null;
		}

		void start_orbit()
		{
			ToOrbit = null;
            THR.Throttle = 0;
			CFG.BR.OffIfOn(BearingMode.Auto);
            update_trajectory();
			if(VesselOrbit.PeR < MinPeR) 
			{
                var StartUT = Math.Min(trajectory.AtTargetUT, VSL.Physics.UT+(VesselOrbit.ApAhead()? VesselOrbit.timeToAp : CorrectionOffset));
                //approach is close enough to directly match orbits
				if(trajectory.DistanceToTarget < REN.ApproachThreshold*2 && 
                   StartUT.Equals(trajectory.AtTargetUT)) 
					match_orbits(); 
                else compute_approach_orbit(StartUT);
//				var transfer_time = Utils.ClampL(TargetOrbit.period*(0.25-AngleDelta(VesselOrbit, TargetOrbit, StartUT)/360), 1);
//				var solver = new LambertSolver(VesselOrbit, TargetOrbit.getRelativePositionAtUT(StartUT+transfer_time), StartUT);
//				var dV = solver.dV4Transfer(transfer_time);
//				var trj = new RendezvousTrajectory(VSL, dV, StartUT, CFG.Target, transfer_time);
//                //if direct rendezvous orbit is possible
//				if(!dV.IsZero() && !trj.KillerOrbit)
//					compute_approach_orbit(StartUT);
//                else circularize();
			} 
            else next_stage();
		}

        void circularize()
        {
            update_trajectory();
            //start from circular orbit and proceed to TTR fitting
            var StartUT = VesselOrbit.ApAhead()? VSL.Physics.UT+VesselOrbit.timeToAp : VSL.Physics.UT+CorrectionOffset;
            var dV = dV4C(VesselOrbit, hV(StartUT), StartUT);
            //compute orbit with desired TTR and activate maneuver autopilot
            dV += dV4TTR(NewOrbit(VesselOrbit, dV, StartUT), TargetOrbit, REN.MaxTTR, REN.MaxDeltaV, MinPeR, StartUT);
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
            public Vector3d ApV;
            public double Dist = -1;

            public Launch(RendezvousAutopilot ren, double startUT, double transfer, double minPeR, double maxPeR, double ApAArc)
            {
                var tN = ren.TargetOrbit.GetOrbitNormal();
                var ApAUT = transfer < 0? startUT : startUT+transfer;
                var startPos = TrajectoryCalculator.BodyRotationAtdT(ren.Body, startUT-ren.VSL.Physics.UT)*ren.VesselOrbit.pos;
                var hVdir = Vector3d.Cross(startPos, tN).normalized;
                ApR = Utils.Clamp(ren.TargetOrbit.getRelativePositionAtUT(ApAUT).magnitude, minPeR, maxPeR);
                Transfer = AtmoSim.FromSurfaceTTA(ren.VSL, ApR-ren.Body.Radius, 
                                                  ApAArc, ren.GTurnCurve, 
                                                  Vector3d.Dot(ren.SurfaceVel, hVdir));
                UT = startUT;
                ApV = QuaternionD.AngleAxis(ApAArc*Mathf.Rad2Deg, Vector3d.Cross(hVdir, startPos))*startPos.normalized*ApR;
                Dist = (ApV-ren.TargetOrbit.getRelativePositionAtUT(UT+Transfer)).magnitude;
            }

            public static bool operator <(Launch a, Launch b) { return a.Dist < b.Dist; }
            public static bool operator >(Launch a, Launch b) { return a.Dist > b.Dist; }
            public override string ToString() 
            { 
                return Utils.Format("UT {}, Transfer {}, Dist {}, ApR {}", 
                                    UT, Transfer, Dist, ApR); 
            }
        }

        double inclinationDelta(double UT)
        {
            var tN = TargetOrbit.GetOrbitNormal();
            var startPos = BodyRotationAtdT(Body, UT-VSL.Physics.UT)*VesselOrbit.pos;
            return Math.Abs(Utils.Angle2(tN, startPos)-90);
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
                startUT = startUT+dT;
                if(!maxInc.Equals(dInc))
                {
                    dT /= -2.1;
                    startUT = maxIncUT+dT;
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
                startUT = startUT+dT;
                if(!bestInc.Equals(dInc))
                {
                    dT /= -2.1;
                    startUT = inPlaneUT+dT;
                }
            }
            return inPlaneUT >= VSL.Physics.UT? inPlaneUT :
                findInPlaneUT(inPlaneUT+Body.rotationPeriod/2, 60);
        }

        IEnumerator<object> launch_window_calculator;
        IEnumerator<object> calculate_launch_window()
        {
            var startUT = VSL.Physics.UT;
            var endUT = startUT + Body.rotationPeriod*REN.MaxTTR;
            var tEndUT = TargetOrbit.GetEndUT();
            if(DiscontiniousOrbit(LastOrbit(TargetOrbit)))
                endUT = tEndUT;
            else if(!double.IsInfinity(tEndUT))
                endUT = Math.Max(endUT, tEndUT);
            var dT = Math.Min((endUT-startUT)/10, Body.rotationPeriod/10);
            var minPeR = MinPeR;
            var maxPeR = minPeR+GLB.ORB.RadiusOffset;
            var ApAArc = Mathf.Lerp((float)((MinPeR-Body.Radius)/Body.Radius/2), 
                                    (float)((MinPeR-Body.Radius)/Body.Radius*3), 
                                    Utils.ClampH((2-GTurnCurve)/2, 1));
            //search for the nearest approach start UT
            //first scan startUT for possible minima
            ToOrbit = new ToOrbitExecutor(TCA);
            var minDis = new MinimumD();
            var minima = new List<double>();
            Launch cur = null;
            while(startUT < endUT)
            {
                #if DEBUG
                if(setp_by_step_computation && 
                   !string.IsNullOrEmpty(TCAGui.StatusMessage))
                { 
                    yield return null; 
                    continue; 
                }
                #endif
                cur = new Launch(this, startUT, cur == null? -1 : cur.Transfer, minPeR, maxPeR, ApAArc);
                minDis.Update(cur.Dist);
                if(minDis) minima.Add(startUT);
                startUT += dT;
                #if DEBUG
                if(setp_by_step_computation) 
                    Status("Push to proceed");
                #endif
                yield return null;
            }
            //then find each of the minima exactly and choose the best
            Launch best = null;
            foreach(var m in minima)
            {
                dT = 100;
                startUT = m;
                Launch min = null;
                while(Math.Abs(dT) > 0.01)
                {
                    #if DEBUG
                    if(setp_by_step_computation && 
                       !string.IsNullOrEmpty(TCAGui.StatusMessage))
                    { 
                        yield return null; 
                        continue; 
                    }
                    #endif
                    cur = new Launch(this, startUT, cur == null? -1 : cur.Transfer, minPeR, maxPeR, ApAArc);
                    if(min == null || cur < min) min = cur;
                    startUT += dT;
                    if(startUT < VSL.Physics.UT || startUT > endUT || cur != min)
                    {
                        dT /= -2.1;
                        startUT = Utils.Clamp(min.UT+dT, VSL.Physics.UT, endUT);
                    }
                    #if DEBUG
                    if(setp_by_step_computation) 
                        Status("Push to proceed");
                    #endif
                    yield return null;
                }
                var resonance = Math.Abs(ResonanceA(CircularOrbit(Body, min.ApV, 
                                                                  Vector3d.Cross(min.ApV, TargetOrbit.GetOrbitNormal()).normalized, 
                                                                  min.UT), TargetOrbit));
                if(min.Dist/maxPeR < REN.CorrectionStart*Math.Max(resonance, 1) && 
                   inclinationDelta(min.UT) < 5 && (best == null || min < best)) 
                    best = min;
            }
            //if the closest approach is too far away or too out of plane with the target,
            //start when in plane
            if(best == null)
                best = new Launch(this, 
                                  findInPlaneUT(VSL.Physics.UT, Body.rotationPeriod/10), 
                                  cur.Transfer, minPeR, maxPeR, ApAArc);
            ToOrbit.LaunchUT = best.UT;
            ToOrbit.ApAUT = best.UT+best.Transfer;
            ToOrbit.Target = best.ApV;
        }

		void to_orbit()
		{
			//setup launch
			CFG.DisableVSC();
            if(VSL.LandedOrSplashed)
            {
                launch_window_calculator = calculate_launch_window();
                stage = Stage.PreLaunch;
            }
            else
            {
                //calculate target vector
                var ApR = Utils.Clamp((TargetOrbit.PeR+TargetOrbit.ApR)/2, MinPeR, MinPeR+GLB.ORB.RadiusOffset);
                var hVdir = Vector3d.Cross(VesselOrbit.pos, TargetOrbit.GetOrbitNormal()).normalized;
                var ascO = AscendingOrbit(ApR, hVdir, GLB.ORB.LaunchSlope);
                ToOrbit = new ToOrbitExecutor(TCA);
                ToOrbit.LaunchUT = VSL.Physics.UT;
                ToOrbit.ApAUT = VSL.Physics.UT+ascO.timeToAp;
                ToOrbit.Target = ToOrbitIniApV = ascO.getRelativePositionAtUT(ToOrbit.ApAUT);
                ToOrbit.StartGravityTurn();
                MinDist.Reset();
                stage = Stage.ToOrbit;
            }
		}

		double correct_launch(bool allow_wait = true)
		{
			var TTR = AngleDelta(TargetOrbit, ToOrbit.Target, ToOrbit.ApAUT)/360*TargetOrbit.period;
            ToOrbit.LaunchUT += TTR/2*Math.Abs(Vector3d.Dot(Body.angularVelocity.normalized, TargetOrbit.GetOrbitNormal().normalized));
			if(allow_wait && ToOrbit.LaunchUT-VSL.Physics.UT <= 0) 
                ToOrbit.LaunchUT += TargetOrbit.period;
			ToOrbit.ApAUT = ToOrbit.LaunchUT+
				AtmoSim.FromSurfaceTTA(VSL, ToOrbit.TargetR-Body.Radius, 
				                       ToOrbit.ArcDistance, GLB.ORB.GTurnCurve, 
				                       Vector3d.Dot(SurfaceVel, Vector3d.Exclude(VesselOrbit.pos, ToOrbit.Target-VesselOrbit.pos).normalized));
            var maxPeR = MinPeR + GLB.ORB.RadiusOffset;
            var targetR = TargetOrbit.getRelativePositionAtUT(ToOrbit.ApAUT);
            ToOrbit.Target = QuaternionD.AngleAxis((VSL.Physics.UT-ToOrbit.LaunchUT)/Body.rotationPeriod*360, Body.angularVelocity.xzy)*
                (targetR.magnitude < maxPeR? ToOrbitIniApV.normalized*TargetOrbit.getRelativePositionAtUT(ToOrbit.ApAUT).magnitude : ToOrbitIniApV);
			return TTR;
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
            var D = Utils.ClampL((TargetOrbit.pos-VesselOrbit.pos).magnitude-VSL.Geometry.MinDistance, 0);
            if(D > REN.Dtol) 
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

        void next_stage()
        {
            if(TargetLoaded) 
                approach_or_brake();
            else
            {
                update_trajectory();
                var direct_approach = new RendezvousTrajectory(VSL, 
                                                               TargetOrbit.vel-VesselOrbit.vel + 
                                                               (TargetOrbit.pos-VesselOrbit.pos).normalized*REN.MaxApproachV,
                                                               VSL.Physics.UT, CFG.Target);
//                Log("dDist {}, cDist {}, direct approach {}", direct_approach.DistanceToTarget, CurrentDistance, direct_approach);//debug
                if(mode != Mode.DeltaV &&
                   direct_approach.FullBrake &&
                   direct_approach.DistanceToTarget < REN.Dtol && 
                   direct_approach.ManeuverDeltaV.magnitude < REN.MaxApproachV*2)
                    approach_or_brake();
                else if(CurrentDistance/VesselOrbit.semiMajorAxis > REN.CorrectionStart) 
                    compute_rendezvou_trajectory();
                else if(CurrentDistance > REN.Dtol)//if(relD > REN.CorrectionStart/4) 
                    fine_tune_approach();
                else 
                    match_orbits();
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
		}

		protected override void UpdateState()
		{
			base.UpdateState();
            IsActive &= CFG.AP2[Autopilot2.Rendezvous] && TargetOrbit != null;
			ControlsActive &= IsActive || 
                VSL.TargetVessel != null;
		}

        #if DEBUG
		double startAlpha;
		double startUT = -1;
		void log_flight()
		{
			var target = ToOrbit != null? ToOrbit.Target : VesselOrbit.getRelativePositionAtUT(VSL.Physics.UT+VesselOrbit.timeToAp);
			var alpha = Utils.ProjectionAngle(VesselOrbit.pos, target, target-VesselOrbit.pos) * Mathf.Deg2Rad;
			var arc = (startAlpha-alpha)*VesselOrbit.radius;
			CSV(VSL.Physics.UT-startUT, VSL.VerticalSpeed.Absolute, Vector3d.Exclude(VesselOrbit.pos, VesselOrbit.vel).magnitude, 
			    arc, VesselOrbit.radius-Body.Radius, 
			    VSL.Engines.Thrust.magnitude,
			    VSL.Physics.M, 90-Utils.Angle2(VesselOrbit.vel.xzy, VSL.Physics.Up));
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
				   VesselOrbit.ApR > MinPeR &&
				   VesselOrbit.radius > MinPeR)
					start_orbit();
				else to_orbit();
				break;
            case Stage.PreLaunch:
                if(launch_window_calculator != null)
                {
                    var i = TRJ.PerFrameIterations;
                    while(i-- > 0) { if(!launch_window_calculator.MoveNext()) break; }
                    if(i < 0) break;
                    launch_window_calculator = null;
                }
                stage = Stage.Launch;
                break;
			case Stage.Launch:
				if(ToOrbit.LaunchUT > VSL.Physics.UT) 
				{
                    TmpStatus("Waiting for launch window...");
                    ToOrbit.UpdateTargetPosition();
					VSL.Info.Countdown = ToOrbit.LaunchUT-VSL.Physics.UT;
					VSL.Controls.WarpToTime = ToOrbit.LaunchUT;
					break;
				}
				if(ToOrbit.Liftoff()) break;
				stage = Stage.ToOrbit;
				MinDist.Reset();
				break;
			case Stage.ToOrbit:
                if(ToOrbit.GravityTurn(ManeuverOffset, GTurnCurve.Value, GLB.ORB.Dist2VelF, REN.Dtol))
				{
					if(ToOrbit.dApA < REN.Dtol)
					{
						update_trajectory();
						MinDist.Update(CurrentDistance);
						if(MinDist) start_orbit();
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
                   trajectory.RelDistanceToTarget < REN.CorrectionStart &&
                   Math.Abs(trajectory.AtTargetPos.magnitude-EndApR)/EndApR < 0.1)
                {
                    clear_nodes();
                    add_trajectory_node_rel();
                    CFG.AP1.On(Autopilot1.Maneuver);
                    stage = Stage.Rendezvou;
                }
                else circularize();
                break;
			case Stage.ComputeRendezvou:
				if(!trajectory_computed()) break;
                optimizer = null;
                if(!trajectory.KillerOrbit &&
                    trajectory.RelDistanceToTarget < REN.CorrectionStart)
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
                   trajectory.RelDistanceToTarget < REN.CorrectionStart &&
                   trajectory.DistanceToTarget < CurrentDistance &&
                   trajectory.TimeToTarget > trajectory.BrakeDuration+trajectory.ManeuverDuration+ManeuverOffset+TimeWarp.CurrentRate)
                {
                    if(trajectory.ManeuverDeltaV.sqrMagnitude > 0.8)
                    {
                        VSL.Controls.StopWarp();
                        if(TimeWarp.CurrentRate > 1 ||
                           trajectory.TimeToStart < trajectory.ManeuverDuration/2+VSL.Torque.NoEngines.TurnTime)
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
                        CorrectionTimer.Period = Math.Max(trajectory.TimeToTarget/100, 10);
                        CorrectionTimer.Reset();
                        stage = Stage.Coast;
                    }
                }
                else if(CurrentDistance/VesselOrbit.semiMajorAxis < REN.CorrectionStart) 
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
                        VSL.Controls.WarpToTime = VSL.Physics.UT+(trajectory.TimeToTarget-trajectory.BrakeDuration+ManeuverOffset)/2;
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
                        var threshold = VSL.Geometry.MinDistance*2-trajectory.AtTargetRelPos.magnitude;
                        #if DEBUG
                        if(!CorrectingManeuver) 
                            Emailer.SendLocalSelf("allista@gmail.com", "TCA.REN: Proximity Alert!", 
                                                  "ETA: {} s\ndV: {} m/s\nDist {} m\nEnd Dist: {} m",
                                                  trajectory.TimeToTarget, 
                                                  (TargetOrbit.vel-VesselOrbit.vel).magnitude,
                                                  (TargetOrbit.pos-VesselOrbit.pos).magnitude,
                                                  trajectory.AtTargetRelPos.magnitude);
                        #endif
                        CorrectingManeuver = threshold > 0;
                        if(CorrectingManeuver)
                        {
                            VSL.Controls.StopWarp();
                            var correction = Vector3d.Exclude(TargetOrbit.pos-VesselOrbit.pos, trajectory.AtTargetRelPos).normalized *
                                threshold/Utils.Clamp(trajectory.TimeToTarget/2, 0.1, 10);
                            TmpStatus("Matching orbits at nearest approach...\n" +
                                      "<color=yellow><b>PROXIMITY ALERT!</b></color> Clearence {0:F1} m", 
                                      trajectory.DistanceToTarget);
                            if(MAN.ManeuverStage == ManeuverAutopilot.Stage.WAITING)
                                correction -= MAN.NodeDeltaV/2;
                            MAN.AddCourseCorrection(correction);
                            THR.CorrectThrottle = false;
                        }
                        else
                        {
                            clear_nodes();
                            add_target_node();
                            MAN.UpdateNode();
                        }
                    }
                    break;
                }
                if(MAN.ManeuverStage == ManeuverAutopilot.Stage.IN_PROGRESS)
                {
                    clear_nodes();
                    add_node_abs(TargetOrbit.vel-VesselOrbit.vel, VSL.Physics.UT);
                    CFG.AP1.On(Autopilot1.Maneuver);
                }
                next_stage();
				break;
			case Stage.Approach:
                TmpStatus("Approaching...");
                THR.DeltaV = 0;
				var dP = TargetOrbit.pos-VesselOrbit.pos;
				var dPm = dP.magnitude;
                if(dPm - VSL.Geometry.MinDistance < REN.Dtol) 
				{ brake(); break; }
                var throttle = VSL.Controls.AlignmentFactor;
                if(throttle.Equals(0)) break;
                var dVr = Vector3d.Dot(VesselOrbit.vel-TargetOrbit.vel, dP/dPm);
				var nVm = Utils.Clamp(dPm*REN.ApproachVelF, 1, REN.MaxApproachV);
				if(dVr < nVm) 
				{
					VSL.Engines.ActivateEngines();
                    THR.DeltaV = (float)Math.Max(nVm-dVr, 1)*throttle;
				}
				else 
                {
                    MVA.MinDeltaV = 0.5f;
                    brake();
                }
				break;
			case Stage.Brake:
                TmpStatus("Braking near target...");
				if(CFG.AP1[Autopilot1.MatchVelNear]) break;
				if(CFG.AP1[Autopilot1.MatchVel])
				{
					if((TargetOrbit.vel-VesselOrbit.vel).magnitude > GLB.THR.MinDeltaV) break;
					CFG.AP1.Off();
					THR.Throttle = 0;
				}
                if((VSL.Physics.wCoM-TargetVessel.CurrentCoM).magnitude-VSL.Geometry.MinDistance > REN.Dtol)
				{ approach(); break; }
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
				if(ToOrbit != null)
				{
					Utils.GLVec(Body.position, ToOrbit.Target.xzy, Color.green);
					Utils.GLVec(Body.position, Vector3d.Cross(VesselOrbit.pos, ToOrbit.Target).normalized.xzy*Body.Radius*1.1, Color.red);
					Utils.GLVec(Body.position, TargetOrbit.getRelativePositionAtUT(ToOrbit.ApAUT).xzy, Color.yellow);
                    Utils.GLVec(Body.position, VesselOrbit.getRelativePositionAtUT(VSL.Physics.UT+VesselOrbit.timeToAp).xzy, Color.magenta);
                    Utils.GLVec(Body.position, VesselOrbit.GetOrbitNormal().normalized.xzy*Body.Radius*1.1, Color.cyan);
                    Utils.GLVec(Body.position, TargetOrbit.GetOrbitNormal().normalized.xzy*Body.Radius*1.1, new Color(1, 0.3f, 0));
				}
				else Utils.GLVec(Body.position, TargetOrbit.getRelativePositionAtUT(VSL.Physics.UT+VesselOrbit.timeToAp).xzy, Color.yellow);
//                Utils.GLVec(Body.position+VesselOrbit.pos.xzy, Vector3d.Cross(TargetOrbit.GetOrbitNormal().normalized, VesselOrbit.pos.normalized).normalized.xzy*50000, new Color(0.3f, 1, 0));
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
            GUILayout.BeginHorizontal();
            GUILayout.Label(new GUIContent("Gravity Turn:", "Sharpness of the gravity turn"), GUILayout.ExpandWidth(false));
            GTurnCurve.Draw("", 0.1f);
            GUILayout.EndHorizontal();
            GUILayout.BeginHorizontal();
            if(computing)
                GUILayout.Label(new GUIContent("Search Mode: "+ModeNames[(int)mode], ModeDesc[(int)mode]), 
                                Styles.grey, GUILayout.ExpandWidth(true));
            else 
            {
                GUILayout.Label("Search Mode:", GUILayout.ExpandWidth(false));
                var choice = Utils.LeftRightChooser(ModeNames[(int)mode], ModeDesc[(int)mode]);
                if(choice > 0) mode = (Mode)(((int)mode+1)%NumModes);
                if(choice < 0) mode = (Mode)(mode > 0? (int)mode-1 : NumModes-1);
            }
            GUILayout.EndHorizontal();
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
                    VSL.Engines.ActivateEnginesAndRun(() => CFG.AP2.XOn(Autopilot2.Rendezvous));
            }
            GUILayout.EndHorizontal();
        }

        public void DrawBestTrajectories()
        {
            if(optimizer != null && mode == Mode.Manual)
                optimizer.DrawBestTrajecotries();
        }

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
                if(bestDeltaV > 1 || t.DistanceToTarget > REN.Dtol) 
                    return t.Quality < Best.Quality;
                //otherwise select the one with larger maneuver dV
                return t.ManeuverDeltaV.sqrMagnitude > bestDeltaV;
            }

            static IEnumerator<char> indicator_seq()
            {
                const string clock = "◐◓◑◒";
                var clen = clock.Length;
                var timer = new RealTimer(0.125);
                var i = 0;
                while(true)
                {
                    yield return clock[i];
                    if(timer.TimePassed)
                    {
                        i = (i+1)%clen;
                        timer.Restart();
                    }
                }
            }
            static IEnumerator<char> _indicator;
            protected static IEnumerator<char> indicator
            {
                get
                {
                    if(_indicator == null)
                        _indicator = indicator_seq();
                    return _indicator;
                }
            }

            protected string BestDesc
            {
                get
                {
                    var tts = Best.TimeToStart;
                    return string.Format("T- {0}  ETA <color=lime>{1}</color>  dV: <color=yellow><b>{2:F1}</b> m/s</color>", 
                                         Utils.formatTimeDelta(tts),
                                         Utils.formatTimeDelta(tts+Best.TransferTime),
                                         Best.GetTotalDeltaV());
                }
            }

            public virtual string Status
            {
                get
                {
                    indicator.MoveNext();
                    var s = indicator.Current+" searching for the best trajectory";
                    if(Best == null) return s+"...";
                    return s+"\n"+BestDesc;
                }
            }
        }

        class TransferOptimizer : RendezvousTrajectoryOptimizer
        {
            readonly double startUT, transferT;

            public TransferOptimizer(RendezvousAutopilot ren,
                                     double maxEndUT, double startUT, double transfer, double dT)

                : base(ren, ren.VSL.Physics.UT+ren.CorrectionOffset, maxEndUT, maxEndUT, false, dT)
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
                    minStartUT = ren.VSL.Physics.UT+ren.CorrectionOffset;
                    if(start-minStartUT < t.ManeuverDuration/2)
                        start = minStartUT+t.ManeuverDuration/2+TimeWarp.fixedDeltaTime*TRJ.PerFrameIterations*10;
                    transfer = Math.Max(t.TransferTime+dt, 0);
                    if(scanned && (start+transfer > maxEndUT || transfer < 1 || t.KillerOrbit ||
                                   !t.KillerOrbit && !Best.KillerOrbit && !is_better) ||
                       !scanned && start+transfer > maxEndUT)
                    {
                        dt /= -2.1;
                        transfer = Math.Max(Best.TransferTime+dt, 1);
                        scanned = true;
                    }
                    yield return t;
                }
            }
        }

        class StartTimeOptimizer : RendezvousTrajectoryOptimizer
        {
            readonly double startUT;

            public StartTimeOptimizer(RendezvousAutopilot ren,
                                      double startUT, double transfer, double dT)

                : base(ren, ren.VSL.Physics.UT+ren.CorrectionOffset, startUT+transfer, -1, false, dT)
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
                    var t = ren.new_trajectory(start, maxStartUT-start);
                    var is_better = trajectory_is_better(t);
                    if(is_better) Best = t;
                    minStartUT = ren.VSL.Physics.UT+ren.CorrectionOffset;
                    start = t.StartUT+dt;
                    if(scanned && (start >= maxStartUT || start <= minStartUT ||
                                   !t.KillerOrbit && !Best.KillerOrbit && !is_better) ||
                       !scanned && start >= maxStartUT)
                    {
                        dt /= -2.1;
                        start = Utils.Clamp(Best.StartUT+dt, minStartUT, maxStartUT);
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

                public static bool Close(Point a, Point b)
                { return Math.Abs(a.start-b.start) < 10 && Math.Abs(a.transfer-b.transfer) < 10; }

                public bool Better(Point b)
                {
                    if(opt.ren.mode == Mode.TimeToTarget) 
                        return 
                            distance + (transfer + start-opt.minStartUT)/100 < 
                            b.distance + (b.transfer + b.start-opt.minStartUT)/100;
                    return distance < b.distance;
                }

                public override string ToString()
                { return Utils.Format("startUT {}, transfer {}, distance {}, trajectory {}", 
                                      start, transfer, distance, trajectory); }

                public bool Draw(bool selected)
                {
                    GUILayout.BeginHorizontal();
                    var tts = trajectory.TimeToStart;
                    var label = string.Format("ETA:  <color=lime>{0}</color>\n" +
                                              "Node: <color={1}>{2}</color>",
                                              Utils.formatTimeDelta(tts+transfer),
                                              tts > opt.ren.ManeuverOffset? "white" : "red",
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
                P0 = P = new Point(Math.Max(startUT, minStartUT+1), startTransfer, this);
                set_dir(new Vector2d(0,1));
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
                var eobt = p.trajectory.EndOrbit;
                return !p.trajectory.KillerOrbit && 
                    p.start > minStartUT && p.transfer > 1 && 
                    (softMaxStart || p.start < maxStartUT) &&
                    (maxEndUT < 0 || p.start+p.transfer < maxEndUT) &&
                    (eobt.patchEndTransition == Orbit.PatchTransitionType.FINAL ||
                     eobt.EndUT-p.trajectory.AtTargetUT > p.trajectory.BrakeDuration+1) &&
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
                    p.start = minStartUT+1;
                    p.transfer = Math.Max(ren.TargetOrbit.EndUT-p.start-ren.ManeuverOffset, 2);
                    p.UpdateTrajectory(true);
                    if(feasible_point(p)) 
                        start_points.Add(p);
                }
            }

            IEnumerable<RendezvousTrajectory> scan_start_time()
            {
                var cur  = P;
                var prev = P;
                var bestP = cur;
                var bestOK = feasible_point(bestP);
                var endUT = Math.Max(P.start+P.transfer*2, maxStartUT);
                var maxTT = maxStartUT-minStartUT;
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
                    bestP = P;
                }
                if(!start_points.Contains(bestP))
                    start_points.Add(bestP);
                set_dir(new Vector2d(1,0));
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
                    var step_k = Point.DistK(prev,cur);
                    prev = cur;
                    path += step_k;
                    cur += dir*dt*step_k;
                    var requestedTT = cur.transfer;
                    cur.UpdateTrajectory(true);
                    if(feasible_point(cur) && cur < bestP)
                    {
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
                if(P0 < P) 
                {
                    set_dir(Point.Delta(P, P0).normalized);
                    P = P0;
                }
                else set_dir(Point.Delta(P0, P).normalized);
                if(P.start < minStartUT+1 && dir.x < 0) 
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
            }

            IEnumerable<RendezvousTrajectory> build_conjugate_set(double dt)
            {
                set_dir(new Vector2d(0,1));
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
                if(P.trajectory.DistanceToTarget < REN.Dtol) best_points.Add(P);
                for(int i = 0, count = start_points.Count; i < count; i++)
                {
                    if(need_to_stop) yield break;
                    progress = (i+1f)/(count+1);
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
                       P.trajectory.DistanceToTarget < REN.Dtol)
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
                    best_points.Sort((a, b) => (a.start+a.transfer).CompareTo(b.start+b.transfer));
                    break;
                case 2:
                    best_points.Sort((a, b) => a.start.CompareTo(b.start));
                    break;
                }
            }

            int sort_order = 0;
            static string[] sorting = {"dV", "ETA", "Start"};
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
                        return "Selected transfer:\n"+BestDesc;
                    indicator.MoveNext();
                    var s = indicator.Current + 
                        (ren.mode == Mode.Manual?
                         " searching for transfers" : " searching for the best transfer");
                    if(Best == null) return s+"...";
                    return s+string.Format(" {0:P0}\n", progress)+BestDesc;
                }
            }
        }
        #endregion

        #if DEBUG
        static void log_patches(Orbit o, string tag)
        { Utils.Log(Utils.formatPatches(o, tag));}
         
        public static bool _CalculatePatch(Orbit p, Orbit nextPatch, double startEpoch, PatchedConics.SolverParameters pars, CelestialBody targetBody)
        {
            p.activePatch = true;
            p.nextPatch = nextPatch;
            p.patchEndTransition = Orbit.PatchTransitionType.FINAL;
            p.closestEncounterLevel = Orbit.EncounterSolutionLevel.NONE;
            p.numClosePoints = 0;
            log_patches(p, "Patch 0");
            int count = Planetarium.Orbits.Count;
            for (int i = 0; i < count; i++)
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
            if (p.patchEndTransition == Orbit.PatchTransitionType.FINAL)
            {
                if (!pars.debug_disableEscapeCheck)
                {
                    if (p.ApR <= p.referenceBody.sphereOfInfluence)
                    {
                        if (p.eccentricity < 1.0)
                        {
                            p.UTsoi = -1.0;
                            p.StartUT = startEpoch;
                            p.EndUT = startEpoch + p.period;
                            p.patchEndTransition = Orbit.PatchTransitionType.FINAL;
                            goto IL_2C0;
                        }
                    }
                    if (double.IsInfinity(p.referenceBody.sphereOfInfluence))
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
            if (nextPatch.eccentricity < 1.0)
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
