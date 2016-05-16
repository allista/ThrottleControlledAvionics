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
	public class RendezvouAutopilot : TargetedTrajectoryCalculator<RendezvousTrajectory, Vessel>
	{
		public new class Config : TCAModule.ModuleConfig
		{
			[Persistent] public float Dtol                = 100f;   //m
			[Persistent] public float StartOffset         = 60f;    //s
			[Persistent] public float MaxTTR              = 3f;     //1/VesselOrbit.period
			[Persistent] public float MaxDeltaV           = 100f;   //m/s
			[Persistent] public float CorrectionStart     = 10000f; //m
			[Persistent] public float CorrectionOffset    = 20f;    //s
			[Persistent] public float CorrectionTimer     = 10f;    //s
			[Persistent] public float ApproachThreshold   = 500f;   //m
			[Persistent] public float MaxApproachV        = 20f;    //parts
			[Persistent] public float ApproachVelF        = 0.01f;  //parts
			[Persistent] public float MaxInclinationDelta = 30;     //deg
			[Persistent] public float GTurnCurve          = 60;
			[Persistent] public float LaunchDeltaVf       = 3;
			[Persistent] public float LaunchTangentK      = 1f;
			[Persistent] public float RendezvouCorrection = 0.8f;
		}
		static Config REN { get { return TCAScenario.Globals.REN; } }

		public RendezvouAutopilot(ModuleTCA tca) : base(tca) {}

		AttitudeControl ATC;
		ThrottleControl THR;
		TranslationControl TRA;
		BearingControl BRC;

		enum Stage { None, Start, Launch, ToOrbit, StartOrbit, ComputeRendezvou, Rendezvou, MatchOrbits, Approach, Brake }
		Stage stage;

		double CurrentDistance = -1;
		RendezvousTrajectory CurrentTrajectory 
		{ get { return new RendezvousTrajectory(VSL, Vector3d.zero, VSL.Physics.UT, Target, REN.StartOffset); } }

		double ApAUT, TTA, MinAccelTime;
		double LaunchUT  = -1;
		double LaunchApR = -1;
		Vector3d TargetApV, ApV;
		FuzzyThreshold<float> ToOrbitDeltaV = new FuzzyThreshold<float>(5, 1);
		SingleAction GearAction = new SingleAction();

		public override void Init()
		{
			base.Init();
			Dtol = REN.Dtol;
			CorrectionTimer.Period = REN.CorrectionTimer;
			CFG.AP2.AddHandler(this, Autopilot2.Rendezvou);
			GearAction.action = () => VSL.vessel.ActionGroups.SetGroup(KSPActionGroup.Gear, false);
			#if DEBUG
			RenderingManager.AddToPostDrawQueue(1, RadarBeam);
			#endif
		}

		#if DEBUG
		public void RadarBeam()
		{
			if(VSL == null || VSL.vessel == null || VSL.refT == null || !CFG.AP2[Autopilot2.Rendezvou]) return;
			if(stage == Stage.ToOrbit)
			{
				GLUtils.GLVec(VSL.Physics.wCoM, tNorm.normalized.xzy*12, Color.green);
				GLUtils.GLVec(VSL.Physics.wCoM, norm.normalized.xzy*10, Color.yellow);
				GLUtils.GLVec(VSL.Physics.wCoM, hVdir.normalized.xzy*10, Color.red);
				GLUtils.GLVec(VSL.Physics.wCoM, VesselOrbit.pos.normalized.xzy*10, Color.blue);
				GLUtils.GLVec(VSL.Physics.wCoM, velN.xzy*10, Color.cyan);
				GLUtils.GLVec(VSL.Physics.wCoM, -Body.angularVelocity.normalized*10, Color.white);
				GLUtils.GLVec(VSL.Physics.wCoM, TargetApV.normalized.xzy*10, Color.magenta);
			}
		}

		public override void Reset()
		{
			base.Reset();
			RenderingManager.RemoveFromPostDrawQueue(1, RadarBeam);
		}
		#endif

		public void RendezvouCallback(Multiplexer.Command cmd)
		{
			reset();
			switch(cmd)
			{
			case Multiplexer.Command.Resume:
				if(!setup()) 
				{
					CFG.AP2.Off();
					return;
				}
				stage = Stage.Start;
				break;

			case Multiplexer.Command.On:
				goto case Multiplexer.Command.Resume;

			case Multiplexer.Command.Off:
				break;
			}
		}

		protected override bool check_target()
		{
			if(!base.check_target()) return false;
			var tVSL = VSL.TargetVessel;
			if(tVSL == null) 
			{
				Status("yellow", "Target should be a vessel");
				return false;
			}
			if(tVSL.LandedOrSplashed)
			{
				Status("yellow", "Target vessel is landed");
				return false;
			}
			if(Math.Abs(tVSL.orbit.inclination-VesselOrbit.inclination) > REN.MaxInclinationDelta)
			{
				Status("yellow", "Target orbit plane is tilted more than {0:F}° with respect to ours.\n" +
				       "You need to change orbit plane before the rendezvou maneuver.", REN.MaxInclinationDelta);
				return false;
			}
			return true;
		}

		protected override void setup_target()
		{ 
			Target = VSL.TargetVessel; 
			SetTarget(Target);
		}

		protected RendezvousTrajectory rendezvou_orbit(RendezvousTrajectory old, ref Vector3d NodeDeltaV)
		{
			double StartUT;
			Vector3d dV = Vector3d.zero;
			if(old != null) 
			{
				var distF = Utils.ClampH(old.DistanceToTarget/REN.CorrectionStart, 1);
				StartUT = AngleDelta2StartUT(old, old.DeltaTA, REN.StartOffset, VesselOrbit.period/4, 
				                             VesselOrbit.period*distF);
				NodeDeltaV += RadiusCorrection(old);
				NodeDeltaV += PlaneCorrection(old)*(1-distF*0.99);
				dV = Node2OrbitDeltaV(StartUT, NodeDeltaV);
				LogF("\nRO: dV {}, dUT {}", NodeDeltaV, StartUT-old.StartUT);//debug
//				CSV(old.TimeToStart, old.TimeToTarget, old.DeltaTA, old.DeltaFi, old.DeltaR, old.DistanceToTarget, NodeDeltaV.x, NodeDeltaV.y, NodeDeltaV.z);//debug
			}
			else 
			{
				var start_offset = VesselOrbit.period/3;
				double alpha, resonanse;
				StartUT = VSL.Physics.UT+
					Utils.ClampL(TimeToResonance(VesselOrbit, Target.orbit, VSL.Physics.UT+REN.StartOffset, out resonanse, out alpha)*VesselOrbit.period
					             -start_offset, REN.StartOffset);
				double AtTargetUT;
				TrajectoryCalculator.ClosestApproach(VesselOrbit, Target.orbit, StartUT, out AtTargetUT, REN.StartOffset);
				LogF("First Time TimeToStart: {}, Correction {}", StartUT-VSL.Physics.UT, AtTargetUT-start_offset-StartUT);//debug
				StartUT = AtTargetUT-start_offset;
			}
			return new RendezvousTrajectory(VSL, dV, StartUT, Target, REN.StartOffset);
		}

		protected RendezvousTrajectory orbit_correction(RendezvousTrajectory old, ref Vector3d NodeDeltaV)
		{
			double StartUT = VSL.Physics.UT+REN.CorrectionOffset;
			Vector3d dV = Vector3d.zero;
			if(old != null) 
			{
				var UT = old.AtTargetUT - old.NewOrbit.period/3;
				if(UT > VSL.Physics.UT+REN.CorrectionOffset) StartUT = UT;
				if(Math.Abs(old.DeltaR) > old.DistanceToTarget) NodeDeltaV += RadiusCorrection(old);
				var distF = (1-Utils.ClampH(old.DistanceToTarget/REN.Dtol*2, 1)*0.99);
				NodeDeltaV += AoPCorrection(old, -old.DeltaTA)*distF;
				NodeDeltaV += PlaneCorrection(old)*distF;
				dV = Node2OrbitDeltaV(StartUT, NodeDeltaV);
				LogF("\nOC: dV {}", NodeDeltaV);//debug
//				CSV(old.TimeToStart, old.TimeToTarget, old.DeltaTA, old.DeltaFi, old.DeltaR, old.DistanceToTarget, 
//				    NodeDeltaV.x, NodeDeltaV.y, NodeDeltaV.z,
//				    TimeToResonance(old.NewOrbit, Target.orbit, StartUT)*old.NewOrbit.period);//debug
			} 
			return new RendezvousTrajectory(VSL, dV, StartUT, Target, REN.CorrectionOffset, 
			                                VSL.Physics.UT+REN.CorrectionOffset, nearest_approach:true);
		}
			
		void compute_rendezvou_trajectory()
		{
			trajectory = null;
			stage = Stage.ComputeRendezvou;
			Vector3d NodeDeltaV = Vector3d.zero;
			setup_calculation(t => rendezvou_orbit(t, ref NodeDeltaV));
		}

		protected override void start_correction()
		{
			trajectory = null;
			stage = Stage.ComputeRendezvou;
			Vector3d NodeDeltaV = Vector3d.zero;
			setup_calculation(t => orbit_correction(t, ref NodeDeltaV));
		}

		void start_orbit()
		{
			update_trajectory();
			if(CurrentDistance < REN.CorrectionStart)
			{
				match_orbits();
				return;
			}
			var dV = Vector3d.zero;
			var old = VesselOrbit;
			var StartUT = VSL.Physics.UT+REN.CorrectionOffset;
			if(VesselOrbit.PeR < MinPeR) 
			{
				StartUT = ApAhead? VSL.Physics.UT+VesselOrbit.timeToAp : VSL.Physics.UT+REN.StartOffset;
				dV += dV4C(old, hV(StartUT), StartUT);
				old = NewOrbit(old, dV, StartUT);
			}
			LogF("\nini orbit\n{}", old);//debug
			dV += dV4TTR(old, Target.orbit, REN.MaxTTR, REN.MaxDeltaV, MinPeR, StartUT);
			LogF("\nstart dV: {}\nstart orbit\n{}", dV, NewOrbit(VesselOrbit, dV, StartUT));//debug
			if(!dV.IsZero())
			{
				add_node(dV, StartUT);
				CFG.AP1.On(Autopilot1.Maneuver);
			}
			stage = Stage.StartOrbit;
		}

		double atm_density(double h)
		{
			if(!Body.atmosphere) return 0;
			var P = Body.GetPressure(h);
			var T = Body.GetTemperature(h);
			return Body.GetDensity(P, T);
		}

		double drag(double s, double h, double v)
		{ return atm_density(h) * v*v * Cd * s/2; }

		double StG(double h) 
		{ 
			var r = Body.Radius+h;
			return Body.gMagnitudeAtCenter/r/r; 
		}

		const double dt = 0.5;
		const double Cd = 0.0006;
		double freefall(double m, double s, double h, double v, double dt)
		{
			var H = h;
			while(v > 0)
			{
				H += v*dt;
				v -= (StG(H) + drag(s, H, v)/m)*dt;
			}
			return H;
		}

		double from_surface_TTA(double ApA)
		{
			var t = 0.0;
			var v = 0.0;
			var h = (double)VSL.Altitude.Absolute;
			var m = (double)VSL.Physics.M;
			var mT = VSL.Engines.MaxThrust;
			var mTm = VSL.Engines.MaxThrustM;
			var mflow = VSL.Engines.MaxMassFlow;
			if(VSL.Engines.NumActive.Equals(0))
			{
				mT = VSL.Engines.NextStageMaxThrust(out mflow);
				mTm = mT.magnitude;
			}
			var s = VSL.Geometry.AreaInDirection(mT);
			var hmove = REN.LaunchTangentK*ApA;
			var thrust = true;
			while(v >= 0)
			{
				if(thrust)
				{
					if(!CheatOptions.InfiniteFuel)
					{
						var dm = mflow*dt;
						if(m < dm) { thrust = false; continue; }
						m -= dm;
					}
					var apa = freefall(m, s, h, v, dt*4);
					var dapa = ApA-apa;
					var vv = dapa/Utils.ClampL(v, 1);
					var hv = hmove/Utils.ClampL(dapa, REN.GTurnCurve)*Utils.Clamp((apa-h)/100, 0, 1)*REN.GTurnCurve;
					if(Body.atmosphere) hv *= Math.Sqrt(Utils.Clamp(VSL.Altitude.Absolute/Body.atmosphereDepth, 0, 1));
					hmove -= hv*dt;
					var T = mTm*Math.Sin(Math.Atan2(vv, hv));
					v += (T/m-StG(h))*dt;
					thrust = ApA-apa > 1;
					LogF("Sim: apa {}, ApA {}, T/m {}", apa, ApA, T/m);
				}
				else v -= StG(h)*dt;
				v -= drag(s, h, v)/m*dt;
				h += v*dt;
				t += dt;
				LogF("Sim: m {}, s {}, v {}, h {}, t {} drag/m {}", 
					m, s, v, h , t, drag(s, h, v)/m);//debug
			}
			return t-dt/2;
		}

		Vector3d hVdir, tNorm, norm, velN; //debug
		void to_orbit()
		{
			//sanity check
			if(VSL.Engines.NumActive > 0 && VSL.OnPlanet && VSL.OnPlanetParams.MaxTWR <= 1)
			{
				Status("red", "TWR < 1, impossible to achive orbit");
				CFG.AP2.Off();
				return;
			}
			//compute trajectory to get the needed apoapsis at needed time
			tNorm = Target.orbit.GetOrbitNormal();//test
			LaunchApR = Math.Max(MinPeR, (Target.orbit.PeR+Target.orbit.ApR)/2);
			hVdir = Vector3d.Cross(Target.orbit.GetOrbitNormal(), VesselOrbit.pos).normalized;
			norm  = Vector3d.Cross(VesselOrbit.pos, hVdir).normalized;
			var LaunchRad = Utils.ClampH(Math.Atan(1/(Body.Radius*REN.LaunchTangentK/(2*LaunchApR) - 
			                                          Body.angularV/Math.Sqrt(2*VSL.Physics.StG*(LaunchApR-Body.Radius)))), 
			                             Math.PI/2);
			velN    = (Math.Sin(LaunchRad)*VesselOrbit.pos.normalized + Math.Cos(LaunchRad)*hVdir).normalized;
			var vel = Math.Sqrt(2*VSL.Physics.G*(LaunchApR-Body.Radius)) / Math.Sin(LaunchRad);
			var v   = 0.0;
			while(vel-v > TRJ.dVtol)
			{
				var V = (v+vel)/2;
				var o = NewOrbit(VesselOrbit, velN*V-VesselOrbit.vel, VSL.Physics.UT);
				if(o.ApR > LaunchApR) vel = V;
				else v = V;
			} vel = (v+vel)/2;
			TTA = from_surface_TTA(LaunchApR-Body.Radius);
			LogF("LaunchAngle {}, Omega0 {}, TTA {}", 
			     LaunchRad*Mathf.Rad2Deg, Body.angularV, TTA);//debug
			LaunchUT = VSL.Physics.UT;
			ApAUT = LaunchUT+TTA;
			var ascO = NewOrbit(VesselOrbit, velN*vel-VesselOrbit.vel, VSL.Physics.UT);
			if(VSL.LandedOrSplashed)
			{
				ApV = ascO.getRelativePositionAtUT(ApAUT);
				double TTR;
				do { TTR = correct_launch(); } 
				while(Math.Abs(TTR) > 1);
			}
			else TargetApV = ascO.getRelativePositionAtUT(ApAUT);
			CFG.DisableVSC();
			stage = VSL.LandedOrSplashed? Stage.Launch : Stage.ToOrbit;
		}

		double correct_launch()
		{
			var TTR = Utils.ProjectionAngle(Target.orbit.getRelativePositionAtUT(ApAUT), 
			    	                        TargetApV, 
			        	                    Target.orbit.getOrbitalVelocityAtUT(ApAUT)) /
				360*Target.orbit.period;
			LaunchUT += TTR;
			if(LaunchUT-VSL.Physics.UT <= 0) 
				LaunchUT += Target.orbit.period;
			ApAUT= LaunchUT+TTA;
			TargetApV = QuaternionD.AngleAxis((VSL.Physics.UT-LaunchUT)/Body.rotationPeriod*360, Body.angularVelocity.xzy)*ApV;

			var angle = Utils.ProjectionAngle(VesselOrbit.pos, TargetApV, VesselOrbit.vel);//debug
			LogF("\nTTR {}, TTL {}, AngleToApV {}, DistToApV {}\n pos {}\nTargetApV {}", 
			     TTR, LaunchUT-VSL.Physics.UT, angle, angle*Mathf.Deg2Rad*LaunchApR/(LaunchApR-Body.Radius),
			     VesselOrbit.pos, TargetApV);//debug

			return TTR;
		}

		void correct_rendezvou()
		{
			var angle = Utils.ProjectionAngle(Target.orbit.getRelativePositionAtUT(ApAUT), 
			                                  TargetApV, 
			                                  Target.orbit.getOrbitalVelocityAtUT(ApAUT));
			TargetApV = QuaternionD.AngleAxis(angle, Body.angularVelocity.xzy)*TargetApV;

			angle = Utils.ProjectionAngle(VesselOrbit.pos, TargetApV, VesselOrbit.vel);//debug
			LogF("\nAngleToApV {}, DistToApV {}\n pos {}\nTargetApV {}", 
			     angle, angle*Mathf.Deg2Rad*LaunchApR/(LaunchApR-Body.Radius),
			     VesselOrbit.pos, TargetApV);//debug
		}

		void match_orbits()
		{
			SetTarget(Target);
			update_trajectory();
			add_target_node();
			CFG.AP1.On(Autopilot1.Maneuver);
			stage = Stage.MatchOrbits;
			MAN.MinDeltaV = 0.5f;
		}

		void approach()
		{
			CFG.AT.On(Attitude.Target);
			stage = Stage.Approach;
		}

		void brake()
		{
			SetTarget(Target);
			stage = Stage.Brake;
			var dV = VesselOrbit.vel-Target.orbit.vel;
			var dVm = dV.magnitude;
			var dist = Target.CurrentCoM-VSL.Physics.wCoM;
			var distm = dist.magnitude;
			if(distm < REN.Dtol && dVm < GLB.THR.MinDeltaV*2) return;
			if(distm > REN.Dtol && Vector3.Dot(dist, dV.xzy) > 0)
				CFG.AP1.On(Autopilot1.MatchVelNear);
			else CFG.AP1.On(Autopilot1.MatchVel);
		}

		void update_trajectory(bool update_distance=true)
		{
			if(trajectory == null) trajectory = CurrentTrajectory;
			else trajectory.UpdateOrbit(VesselOrbit);
			if(update_distance) CurrentDistance = trajectory.DistanceToTarget;
		}

		protected override void reset()
		{
			base.reset();
			stage = Stage.None;
			CurrentDistance = -1;
			CFG.AP1.Off();
		}

		protected override void UpdateState()
		{
			base.UpdateState();
			IsActive &= CFG.AP2[Autopilot2.Rendezvou];
			ControlsActive = IsActive || VSL.TargetVessel != null;
		}

		SingleAction pause = new SingleAction();
		protected override void Update()
		{
			if(!IsActive) return;
			switch(stage)
			{
			case Stage.Start:
				if(VSL.InOrbit && 
				   VesselOrbit.ApR > MinPeR &&
				   VesselOrbit.radius > MinPeR) start_orbit();
				else to_orbit();
				break;
			case Stage.Launch:
				//correct launch time
				if(LaunchUT > VSL.Physics.UT) 
				{
					correct_launch();
					VSL.Info.Countdown = LaunchUT-VSL.Physics.UT-MinAccelTime;
					WRP.WarpToTime = LaunchUT;
					break;
				}
				if(VSL.Engines.Active.Count == 0) VSL.ActivateNextStage();
//				CSV(VSL.Physics.UT, VesselOrbit.radius-Body.Radius, VSL.VerticalSpeed.Absolute, 90);//debug
				if(VSL.VerticalSpeed.Absolute/VSL.Physics.G < 5)
				{ 
					CFG.VTOLAssistON = true;
					THR.Throttle = 1; 
					break; 
				}
				CFG.VTOLAssistON = false;
				CFG.StabilizeFlight = false;
				CFG.HF.Off();
				stage = Stage.ToOrbit;
				break;
			case Stage.ToOrbit:
				var dApA   = LaunchApR-VesselOrbit.ApR;
				if(dApA < REN.Dtol)
				{
					pause.Run(PauseMenu.Display);//debug
					update_trajectory();
					if(CurrentDistance < REN.CorrectionStart)
					{ match_orbits(); break; }
				}

				ApAUT = VSL.Physics.UT+VesselOrbit.timeToAp;
				var vel    = Vector3d.zero;
				var cApV   = VesselOrbit.getRelativePositionAtUT(ApAUT);
				var in_atm = Body.atmosphere && VesselOrbit.radius < Body.Radius+Body.atmosphereDepth;
				var h2ApA  = VesselOrbit.ApA-VSL.Altitude.Absolute;
				var dFi    = 90-Vector3d.Angle(VesselOrbit.GetOrbitNormal(), TargetApV);
				if(VesselOrbit.ApA/(LaunchApR-Body.Radius) > REN.RendezvouCorrection) correct_rendezvou();
				var hv     = Vector3d.Exclude(VesselOrbit.pos, VesselOrbit.vel);
				var alpha  = Utils.ProjectionAngle(cApV, TargetApV, Vector3d.Cross(VesselOrbit.GetOrbitNormal(), cApV))*Mathf.Deg2Rad*Body.Radius;
				if(alpha > REN.Dtol)
				{
					var hvel = alpha/Utils.ClampL(dApA, REN.GTurnCurve)*Utils.Clamp(h2ApA/100, 0, 1)*REN.GTurnCurve;
					if(Body.atmosphere) hvel *= Math.Sqrt(Utils.Clamp(VSL.Altitude.Absolute/Body.atmosphereDepth, 0, 1));
					vel += hv.normalized*hvel;
				}
				vel += VSL.Physics.Up.xzy*dApA/Utils.ClampL(VSL.VerticalSpeed.Absolute, 1);
				vel *= VSL.Physics.StG/Utils.G0;
				if(!in_atm || dApA > 0 || alpha > 0)
					vel += (QuaternionD.AngleAxis(dFi, VesselOrbit.pos) * hv) - hv;

//				CSV(VSL.Physics.UT, VesselOrbit.radius-Body.Radius, VSL.VerticalSpeed.Absolute, 
//				    Math.Atan2(dApA/Utils.ClampL(VSL.VerticalSpeed.Absolute, 1), hvel)*Mathf.Rad2Deg);//debug
				LogF("\nApV {}\ndApA {}, {}\nalpha {}, {}\ndFi {}, {}", cApV,
				     dApA, VSL.Physics.Up.xzy*dApA/Utils.ClampL(VSL.VerticalSpeed.Absolute, 1)*VSL.Physics.StG/9.81,
				     alpha, hv.normalized*alpha/Utils.ClampL(dApA, REN.GTurnCurve)*Utils.Clamp(h2ApA/100, 0, 1)*REN.GTurnCurve*VSL.Physics.StG/9.81,
				     dFi, (!in_atm || dApA > 0 || alpha > 0)? (QuaternionD.AngleAxis(dFi, VesselOrbit.pos) * hv) - hv : Vector3d.zero);//debug

				if(dApA < REN.Dtol && VesselOrbit.timeToAp < REN.StartOffset) ToOrbitDeltaV.Value = 0;
				else ToOrbitDeltaV.Value = (float)vel.magnitude;

				if(ToOrbitDeltaV.Value > 1)
				{
					VSL.ActivateNextStageOnFlameout();
					if(ToOrbitDeltaV && VSL.Controls.RCSAvailable) 
						CFG.AT.OnIfNot(Attitude.KillRotation);
					else 
					{
						CFG.AT.OnIfNot(Attitude.Custom);
						BRC.ForwardDirection = hV(VSL.Physics.UT).xzy;
	//					var velW = VSL.IsActiveVessel? vel.xzy : (vel+Body.GetFrameVel()).xzy;
	//					if(Body.inverseRotation) velW -= Body.getRFrmVel(VesselOrbit.pos.xzy + Body.position);
						ATC.SetCustomRotationW(VSL.Engines.MaxThrust, -vel.xzy);
						LogF("\nvel {}\nThrust {}\nvel*Thrust {}", -vel.xzy, VSL.Engines.MaxThrust, Vector3d.Dot(VSL.Engines.MaxThrust, -vel.normalized.xzy));//debug
					}
					if(VSL.Controls.RCSAvailable)
					{
						if(ToOrbitDeltaV || ATC.AttitudeError > GLB.ATCB.AttitudeErrorThreshold)
							TRA.AddDeltaV(-VSL.LocalDir(vel.xzy));
						if(ToOrbitDeltaV) THR.Throttle = 0;
						else THR.DeltaV = ToOrbitDeltaV;
					}
					else THR.DeltaV = ToOrbitDeltaV;
					break;
				}
				CFG.AT.OnIfNot(Attitude.KillRotation);
				THR.Throttle = 0;
				if(in_atm) break;
				LaunchUT = -1;
				start_orbit();
				break;
			case Stage.StartOrbit:
//				CSV(VSL.Physics.UT, VesselOrbit.radius-Body.Radius, VSL.VerticalSpeed.Absolute, 0);//debug
				if(CFG.AP1[Autopilot1.Maneuver]) break;
				CurrentDistance = -1;
				update_trajectory(false);
				if(trajectory.DistanceToTarget < REN.CorrectionStart ||
				   (trajectory.TimeToTarget+trajectory.TimeToStart)/VesselOrbit.period > REN.MaxTTR) 
					stage = Stage.Rendezvou;
				else compute_rendezvou_trajectory();
				break;
			case Stage.ComputeRendezvou:
				if(!trajectory_computed()) break;
				if(trajectory.ManeuverDeltaV.magnitude > GLB.THR.MinDeltaV*5 &&
				   trajectory.DistanceToTarget < REN.CorrectionStart &&
				   (CurrentDistance < 0 || trajectory.DistanceToTarget < CurrentDistance))
				{
					CorrectionTimer.Start();
					CurrentDistance = trajectory.DistanceToTarget;
					add_trajectory_node();
					CFG.AP1.On(Autopilot1.Maneuver);
					stage = Stage.Rendezvou;
				}
				else if(trajectory.DistanceToTarget < REN.CorrectionStart) match_orbits();
				else 
				{
					Status("red", "Failed to compute rendezvou trajectory.\nPlease, try again.");
					CFG.AP2.Off();
					return;
				}
				break;
			case Stage.Rendezvou:
				if(CFG.AP1[Autopilot1.Maneuver]) break;
				if(!CorrectionTimer.Check && CurrentDistance >= 0) break;
				CorrectionTimer.Reset();
				update_trajectory();
				if(CurrentDistance < REN.ApproachThreshold ||
				   CurrentDistance < REN.CorrectionStart/2 && 
				   trajectory.TimeToTarget < ManeuverAutopilot.TTB(VSL, (float)trajectory.BreakDeltaV.magnitude)+REN.StartOffset)
					match_orbits();
				else start_correction();
				break;
			case Stage.MatchOrbits:
				if(CFG.AP1[Autopilot1.Maneuver]) break;
				update_trajectory();
				var dist = (Target.orbit.pos-VesselOrbit.pos).magnitude;
				if(dist > REN.CorrectionStart) start_orbit();
				else if(dist > REN.CorrectionStart/2) start_correction();
				else if(dist > REN.Dtol) approach();
				else brake();
				break;
			case Stage.Approach:
				var dP = Target.orbit.pos-VesselOrbit.pos;
				var dPm = dP.magnitude;
				if(dPm - VSL.Geometry.R < REN.Dtol) 
				{ brake(); break; }
				if(ATC.AttitudeError > 1) break;
				var dV = Vector3d.Dot(VesselOrbit.vel-Target.orbit.vel, dP/dPm);
				var nV = Utils.Clamp(dPm*REN.ApproachVelF, 1, REN.MaxApproachV);
				if(dV+GLB.THR.MinDeltaV < nV) THR.DeltaV = (float)(nV-dV);
				else brake();
				break;
			case Stage.Brake:
				if(CFG.AP1[Autopilot1.MatchVelNear]) break;
				if(CFG.AP1[Autopilot1.MatchVel])
				{
					if((Target.orbit.vel-VesselOrbit.vel).magnitude > GLB.THR.MinDeltaV) break;
					CFG.AP1.Off();
				}
				if((VSL.Physics.wCoM-Target.CurrentCoM).magnitude-VSL.Geometry.R > REN.Dtol)
				{ approach(); break; }
				CFG.AP2.Off();
				CFG.AT.OnIfNot(Attitude.KillRotation);
				break;
			}
		}

		public override void Draw()
		{
			if(ControlsActive)
			{
				if(computing) 
				{
					GUILayout.Label("Computing...", Styles.inactive_button, GUILayout.ExpandWidth(false));
					GLUtils.GLVec(Body.position, current.AtTargetPos.xzy, Color.green);//debug
					GLUtils.GLVec(Body.position, current.TargetPos.xzy, Color.magenta);//debug
				}
				else if(Utils.ButtonSwitch("Rendezvou", CFG.AP2[Autopilot2.Rendezvou],
				                           "Compute and perform a rendezvou maneuver, then brake near the target.", 
				                           GUILayout.ExpandWidth(false)))
					CFG.AP2.XToggle(Autopilot2.Rendezvou);
			}
			else GUILayout.Label(new GUIContent("Rendezvou", "Compute and perform a rendezvou maneuver, then brake near the target."), 
			                     Styles.inactive_button, GUILayout.ExpandWidth(false));
		}
	}
}

