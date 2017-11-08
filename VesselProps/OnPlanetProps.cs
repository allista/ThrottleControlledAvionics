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
using System.Linq;
using System.Collections.Generic;
using UnityEngine;
using ModuleWheels;
using AT_Utils;

namespace ThrottleControlledAvionics
{

	public class OnPlanetProps : VesselProps
	{
		public OnPlanetProps(VesselWrapper vsl) : base(vsl) {}

		public List<Parachute> Parachutes = new List<Parachute>();
		public List<Parachute> UnusedParachutes = new List<Parachute>();
		public List<Parachute> ActiveParachutes = new List<Parachute>();
		public List<ModuleWheelDeployment> LandingGear = new List<ModuleWheelDeployment>();
		public List<LaunchClamp> LaunchClamps = new List<LaunchClamp>();
		public bool HaveLaunchClamps { get; private set; }
		public bool HaveParachutes { get; private set; }
		public bool HaveUsableParachutes { get; private set; }
		public bool ParachutesActive { get; private set; }
		public bool ParachutesDeployed { get; private set; }
		public int  NearestParachuteStage { get; private set; }

		public Vector3 Fwd { get; private set; }  //fwd unit vector of the Control module in world space
		public Vector3 FwdL { get; private set; }  //fwd unit vector of the Control module in local space
		public bool    NoseUp { get; private set; }  //if the forward is refT.forward or refT.up
		public Vector3 Heading { get; private set; }  //bearing unit vector of the Control module in world space

        public Vector3 Lift { get; private set; } //current lift vector
        public Vector3 Drag { get; private set; } //current drag vector
        public float  vLift { get; private set; } //current vertical lift kN
        public Vector3 AeroForce { get; private set; } //current lift+drag vector
        public Vector3 AeroTorque { get; private set; } //current torque produced by aero forcess
        public Vector3 MaxAeroForceL { get; private set; } //local statistically maximum aero force

		public float   MaxTWR { get; private set; }
		public float   MaxDTWR { get; private set; }
		public float   DTWR { get; private set; }
		public float   DTWRf { get { return DTWR_filter.Value; } }
		AsymmetricFiterF DTWR_filter = new AsymmetricFiterF();

		public float   CurrentThrustAccelerationTime { get; private set; }
		public float   CurrentThrustDecelerationTime { get; private set; }

		public float   VSF; //vertical speed factor
		public float   GeeVSF; //the value of VSF that provides the thrust equal to the gravity force
		public float   MinVSF; //minimum allowable VSF to have enough control authority

		public float   TWRf { get; private set; } //all VSC engines should be pointed down when this is low: [1e-9, 1]

		public bool    HaveLandingGear { get; private set; }
		public float   GearDeployTime { get; private set; }

		public bool    GearDeploying
		{ get { return LandingGear.Count > 0 && LandingGear.All(g => g.fsm.CurrentState == g.st_deploying); } }

		public bool    GearDeployed
		{ get { return LandingGear.Count == 0 || LandingGear.All(g => g.fsm.CurrentState == g.st_deployed); } }

		public bool    GearRetracting
		{ get { return LandingGear.Count > 0 && LandingGear.All(g => g.fsm.CurrentState == g.st_retracting); } }

		public bool    GearRetracted
		{ get { return LandingGear.Count == 0 || LandingGear.All(g => g.fsm.CurrentState == g.st_retracted); } }

		public bool AddGear(PartModule pm)
		{
			var wheel = pm as ModuleWheelDeployment;
			if(wheel != null)
			{
				var animator = wheel.part.FindModelAnimator(wheel.animationTrfName, wheel.animationStateName);
				if(animator != null)
				{
					LandingGear.Add(wheel);
					GearDeployTime = Mathf.Max(GearDeployTime, animator.clip.length);
					HaveLandingGear = true;
				}
				return true;
			}
			return false;
		}

		public bool AddParachute(PartModule pm)
		{
			var parachute = pm as ModuleParachute;
			if(parachute != null) 
			{
				Parachutes.Add(new Parachute(VSL, parachute));
				return true;
			}
			return false;
		}

		public bool AddLaunchClamp(PartModule pm)
		{
			var clamp = pm as LaunchClamp;
			if(clamp != null)
			{
				LaunchClamps.Add(clamp);
				HaveLaunchClamps = true;
				return true;
			}
			return false;
		}

		public override void Clear()
		{
			GearDeployTime = 0;
            MaxAeroForceL = Vector3.zero;
			HaveLandingGear = false;
			HaveLaunchClamps = false;
			LaunchClamps.Clear();
			LandingGear.Clear();
			Parachutes.Clear();
		}

        void update_aero_forces()
        {
            var lift = Vector3.zero;
            var drag = Vector3.zero;
            var torque = Vector3.zero;
            if(VSL.vessel.dynamicPressurekPa > 0)
            {
                for(int i = 0, VSLvesselPartsCount = VSL.vessel.Parts.Count; i < VSLvesselPartsCount; i++)
                {
                    var p = VSL.vessel.Parts[i];
                    var r = p.Rigidbody.worldCenterOfMass-VSL.Physics.wCoM;
                    var d = -p.dragVectorDir * p.dragScalar;
                    torque += Vector3.Cross(r, d);
                    drag += d;
                    if(!p.hasLiftModule)
                    {
                        var lift_mod = p.transform.rotation * (p.bodyLiftScalar * p.DragCubes.LiftForce);
                        torque += Vector3.Cross(r, lift_mod);
                        lift += lift_mod;
                    }
                    var lift_srf = p.Modules.GetModules<ModuleLiftingSurface>();
                    if(lift_srf.Count > 0)
                    {
                        var srf_lift = Vector3.zero;
                        var srf_drag = Vector3.zero;
                        lift_srf.ForEach(m => { srf_lift += m.liftForce; srf_drag += m.dragForce; });
                        torque += Vector3.Cross(r, srf_lift+srf_drag);
                        lift += srf_lift; 
                        drag += srf_drag;
                    }
                }
            }
            Lift = lift;
            Drag = drag;
            AeroTorque = torque;
        }

		public override void Update()
		{
            TWRf = 1;
			CurrentThrustAccelerationTime = 0f; 
            CurrentThrustDecelerationTime = 0f;
            update_aero_forces();
			//calculate total downward thrust and slow engines' corrections
            AeroForce = Lift+Drag;
            var relAeroForce = AeroForce/(float)Utils.ClampL(VSL.vessel.dynamicPressurekPa, 1);
            if(relAeroForce.sqrMagnitude > MaxAeroForceL.sqrMagnitude)
                MaxAeroForceL = VSL.LocalDir(relAeroForce);
            vLift = Vector3.Dot(AeroForce, VSL.Physics.Up);
            MaxTWR = (VSL.Engines.MaxThrustM+Mathf.Max(vLift, 0))/VSL.Physics.mg;
            DTWR = Mathf.Max((vLift-Vector3.Dot(VSL.Engines.Thrust, VSL.Physics.Up))/VSL.Physics.mg, 0);
			DTWR_filter.TauUp = VSL.Engines.AccelerationTime90;
			DTWR_filter.TauDown = VSL.Engines.DecelerationTime10;
			DTWR_filter.Update(DTWR);
			GeeVSF = 1/Utils.ClampL(MaxTWR, 1);
			var mVSFtor = (VSL.Torque.MaxPitchRoll.AA_rad > 0)? 
				Utils.ClampH(GLB.VSC.MinVSFf/VSL.Torque.MaxPitchRoll.AA_rad, GLB.VSC.MaxVSFtwr*GeeVSF) : 0;
			MinVSF = Mathf.Lerp(0, mVSFtor, Mathf.Pow(VSL.Controls.Steering.sqrMagnitude, 0.25f));
            var down_thrust = Mathf.Max(vLift, 0);
			var slow_thrust = 0f;
			var fast_thrust = 0f;
			for(int i = 0; i < VSL.Engines.NumActive; i++)
			{
				var e = VSL.Engines.Active[i];
				e.VSF = 1;
				if(e.isVSC)
				{
					var dcomponent = -Vector3.Dot(e.wThrustDir, VSL.Physics.Up);
					if(dcomponent <= 0) e.VSF = VSL.HasUserInput? 0 : GeeVSF*VSL.Controls.InvAlignmentFactor;
					else 
					{
						var dthrust = e.nominalCurrentThrust(e.limit)*dcomponent;
						if(e.useEngineResponseTime && dthrust > 0) 
						{
							slow_thrust += dthrust;
							CurrentThrustAccelerationTime += e.engineAccelerationSpeed*dthrust;
							CurrentThrustDecelerationTime += e.engineDecelerationSpeed*dthrust;
						}
						else fast_thrust = dthrust;
						down_thrust += dthrust;
					}
				}
			}
			MaxDTWR = Utils.EWA(MaxDTWR, down_thrust/VSL.Physics.mg, 0.1f);
			if(refT != null)
			{
				Fwd = Vector3.Cross(VSL.refT.right, -VSL.Engines.MaxThrust).normalized;
				FwdL = refT.InverseTransformDirection(Fwd);
				NoseUp = Mathf.Abs(Vector3.Dot(Fwd, VSL.refT.forward)) >= 0.9;
				Heading = Vector3.ProjectOnPlane(Fwd, VSL.Physics.Up).normalized;
			}
			var controllable_thrust = slow_thrust+fast_thrust;
			if(controllable_thrust > 0)
			{
				//correct setpoint for current TWR and slow engines
				var rel_slow_thrust = slow_thrust*slow_thrust/controllable_thrust;
				if(CurrentThrustAccelerationTime > 0) 
                    CurrentThrustAccelerationTime = rel_slow_thrust/CurrentThrustAccelerationTime*GLB.VSC.ASf;
				if(CurrentThrustDecelerationTime > 0) 
                    CurrentThrustDecelerationTime = rel_slow_thrust/CurrentThrustDecelerationTime*GLB.VSC.DSf;
				//TWR factor
				var vsf = 1f;
                if(VSL.VerticalSpeed.Absolute < 0)
                    vsf = Utils.Clamp(1 -(Utils.ClampH(CFG.VerticalCutoff, 0)-VSL.VerticalSpeed.Absolute)/GLB.TDC.VSf, 1e-9f, 1);
				var twr = VSL.Engines.Slow? DTWR_filter.Value : MaxTWR*Utils.Sin45; //MaxTWR at 45deg
				TWRf = Utils.Clamp(twr/GLB.TDC.TWRf, 1e-9f, 1)*vsf;
			}
			//parachutes
			UnusedParachutes.Clear();
			ActiveParachutes.Clear();
			ParachutesActive = false;
			ParachutesDeployed = false;
			NearestParachuteStage = 0;
			for(int i = 0, count = Parachutes.Count; i < count; i++)
			{
				var p = Parachutes[i];
				if(!p.Valid) continue;
				p.Update();
				if(p.Active) ActiveParachutes.Add(p);
				if(p.Usable) UnusedParachutes.Add(p);
				if(p.Stage > NearestParachuteStage) 
					NearestParachuteStage = p.Stage;
				ParachutesDeployed |= p.Deployed;
			}
			ParachutesActive = ActiveParachutes.Count > 0;
			HaveUsableParachutes = UnusedParachutes.Count > 0;
			HaveParachutes = HaveUsableParachutes || ParachutesActive;
		}

		public delegate bool ParachuteCondition(Parachute p);
		void activate_parachutes(ParachuteCondition cond = null)
		{
			for(int i = 0, count = UnusedParachutes.Count; i < count; i++)
			{
				var p = UnusedParachutes[i];
				if(cond == null || cond(p))
					p.parachute.Deploy();
			}
		}

		public void ActivateParachutesAtDeplyomentAltitude()
		{
			if(CFG.AutoParachutes)
				activate_parachutes(p => p.AtDeploymentAltitude);
		}

		public void ActivateParachutesBeforeUnsafe()
		{
			if(CFG.AutoParachutes)
				activate_parachutes(p => p.SafetyState == Parachute.State.BeforeUnsafe || 
				                    p.AtDeploymentAltitude);
		}

		public void ActivateParachutesASAP()
		{
            var P = vessel.staticPressurekPa*Parachute.Atm; //atm
			if(CFG.AutoParachutes) 
                activate_parachutes(p => p.CanBeDeployed(P));
		}

		public void CutActiveParachutes()
		{
			if(CFG.AutoParachutes && ActiveParachutes.Count > 0)
			{
				ActiveParachutes.ForEach(p => p.parachute.CutParachute());
				ActiveParachutes.Clear();
				ParachutesActive = false;
			}
		}

		public void ActivateLaunchClamps()
		{
			if(HaveLaunchClamps)
				LaunchClamps.ForEach(cl => cl.Release());
		}

		public class Parachute : VesselProps
		{
			public const double Atm = 0.0098692326671601278; //kPa => atmposhere: 1/1013.25kPa

			public enum State { None, Unsafe, AfterUnsafe, Safe, BeforeUnsafe };
			public State SafetyState { get; private set; }

			public readonly ModuleParachute parachute;

			public Parachute(VesselWrapper VSL, ModuleParachute p) : base(VSL)
			{ parachute = p; }

			public int Stage { get { return parachute.part.inverseStage; } }

			public bool Valid { get { return parachute != null && parachute.part != null && parachute.vessel != null; } }

			public bool Usable
			{
				get
				{
					return !parachute.part.ShieldedFromAirstream && 
						parachute.deploymentState == ModuleParachute.deploymentStates.STOWED;
				}
			}

			public bool Deployed
			{
				get
				{
					return parachute.deploymentState == ModuleParachute.deploymentStates.DEPLOYED && 
						parachute.part.maximum_drag/parachute.fullyDeployedDrag > 0.9;
				}
			}

			public bool Active
			{
				get
				{
					return parachute.deploymentState == ModuleParachute.deploymentStates.ACTIVE ||
						parachute.deploymentState == ModuleParachute.deploymentStates.SEMIDEPLOYED ||
						parachute.deploymentState == ModuleParachute.deploymentStates.DEPLOYED;
				}
			}

			public bool CanBeDeployed(double pressure)
			{
				return parachute.deploymentSafeState == ModuleParachute.deploymentSafeStates.SAFE &&
					pressure >= parachute.minAirPressureToOpen;
			}

			public bool AtDeploymentAltitude
			{
				get
				{
					return VSL.Altitude.Relative < parachute.deployAltitude
						+Mathf.Abs(VSL.VerticalSpeed.Absolute)*(1/Utils.ClampL(parachute.semiDeploymentSpeed, 0.1f)+10);
				}
			}

			public override void Update()
			{
				switch(parachute.deploymentSafeState)
				{
				case ModuleParachute.deploymentSafeStates.SAFE:
					if(SafetyState == State.Unsafe)
						SafetyState = State.AfterUnsafe;
					else if(SafetyState == State.AfterUnsafe)
					{
						if(parachute.shockTemp < parachute.chuteMaxTemp * parachute.safeMult * 0.9f)
							SafetyState = State.Safe;
					}
					else if(SafetyState == State.Safe)
					{
						if(parachute.shockTemp > parachute.chuteMaxTemp * parachute.safeMult * 0.9f)
							SafetyState = State.BeforeUnsafe;
					}
					else SafetyState = State.Safe;
					break;
				case ModuleParachute.deploymentSafeStates.RISKY:
				case ModuleParachute.deploymentSafeStates.UNSAFE:
					SafetyState = State.Unsafe;
					break;
				}
			}

			public override string ToString()
			{
				if(!Valid) return Utils.Format("Invalid Parachute: {}", parachute);
				return Utils.Format("{}[{}], Stage {}, Usable {}, Deployed {}, Active {}, AtDeploymentAltitude {}", 
				                    parachute, SafetyState, Stage, Usable, Deployed, Active, AtDeploymentAltitude);
			}
		}
	}
}

