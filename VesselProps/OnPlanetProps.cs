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
		public bool HaveParachutes { get; private set; }
		public bool HaveUsableParachutes { get; private set; }
		public bool ParachutesActive { get; private set; }
		public bool ParachutesDeployed { get; private set; }
		public int  NearestParachuteStage { get; private set; }

		public Vector3 Fwd { get; private set; }  //fwd unit vector of the Control module in world space
		public Vector3 FwdL { get; private set; }  //fwd unit vector of the Control module in local space
		public bool    NoseUp { get; private set; }  //if the forward is refT.forward or refT.up
		public Vector3 Heading { get; private set; }  //bearing unit vector of the Control module in world space

		public float   MaxTWR { get; private set; }
		public float   MaxDTWR { get; private set; }
		public float   DTWR { get; private set; }
		public float   DTWRf { get { return DTWR_filter.Value; } }
		LowPassFilterF DTWR_filter = new LowPassFilterF();

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

		public override void Clear()
		{
			HaveLandingGear = false;
			GearDeployTime = 0;
			LandingGear.Clear();
			Parachutes.Clear();
		}

		static void update_aero_forces(Part p, ref Vector3 lift, ref Vector3 drag)
		{
			drag += p.dragVector;
			if(!p.hasLiftModule)
				lift += p.partTransform.InverseTransformDirection(p.bodyLiftLocalVector);
			var lift_srf = p.Modules.GetModules<ModuleLiftingSurface>();
			if(lift_srf.Count > 0)
			{
				var srf_lift = Vector3.zero;
				var srf_drag = Vector3.zero;
				lift_srf.ForEach(m => { srf_lift += m.liftForce; srf_drag += m.dragForce; });
				lift += srf_lift; drag += srf_drag;
			}
		}

		public Vector3 MainAeroForce()
		{
			var lift = Vector3.zero;
			var drag = Vector3.zero;
			VSL.vessel.Parts.ForEach(p => update_aero_forces(p, ref lift, ref drag));
			return lift.sqrMagnitude > drag.sqrMagnitude? lift : VSL.Geometry.MaxAreaDirection;
		}

		public override void Update()
		{
			CurrentThrustAccelerationTime = 0f; CurrentThrustDecelerationTime = 0f; TWRf = 1;
			//calculate total downward thrust and slow engines' corrections
			MaxTWR  = VSL.Engines.MaxThrustM/VSL.Physics.mg;
			DTWR = Vector3.Dot(VSL.Engines.Thrust, VSL.Physics.Up) < 0? 
				Vector3.Project(VSL.Engines.Thrust, VSL.Physics.Up).magnitude/VSL.Physics.mg : 0f;
			DTWR_filter.Tau = Mathf.Max(VSL.Engines.AccelerationTime, VSL.Engines.DecelerationTime);
			DTWR_filter.Update(DTWR);
			GeeVSF = 1/Utils.ClampL(MaxTWR, 1);
			var mVSFtor = (VSL.Torque.MaxPitchRoll.AA_rad > 0)? 
				Utils.ClampH(GLB.VSC.MinVSFf/VSL.Torque.MaxPitchRoll.AA_rad, GLB.VSC.MaxVSFtwr*GeeVSF) : 0;
			MinVSF = Mathf.Lerp(0, mVSFtor, Mathf.Pow(VSL.Controls.Steering.sqrMagnitude, 0.25f));
			var down_thrust = 0f;
			var slow_thrust = 0f;
			var fast_thrust = 0f;
			for(int i = 0; i < VSL.Engines.NumActive; i++)
			{
				var e = VSL.Engines.Active[i];
				e.VSF = 1;
				if(e.thrustInfo == null) continue;
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
				if(CurrentThrustAccelerationTime > 0) CurrentThrustAccelerationTime = rel_slow_thrust/CurrentThrustAccelerationTime*GLB.VSC.ASf;
				if(CurrentThrustDecelerationTime > 0) CurrentThrustDecelerationTime = rel_slow_thrust/CurrentThrustDecelerationTime*GLB.VSC.DSf;
				//TWR factor
				var vsf = CFG.VSCIsActive && VSL.VerticalSpeed.Absolute < 0? 
					Utils.Clamp(1-(Utils.ClampH(CFG.VerticalCutoff, 0)-VSL.VerticalSpeed.Absolute)/GLB.TDC.VSf, 1e-9f, 1) : 1;
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
			var P = vessel.staticPressurekPa*Parachute.Atm; //atm
			for(int i = 0, count = UnusedParachutes.Count; i < count; i++)
			{
				var p = UnusedParachutes[i];
				if(p.CanBeDeployed(P) && (cond == null || cond(p)))
					p.parachute.Deploy();
			}
		}

		public void ActivateParachutesAtDeplyomentAltitude()
		{
			if(!CFG.AutoParachutes) return;
			activate_parachutes(p => p.AtDeploymentAltitude);
		}

		public void ActivateParachutesBeforeUnsafe()
		{
			if(!CFG.AutoParachutes) return;
			activate_parachutes(p => p.SafetyState == Parachute.State.BeforeUnsafe || 
			                    p.AtDeploymentAltitude);
		}

		public void ActivateParachutesASAP()
		{
			if(!CFG.AutoParachutes) return;
			activate_parachutes();
		}

		public void CutActiveParachutes()
		{
			if(!CFG.AutoParachutes || ActiveParachutes.Count == 0) return;
			ActiveParachutes.ForEach(p => p.parachute.CutParachute());
			ActiveParachutes.Clear();
			ParachutesActive = false;
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

