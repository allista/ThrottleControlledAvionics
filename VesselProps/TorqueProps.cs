//  Author:
//       Allis Tauri <allista@gmail.com>
//
//  Copyright (c) 2016 Allis Tauri
//
// This work is licensed under the Creative Commons Attribution-ShareAlike 4.0 International License. 
// To view a copy of this license, visit http://creativecommons.org/licenses/by-sa/4.0/ 
// or send a letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.
//
using System.Collections.Generic;
using UnityEngine;
using AT_Utils;

namespace ThrottleControlledAvionics
{
	public class TorqueProps : VesselProps
	{
		readonly LowPassFilterF MaxAAFilter = new LowPassFilterF();

		public List<ModuleReactionWheel> Wheels = new List<ModuleReactionWheel>();

		public Vector6  EnginesLimits { get; private set; } = Vector6.zero; //torque limits of engines
		public Vector6  WheelsLimits { get; private set; } = Vector6.zero; //torque limits of reaction wheels
		public Vector6  RCSLimits { get; private set; } = Vector6.zero; //torque limits of rcs

		public TorqueInfo Imbalance; //current torque imbalance, set by UpdateImbalance
		public TorqueInfo Engines; //current torque applied to the vessel by engines
		public TorqueInfo NoEngines; //current maximum torque
        public TorqueInfo SlowMaxPossible; //current maximum torque
        public TorqueInfo Instant; //current maximum torque
		public TorqueInfo MaxCurrent; //current maximum torque
		public TorqueInfo MaxPitchRoll; //current maximum torque
		public TorqueInfo MaxPossible; //theoretical maximum torque from active engines, wheels and RCS
		public TorqueInfo MaxEngines; //theoretical maximum torque from active engines

        public bool     Gimball { get; private set; } //if there are stirring engines that have gimball
        public bool     Slow { get; private set; } //if there are MAIN or MANEUVER engines with non-zero acceleration/deceleration speed
		public Vector3  EnginesResponseTime { get; private set; }
		public float    EnginesResponseTimeM { get; private set; }

		public float MaxAAMod { get; private set; }

		public TorqueProps(VesselWrapper vsl) : base(vsl) 
		{ MaxAAFilter.Tau = GLB.MaxAAFilter; }

		public static Vector3 AngularAcceleration(Vector3 torque, Vector3 MoI)
		{
			return new Vector3
				(MoI.x.Equals(0)? float.MaxValue : torque.x/MoI.x,
				 MoI.y.Equals(0)? float.MaxValue : torque.y/MoI.y,
				 MoI.z.Equals(0)? float.MaxValue : torque.z/MoI.z);
		}

		public Vector3 AngularAcceleration(Vector3 torque)
		{ return AngularAcceleration(torque, VSL.Physics.MoI); }

		public bool HavePotentialControlAuthority
		{ 
			get 
			{ 
				var max_res = MaxPossible.AngularDragResistance;
				return max_res > 1.1*MaxCurrent.AngularDragResistance &&
					max_res > VSL.Body.atmDensityASL*GLB.ATCB.DragResistanceF; 
			} 
		}

		public override void Update()
		{
			//engines
            Slow = false;
			EnginesResponseTime = Vector3.zero;
			EnginesResponseTimeM = 0f;
			EnginesLimits = Vector6.zero;
			var MaxEnginesLimits = Vector6.zero;
			var EnginesSpecificTorque = Vector6.zero;
			var TorqueResponseSpeed = Vector6.zero;
			var TotalSlowTorque = Vector6.zero;
            var TotalSlowSpecificTorque = Vector6.zero;
			for(int i = 0, count = VSL.Engines.Active.Steering.Count; i < count; i++)
			{
				var e = VSL.Engines.Active.Steering[i];
                var max_torque = e.defSpecificTorque*e.nominalFullThrust;
                EnginesLimits.Add(e.defCurrentTorque);
                EnginesSpecificTorque.Add(e.defSpecificTorque);
                MaxEnginesLimits.Add(max_torque);
				if(e.useEngineResponseTime)
				{
                    TotalSlowTorque.Add(max_torque);
                    TotalSlowSpecificTorque.Add(e.defSpecificTorque);
                    TorqueResponseSpeed.Add(max_torque*Mathf.Max(e.engineAccelerationSpeed, e.engineDecelerationSpeed));
				}
                Gimball |= e.gimbal != null && e.gimbal.gimbalActive && !e.gimbal.gimbalLock;
			}
			//wheels
			WheelsLimits = Vector6.zero;
			for(int i = 0, count = Wheels.Count; i < count; i++)
			{
				var w = Wheels[i];
				if(w.State != ModuleReactionWheel.WheelState.Active) continue;
                var torque = new Vector3(w.PitchTorque, w.RollTorque, w.YawTorque)*w.authorityLimiter/100;
				WheelsLimits.Add(torque);
				WheelsLimits.Add(-torque);
			}
			//RCS
			RCSLimits = Vector6.zero;
			var RCSSpecificTorque = Vector6.zero;
			for(int i = 0; i < VSL.Engines.NumActiveRCS; i++)
			{
				var r = VSL.Engines.ActiveRCS[i];
				for(int j = 0, tcount = r.rcs.thrusterTransforms.Count; j < tcount; j++)
				{
					var t = r.rcs.thrusterTransforms[j];
					if(t == null) continue;
					var specificTorque = refT.InverseTransformDirection(Vector3.Cross(t.position-VSL.Physics.wCoM, t.up));
					RCSLimits.Add(specificTorque*r.rcs.thrusterPower);
					RCSSpecificTorque.Add(specificTorque);
				}
			}
			//torque and angular acceleration
			Engines.Update(EnginesLimits.Max);
			NoEngines.Update(RCSLimits.Max+WheelsLimits.Max);
			MaxEngines.Update(MaxEnginesLimits.Max);
			MaxCurrent.Update(NoEngines.Torque+Engines.Torque);
			MaxPossible.Update(NoEngines.Torque+MaxEngines.Torque);
            MaxPitchRoll.Update(Vector3.ProjectOnPlane(MaxCurrent.Torque, VSL.Engines.CurrentThrustDir).AbsComponents());
            SlowMaxPossible.Update(TotalSlowTorque.Max);
            Instant.Update(MaxPossible.Torque-SlowMaxPossible.Torque);
			//specifc torque
			Engines.SpecificTorque = EnginesSpecificTorque.Max;
			NoEngines.SpecificTorque = RCSSpecificTorque.Max;
			MaxEngines.SpecificTorque = Engines.SpecificTorque;
			MaxCurrent.SpecificTorque = Engines.SpecificTorque+NoEngines.SpecificTorque;
			MaxPossible.SpecificTorque = MaxCurrent.SpecificTorque;
            MaxPitchRoll.SpecificTorque = Vector3.ProjectOnPlane(MaxCurrent.SpecificTorque, VSL.Engines.CurrentThrustDir).AbsComponents();
            SlowMaxPossible.SpecificTorque = TotalSlowSpecificTorque.Max;
            Instant.SpecificTorque = MaxPossible.SpecificTorque-SlowMaxPossible.SpecificTorque;
			//torque response time
			if(!TorqueResponseSpeed.IsZero()) 
			{ 
                EnginesResponseTime = Vector3.Scale(SlowMaxPossible.Torque, TorqueResponseSpeed.Max.Inverse(0));
                EnginesResponseTime = EnginesResponseTime.ScaleChain(SlowMaxPossible.Torque, MaxPossible.Torque.Inverse(0));
				EnginesResponseTimeM = EnginesResponseTime.MaxComponentF();
                Slow = true;
			}
			//Max AA filter
			if(MaxCurrent.AA_rad > 0)
			{
				MaxAAMod = MaxAAFilter.Update(MaxCurrent.AA_rad)/MaxCurrent.AA_rad;
				MaxAAMod *= MaxAAMod*MaxAAMod;
			}
			else MaxAAMod = 1;
		}

        public void UpdateImbalance(bool useDefTorque, params IList<EngineWrapper>[] engines)
		{
            var torque = CalculateImbalance(useDefTorque, engines);
//            if(VSL.OnPlanet) torque += VSL.LocalDir(VSL.OnPlanetParams.AeroTorque);
			Imbalance.Update(EnginesLimits.Clamp(torque));
		}

		public static Vector3 CalculateImbalance(bool useDefTorque, params IList<EngineWrapper>[] engines)
		{
			var torque = Vector3.zero;
			for(int i = 0; i < engines.Length; i++)
			{
				for(int j = 0; j < engines[i].Count; j++)
				{
					var e = engines[i][j];
                    torque += e.Torque(e.throttle * e.limit, useDefTorque);
				}
			}
			return torque;
		}
	}


	public class TorqueInfo : VesselProps
	{
		public TorqueInfo(VesselWrapper vsl) : base(vsl) {}

		public Vector3 Torque;
		public Vector3 AA { get; private set; } //angular acceleration vector in radians
		public float   AA_rad { get; private set; } = 0f ;//angular acceleration in radians
		public float   TurnTime { get; private set; }
		public Vector3 SpecificTorque; //change in torque per thrust

		public float   AngularDragResistance 
		{ 
			get 
			{ 
				var s = VSL.Geometry.BoundsSideAreas;
				return 
					Torque.x/(s.y+s.z) +
					Torque.y/(s.x+s.z) +
					Torque.z/(s.y+s.x);
			} 
		}

		public float AngularDragResistanceAroundAxis(Vector3 axis)
		{ 
			axis = axis.AbsComponents().normalized;
			var s = VSL.Geometry.BoundsSideAreas;
			return 
				Torque.x*axis.x/(s.y+s.z) +
				Torque.y*axis.y/(s.x+s.z) +
				Torque.z*axis.z/(s.y+s.x);
		}


		public float AngularAccelerationAroundAxis(Vector3 axis)
		{ return Mathf.Abs(Vector3.Dot(AA, axis.AbsComponents().normalized)); }

		public float MinStopTime()
		{
			if(VSL.Physics.NoRotation) return 0;
			var aa = AngularAccelerationAroundAxis(VSL.vessel.angularVelocity);
			return aa.Equals(0) ? float.MaxValue : VSL.vessel.angularVelocity.magnitude / aa;
		}

        public float RotationTime2Phase(float angle, float aa, float throttle)
        { return 2*Mathf.Sqrt(angle/aa/throttle/Mathf.Rad2Deg); }

		public float RotationTime2Phase(float angle, float throttle = 1)
        { return RotationTime2Phase(angle, AA_rad, throttle); }

        public float RotationTime2Phase(float angle, Vector3 axis, float throttle = 1)
        { return RotationTime2Phase(angle, AngularAccelerationAroundAxis(axis), throttle); }

        public float RotationTime3Phase(float angle, float aa, float accel_part, float throttle)
        {
            var ak2 = 2*accel_part*angle;
            return (angle+ak2)/Mathf.Sqrt(ak2*aa*throttle*Mathf.Rad2Deg);
        }

        public float RotationTime3Phase(float angle, float accel_part, float throttle = 1)
        { return RotationTime3Phase(angle, AA_rad, accel_part, throttle); }

        public float RotationTime3Phase(float angle, Vector3 axis, float accel_part, float throttle = 1)
        { return RotationTime3Phase(angle, AngularAccelerationAroundAxis(axis), accel_part, throttle); }


		public void Update(Vector3 torque)
		{
			Torque = torque;
			AA = VSL.Torque.AngularAcceleration(Torque);
			AA_rad = AA.magnitude;
            TurnTime = RotationTime2Phase(180);
		}

		public static implicit operator bool(TorqueInfo info) { return info.AA_rad > 0; }
	}
}

