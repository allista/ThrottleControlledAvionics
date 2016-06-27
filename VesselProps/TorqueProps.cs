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

namespace ThrottleControlledAvionics
{
	public class TorqueProps : VesselProps
	{
		public TorqueProps(VesselWrapper vsl) : base(vsl) 
		{ MaxAAFilter.Tau = GLB.MaxAAFilter; }

		readonly LowPassFilterF MaxAAFilter = new LowPassFilterF();

		public List<ModuleReactionWheel> Wheels = new List<ModuleReactionWheel>();

		public Vector6  EnginesLimits { get; private set; } = Vector6.zero; //torque limits of engines
		public Vector6  WheelsLimits { get; private set; } = Vector6.zero; //torque limits of reaction wheels
		public Vector6  RCSLimits { get; private set; } = Vector6.zero; //torque limits of rcs

		public Vector3  EnginesTorque { get; private set; } //current torque applied to the vessel by engines
		public Vector3  NoEnginesTorque { get; private set; } //current maximum torque
		public Vector3  MaxTorque { get; private set; } //current maximum torque
		public Vector3  MaxPossibleTorque { get; private set; } //theoretical maximum torque from active engines, wheels and RCS

		public Vector3  MaxAA { get; private set; } //current maximum angular acceleration
		public Vector3  NoEnginesAA { get; private set; } //current maximum angular acceleration
		public Vector3  MaxPossibleAA { get; private set; }
		public Vector3  MaxPitchRollAA { get; private set; }

		public float    MaxAA_rad { get; private set; }
		public float    MaxPitchRollAA_rad { get; private set; }
		public float    MaxPossibleAA_rad { get; private set; }
		public float    NoEnginesAA_rad { get; private set; }

		public float    MinTurnTime { get; private set; }
		public float    NoEnginesTurnTime { get; private set; }

		public float    MaxAngularDragResistance { get { return AngularDragResistance(MaxTorque); } }
		public float    MaxPossibleAngularDragResistance { get { return AngularDragResistance(MaxPossibleTorque); } }
		public float    NoEnginesAngularDragResistance { get { return AngularDragResistance(NoEnginesTorque); } }

		public float    MaxAAMod { get; private set; }

		public Vector3 AngularAcceleration(Vector3 torque)
		{
			return new Vector3
				(VSL.Physics.MoI.x.Equals(0)? float.MaxValue : torque.x/VSL.Physics.MoI.x,
				 VSL.Physics.MoI.y.Equals(0)? float.MaxValue : torque.y/VSL.Physics.MoI.y,
				 VSL.Physics.MoI.z.Equals(0)? float.MaxValue : torque.z/VSL.Physics.MoI.z);
		}

		public float AngularAccelerationInDirection(Vector3 dir)
		{ return Mathf.Abs(Vector3.Dot(VSL.Torque.MaxAA, dir)); }

		//rotation with zero start and end angular velocities and constant angular acceleration
		public float MinRotationTime(float angle, float AA_rad)
		{ return 2*Mathf.Sqrt(angle/AA_rad/Mathf.Rad2Deg); }

		public float MinRotationTime(float angle)
		{ return MinRotationTime(angle, MaxAA_rad); }

		public float MaxPossibleTorqueRotationTime(float angle)
		{ return MinRotationTime(angle, MaxPossibleAA_rad); }

		public float AngularDragResistance(Vector3 torque) 
		{ 
			var s = VSL.Geometry.BoundsSideAreas;
			return 
				torque.x/(s.y+s.z) +
				torque.y/(s.x+s.z) +
				torque.z/(s.y+s.x);
		}

		public bool HavePotentialControlAuthority
		{ 
			get 
			{ 
				var max_res = MaxPossibleAngularDragResistance;
				return max_res > 1.1*MaxAngularDragResistance &&
					max_res > VSL.Body.atmDensityASL*GLB.ATCB.DragResistanceF; 
			} 
		}

		public override void Update()
		{
			//engines
			EnginesLimits = Vector6.zero;
			var MaxEnginesLimits = Vector6.zero;
			for(int i = 0, count = VSL.Engines.Steering.Count; i < count; i++)
			{
				var e = VSL.Engines.Steering[i];
				EnginesLimits.Add(e.currentTorque);
				MaxEnginesLimits.Add(e.specificTorque*e.nominalCurrentThrust(1));
			}
			//wheels
			WheelsLimits = Vector6.zero;
			for(int i = 0, count = Wheels.Count; i < count; i++)
			{
				var w = Wheels[i];
				if(!w.operational) continue;
				var torque = new Vector3(w.PitchTorque, w.RollTorque, w.YawTorque);
				WheelsLimits.Add(torque);
				WheelsLimits.Add(-torque);
			}
			//RCS
			RCSLimits = Vector6.zero;
			for(int i = 0; i < VSL.Engines.NumActiveRCS; i++)
			{
				var r = VSL.Engines.ActiveRCS[i];
				for(int j = 0, tcount = r.rcs.thrusterTransforms.Count; j < tcount; j++)
				{
					var t = r.rcs.thrusterTransforms[j];
					if(t == null) continue;
					RCSLimits.Add(refT.InverseTransformDirection(Vector3.Cross(t.position-VSL.Physics.wCoM, t.up)*r.rcs.thrusterPower));
				}
			}
			//torque and angular acceleration
			NoEnginesTorque = RCSLimits.Max+WheelsLimits.Max;
			MaxTorque = NoEnginesTorque+EnginesLimits.Max;
			MaxPossibleTorque = NoEnginesTorque+MaxEnginesLimits.Max;

			MaxAA = AngularAcceleration(MaxTorque);
			MaxAA_rad = MaxAA.magnitude;
			NoEnginesAA = AngularAcceleration(NoEnginesTorque);
			NoEnginesAA_rad = NoEnginesAA.magnitude;
			MaxPossibleAA = AngularAcceleration(MaxPossibleTorque);
			MaxPossibleAA_rad = MaxPossibleAA.magnitude;
			MaxPitchRollAA = Vector3.ProjectOnPlane(MaxAA, refT.InverseTransformDirection(VSL.Engines.Thrust)).AbsComponents();
			MaxPitchRollAA_rad = MaxPitchRollAA.magnitude;

			MinTurnTime = MinRotationTime(180, MaxAA_rad);
			NoEnginesTurnTime = MinRotationTime(180, NoEnginesAA_rad);

			if(MaxAA_rad > 0)
			{
				MaxAAMod = MaxAAFilter.Update(MaxAA_rad)/MaxAA_rad;
				MaxAAMod *= MaxAAMod*MaxAAMod;
			}
			else MaxAAMod = 1;
		}

		public void UpdateTorque(params IList<EngineWrapper>[] engines)
		{
			EnginesTorque = Vector3.zero;
			for(int i = 0; i < engines.Length; i++)
			{
				for(int j = 0; j < engines[i].Count; j++)
				{
					var e = engines[i][j];
					EnginesTorque += e.Torque(e.throttle * e.limit);
				}
			}
			EnginesTorque = EnginesLimits.Clamp(EnginesTorque);
		}
	}
}

