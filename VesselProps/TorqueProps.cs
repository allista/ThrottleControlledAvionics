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

		public TorqueInfo Engines; //current torque applied to the vessel by engines
		public TorqueInfo NoEngines; //current maximum torque
		public TorqueInfo MaxCurrent; //current maximum torque
		public TorqueInfo MaxPitchRoll; //current maximum torque
		public TorqueInfo MaxPossible; //theoretical maximum torque from active engines, wheels and RCS

		public float MaxAAMod { get; private set; }

		public TorqueProps(VesselWrapper vsl) : base(vsl) 
		{ MaxAAFilter.Tau = GLB.MaxAAFilter; }

		public Vector3 AngularAcceleration(Vector3 torque)
		{
			return new Vector3
				(VSL.Physics.MoI.x.Equals(0)? float.MaxValue : torque.x/VSL.Physics.MoI.x,
				 VSL.Physics.MoI.y.Equals(0)? float.MaxValue : torque.y/VSL.Physics.MoI.y,
				 VSL.Physics.MoI.z.Equals(0)? float.MaxValue : torque.z/VSL.Physics.MoI.z);
		}

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
			NoEngines.Update(RCSLimits.Max+WheelsLimits.Max);
			MaxCurrent.Update(NoEngines.Torque+EnginesLimits.Max);
			MaxPossible.Update(NoEngines.Torque+MaxEnginesLimits.Max);
			MaxPitchRoll.Update(Vector3.ProjectOnPlane(MaxCurrent.Torque, VSL.Engines.CurrentThrustDir));

			if(MaxCurrent.AA_rad > 0)
			{
				MaxAAMod = MaxAAFilter.Update(MaxCurrent.AA_rad)/MaxCurrent.AA_rad;
				MaxAAMod *= MaxAAMod*MaxAAMod;
			}
			else MaxAAMod = 1;
		}

		public void UpdateTorque(params IList<EngineWrapper>[] engines)
		{
			var torque = Vector3.zero;
			for(int i = 0; i < engines.Length; i++)
			{
				for(int j = 0; j < engines[i].Count; j++)
				{
					var e = engines[i][j];
					torque += e.Torque(e.throttle * e.limit);
				}
			}
			Engines.Update(EnginesLimits.Clamp(torque));
		}
	}


	public class TorqueInfo : VesselProps
	{
		public TorqueInfo(VesselWrapper vsl) : base(vsl) {}

		public Vector3 Torque;
		public Vector3 AA { get; private set; } //angular acceleration vector in radians
		public float   AA_rad { get; private set; } = 0f ;//angular acceleration in radians
		public float   TurnTime { get; private set; }

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

		//rotation with zero start and end angular velocities and constant angular acceleration
		public float MinRotationTime(float angle)
		{ return 2*Mathf.Sqrt(angle/AA_rad/Mathf.Rad2Deg); }


		public void Update(Vector3 torque)
		{
			Torque = torque;
			AA = VSL.Torque.AngularAcceleration(Torque);
			AA_rad = AA.magnitude;
			TurnTime = MinRotationTime(180);
		}
	}
}

