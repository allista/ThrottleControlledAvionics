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

	public class PhysicalProps : VesselProps
	{
		public PhysicalProps(VesselWrapper vsl) : base(vsl) {}

		public Vector3d   Up { get; private set; }  //up unit vector in world space
		public Vector3d   UpL { get; private set; }  //up unit vector in world space

		public float      M { get; private set; } //mass
		public float      StG { get; private set; } //gee at position
		public float      G { get; private set; } //gee - centrifugal acceleration
		public double     UT { get; private set; } //Planetarium.GetUniversalTime
		public Vector3    wCoM { get; private set; } //center of mass in world space
		public Vector3    MoI { get; private set; } = Vector3.one; //main diagonal of inertia tensor
		public Matrix3x3f InertiaTensor { get; private set; }

		public override void Update()
		{
			UT   = Planetarium.GetUniversalTime();
			wCoM = vessel.CurrentCoM;
			refT = vessel.ReferenceTransform;
			Up   = (wCoM - vessel.mainBody.position).normalized;
			UpL  = VSL.refT.InverseTransformDirection(Up);
			M    = vessel.GetTotalMass();
			StG  = (float)(vessel.mainBody.gMagnitudeAtCenter/(vessel.mainBody.position - wCoM).sqrMagnitude);
			G    = Utils.ClampL(StG-(float)vessel.CentrifugalAcc.magnitude, 1e-5f);
		}

		public Vector3d NorthDirW { get { return Vector3.ProjectOnPlane(VSL.mainBody.position+VSL.mainBody.transform.up*(float)VSL.mainBody.Radius-wCoM, Up).normalized; } }
//		public Vector3d EastDirW { get { return (QuaternionD.AngleAxis(90, Up) * NorthDirW).normalized; } }

		public double Bearing(Vector3d dir)
		{
			dir.Normalize();
			var north = NorthDirW;
			var east  = (QuaternionD.AngleAxis(90, Up) * north).normalized;
			return Utils.ClampAngle(Math.Atan2(Vector3d.Dot(dir, east), Vector3d.Dot(dir, north))*Mathf.Rad2Deg);
		}

		public Vector3d Direction(double bearing)
		{ return (QuaternionD.AngleAxis(bearing, Up) * NorthDirW).normalized; }

		// From MechJeb2:
		// KSP's calculation of the vessel's moment of inertia is broken.
		// This function is somewhat expensive :(
		// Maybe it can be optimized more.
		static readonly Vector3[] unitVectors = { new Vector3(1, 0, 0), new Vector3(0, 1, 0), new Vector3(0, 0, 1) };
		public void UpdateMoI()
		{
			if(vessel == null || vessel.rootPart.Rigidbody == null) return;
			var CoM = vessel.CurrentCoM;
			InertiaTensor = new Matrix3x3f();
			var vesselTransform = vessel.GetTransform();
			var inverseVesselRotation = Quaternion.Inverse(vesselTransform.rotation);
			for(int pi = 0, vesselpartsCount = vessel.parts.Count; pi < vesselpartsCount; pi++)
			{
				Part p = vessel.parts[pi];
				var rb = p.Rigidbody;
				if(rb == null) continue;
				//Compute the contributions to the vessel inertia tensor due to the part inertia tensor
				Vector3 principalMoments = rb.inertiaTensor;
				Quaternion principalAxesRot = inverseVesselRotation * p.transform.rotation * rb.inertiaTensorRotation;
				Quaternion invPrincipalAxesRot = Quaternion.Inverse(principalAxesRot);
				for(int j = 0; j < 3; j++)
				{
					Vector3 partInertiaTensorTimesjHat = principalAxesRot * Vector3.Scale(principalMoments, invPrincipalAxesRot * unitVectors[j]);
					for(int i = 0; i < 3; i++)
						InertiaTensor.Add(i, j, Vector3.Dot(unitVectors[i], partInertiaTensorTimesjHat));
				}
				//Compute the contributions to the vessel inertia tensor due to the part mass and position
				float partMass = rb.mass;
				Vector3 partPosition = vesselTransform.InverseTransformDirection(rb.worldCenterOfMass - CoM);
				for(int i = 0; i < 3; i++)
				{
					InertiaTensor.Add(i, i, partMass * partPosition.sqrMagnitude);
					for(int j = 0; j < 3; j++)
						InertiaTensor.Add(i, j, -partMass * partPosition[i] * partPosition[j]);
				}
			}
			MoI = new Vector3(InertiaTensor[0, 0], InertiaTensor[1, 1], InertiaTensor[2, 2]);
			MoI = refT.InverseTransformDirection(vessel.transform.TransformDirection(MoI));
		}
	}
}

