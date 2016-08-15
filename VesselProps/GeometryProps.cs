//  Author:
//       Allis Tauri <allista@gmail.com>
//
//  Copyright (c) 2016 Allis Tauri
//
// This work is licensed under the Creative Commons Attribution-ShareAlike 4.0 International License. 
// To view a copy of this license, visit http://creativecommons.org/licenses/by-sa/4.0/ 
// or send a letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.
//
using UnityEngine;
using AT_Utils;

namespace ThrottleControlledAvionics
{

	public class GeometryProps : VesselProps
	{
		public GeometryProps(VesselWrapper vsl) : base(vsl) {}

		public Bounds  B { get; private set; } //bounds, including exhaust tails
		public Vector3 C { get; private set; } //center
		public float   H { get; private set; } //height
		public float   R { get; private set; } //radius
		public float   D { get; private set; } //diamiter
		public float Area { get; private set; }
		public Vector3 BoundsSideAreas { get; private set; }

		public float DistToBounds(Vector3 world_point)
		{ return Mathf.Sqrt(B.SqrDistance(refT.InverseTransformPoint(world_point))); }

		public override void Update()
		{
			//update physical bounds
			var b = vessel.Bounds(refT);
			C = refT.TransformPoint(b.center);
			H = Mathf.Abs(Vector3.Dot(refT.TransformDirection(b.extents), VSL.Physics.Up))+
				Vector3.Dot(VSL.Physics.wCoM-C, VSL.Physics.Up);
			R = b.extents.magnitude;
			D = R*2;
			BoundsSideAreas = new Vector3(B.extents.y*B.extents.z, //right
										  B.extents.x*B.extents.z, //up
										  B.extents.x*B.extents.y);//forward
			Area = (BoundsSideAreas.x+BoundsSideAreas.y+BoundsSideAreas.z)*2;
			//update exhaust bounds
			foreach(var e in VSL.Engines.All)
			{
				if(!e.Valid(VSL) || !e.engine.exhaustDamage) continue;
				for(int k = 0, tCount = e.engine.thrustTransforms.Count; k < tCount; k++)
				{
					var t = e.engine.thrustTransforms[k];
					if(t == null) continue;
					var term = refT.InverseTransformPoint(t.position + t.forward * e.engine.exhaustDamageMaxRange*GLB.ExhaustSafeDist);
					b.Encapsulate(term);
				}
			}
			B = b;
		}

		public float AreaInDirection(Vector3 wdir)
		{
			wdir.Normalize();
			return Vector3.Dot(BoundsSideAreas, new Vector3(
				Mathf.Abs(Vector3.Dot(wdir, VSL.refT.right)), 
				Mathf.Abs(Vector3.Dot(wdir, VSL.refT.up)),
				Mathf.Abs(Vector3.Dot(wdir, VSL.refT.forward))));
		}

		public Vector3 MinAreaDirection
		{
			get
			{
				var minI = BoundsSideAreas.MinI();
				switch(minI)
				{
				case 0:
					return VSL.refT.right;
				case 1:
					return VSL.refT.up;
				case 2:
					return -VSL.refT.forward;
				default:
					return VSL.refT.up;
				}
			}
		}

		public Vector3 MaxAreaDirection
		{
			get
			{
				var maxI = BoundsSideAreas.MaxI();
				switch(maxI)
				{
				case 0:
					return VSL.refT.right;
				case 1:
					return VSL.refT.up;
				case 2:
					return -VSL.refT.forward;
				default:
					return VSL.refT.up;
				}
			}
		}
	}
}

