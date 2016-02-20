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

namespace ThrottleControlledAvionics
{

	public class GeometryProps : VesselProps
	{
		public GeometryProps(VesselWrapper vsl) : base(vsl) {}

		public Bounds  B { get; private set; } //bounds, including exhaust tails
		public Vector3 C { get; private set; } //center
		public float   H { get; private set; } //height
		public float   R { get; private set; } //radius

		public float DistToBounds(Vector3 world_point)
		{ return Mathf.Sqrt(B.SqrDistance(refT.InverseTransformPoint(world_point))); }

		public override void Update()
		{
			//update physical bounds
			var b = new Bounds();
			bool inited = false;
			var parts = vessel.parts;
			for(int i = 0, partsCount = parts.Count; i < partsCount; i++)
			{
				Part p = parts[i];
				if(p == null) continue;
				var meshes = p.FindModelComponents<MeshFilter>();
				for(int mi = 0, meshesLength = meshes.Length; mi < meshesLength; mi++)
				{
					//skip meshes without renderer
					var m = meshes[mi];
					if(m.renderer == null || !m.renderer.enabled) continue;
					var bounds = Utils.BoundCorners(m.sharedMesh.bounds);
					for(int j = 0; j < 8; j++)
					{
						var c = refT.InverseTransformPoint(m.transform.TransformPoint(bounds[j]));
						if(inited) b.Encapsulate(c);
						else { b = new Bounds(c, Vector3.zero); inited = true; }
					}
				}
			}
			C = refT.TransformPoint(b.center);
			H = Mathf.Abs(Vector3.Dot(refT.TransformDirection(b.extents), VSL.Physics.Up))+
				Vector3.Dot(C-vessel.CurrentCoM, VSL.Physics.Up);
			R = b.extents.magnitude;
			//update exhaust bounds
			foreach(var e in VSL.Engines.All)
			{
				if(!e.Valid(VSL) || !e.engine.exhaustDamage) continue;
				for(int k = 0, tCount = e.engine.thrustTransforms.Count; k < tCount; k++)
				{
					var t = e.engine.thrustTransforms[k];
					if(t == null) continue;
					var term = refT.InverseTransformPoint(t.position + t.forward * e.engine.exhaustDamageMaxRange*GLB.ExhaustSafeDist);
					if(inited) b.Encapsulate(term);
					else { b = new Bounds(term, Vector3.zero); inited = true; }
				}
			}
			B = b;
		}
	}
}

