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

	public class GeometryProps : VesselProps
	{
		public GeometryProps(VesselWrapper vsl) : base(vsl) {}

		public Bounds  B { get; private set; } //bounds, including exhaust tails
		public Vector3 C { get; private set; } //center
		public float   H { get; private set; } //height
		public float   R { get; private set; } //radius
        public float   E { get; private set; } //radius including engines' exhaust
		public float   D { get; private set; } //diamiter
        public Vector3 RelC { get; private set; } //center relative to CoM
		public float   Area { get; private set; }
        public float   AreaWithBrakes { get; private set; }
		public Vector3 BoundsSideAreas { get; private set; }

		public float DistToBounds(Vector3 world_point)
		{ return Mathf.Sqrt(B.SqrDistance(refT.InverseTransformPoint(world_point))); }

		public override void Update()
		{
			//update physical bounds
			var b = vessel.Bounds(refT);
			C = refT.TransformPoint(b.center);
            RelC = C-VSL.vessel.CoM;
			H = Mathf.Abs(Vector3.Dot(refT.TransformDirection(b.extents), VSL.Physics.Up)) -
                Vector3.Dot(RelC, VSL.Physics.Up);
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
            E = b.extents.magnitude;
			B = b;
		}

        Timer brakes_measured_timer = new Timer();
        IEnumerator<YieldInstruction> measure_area_with_brakes_and_run(Callback action)
        {
            var brakes = VSL.vessel.ActionGroups[KSPActionGroup.Brakes];
            VSL.BrakesOn();
            brakes_measured_timer.Reset();
            AreaWithBrakes = BoundsSideAreas.MinComponentF();
            while(!brakes_measured_timer.TimePassed)
            {
                TCAGui.Status(0.1, "Testing aero-brakes...");
                var min_area = BoundsSideAreas.MinComponentF();
                if(min_area > AreaWithBrakes)
                {
                    AreaWithBrakes = min_area;
                    brakes_measured_timer.Reset();
                }
                yield return null;
            }
            if(!brakes)
                VSL.BrakesOn(false);
            action();
        }

        public void MeasureAreaWithBrakesAndRun(Callback action)
        {
            if(CFG.AutoBrakes)
            {
                VSL.TCA.StartCoroutine(measure_area_with_brakes_and_run(action));
                return;
            }
            Utils.Message("TCA is not allowed to use brakes. Check Advanced Tab.");
            action();
        }

        public void ResetAreaWithBrakes() { AreaWithBrakes = 0; }

		public float AreaInDirection(Vector3 wdir)
		{
			wdir.Normalize();
			return Vector3.Dot(BoundsSideAreas, new Vector3(
				Mathf.Abs(Vector3.Dot(wdir, VSL.refT.right)), 
				Mathf.Abs(Vector3.Dot(wdir, VSL.refT.up)),
				Mathf.Abs(Vector3.Dot(wdir, VSL.refT.forward))));
		}

        public float MinArea
        { get { return BoundsSideAreas.MinComponentF(); } }

        public float MaxArea
        { get { return BoundsSideAreas.MaxComponentF(); } }

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
					return VSL.refT.forward;
				default:
					return VSL.refT.up;
				}
			}
		}

		public Vector3 MaxDragDirection
		{
			get
			{
				float[] drag = new float[6];
				for(int i = 0, parts = VSL.vessel.Parts.Count; i < parts; i++)
				{
					var p = VSL.vessel.Parts[i];
					for(int f = 0; f < 6; f++)
						drag[f] += p.DragCubes.WeightedDrag[f];
				}
				int maxI = 0;
				float max = 0;
				for(int f = 0; f < 6; f++)
				{
					if(drag[f] > max)
					{
						max = drag[f];
						maxI = f;
					}
				}
				switch((DragCube.DragFace)maxI)
				{
				case DragCube.DragFace.XP:
					return VSL.refT.right;
				case DragCube.DragFace.XN:
					return -VSL.refT.right;
				case DragCube.DragFace.YP:
					return VSL.refT.up;
				case DragCube.DragFace.YN:
					return -VSL.refT.up;
				case DragCube.DragFace.ZP:
					return VSL.refT.forward;
				}
				return -VSL.refT.forward;
			}
		}

		public double MinDistance
		{
			get
			{
                var shift = RelC.magnitude;
				if(CFG.Target == null) return E+shift;
				var tgtVessel = CFG.Target.GetVessel();
                if(tgtVessel == null) return E+shift;
                return E+shift+tgtVessel.Radius(true);
			}
		}
	}
}

