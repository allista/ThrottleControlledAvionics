//  Author:
//       Allis Tauri <allista@gmail.com>
//
//  Copyright (c) 2015 Allis Tauri
//
// This work is licensed under the Creative Commons Attribution-ShareAlike 4.0 International License. 
// To view a copy of this license, visit http://creativecommons.org/licenses/by-sa/4.0/ 
// or send a letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.

using System;
using System.Collections.Generic;
using UnityEngine;

namespace ThrottleControlledAvionics
{
	//convergent with Anatid's Vector6, but not taken from it
	public class Vector6 
	{
		public static Vector6 zero { get { return new Vector6(); } }

		public Vector3 positive, negative;

		public Vector6() {}

		public Vector6(Vector3 pos, Vector3 neg) { positive = pos; negative = neg; }

		public Vector6(Vector6 other)
			: this(other.positive, other.negative) {}

		public Vector6(float xp, float yp, float zp,
		               float xn, float yn, float zn)
			: this(new Vector3(xp, yp, zp), new Vector3(xn, yn, zn)) {}

		public static Vector6 operator+(Vector6 first, Vector6 second)
		{ 
			var sum = new Vector6();
			sum.positive = first.positive+second.positive; 
			sum.negative = first.negative+second.negative; 
			return sum;
		}

		public bool IsZero() { return positive.IsZero() && negative.IsZero(); }

		public float this[int i]
		{
			get { return i<3? positive[i] : negative[i-3]; }
			set 
			{ 
				if(i<3) positive[i] = value;
				else negative[i-3] = value; 
			}
		}

		public void Add(Vector6 vec)
		{
			positive += vec.positive; 
			negative += vec.negative; 
		}

		public void Add(Vector3 vec)
		{
			for(int i = 0; i < 3; i++)
			{
				if(vec[i] >= 0) positive[i] = positive[i]+vec[i];
				else negative[i] = negative[i]+vec[i];
			}
		}

		public void Add(List<Vector3> vecs) { vecs.ForEach(Add); }

		public Vector3 Clamp(Vector3 vec)
		{
			var cvec = Vector3.zero;
			for(int i = 0; i < 3; i++)
			{
				var vi = vec[i];
				cvec[i] = vi >= 0 ? 
					Mathf.Min(positive[i], vi) : 
					Mathf.Max(negative[i], vi);
			}
			return cvec;
		}

		public Vector3 Scale(Vector3 vec)
		{
			var svec = Vector3.zero;
			for(int i = 0; i < 3; i++)
			{
				var vi = vec[i];
				svec[i] = vi >= 0 ? 
					positive[i]*Mathf.Abs(vi) : 
					negative[i]*Mathf.Abs(vi);
			}
			return svec;
		}

		public void Scale(Vector6 other)
		{
			positive.Scale( other.positive);
			negative.Scale(-other.negative);
		}

		public Vector6 Scaled(Vector6 other)
		{
			var s = new Vector6(this);
			s.Scale(other);
			return s;
		}

		public Vector3 Max
		{
			get
			{
				var mvec = Vector3.zero;
				for(int i = 0; i < 3; i++)
					mvec[i] = Mathf.Max(-negative[i], positive[i]);
				return mvec;
			}
		}

		public Vector3 MaxInPlane(Vector3 normal)
		{
			var maxm = 0f;
			var max  = Vector3.zero;
			var cvec = Vector3.zero;
			for(int i = 0; i < 3; i++)
			{
				cvec[i] = positive[i];
				cvec = Vector3.ProjectOnPlane(cvec, normal);
				var cvecm = cvec.sqrMagnitude;
				if(cvecm > maxm) { max = cvec; maxm = cvecm; }
				cvec[i] = negative[i];
				cvec = Vector3.ProjectOnPlane(cvec, normal);
				cvecm = cvec.sqrMagnitude;
				if(cvecm > maxm) { max = cvec; maxm = cvecm; }
				cvec[i] = 0;
			}
			return max;
		}

		public Vector3 SumInPlane(Vector3 normal)
		{
			var sum = Vector3.zero;
			var cvec = Vector3.zero;
			for(int i = 0; i < 3; i++)
			{
				cvec[i] = positive[i];
				sum += Vector3.ProjectOnPlane(cvec, normal);
				cvec[i] = negative[i];
				sum += Vector3.ProjectOnPlane(cvec, normal);
				cvec[i] = 0;
			}
			return sum;
		}

		public Vector3 Project(Vector3 normal)
		{
			var proj = 0f;
			var cvec = Vector3.zero;
			for(int i = 0; i < 3; i++)
			{
				cvec[i] = positive[i];
				var projm = Vector3.Dot(cvec, normal);
				if(projm > 0) proj += projm;
				cvec[i] = negative[i];
				projm = Vector3.Dot(cvec, normal);
				if(projm > 0) proj += projm;
				cvec[i] = 0;
			}
			return proj*normal;
		}

		public Vector6 Transform(Transform T)
		{
			var tV = new Vector6();
			for(int i = 0; i < 3; i++)
			{
				tV.Add(T.TransformDirection(negative.Component(i)));
				tV.Add(T.TransformDirection(positive.Component(i)));
			}
			return tV;
		}

		public Vector6 InverseTransform(Transform T)
		{
			var tV = new Vector6();
			for(int i = 0; i < 3; i++)
			{
				tV.Add(T.InverseTransformDirection(negative.Component(i)));
				tV.Add(T.InverseTransformDirection(positive.Component(i)));
			}
			return tV;
		}

		public Vector6 Local2Local(Transform fromT, Transform toT)
		{
			var tV = new Vector6();
			for(int i = 0; i < 3; i++)
			{
				tV.Add(toT.InverseTransformDirection(fromT.TransformDirection(negative.Component(i))));
				tV.Add(toT.InverseTransformDirection(fromT.TransformDirection(positive.Component(i))));
			}
			return tV;
		}

		public override string ToString()
		{ 
			return string.Format("Vector6:\nMax {0}\n+ {1}\n- {2}", 
		                         Max, Utils.formatVector(positive), Utils.formatVector(negative)); 
		}
	}
}

