//   Vector6.cs
//
//  Author:
//       Allis Tauri <allista@gmail.com>
//
//  Copyright (c) 2015 Allis Tauri
//
// This work is licensed under the Creative Commons Attribution 4.0 International License. 
// To view a copy of this license, visit http://creativecommons.org/licenses/by/4.0/ 
// or send a letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.

using System;
using System.Collections.Generic;
using UnityEngine;

namespace ThrottleControlledAvionics
{
	//convergent with Anatid's Vector6, but not taken from it
	public class Vector6 
	{
		public Vector3 positive = Vector3.zero, negative = Vector3.zero;

		public static Vector6 operator+(Vector6 first, Vector6 second)
		{ 
			var sum = new Vector6();
			sum.positive = first.positive+second.positive; 
			sum.negative = first.negative+second.negative; 
			return sum;
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
				cvec.Zero();
				cvec[i] = positive[i];
				cvec = Vector3.ProjectOnPlane(cvec, normal);
				var cvecm = cvec.sqrMagnitude;
				if(cvecm > maxm) { max = cvec; maxm = cvecm; }
				cvec[i] = negative[i];
				cvec = Vector3.ProjectOnPlane(cvec, normal);
				cvecm = cvec.sqrMagnitude;
				if(cvecm > maxm) { max = cvec; maxm = cvecm; }
			}
			return max;
		}

		public Vector3 Project(Vector3 normal)
		{
			var proj = Vector3.zero;
			var cvec = Vector3.zero;
			for(int i = 0; i < 3; i++)
			{
				cvec.Zero();
				cvec[i] = positive[i];
				var projm = Vector3.Dot(cvec, normal);
				if(projm > 0) proj += normal*projm;
				cvec[i] = negative[i];
				projm = Vector3.Dot(cvec, normal);
				if(projm > 0) proj += normal*projm;
			}
			return proj;
		}

		public override string ToString()
		{ return string.Format("Vector6:\nMax {0}\n+ {1}\n- {2}", Max, positive, negative); }
	}
}

