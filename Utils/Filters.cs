//  Author:
//       Allis Tauri <allista@gmail.com>
//
//  Copyright (c) 2015 Allis Tauri
//
// This work is licensed under the Creative Commons Attribution-ShareAlike 4.0 International License. 
// To view a copy of this license, visit http://creativecommons.org/licenses/by-sa/4.0/ 
// or send a letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.

using System;
using UnityEngine;

namespace ThrottleControlledAvionics
{
	public class TimeAverage
	{
		public float dT;
		protected int n;
		protected float t, val, new_val;
		public float Value { get; protected set; }
		public float Speed { get; protected set; }
		public TimeAverage(float dt = 0.05f) { dT = dt; }

		public bool Update(float v)
		{
			val += v;
			t += TimeWarp.fixedDeltaTime;
			n += 1;
			if(t < dT) return false;
			new_val = val/n;
			Speed = (new_val-Value)/t;
			Value = new_val;
			val = t = n = 0;
			return true;
		}

		public void Reset() { Value = Speed = val = t = n = 0; }

		public static implicit operator float(TimeAverage td) { return td.Value; }
		public override string ToString() { return string.Format("Value: {0}, Speed: {1}", Value, Speed); }
	}

	public class EWA
	{
		float old, value;

		public EWA(float v = 0) { old = v; value = v; }

		public float Update(float v, float ratio = 0.1f)
		{
			value = Utils.EWA(old, v, ratio);
			old = value;
			return value;
		}
		public static implicit operator float(EWA ewa) { return ewa.value; }
		public override string ToString() { return value.ToString(); }
		public string ToString(string F) { return value.ToString(F); }
	}

	public abstract class LowPassFilter<T>
	{
		protected T prev;
		public float Tau = 1;
		protected float alpha { get { return (TimeWarp.fixedDeltaTime/(Tau+TimeWarp.fixedDeltaTime)); } }

		public T Value { get { return prev; } }
		public abstract T Update(T cur);
		public void Reset() { prev = default(T); }
	}

	public class LowPassFilterF : LowPassFilter<float>
	{
		public override float Update(float cur)
		{
			prev = prev +  alpha * (cur-prev);
			return prev;
		}
	}

	public class LowPassFilterV : LowPassFilter<Vector3>
	{
		public override Vector3 Update(Vector3 cur)
		{
			prev = prev + alpha * (cur-prev);
			return prev;
		}
	}

	public class LowPassFilterVd : LowPassFilter<Vector3d>
	{
		public override Vector3d Update(Vector3d cur)
		{
			prev = prev + alpha * (cur-prev);
			return prev;
		}
	}

	public class LowPassFilterVV
	{
		Vector3 prev;
		public Vector3 Value { get { return prev; } }

		public Vector3 Update(Vector3 cur, Vector3 tau)
		{
			var output = Vector3.zero;
			output.x = prev.x + (TimeWarp.fixedDeltaTime/(tau.x+TimeWarp.fixedDeltaTime)) * (cur.x-prev.x);
			output.y = prev.y + (TimeWarp.fixedDeltaTime/(tau.y+TimeWarp.fixedDeltaTime)) * (cur.y-prev.y);
			output.z = prev.z + (TimeWarp.fixedDeltaTime/(tau.z+TimeWarp.fixedDeltaTime)) * (cur.z-prev.z);
			prev = output;
			return output;
		}
	}

	public class LowPassFilterVVd
	{
		Vector3d prev;
		public Vector3d Value { get { return prev; } }

		public Vector3d Update(Vector3d cur, Vector3d tau)
		{
			var output = Vector3d.zero;
			output.x = prev.x + (TimeWarp.fixedDeltaTime/(tau.x+TimeWarp.fixedDeltaTime)) * (cur.x-prev.x);
			output.y = prev.y + (TimeWarp.fixedDeltaTime/(tau.y+TimeWarp.fixedDeltaTime)) * (cur.y-prev.y);
			output.z = prev.z + (TimeWarp.fixedDeltaTime/(tau.z+TimeWarp.fixedDeltaTime)) * (cur.z-prev.z);
			prev = output;
			return output;
		}
	}

	public class FuzzyThreshold<T> where T : IComparable
	{
		public T Upper, Lower;
		T value;
		public T Value 
		{ 
			get { return value; } 
			set 
			{ 
				this.value = value; 
				if(this.value.CompareTo(Lower) < 0) On = true;
				else On &= this.value.CompareTo(Upper) < 0;
			}
		}
		public bool On { get; protected set; } = false;

		public FuzzyThreshold() { value = default(T); }
		public FuzzyThreshold(T upper, T lower) { Upper = upper; Lower = lower; }

		public static implicit operator bool(FuzzyThreshold<T> t) { return t.On; }
		public static implicit operator T(FuzzyThreshold<T> t) { return t.value; }

		public override string ToString()
		{ return string.Format("[FuzzyThreshold: Value={0}, On={1}, Upper={2}, Lower={3}]", Value, On, Upper, Lower); }
	}
}

