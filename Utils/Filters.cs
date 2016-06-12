//  Author:
//       Allis Tauri <allista@gmail.com>
//
//  Copyright (c) 2015 Allis Tauri
//
// This work is licensed under the Creative Commons Attribution-ShareAlike 4.0 International License. 
// To view a copy of this license, visit http://creativecommons.org/licenses/by-sa/4.0/ 
// or send a letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.

using System;
using System.Linq;
using System.Collections.Generic;
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
			if(float.IsNaN(v)) return false;
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

	/// <summary>
	/// Exponentially weighted average.
	/// </summary>
	public class EWA
	{
		float old, value;

		public EWA(float v = 0) { old = v; value = v; }

		public float Update(float v, float ratio = 0.1f)
		{
			if(float.IsNaN(v)) return value;
			value = Utils.EWA(old, v, ratio);
			old = value;
			return value;
		}
		public static implicit operator float(EWA ewa) { return ewa.value; }
		public override string ToString() { return value.ToString(); }
		public string ToString(string F) { return value.ToString(F); }
	}

	public abstract class LowPassFilter
	{
		public static float alpha(float tau) { return TimeWarp.fixedDeltaTime/(tau+TimeWarp.fixedDeltaTime);}
		public static float tau(float alpha) { return TimeWarp.fixedDeltaTime*(1-alpha)/alpha; }
	}

	public abstract class LowPassFilterBase<T>
	{
		protected T value;

		public T Value { get { return value; } }
		public abstract T Update(T cur);
		public void Reset() { value = default(T); }
		public void Set(T val) { value = val; }

		public static implicit operator T(LowPassFilterBase<T> f) { return f.value; }
		public override string ToString() { return value.ToString(); }
	}

	/// <summary>
	/// Low-pass filter.
	/// A more convenient form of exponentially weighted average.
	/// </summary>
	public abstract class LowPassFilter<T> : LowPassFilterBase<T>
	{
		protected float alpha;
		public float Tau 
		{ 
			get { return LowPassFilter.tau(alpha); } 
			set { alpha = LowPassFilter.alpha(value); } 
		}

		#if DEBUG
		public string DebugInfo
		{ get { return string.Format("LowPassFilter: [Tau {0}, Value {1}]", Tau, Value); } }
		#endif
	}

	/// <summary>
	/// Asymmetric low-pass filter with different Tau parameter
	/// for values that are less then and grater than average.
	/// </summary>
	public abstract class AsymmetricFilter<T> : LowPassFilterBase<T> where T : IComparable
	{
		protected float alphaF, alphaR;
		public float TauF 
		{ 
			get { return LowPassFilter.tau(alphaF); } 
			set { alphaF = LowPassFilter.alpha(value); } 
		}
		public float TauR
		{ 
			get { return LowPassFilter.tau(alphaR); } 
			set { alphaR = LowPassFilter.alpha(value); } 
		}

		protected float _alpha(T cur) 
		{ return cur.CompareTo(value) > 0? alphaF : alphaR; }


		#if DEBUG
		public string DebugInfo
		{ get { return string.Format("AsymmetricFilter: [TauF {0}, TauR {1}, Value {2}]", TauF, TauR, value); } }
		#endif
	}

	public abstract class ClampedAsymmetricFilter<T> : AsymmetricFilter<T>
		where T : IComparable
	{ public T Min, Max; }

	public abstract class ConditionalLowPassFilter<T> : LowPassFilter<T>
	{ protected abstract bool do_filter(T cur); }

	public class EquilibriumLowPassFilterVd : ConditionalLowPassFilter<Vector3d>
	{
		protected override bool do_filter(Vector3d cur)
		{ return Vector3d.Dot(cur, value) <= 0; }

		public override Vector3d Update(Vector3d cur)
		{
			if(cur.IsNaN()) return value;
			value = do_filter(cur)? value +  alpha * (cur-value) : cur;
			return value;
		}
	}

	public class LowPassFilterF : LowPassFilter<float>
	{
		public override float Update(float cur)
		{
			if(float.IsNaN(cur)) return value;
			value = value +  alpha * (cur-value);
			return value;
		}
	}

	public class LowPassFilterD : LowPassFilter<double>
	{
		public override double Update(double cur)
		{
			if(double.IsNaN(cur)) return value;
			value = value +  alpha * (cur-value);
			return value;
		}
	}

	public class LowPassFilterV : LowPassFilter<Vector3>
	{
		public override Vector3 Update(Vector3 cur)
		{
			if(cur.IsNaN()) return value;
			value = value + alpha * (cur-value);
			return value;
		}
	}

	public class LowPassFilterVd : LowPassFilter<Vector3d>
	{
		public override Vector3d Update(Vector3d cur)
		{
			if(cur.IsNaN()) return value;
			value = value + alpha * (cur-value);
			return value;
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
		Vector3d value;
		public Vector3d Value { get { return value; } }

		public Vector3d Update(Vector3d cur, Vector3d tau)
		{
			var output = Vector3d.zero;
			output.x = value.x + (TimeWarp.fixedDeltaTime/(tau.x+TimeWarp.fixedDeltaTime)) * (cur.x-value.x);
			output.y = value.y + (TimeWarp.fixedDeltaTime/(tau.y+TimeWarp.fixedDeltaTime)) * (cur.y-value.y);
			output.z = value.z + (TimeWarp.fixedDeltaTime/(tau.z+TimeWarp.fixedDeltaTime)) * (cur.z-value.z);
			value = output;
			return output;
		}
	}

	public class AsymmetricFiterF : AsymmetricFilter<float>
	{
		public override float Update(float cur)
		{
			if(float.IsNaN(cur)) return value;
			value = value + (cur > value? alphaF : alphaR) * (cur-value);
			return value;
		}
	}

	public class ClampedAssymetricFilterF : ClampedAsymmetricFilter<float>
	{
		public override float Update(float cur)
		{
			if(float.IsNaN(cur)) return value;
			value = Utils.Clamp(value + (cur > value? alphaF : alphaR) * (cur-value), Min, Max);
			return value;
		}
	}

	public class AsymmetricFiterD : AsymmetricFilter<double>
	{
		public override double Update(double cur)
		{
			if(double.IsNaN(cur)) return value;
			value = value + (cur > value? alphaF : alphaR) * (cur-value);
			return value;
		}
	}

	public class ClampedAssymetricFilterD : ClampedAsymmetricFilter<double>
	{
		public override double Update(double cur)
		{
			if(double.IsNaN(cur)) return value;
			value = Utils.Clamp(value + (cur > value? alphaF : alphaR) * (cur-value), Min, Max);
			return value;
		}
	}

	public class FuzzyThreshold<T> where T : IComparable
	{
		public bool Inverse;
		public T Upper, Lower;
		T value;
		public T Value 
		{ 
			get { return value; } 
			set 
			{ 
				this.value = value; 
				if(Inverse)
				{
					if(this.value.CompareTo(Upper) > 0) On = true;
					else On &= this.value.CompareTo(Lower) > 0;
				}
				else
				{
					if(this.value.CompareTo(Lower) < 0) On = true;
					else On &= this.value.CompareTo(Upper) < 0;
				}
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

	public abstract class Extremum<T> where T : IComparable
	{
		protected T v2, v1, v0;
		protected int i;

		public void Update(T cur) { v2 = v1; v1 = v0; v0 = cur; if(i < 3) i++; }
		public abstract bool True { get; }
		public T Value { get { return v1; } }
		public void Reset() { v2 = v1 = v0 = default(T); i = 0; }

		public static implicit operator bool(Extremum<T> e) { return e.True; }
		public static implicit operator T(Extremum<T> e) { return e.Value; }
	}

	public class MinimumF: Extremum<float>
	{ public override bool True { get { return i > 2 && v2 >= v1 && v1 < v0; } } }

	public class MinimumD: Extremum<double>
	{ public override bool True { get { return i > 2 && v2 >= v1 && v1 < v0; } } }

	public abstract class StateMap<T>
	{
		readonly protected int order;
		readonly protected Queue<T> states;
		readonly protected T[] values;

		protected StateMap(int order)
		{
			this.order = order+1;
			states = new Queue<T>(this.order);
			values = new T[this.order];
		}

		public T this[int i] { get { return values[i]; } }

		public int MaxOrder { get { return order-1; } }

		public int Count { get { return states.Count; } }

		public static implicit operator bool(StateMap<T> m) { return m.states.Count == m.order; }

		public void Update(T cur)
		{
			states.Enqueue(cur);
			if(states.Count > order)
				states.Dequeue();
			update_values();
		}

		protected abstract void update_values();

		public override string ToString()
		{
			return string.Format("[StateMap: MaxOrder={0}, Available={1}, Values=({2})]", 
			                     MaxOrder, Count-1, values.Aggregate("", (s, v) => s+(string.IsNullOrEmpty(s)? "" : ", ")+v));
		}
	}

	public class DifferentialF: StateMap<float>
	{
		public DifferentialF(int order = 1) : base(order) {}

		protected override void update_values()
		{
			var  i = 0;
			var  d = 0.0f;
			foreach(var s in states)
			{
				if(i == 0) values[i] = s;
				else 
				{
					var n = i == 1? s-values[0] : values[i - 1] - d;
					d = values[i];
					values[i] = n;
				}
				i++;
			}
		}
	}

	public class DifferentialD: StateMap<double>
	{
		public DifferentialD(int order = 1) : base(order) {}

		protected override void update_values()
		{
			var  i = 0;
			var  d = 0.0;
			foreach(var s in states)
			{
				if(i == 0) values[i] = s;
				else 
				{
					var n = i == 1? s-values[0] : values[i - 1] - d;
					d = values[i];
					values[i] = n;
				}
				i++;
			}
		}
	}
}

