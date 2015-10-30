//   Condition.cs
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
	public class Condition : TypedConfigNodeObject
	{
		public delegate void Selector(Action<Condition> callback);

		protected static TCAGlobals GLB { get { return TCAScenario.Globals; } }

		public Condition Prev;
		[Persistent] public bool negatable = true;
		[Persistent] public bool or;
		[Persistent] public bool not;
		[Persistent] public Condition Next;

		public bool Edit;
		public Selector SelectCondition;

		Vector2 scroll;

		public override void Load(ConfigNode node)
		{
			base.Load(node);
			not &= negatable;
			if(Next != null)
			{
				if(Next.GetType() == typeof(Condition)) Next = null;
				else Next.Prev = this;
			}
		}

		public void SetSelector(Selector selector)
		{
			SelectCondition = selector;
			if(Next != null) Next.SetSelector(selector);
		}

		public bool True(VesselWrapper VSL)
		{ 
			var value = Evaluate(VSL);
			if(not) value = !value;
			if(Next != null) 
				value = Next.or? 
					value || Next.True(VSL) : 
					value && Next.True(VSL);
			return value;
		}

		protected virtual bool Evaluate(VesselWrapper VSL) { return false; }
		protected virtual void DrawThis() {}

		public void Draw()
		{
			not &= negatable;
			scroll = GUILayout.BeginScrollView(scroll, GUILayout.ExpandWidth(true), GUILayout.ExpandHeight(false));
			GUILayout.BeginVertical();
			GUILayout.BeginHorizontal();
			if(Prev != null && GUILayout.Button(or? "OR" : "AND", Styles.white, GUILayout.Width(40))) or = !or;
			if(negatable && GUILayout.Button(not? "NOT" : "", Styles.red, GUILayout.Width(40))) not = !not;
			DrawThis();
			if(Prev != null && GUILayout.Button("X", Styles.red_button, GUILayout.Width(20))) Delete();
			if(Next != null) Next.Draw();
			else
			{
				if(GUILayout.Button("+", Styles.green_button, GUILayout.Width(20))) 
				{ if(SelectCondition != null) SelectCondition(Add); }
			}
			GUILayout.EndHorizontal();
			GUILayout.EndVertical();
			GUILayout.EndScrollView();
		}

		public bool Delete()
		{
			if(Prev == null) return false;
			if(Next == null) { Prev.Next = null; return true; }
			Prev.Next = Next; Next.Prev = Prev;
			return true;
		}

		public void Add(Condition next) 
		{ 
			Next = next; 
			Next.Prev = this; 
			Next.SetSelector(SelectCondition); 
		}
	}

	[HiddenComponent]
	public class FloatCondition : Condition
	{ 
		public enum CompareOperator { GT, LS, EQ }
		protected static readonly Dictionary<CompareOperator,string> OperatorNames = new Dictionary<CompareOperator, string>
		{ {CompareOperator.GT , ">"}, {CompareOperator.LS , "<"}, {CompareOperator.EQ , "="} };

		[Persistent] public float Value;
		[Persistent] public CompareOperator Operator;
		[Persistent] public float Error = 0.1f;
		[Persistent] public float Period;

		protected string Title;
		protected string Suffix;

		protected readonly FloatField ValueField = new FloatField();
		protected readonly FloatField ErrorField = new FloatField();
		protected readonly FloatField TimerField = new FloatField();
		protected readonly Timer WaitTimer = new Timer();

		public FloatCondition()
		{ Title = Utils.ParseCamelCase(GetType().Name.Replace(typeof(Condition).Name, "")); }

		protected virtual float VesselValue(VesselWrapper VSL) { return 0; }

		public override void Load(ConfigNode node)
		{
			base.Load(node);
			negatable = Period.Equals(0);
			not &= negatable;
		}

		protected override void DrawThis()
		{
			if(Edit) 
			{ 
				GUILayout.Label(Title, Styles.white, GUILayout.ExpandWidth(false));
				if(GUILayout.Button(OperatorNames[Operator], Styles.yellow, GUILayout.ExpandWidth(false)))
					Operator = (CompareOperator)(((int)Operator+1)%3);
				ValueField.Draw(Value, false);
				if(Operator == CompareOperator.EQ)
				{
					GUILayout.Label("Error", Styles.white, GUILayout.ExpandWidth(false));
					ErrorField.Draw(Error, false);
				}
				GUILayout.Label("Wait for:", Styles.white, GUILayout.ExpandWidth(false));
				TimerField.Draw(Period, false);
				if(GUILayout.Button("Done", Styles.green_button, GUILayout.ExpandWidth(false)))
				{
					if(ValueField.UpdateValue(Value)) Value = ValueField.Value;
					if(ErrorField.UpdateValue(Error)) Error = ErrorField.Value;
					if(TimerField.UpdateValue(Period))
					{
						Period = Utils.ClampL(TimerField.Value, 0);
						negatable = Period.Equals(0);
						WaitTimer.Period = Period;
						WaitTimer.Reset();
					}
					Edit = false; 
				}
			} 
			else 
			{
				var condition_string = string.Format("{0} {1} {2:F1}", Title, OperatorNames[Operator], Value);
				condition_string += Operator == CompareOperator.EQ? 
					string.Format("+/-{0:F2}{1}", Error, Suffix) : Suffix;
				if(Period > 0) condition_string += string.Format(" for {0:F1}s", Period);
				Edit |= GUILayout.Button(condition_string, Styles.normal_button, GUILayout.ExpandWidth(true));
			}
		}

		protected override bool Evaluate(VesselWrapper VSL)
		{
			var ret = false;
			switch(Operator)
			{
			case CompareOperator.GT:
				ret = VesselValue(VSL) > Value;
				break;
			case CompareOperator.EQ:
				ret = Mathf.Abs(VesselValue(VSL)-Value) < Error;
				break;
			case CompareOperator.LS:
				ret = VesselValue(VSL) < Value;
				break;
			}
			if(ret) { if(WaitTimer.Check) { WaitTimer.Reset(); return true; } }
			else WaitTimer.Reset();
			return ret;
		}
	}

//	[HiddenComponent]
//	public class IntCondition : Condition
//	{ 
//		[Persistent] public int Value;
//		protected string Title;
//		protected int Min = 0, Max = int.MaxValue;
//
//		protected virtual void OnValueChanged() {}
//
//		protected override void DrawThis()
//		{
//			GUILayout.BeginHorizontal();
//			if(Edit)
//			{ 
//				GUILayout.Label(Title, Styles.label);
//				var new_value = Utils.IntSelector(Value, Min, Max);
//				if(new_value != Value) 
//				{ 
//					Value = new_value;
//					OnValueChanged(); 
//					Edit = false;
//				} 
//			}
//			else Edit |= GUILayout.Button(string.Format ("{0} {1:D}", Title, Value), Styles.normal_button);
//			GUILayout.EndHorizontal();
//		}
//	}
}

