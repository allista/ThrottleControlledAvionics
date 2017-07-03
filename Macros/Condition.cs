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
using AT_Utils;

namespace ThrottleControlledAvionics
{
	public class Condition : TypedConfigNodeObject
	{
		public delegate void Selector(Action<Condition> callback);

		internal static Globals GLB { get { return Globals.Instance; } }

		protected string Name;
        protected GUIContent Label;
		public Condition Prev;
		[Persistent] public bool negatable = true;
		[Persistent] public bool or;
		[Persistent] public bool not;
		[Persistent] public Condition Next;

		public bool Edit;
		public Selector SelectCondition;

		Vector2 scroll;

		public Condition() 
		{ Name = Utils.ParseCamelCase(GetType().Name.Replace(typeof(Condition).Name, "")); }

        public Condition(ComponentInfo info) : this()
        {
            if(!string.IsNullOrEmpty(info.Name)) Name = info.Name;
            Label = new GUIContent(Name, info.Description);
        }

		public override void Load(ConfigNode node)
		{
			base.Load(node);
            var t = GetType();
            Label = Components.Conditions.ContainsKey(t)? 
                Components.Conditions[t].Label : new GUIContent(Name);
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
        protected virtual void DrawThis() { GUILayout.Label(Label, Styles.label, GUILayout.ExpandWidth(false)); }

		public void Draw()
		{
			not &= negatable;
			scroll = GUILayout.BeginScrollView(scroll, GUILayout.ExpandWidth(true), GUILayout.ExpandHeight(false));
			GUILayout.BeginVertical();
			GUILayout.BeginHorizontal();
			if(Prev != null && GUILayout.Button(or? "OR" : "AND", Styles.white, GUILayout.Width(40))) or = !or;
			if(negatable && GUILayout.Button(not? "NOT" : "", Styles.red, GUILayout.Width(40))) not = !not;
			DrawThis();
            if(Prev != null && GUILayout.Button(new GUIContent("X", "Delete"), Styles.close_button, GUILayout.Width(20))) Delete();
			if(Next != null) Next.Draw();
			else
			{
                if(GUILayout.Button(new GUIContent("+", "Add new condition"), Styles.active_button, GUILayout.Width(20))) 
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

    [ComponentInfo(Hidden = true)]
	public class FloatCondition : Condition
	{ 
		public enum CompareOperator { GT, LS, EQ }
		protected static readonly Dictionary<CompareOperator,string> OperatorNames = new Dictionary<CompareOperator, string>
		{ {CompareOperator.GT , ">"}, {CompareOperator.LS , "<"}, {CompareOperator.EQ , "="} };

		[Persistent] public CompareOperator Operator;
		[Persistent] public FloatField Value  = new FloatField();
		[Persistent] public FloatField Error  = new FloatField();
		[Persistent] public FloatField Period = new FloatField(min:0);

		protected string Suffix;
		protected readonly Timer WaitTimer = new Timer();
		protected virtual float VesselValue(VesselWrapper VSL) { return 0; }

		public FloatCondition() { Error.Value = 0.1f; }

		public override void Load(ConfigNode node)
		{
			base.Load(node);
			negatable = Period.Equals(0);
			not &= negatable;
			WaitTimer.Period = Period;
			WaitTimer.Reset();
		}

		protected override void DrawThis()
		{
			if(Edit) 
			{ 
                GUILayout.Label(Label, Styles.white, GUILayout.ExpandWidth(false));
				if(GUILayout.Button(OperatorNames[Operator], Styles.yellow, GUILayout.ExpandWidth(false)))
					Operator = (CompareOperator)(((int)Operator+1)%3);
				Value.Draw(Suffix);
				if(Operator == CompareOperator.EQ)
				{
                    GUILayout.Label(new GUIContent("Error", "Interval of tolerance"), 
                                    Styles.white, GUILayout.ExpandWidth(false));
					Error.Draw(Suffix);
				}
                GUILayout.Label(new GUIContent("Wait for:", "The condition should be met at least this number of seconds"), 
                                Styles.white, GUILayout.ExpandWidth(false));
				Period.Draw("s");
				if(GUILayout.Button("Done", Styles.confirm_button, GUILayout.ExpandWidth(false)))
				{
					Value.UpdateValue();
					Error.UpdateValue();
					if(Period.UpdateValue())
					{
						negatable = Period.Equals(0);
						WaitTimer.Period = Period;
						WaitTimer.Reset();
					}
					Edit = false; 
				}
			} 
			else 
			{
				var condition_string = string.Format("{0} {1} {2:F1}", Name, OperatorNames[Operator], Value);
				condition_string += Operator == CompareOperator.EQ? 
					string.Format("+/-{0:F2}{1}", Error, Suffix) : Suffix;
				if(Period > 0) condition_string += string.Format(" for {0:F1}s", Period);
                Edit |= GUILayout.Button(new GUIContent(condition_string, Label.tooltip), 
                                         Styles.normal_button, GUILayout.ExpandWidth(true));
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
			if(ret) { if(WaitTimer.TimePassed) { WaitTimer.Reset(); return true; } }
			else WaitTimer.Reset();
			return false;
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

