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
using UnityEngine;

namespace ThrottleControlledAvionics
{
	public class Condition : TypedConfigNodeObject
	{
		public delegate void Selector(Action<Condition> callback);

		protected static TCAGlobals GLB { get { return TCAScenario.Globals; } }

		public Condition Prev;
		public bool negatable = true;
		[Persistent] public bool or;
		[Persistent] public bool not;
		[Persistent] public Condition Next;

		public bool Edit;
		public Selector SelectCondition;

		Vector2 scroll;

		public override void Load(ConfigNode node)
		{
			base.Load(node);
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
}

