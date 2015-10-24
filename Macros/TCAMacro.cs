//   ActionNode.cs
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
	public static class Components
	{
		public static SortedList<string, ComponentDB<Condition>.Factory> Conditions
		{ get { return ComponentDB<Condition>.Components; } }

		public static SortedList<string, ComponentDB<MacroNode>.Factory> Actions
		{ get { return ComponentDB<MacroNode>.Components; } }

		public static bool ConditionSelector(out Condition condition)
		{ return ComponentDB<Condition>.Selector(out condition); }

		public static bool ActionSelector(out MacroNode action)
		{ return ComponentDB<MacroNode>.Selector(out action); }
	}

	public class Condition : TypedConfigNodeObject
	{
		protected static TCAGlobals GLB { get { return TCAScenario.Globals; } }

		[Persistent] public bool or;
		[Persistent] public Condition Next;
		[Persistent] readonly PersistentBaseList<Condition> Alternatives = new PersistentBaseList<Condition>();

		public bool Edit;
		public bool SelectAlternative, SelectNext;

		public bool True(VesselWrapper VSL)
		{ 
			var value = Evaluate(VSL);
			for(int i = 0, count = Alternatives.Count; i < count; i++) 
				value |= Alternatives[i].True(VSL);
			if(Next != null)
			{
				if(or) return value || Next.True(VSL);
				else return value && Next.True(VSL);
			}
			return value;
		}

		protected virtual bool Evaluate(VesselWrapper VSL) { return false; }
		protected virtual void DrawThis() {}

		public void Draw()
		{
			if(Alternatives.Count > 0) GUILayout.Label("(");
			DrawThis();
			if(GUILayout.Button("|", Styles.normal_button, GUILayout.Width(20))) 
				SelectAlternative = !SelectAlternative;
			if(Alternatives.Count > 0) 
			{
				for(int i = 0, count = Alternatives.Count; i < count; i++) 
				{ Alternatives[i].Draw(); if(i < count-1) GUILayout.Label("|"); }
				GUILayout.Label(")");
			}
			if(Next != null) 
			{ 
				if(GUILayout.Button(or? "OR" : "AND", Styles.normal_button, GUILayout.Width(20))) or = !or;
				Next.Draw();
			}
			else 
			{ 
				if(GUILayout.Button("+", Styles.normal_button, GUILayout.Width(20))) 
					SelectNext = !SelectNext;
			}
		}

		public void AddAlternative(Condition alt) { Alternatives.Add(alt); }
		public bool RemoveAlternative(Condition alt) { return Alternatives.Remove(alt); }

		public void Or(Condition next) { or = true; Next = next; }
		public void And(Condition next) { or = false; Next = next; }
	}

	public class MacroNode : TypedConfigNodeObject
	{
		protected static TCAGlobals GLB { get { return TCAScenario.Globals; } }

		[Persistent] public string Name;
		[Persistent] public Condition Condition = new TrueCondition();
		[Persistent] public bool Active;
		[Persistent] public bool Paused;
		[Persistent] public MacroNode Parent;
		[Persistent] public PersistentBaseList<MacroNode> Children = new PersistentBaseList<MacroNode>();
		public bool HasChildren { get { return Children.Count > 0; } }

		public bool Edit, SelectNext, SelectCondition;

		/// <summary>
		/// Perform the Action on a specified VSL.
		/// Returns true if it is not finished and should be called on next update. 
		/// False if it is finished.
		/// </summary>
		/// <param name="VSL">VesselWrapper</param>
		protected virtual bool Action(VesselWrapper VSL) { return false; }
		protected virtual string Title
		{
			get
			{
				var title = Name;
				if(Paused) title += " (paused)";
				return title;
			}
		}
		protected virtual void DrawThis() 
		{ 
			if(GUILayout.Button(Title, Edit? Styles.yellow_button : 
				(Active? Styles.green_button : Styles.normal_button), GUILayout.ExpandWidth(true)))
				Edit = !Edit;
		}

		public void Draw()
		{
			GUILayout.BeginVertical();
			GUILayout.BeginHorizontal();
			Condition.Draw();
			if(Edit && GUILayout.Button("Replace", Styles.red_button, GUILayout.Width(70)))
				SelectCondition = !SelectCondition;
			GUILayout.EndHorizontal();
			GUILayout.BeginHorizontal();
			GUILayout.Space(20); DrawThis();
			GUILayout.EndHorizontal();
			GUILayout.BeginHorizontal();
			GUILayout.Space(40);
			GUILayout.BeginVertical();
			var del = new List<int>();
			for(int i = 0, count = Children.Count; i < count; i++)
			{ 
				var child = Children[i];
				GUILayout.BeginHorizontal();
				child.Draw();
				if(Edit)
				{
					if(GUILayout.Button("^", Styles.normal_button, GUILayout.Width(20)))
						MoveUp(i);
					if(child.Children.Count < 2 &&
						GUILayout.Button("X", Styles.red_button, GUILayout.Width(20)))
						del.Add(i);
				}
				GUILayout.EndHorizontal();
			}
			if(Edit)
			{
				for(int i = 0, count = del.Count; i < count; i++) RemoveAt(del[i]);
				if(GUILayout.Button("Add Next", Styles.normal_button, GUILayout.ExpandWidth(true)))
					SelectNext = !SelectNext;
			}
			GUILayout.EndVertical();
			GUILayout.EndHorizontal();
			GUILayout.EndVertical();
		}

		public bool Execute(VesselWrapper VSL)
		{
			if(Paused) return true;
			if(Active) return Action(VSL);
			if(Condition.True(VSL))
			{
				Active = true;
				return Action(VSL);
			}
			return true;
		}

		public MacroNode Next(VesselWrapper VSL)
		{
			for(int i = 0, count = Children.Count; i < count; i++)
			{
				var child = Children[i];
				if(child.Condition.True(VSL)) return child;
			}
			return null;
		}

		public void CopyFrom(MacroNode mn)
		{
			var node = new ConfigNode();
			mn.Save(node);
			Load(node);
		}

		public MacroNode GetCopy()
		{
			var mn = new MacroNode();
			mn.CopyFrom(this);
			return mn;
		}

		public void Add(MacroNode child)
		{
			child.Parent = this;
			Children.Add(child);
		}

		public bool Remove(MacroNode child)
		{ return RemoveAt(Children.IndexOf(child)); }

		protected bool RemoveAt(int i)
		{
			if(i < 0 || i >= Children.Count) return false;
			var child = Children[i];
			if(child.Children.Count > 1) return false;
			if(child.HasChildren)
				Children[i] = child.Children[0];
			else Children.Remove(child);
			return true;
		}

		public bool MoveUp(int i)
		{
			if(i <= 0 || i >= Children.Count) return false;
			var child  = Children[i];
			var child1 = Children[i-1];
			Children[i-1] = child;
			Children[i] = child1;
			return true;
		}

		public bool MoveDown(int i)
		{
			if(i < 0 || i >= Children.Count-1) return false;
			var child  = Children[i];
			var child1 = Children[i+1];
			Children[i+1] = child;
			Children[i] = child1;
			return true;
		}
	}

	public class TCAMacro : MacroNode
	{
		[Persistent] public MacroNode Root;

		protected override bool Action(VesselWrapper VSL)
		{
			if(Root == null) return false;
			if(Root.Execute(VSL)) return true;
			if(Root.HasChildren) 
			{
				var next = Root.Next(VSL);
				if(next == null) return true;
				Root = next;
				return Root.Execute(VSL);
			}
			Root = null;
			return false;
		}

		protected override string Title 
		{
			get 
			{
				var title = Name;
				if(Root != null) title += " ["+Root.Name+"]";
				if(Paused) title += " (paused)";
				return title;
			}
		}

		protected override void DrawThis()
		{
			GUILayout.BeginVertical();
			if(Edit) 
			{
				GUILayout.BeginHorizontal();
				Name = GUILayout.TextField(Name, GUILayout.ExpandWidth(true), GUILayout.MinWidth(50));
				if(GUILayout.Button("Done", Styles.green_button, GUILayout.Width(40))) Edit = false;
				GUILayout.EndHorizontal();
			}
			else base.DrawThis();
			if(Edit && Root != null) 
			{
				GUILayout.BeginHorizontal();
				GUILayout.Space(20);
				Root.Draw();
				GUILayout.EndHorizontal();
			}
			GUILayout.EndVertical();
		}
	}

	public class TCAMacroEditor
	{
		public int Height = 400;
		public TCAMacro Macro;
		Vector2 scroll;

		public void Draw()
		{
			if(Macro == null) return;
			GUILayout.BeginVertical(Styles.white);
			scroll = GUILayout.BeginScrollView(scroll, GUILayout.Height(Height));
			Macro.Draw();
			GUILayout.EndScrollView();
			GUILayout.EndVertical();
		}


	}
}

