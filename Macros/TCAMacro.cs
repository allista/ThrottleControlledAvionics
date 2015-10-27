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
		public delegate void Select(Action<Condition> callback);

		protected static TCAGlobals GLB { get { return TCAScenario.Globals; } }

		[Persistent] public bool or;
		[Persistent] public bool not;
		[Persistent] public Condition Prev;
		[Persistent] public Condition Next;
		[Persistent] readonly PersistentBaseList<Condition> Alternatives = new PersistentBaseList<Condition>();
		public bool HasAlternatives { get { return Alternatives.Count > 0; } }

		public bool Edit;
		public Select SelectCondition;

		public void SetSelector(Select selector)
		{
			SelectCondition = selector;
			for(int i = 0, count = Alternatives.Count; i < count; i++) 
				Alternatives[i].SetSelector(selector);
			if(Next != null) Next.SetSelector(selector);
		}

		public bool True(VesselWrapper VSL)
		{ 
			var value = Evaluate(VSL);
			if(not) value = !value;
			for(int i = 0, count = Alternatives.Count; i < count; i++) 
				value |= Alternatives[i].True(VSL);
			if(Next != null)
				return or? 
					value || Next.True (VSL) : 
					value && Next.True (VSL);
			return value;
		}

		protected virtual bool Evaluate(VesselWrapper VSL) { return false; }
		protected virtual void DrawThis() {}

		public void Draw()
		{
			if(Prev == null) GUILayout.Label("IF:", Styles.label, GUILayout.Width(20));
			GUILayout.BeginHorizontal(Styles.white);
			GUILayout.Label("(", Styles.label, GUILayout.Width(20));
			if(GUILayout.Button(not? "NOT" : "", Styles.normal_button, GUILayout.Width(25))) not = !not;
			DrawThis();
			if(Alternatives.Count > 0) 
			{
				var del = new List<int>();
				for(int i = 0, count = Alternatives.Count; i < count; i++) 
				{ 
					Alternatives[i].Draw();
					if(GUILayout.Button("x", Styles.red_button, GUILayout.Width(20)))
						del.Add(i);
					if(i < count-1) GUILayout.Label("OR"); 
				}
				for (int i = 0, count = del.Count; i < count; i++) 
					Alternatives.List.RemoveAt(del[i]);
			}
			if(GUILayout.Button(new GUIContent(")", "Add a condition inside the braces"), 
				Styles.normal_button, GUILayout.Width(20)))
			{ if(SelectCondition != null) SelectCondition(AddAlternative); }
			if(Prev != null && GUILayout.Button("X", Styles.red_button, GUILayout.Width(20)))
				Delete();
			GUILayout.EndHorizontal();
			if(Next != null) 
			{ 
				if(GUILayout.Button(or? "OR" : "AND", Styles.normal_button, GUILayout.Width(20))) or = !or;
				Next.Draw();
			}
			else 
			{ 
				if(GUILayout.Button("+", Styles.green_button, GUILayout.Width(20))) 
				{ if(SelectCondition != null) SelectCondition(And); }
			}
		}

		public void AddAlternative(Condition alt) { Alternatives.Add(alt); }
		public bool RemoveAlternative(Condition alt) { return Alternatives.Remove(alt); }
		public bool Delete()
		{
			if(Prev == null) return false;
			if(Next == null) { Prev.Next = null; return true; }
			Prev.Next = Next; Next.Prev = Prev;
			return true;
		}

		public void Or(Condition next) { or = true; Next = next; Next.Prev = this; }
		public void And(Condition next) { or = false; Next = next; Next.Prev = this; }
	}

	public class MacroNode : TypedConfigNodeObject
	{
		public delegate void Select(Action<MacroNode> callback);

		protected static TCAGlobals GLB { get { return TCAScenario.Globals; } }

		[Persistent] public string Name;
		[Persistent] public Condition Condition = new TrueCondition();
		[Persistent] public bool Active;
		[Persistent] public bool Paused;
		[Persistent] public bool Done;
		[Persistent] public MacroNode Parent;
		[Persistent] public PersistentBaseList<MacroNode> Children = new PersistentBaseList<MacroNode>();
		public bool HasChildren { get { return Children.Count > 0; } }

		public bool Edit;
		public Select SelectNode;

		public virtual void SetSelector(Select selector)
		{
			SelectNode = selector;
			for(int i = 0, count = Children.Count; i < count; i++) 
				Children[i].SetSelector(selector);
		}

		public virtual void SetConditionSelector(Condition.Select selector)
		{
			Condition.SetSelector(selector);
			for(int i = 0, count = Children.Count; i < count; i++) 
				Children[i].SetConditionSelector(selector);
		}

		/// <summary>
		/// Perform the Action on a specified VSL.
		/// Returns true if it is not finished and should be called on next update. 
		/// False if it is finished.
		/// </summary>
		/// <param name="VSL">VesselWrapper</param>
		protected virtual bool Action(VesselWrapper VSL) { return false; }
		public virtual string Title
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
			if(Edit && Condition.Next == null && !Condition.HasAlternatives &&
				GUILayout.Button("Replace", Styles.red_button, GUILayout.Width(70)))
			{ if(SelectNode != null) Condition.SelectCondition((cnd) => Condition = cnd); }
			GUILayout.EndHorizontal();
			GUILayout.BeginHorizontal();
			if(Children.Count < 2 && Parent != null)
			{ if(GUILayout.Button("X", Styles.red_button, GUILayout.Width(20))) Parent.Remove(this); }
			else GUILayout.Space(20); 
			DrawThis();
			if(Active && GUILayout.Button("||", Paused? Styles.green_button : Styles.yellow_button, GUILayout.Width(20)))
				Paused = !Paused;
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
				{ if(SelectNode != null) SelectNode(Add); }
			}
			GUILayout.EndVertical();
			GUILayout.EndHorizontal();
			GUILayout.EndVertical();
		}

		public bool Execute(VesselWrapper VSL)
		{
			if(Done) return false;
			if(Paused) return true;
			Active |= Condition.True(VSL);
			Done = Active && !Action(VSL);
			Active &= !Done;
			return !Done;
		}

		public virtual void Rewind()
		{
			Active = Done = Paused = false;
			for (int i = 0, count = Children.Count; i < count; i++) 
				Children[i].Rewind ();
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
			var constInfo = GetType().GetConstructor(null);
			if(constInfo == null) return null;
			var mn = (MacroNode)constInfo.Invoke(null);
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

	public abstract class ProxyMacro : MacroNode
	{
		[Persistent] public MacroNode Macro;

		public override void SetSelector(Select selector)
		{
			Macro.SetSelector(selector);
			base.SetSelector(selector);
		}

		public override void SetConditionSelector(Condition.Select selector)
		{
			Macro.SetConditionSelector(selector);
			base.SetConditionSelector(selector);
		}

		public override void Rewind ()
		{
			Macro.Rewind();
			base.Rewind ();
		}
	}

	public class TCAMacro : ProxyMacro
	{
		[Persistent] public MacroNode Current;

		public override void Rewind ()
		{
			Current = null;
			Macro.Rewind();
			base.Rewind();
		}

		protected override bool Action(VesselWrapper VSL)
		{
			if(Macro == null) return false;
			if(Current == null) Current = Macro;
			if(Current.Execute(VSL)) return true;
			if(Current.HasChildren) 
			{
				var next = Current.Next(VSL);
				if(next != null)
				{
					Current.Active = false;
					Current = next;
					Current.Execute(VSL);
				}
				return true;
			}
			return false;
		}

		public override string Title 
		{
			get 
			{
				var title = Name;
				if(Current != null) title += " ["+Current.Name+"]";
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
				Edit &= !GUILayout.Button ("Done", Styles.green_button, GUILayout.Width (40));
				GUILayout.EndHorizontal();
			}
			else base.DrawThis();
			if(Edit) 
			{
				GUILayout.BeginHorizontal();
				if(Macro != null)
				{
					if(GUILayout.Button("X", Styles.red_button, GUILayout.Width(20))) Macro = null;
					else Macro.Draw();
				}
				else 
				{
					GUILayout.Space(20);
					if(Edit && GUILayout.Button("Select First Action", Styles.normal_button, GUILayout.ExpandWidth(true)))
					{ if(SelectNode != null) SelectNode((n) => Macro = n); }
				}
				GUILayout.EndHorizontal();
			}
			GUILayout.EndVertical();
		}
	}

	public class TCAMacroLibrary : TypedConfigNodeObject
	{
		[Persistent] public PersistentBaseList<TCAMacro> DB = new PersistentBaseList<TCAMacro>();

		public void SaveMacro(TCAMacro macro, bool overwrite = false)
		{
			var old_macro = DB.List.FindIndex(m => m.Name == macro.Name);
			if(old_macro < 0) 
			{ 
				DB.Add((TCAMacro)macro.GetCopy()); 
				DB.List.Sort((a,b) => a.Name.CompareTo(b.Name)); 
			}
			else if(overwrite) DB[old_macro] = (TCAMacro)macro.GetCopy();
		}

		public bool Remove(TCAMacro macro) { return DB.List.Remove(macro); }
		public TCAMacro Get(string name) { return DB.List.Find(m => m.Name == name); }

		Vector2 scroll;
		public bool Selector(out TCAMacro macro)
		{
			var ret = false;
			macro = null;
			scroll = GUILayout.BeginScrollView(scroll, Styles.white, GUILayout.Height(110));
			GUILayout.BeginVertical();
			foreach(var m in DB.List)
			{
				if(GUILayout.Button(m.Name, Styles.normal_button, GUILayout.ExpandWidth(true)))
				{ macro = (TCAMacro)m.GetCopy(); ret = true; }
			}
			GUILayout.EndVertical();
			GUILayout.EndScrollView();
			return ret;
		}

		public bool Selector(out MacroNode macro)
		{
			macro = null;
			TCAMacro m = null;
			if(Selector(out m)) 
			{ macro = m; return true; }
			return false;
		}
	}
}

