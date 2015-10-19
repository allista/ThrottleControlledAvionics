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

namespace ThrottleControlledAvionics
{
	public class Condition : TypedConfigNodeObject
	{
		[Persistent] public bool or;
		[Persistent] public Condition Next;
		[Persistent] readonly PersistentBaseList<Condition> Alternatives = new PersistentBaseList<Condition>();

		protected static TCAGlobals GLB { get { return TCAScenario.Globals; } }

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

		public virtual void Draw() {}
		protected virtual bool Evaluate(VesselWrapper VSL) { return false; }

		public void AddAlternative(Condition alt) { Alternatives.Add(alt); }
		public bool RemoveAlternative(Condition alt) { return Alternatives.Remove(alt); }

		public void Or(Condition next) { or = true; Next = next; }
		public void And(Condition next) { or = false; Next = next; }
	}

	public class MacroNode : TypedConfigNodeObject
	{
		[Persistent] public string Name;
		[Persistent] public Condition Condition = new TrueCondition();
		[Persistent] public bool Active;
		[Persistent] public MacroNode Parent;
		[Persistent] public PersistentBaseList<MacroNode> Children = new PersistentBaseList<MacroNode>();
		public bool HasChildren { get { return Children.Count > 0; } }

		public VesselWrapper VSL;
		protected VesselConfig CFG { get { return VSL.CFG; } }
		protected static TCAGlobals GLB { get { return TCAScenario.Globals; } }

		public virtual bool Action() { return false; }
		public virtual void Draw() {}

		public bool Execute()
		{
			if(Active) return Action();
			if(Condition.True(VSL))
			{
				Active = true;
				return Action();
			}
			return true;
		}

		public MacroNode Next
		{
			get
			{
				for(int i = 0, count = Children.Count; i < count; i++)
				{
					var child = Children.List[i];
					if(child.Condition.True(VSL)) return child;
				}
				return null;
			}
		}
	}

	public class TCAMacro : MacroNode
	{
		[Persistent] public MacroNode Root;

		public override bool Action()
		{
			if(Root == null) return false;
			if(Root.Execute()) return true;
			if(Root.HasChildren) 
			{
				var next = Root.Next;
				if(next == null) return true;
				Root = next;
				return Root.Execute();
			}
			Root = null;
			return false;
		}
	}
}

