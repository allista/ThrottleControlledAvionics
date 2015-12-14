//  Author:
//       Allis Tauri <allista@gmail.com>
//
//  Copyright (c) 2015 Allis Tauri
//
// This work is licensed under the Creative Commons Attribution-ShareAlike 4.0 International License. 
// To view a copy of this license, visit http://creativecommons.org/licenses/by-sa/4.0/ 
// or send a letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.

﻿using System;
using System.IO;
using System.Linq;
using System.Collections.Generic;
using System.Text.RegularExpressions;

namespace ThrottleControlledAvionics
{
	public class MD2Unity
	{
		static readonly Regex section = new Regex(@"^\s*(#+)\s*(.*)");
		static readonly Regex italic  = new Regex(@"(\*|_)(.*?)\1");
		static readonly Regex bold    = new Regex(@"(\*\*|__)(.*?)\1");

		StreamReader file;
		public MD2Unity(StreamReader f) { file = f; }

		public MDSection Root;
		MDSection sec;

		public static string ConvertEmphasis(string line)
		{
			//replace bold and italics
			line = bold.Replace(line, "<b><color=yellow>$2</color></b>");
			line = italic.Replace(line, "<i><color=#55ffffff>$2</color></i>");
			return line;
		}

		public MDSection Parse()
		{
			Root = new MDSection("", 0);
			while(!file.EndOfStream)
			{
				var line = file.ReadLine();
				//check for section
				var match = section.Match(line);
				if(match.Success)
				{
					var level = match.Groups[1].Value.Length;
					var title = ConvertEmphasis(match.Groups[2].Value);
					if(sec == null) sec = Root.AddSubsection(title, level); //first section
					else if(level > sec.Level)
					{ sec.Flush(); sec = sec.AddSubsection(title, level); } //child
					else { sec.Flush(); sec = sec.ParentForLevel(level).AddSubsection(title, level); } //sibling or cousin
				}
				else sec.AddLine(line);
			}
			if(sec != null) sec.Flush();
			//if the document has a single top section, chroot to it
			if(Root.NoText && Root.Subsections.Count == 1)
			{
				Root = Root.Subsections[0];
				Root.Parent = null;
			}
			return Root;
		}

		public static MDSection Parse(StreamReader file)
		{
			var parser = new MD2Unity(file);
			return parser.Parse();
		}
	}

	public class Par
	{
		public virtual string Text { get; set; } = "";

		public void Reset() { Text = ""; }

		public virtual void AddLine(string line)
		{
			if(!string.IsNullOrEmpty(Text) && Text[Text.Length-1] != ' ') Text += " ";
			Text += line;
		}

		public override string ToString() { return this? Text+"\n\n" : ""; }

		public static implicit operator bool(Par p) { return p != null && !string.IsNullOrEmpty(p.Text); }
	}

	public class MDList : Par
	{
		public static readonly Regex item = new Regex(@"^(\s*)(\*|[0-9]+\.)\s+(.*)$");

		readonly List<Par> items = new List<Par>();
		public MDList Root { get { return Parent == null? this : Parent.Root; } }
		public MDList Parent;

		const string _indent = "    ";
		public readonly int OrigIndent = 0;
		public static string indent(int level = 1)
		{
			var ind = "";
			for(int i = 0; i < level; i++) ind += _indent;	
			return ind;
		}

		public int Level = 1;
		public override string Text
		{ get { return items.Aggregate("", (s, i) => s+(i is MDList? i.Text : indent(Level)+i.Text+"\n")); } }
		public override string ToString() { return Root.Text+"\n"; }

		public MDList(int ind) { OrigIndent = ind; }

		public Par AddItem()
		{
			var it = new Par();
			items.Add(it);
			return it;
		}

		public MDList AddSublist(int ind)
		{
			var lst = new MDList(ind);
			lst.Parent = this;
			lst.Level = Level+1;
			items.Add(lst);
			return lst;
		}
	}

	public class MDSection : Par
	{
		const string _hr = "\n--------------------------------------------------\n";
		public static readonly Regex hr = new Regex(@"^\s{0,3}\*\s{0,2}\*\s{0,2}\*\s*$");
		public static readonly Regex empty = new Regex(@"^\s*$");

		public readonly int Level = 1;
		public readonly List<Par> Pars = new List<Par>();

		public string Title = "";
		public MDSection Parent;
		public MDSection Root { get { return Parent == null? this : Parent.Root; } }
		public readonly List<MDSection> Subsections = new List<MDSection>();
		public bool NoText { get { return Pars.Count == 0; } }
		public bool NoTitle { get { return string.IsNullOrEmpty(Title); } }

		public MDSection(string title, int level)
		{ Title = title; Level = level; }

		Par par, it;
		MDList list;

		void new_par()
		{
			par = new Par();
			Pars.Add(par);
		}

		void new_list(int ind)
		{
			list = new MDList(ind);
			Pars.Add(list);
		}

		public MDSection AddSubsection(string title, int level)
		{
			var ss = new MDSection(title, level);
			ss.Parent = this;
			Subsections.Add(ss);
			return ss;
		}

		public MDSection ParentForLevel(int level)
		{
			if(Parent == null || level > Level) return null;
			return Parent.Level < level? Parent : Parent.ParentForLevel(level);
		}

		public void Flush() { par = null; list = null; it = null; }

		public override void AddLine(string line)
		{
			//end of a paragraph
			if(empty.IsMatch(line)) { Flush(); return; }
			//paragraph or list
			if(!par)
			{ //handle lists
				var match = MDList.item.Match(line);
				if(match.Success)
				{
					var ind = match.Groups[1].Value.Length;
					if(list == null) new_list(ind);
					else if(list.OrigIndent < ind) 
						list = list.AddSublist(ind);
					else if(list.OrigIndent > ind && list.Parent != null)
						list = list.Parent;
					it = list.AddItem();
					it.AddLine(match.Groups[2].Value+" "+MD2Unity.ConvertEmphasis(match.Groups[3].Value));
					return;
				}
				else if(it != null) { it.AddLine(MD2Unity.ConvertEmphasis(line)); return; }
			}
			//append the line to the current paragraph
			if(par == null) new_par();
			if(hr.IsMatch(line)) par.AddLine(_hr);
			else par.AddLine(MD2Unity.ConvertEmphasis(line));
		}

		public override string Text
		{
			get
			{
				var text = "";
				switch(Level)
				{
				case 0: break;
				case 1: text += string.Format("<size=25><b><color=green>{0}</color></b></size>\n\n", Title); break;
				case 2: text += string.Format("<size=20><b><color=green>{0}</color></b></size>\n\n", Title); break;
				case 3: text += string.Format("<size=15><b><color=green>{0}</color></b></size>\n\n", Title); break;
				default: text += string.Format("<b><color=green>{0}</color></b>\n\n", Title); break;
				}
				text += Pars.Aggregate("", (s, p) => s + p);
				text += Subsections.Aggregate("", (s, ss) => s + ss.Text);
				return text;
			}
		}
		public override string ToString() { return Text; }

		#if DEBUG
		public string ShowStructure()
		{
			var text = "";
			var indent = MDList.indent(Level);
			var pindent = indent+indent;
			text += indent+string.Format("Section: level {0}, title '{1}'\n", Level, Title);
			if(Subsections.Count > 0) 
				text += indent+Subsections.Aggregate("Subsections: ", (s, ss) => s + ss.Title+" : ")+"\n";
			text += Pars.Aggregate("", (s, p) => s+pindent+p.GetType().Name+":\n"+pindent+p);
			if(Subsections.Count > 0)
			{
				if(Pars.Count > 0) text += "\n";
				text += Subsections.Aggregate("", (s, ss) => s + ss.ShowStructure());
			}
			return text+"\n";
		}
		#endif
	}
}

