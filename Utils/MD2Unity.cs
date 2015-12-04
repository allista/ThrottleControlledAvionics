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
		static readonly Regex empty  = new Regex(@"^\s*$");
		static readonly Regex italic = new Regex(@"(\*|_)(.*)\1");
		static readonly Regex bold   = new Regex(@"(\*\*|__)(.*)\1");
		static readonly Regex item   = new Regex(@"(\s*)(\*|[0-9]+\.)\s*(.*)");

		StreamReader file;
		public MD2Unity(StreamReader f) { file = f; }

		readonly Par par = new Par();
		MDList list;
		Par it;

		string flush()
		{
			var text = "";
			if(par)
			{
				text += par;
				par.Reset();
			}
			else if(list != null)
			{
				text += list;
				list = null;
				it = null;
			}
			return text;
		}

		public string Parse()
		{
			var text = "";
			par.Reset(); list = null; it = null;
			while(!file.EndOfStream)
			{
				var line = file.ReadLine();
				//support only the first three levels of headers
				if(line.StartsWith("####")) line = line.TrimStart(new [] {'#'});
				//check header
				if(line.StartsWith("###"))
					text += string.Format("\n<size=15><b>{0}</b></size>\n\n", line.TrimStart(new [] {'#'}));
				else if(line.StartsWith("##"))
					text += string.Format("\n<size=20><b>{0}</b></size>\n\n", line.TrimStart(new [] {'#'}));
				else if(line.StartsWith("#"))
					text += string.Format("<size=25><b>{0}</b></size>\n\n", line.TrimStart(new [] {'#'}));
				//end of the paragraph or list
				else if(empty.IsMatch(line)) text += flush();
				else 
				{
					//replace bold and italics
					line = bold.Replace(line, "<b>$2</b>");
					line = italic.Replace(line, "<i>$2</i>");
					if(!par) //handle lists
					{
						var match = item.Match(line);
						if(match.Success)
						{
							var ind = match.Groups[1].Value.Length;
							if(list == null) list = new MDList(ind);
							else if(list.orig_indent < ind) 
								list = list.AddSublist(ind);
							else if(list.orig_indent > ind && list.parent != null)
								list = list.parent;
							it = list.AddItem();
							it.AddLine(match.Groups[2].Value+" "+match.Groups[3].Value);
							continue;
						}
						else if(it != null) { it.AddLine(line); continue; }
					}
					//append the line to the current paragraph
					par.AddLine(line);
				}
			}
			return text+flush();
		}

		public static string Parse(StreamReader file)
		{
			var parser = new MD2Unity(file);
			return parser.Parse();
		}

		internal class Par
		{
			internal virtual string text { get; set; } = "";

			internal void Reset() { text = ""; }

			internal void AddLine(string line)
			{
				if(!string.IsNullOrEmpty(text) && 
				   text[text.Length-1] != ' ') text += " ";
				text += line;
			}

			public override string ToString() { return text+"\n"; }

			public static implicit operator bool(Par p) { return !string.IsNullOrEmpty(p.text); }
		}

		internal class MDList : Par
		{
			readonly List<Par> items = new List<Par>();
			internal MDList root { get { return parent == null? this : parent.root; } }
			internal MDList parent;

			internal int orig_indent = 0;
			const string _indent = "    ";
			static string indent(int level = 1)
			{
				var ind = "";
				for(int i = 0; i < level; i++) ind += _indent;	
				return ind;
			}

			internal int level = 1;
			internal override string text
			{ get { return items.Aggregate("", (s, i) => s+(i is MDList? i.text : indent(level)+i.text+"\n")); } }
			public override string ToString() { return root.text; }

			public MDList(int ind) { orig_indent = ind; }

			internal Par AddItem()
			{
				var it = new Par();
				items.Add(it);
				return it;
			}

			internal MDList AddSublist(int ind)
			{
				var lst = new MDList(ind);
				lst.parent = this;
				lst.level = level+1;
				items.Add(lst);
				return lst;
			}
		}
	}
}

