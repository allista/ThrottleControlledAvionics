//  Author:
//       Allis Tauri <allista@gmail.com>
//
//  Copyright (c) 2015 Allis Tauri
//
// This work is licensed under the Creative Commons Attribution-ShareAlike 4.0 International License. 
// To view a copy of this license, visit http://creativecommons.org/licenses/by-sa/4.0/ 
// or send a letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.

namespace ThrottleControlledAvionics
{
	[RequireModules(null)]
	public class MacroProcessor : TCAModule
	{
		public MacroProcessor(ModuleTCA tca) : base(tca) {}

		protected override void UpdateState()
		{ IsActive = CFG.MacroIsActive && CFG.SelectedMacro != null; }

		protected override void Update()
		{
			if(!IsActive) return;
			CFG.MacroIsActive &= CFG.SelectedMacro.Execute(VSL);
			if(!CFG.MacroIsActive) CFG.SelectedMacro.Rewind();
		}
	}
}

