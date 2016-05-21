//  Author:
//       Allis Tauri <allista@gmail.com>
//
//  Copyright (c) 2016 Allis Tauri
//
// This work is licensed under the Creative Commons Attribution-ShareAlike 4.0 International License. 
// To view a copy of this license, visit http://creativecommons.org/licenses/by-sa/4.0/ 
// or send a letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.
//

namespace ThrottleControlledAvionics
{
	public abstract class ControlPanel : DrawableComponent
	{
		protected ControlPanel(ModuleTCA tca) : base(tca) { InitModuleFields(); }

		public virtual void Update() {}
		public virtual void Reset() {}

		protected void follow_me() { if(SQD != null) SQD.FollowMe(); }
	}
}

