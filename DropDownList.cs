//This code is based on code from Extraplanetary Launchpads plugin. DropDownList class.
//And there it was taken from TriggerAu's plugin framework
using System;
using System.Collections.Generic;
using UnityEngine;

namespace ThrottleControlledAvionics 
{
    public class DropDownList
    {
        //properties to use
		string widest_item;
		List<string> items = new List<string>();
		internal List<string> Items 
		{ 
			get { return items; }
			set
			{
				items = value;
				widest_item = "";
				foreach(string i in items) 
				{ if(i.Length > widest_item.Length) widest_item = i; }
			}
		}
        internal int SelectedIndex { get; private set; }
        internal string SelectedValue { get { return Items[SelectedIndex]; } }

        internal bool ListVisible;

		internal Rect windowRect;
        Rect rectButton = new Rect();
        Rect rectListBox = new Rect();
		Rect rectScrollView = new Rect();
		Vector2 scroll_view;

        internal GUIStyle styleListItem = new GUIStyle();
        internal GUIStyle styleListBox = new GUIStyle();
        internal GUIStyle styleListBlocker = new GUIStyle();
        internal int ListItemHeight = 25;

        //Constructors
        public DropDownList()
        {
            ListVisible = false;
            SelectedIndex = 0;
			Items = new List<string>();
        }
		public DropDownList(List<String> Items, int SelectedIndex = 0) : this() 
		{ this.Items = Items; this.SelectedIndex =  SelectedIndex; }

		public void SelectItem (int index)
		{
			if(Items.Count < 1 || index < 0) index = 0;
			else if(index >= Items.Count) index = Items.Count - 1;
			SelectedIndex = index;
		}

        //Draw the button behind everything else to catch the first mouse click
        internal void DrawBlockingSelector()
        {
            //do we need to draw the blocker
            if(ListVisible)
            {
                //This will collect the click event before any other controls under the listrect
				if(GUI.Button(rectScrollView, "", styleListBlocker))
                {
					SelectedIndex = (int)Math.Floor((Event.current.mousePosition.y - rectScrollView.y + scroll_view.y) / ListItemHeight);
					if(SelectedIndex >= Items.Count) SelectedIndex = Items.Count - 1;
                    ListVisible = false;
                }
            }
        }

        //Draw the actual button for the list
        internal bool DrawButton()
        {
            bool blnReturn = false;
            //this is the dropdown button - toggle list visible if clicked
			GUILayout.BeginHorizontal();
			Vector2 button_size = styleListBox.CalcSize(new GUIContent(widest_item));
			if(GUILayout.Button(SelectedValue, styleListBox, GUILayout.MinWidth(button_size.x)))
            {
                ListVisible = !ListVisible;
                blnReturn = true;
            }
            //get the drawn button rectangle
            if (Event.current.type == EventType.repaint)
            	rectButton = GUILayoutUtility.GetLastRect();

            //draw a dropdown symbol on the right edge
			if(GUILayout.Button("\\/", styleListBox, GUILayout.MaxWidth(20)))
			{
				ListVisible = !ListVisible;
				blnReturn = true;
			}
			GUILayout.EndHorizontal();
            return blnReturn;
        }

        //Draw the hovering dropdown
        internal void DrawDropDown()
        {
            if (ListVisible)
            {
                //work out the list of items box
				rectScrollView = new Rect(rectButton)
				{
					y = rectButton.y + rectButton.height,
					width  = rectButton.width + GUI.skin.verticalScrollbar.fixedWidth+1,
					height = windowRect != default(Rect)? 
						windowRect.height - rectButton.y - rectButton.height : ListItemHeight * 2 + 1
				};
                rectListBox = new Rect(rectButton)
                {
					x = 0, y = 0,
                    height = Items.Count * ListItemHeight
                };
                //and draw it
				scroll_view = GUI.BeginScrollView(rectScrollView, scroll_view, rectListBox);
				GUI.Box(rectListBox, "", styleListBox);
                //now draw each listitem
                for (int i = 0; i < Items.Count; i++)
                {
                    Rect ListButtonRect = new Rect(rectListBox) { y = rectListBox.y + (i * ListItemHeight), height = ListItemHeight };
                    if(GUI.Button(ListButtonRect, Items[i], styleListItem))
                    {
                        ListVisible = false;
                        SelectedIndex = i;
                    }
                }
				GUI.EndScrollView();
                //maybe put this here to limit what happens in pre/post calls
                //CloseOnOutsideClick();
            }

        }

        internal bool CloseOnOutsideClick()
        {
			if(ListVisible && Event.current.type == EventType.mouseDown && !rectScrollView.Contains(Event.current.mousePosition))
            {
                ListVisible = false;
                return true;
            }
            return false;
        }
    }
}
