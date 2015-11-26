//  Author:
//       Allis Tauri <allista@gmail.com>
//
//  Copyright (c) 2015 Allis Tauri
//
// This work is licensed under the Creative Commons Attribution-ShareAlike 4.0 International License. 
// To view a copy of this license, visit http://creativecommons.org/licenses/by-sa/4.0/ 
// or send a letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.

using System;
using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using System.Reflection;
using System.Linq;

namespace ThrottleControlledAvionics
{
	[AttributeUsage (AttributeTargets.Class | AttributeTargets.Property | AttributeTargets.Field)]
	public class UI_IntRange : UI_Control
	{
		public int minValue;
		public int maxValue = 10;
		public int stepIncrement = 1;

		public static string UIControlName = "Integer Range";

		static UI_IntRange() {}

		protected static bool ParseInt(out int value, ConfigNode node, string valueName, string FieldUIControlName)
		{
			value = 0;
			var sval = node.GetValue(valueName);
			if (sval == null) 
			{
				Utils.Log("Config of {0} has no value {1}.", FieldUIControlName, valueName);
				return false;
			}
			if (!int.TryParse(sval, out value)) 
			{
				Utils.Log("Saved {0} of {1}: {2} cannot be parsed as int.", valueName, FieldUIControlName, sval);
				return false;
			}
			return true;
		}

		public override void Load(ConfigNode node, object host)
		{
			base.Load(node, host);
			if(!ParseInt(out this.minValue, node, "minValue", UIControlName))
				return;
			if(!ParseInt(out this.maxValue, node, "maxValue", UIControlName))
				return;
			if(!ParseInt(out this.stepIncrement, node, "stepIncremet", UIControlName))
				return;
		}

		public override void Save (ConfigNode node, object host)
		{
			base.Save(node, host);
			node.AddValue("minValue", this.minValue.ToString ());
			node.AddValue("maxValue", this.maxValue.ToString ());
			node.AddValue("stepIncremet", this.stepIncrement.ToString ());
		}
	}

	[UI_IntRange]
	public class UIPartActionIntRange : UIPartActionFieldItem
	{
		public UIProgressSlider slider;
		public SpriteText fieldName, fieldValue;
		public UIButton incSmallDown, incSmallUp;
		public SpriteText incSmallDownLabel, incSmallUpLabel;
		int value;

		protected UI_IntRange progBarControl { get { return (UI_IntRange)control; } }

		int GetFieldValue()
		{ return isModule? field.GetValue<int>(partModule) : field.GetValue<int>(part); }

		void OnValueChanged(IUIObject obj)
		{
			var slider_value = Mathf.RoundToInt(Mathf.Lerp((float)progBarControl.minValue, (float)progBarControl.maxValue, slider.Value));
			var reminder = slider_value % progBarControl.stepIncrement;
			if(reminder > 0) 
				value = reminder < progBarControl.stepIncrement * 0.5? 
					slider_value - reminder : 
					slider_value + progBarControl.stepIncrement - reminder;
			else value = slider_value;
			slider.Value = Mathf.InverseLerp(progBarControl.minValue, progBarControl.maxValue, value);
			field.SetValue(value, field.host);
			if((control.affectSymCounterparts & scene) != (UI_Scene)0)
				SetSymCounterpartValue(value);
			if (HighLogic.LoadedSceneIsEditor)
				GameEvents.onEditorShipModified.Fire(EditorLogic.fetch.ship);
		}

		void OnButton(bool up)
		{
			value = Mathf.Clamp(value + (up? progBarControl.stepIncrement : -progBarControl.stepIncrement), 
			                    progBarControl.minValue, progBarControl.maxValue);
			slider.Value = Mathf.InverseLerp(progBarControl.minValue, progBarControl.maxValue, value);
			field.SetValue(value, field.host);
			if((control.affectSymCounterparts & scene) != (UI_Scene)0)
				SetSymCounterpartValue(value);
			if (HighLogic.LoadedSceneIsEditor)
				GameEvents.onEditorShipModified.Fire(EditorLogic.fetch.ship);
		}

		void SetSliderValue(float rawValue)
		{ slider.Value = Mathf.Clamp01((rawValue - progBarControl.minValue)/(progBarControl.maxValue - progBarControl.minValue)); }

		public override void Setup(UIPartActionWindow window, Part part, PartModule partModule, UI_Scene scene, UI_Control control, BaseField field)
		{
			base.Setup(window, part, partModule, scene, control, field);
			SetSliderValue(GetFieldValue());
			incSmallUp.SetValueChangedDelegate(obj => OnButton(true));
			incSmallDown.SetValueChangedDelegate(obj => OnButton(false));
			slider.AddValueChangedDelegate(new EZValueChangedDelegate(OnValueChanged));
			slider.ignoreDefault = true;
		}

		public override void UpdateItem()
		{
			value = GetFieldValue ();
			fieldName.Text = field.guiName;
			fieldValue.Text = value.ToString(field.guiFormat);
			SetSliderValue(value);
		}

		//adapted from KSPApiExtensions by Swamp-Ig
		public static UIPartActionIntRange CreateTemplate()
		{
			// Create the control
			var editGo = new GameObject("UIPartActionFloatEdit", typeof(UIPartActionIntRange));
			UIPartActionIntRange edit = editGo.GetComponent<UIPartActionIntRange>();
			editGo.SetActive(false);

			UIPartActionButton evtp = UIPartActionController.Instance.eventItemPrefab;
			GameObject srcTextGo = evtp.transform.Find("Text").gameObject;
			GameObject srcBackgroundGo = evtp.transform.Find("Background").gameObject;
			GameObject srcButtonGo = evtp.transform.Find("Btn").gameObject;

			var paFlt = (UIPartActionFloatRange)UIPartActionController.Instance.fieldPrefabs.Find(cls => cls.GetType() == typeof(UIPartActionFloatRange));
			GameObject srcSliderGo = paFlt.transform.Find("Slider").gameObject;

			// Start building our control
			var backgroundGo = (GameObject)Instantiate(srcBackgroundGo);
			backgroundGo.transform.parent = editGo.transform;
			//slider
			var sliderGo = (GameObject)Instantiate(srcSliderGo);
			sliderGo.transform.parent = editGo.transform;
			sliderGo.transform.localScale = new Vector3(0.85f, 1, 1);
			edit.slider = sliderGo.GetComponent<UIProgressSlider>();
			edit.slider.ignoreDefault = true;
			//name
			var fieldNameGo = (GameObject)Instantiate(srcTextGo);
			fieldNameGo.transform.parent = editGo.transform;
			fieldNameGo.transform.localPosition = new Vector3(20, -8, 0);
			edit.fieldName = fieldNameGo.GetComponent<SpriteText>();
			//value
			var fieldValueGo = (GameObject)Instantiate(srcTextGo);
			fieldValueGo.transform.parent = editGo.transform;
			fieldValueGo.transform.localPosition = new Vector3(150, -8, 0);
			edit.fieldValue = fieldValueGo.GetComponent<SpriteText>();
			//down button
			var incSmallDownGo = (GameObject)Instantiate(srcButtonGo);
			incSmallDownGo.transform.parent = edit.transform;
			incSmallDownGo.transform.localScale = new Vector3(0.35f, 1.1f, 1f);
			incSmallDownGo.transform.localPosition = new Vector3(11, -9, 0); // <31.5
			edit.incSmallDown = incSmallDownGo.GetComponent<UIButton>();
			//down label
			var incSmallDownLabelGo = (GameObject)Instantiate(srcTextGo);
			incSmallDownLabelGo.transform.parent = editGo.transform;
			incSmallDownLabelGo.transform.localPosition = new Vector3(7.5f, -7, 0); //<28
			edit.incSmallDownLabel = incSmallDownLabelGo.GetComponent<SpriteText>();
			edit.incSmallDownLabel.Text = "<";
			//up button
			var incSmallUpGo = (GameObject)Instantiate(srcButtonGo);
			incSmallUpGo.transform.parent = edit.transform;
			incSmallUpGo.transform.localScale = new Vector3(0.35f, 1.1f, 1f);
			incSmallUpGo.transform.localPosition = new Vector3(190, -9, 0);
			edit.incSmallUp = incSmallUpGo.GetComponent<UIButton>();
			//up label
			var incSmallUpLabelGo = (GameObject)Instantiate(srcTextGo);
			incSmallUpLabelGo.transform.parent = editGo.transform;
			incSmallUpLabelGo.transform.localPosition = new Vector3(187.5f, -7, 0); //<168
			edit.incSmallUpLabel = incSmallUpLabelGo.GetComponent<SpriteText>();
			edit.incSmallUpLabel.Text = ">";

			return edit;
		}
	}

	//adapted from KSPApiExtensions by Swamp-Ig
	[KSPAddon(KSPAddon.Startup.Instantly, true)]
	class UIPartActionsExtendedRegistration : MonoBehaviour
	{
		static bool loaded;
		bool isRunning;

		public void Start()
		{
			if(loaded)
			{
				// prevent multiple copies of same object
				Destroy(gameObject);
				return;
			}
			loaded = true;
			DontDestroyOnLoad(gameObject);
		}

		public void OnLevelWasLoaded(int level)
		{
			if(isRunning) StopCoroutine("Register");
			if(!HighLogic.LoadedSceneIsEditor && !HighLogic.LoadedSceneIsFlight) return;
			isRunning = true;
			StartCoroutine("Register");
		}

		internal IEnumerator Register()
		{
			UIPartActionController controller;
			while((controller = UIPartActionController.Instance) == null)
				yield return false;
			FieldInfo typesField = (from fld in controller.GetType().GetFields(BindingFlags.NonPublic | BindingFlags.Instance)
			                    	where fld.FieldType == typeof(List<Type>)
			                    	select fld).First();
			List<Type> fieldPrefabTypes;
			while((fieldPrefabTypes = (List<Type>)typesField.GetValue(controller)) == null
			      || fieldPrefabTypes.Count == 0 
			      || !UIPartActionController.Instance.fieldPrefabs.Find(cls => cls.GetType() == typeof(UIPartActionFloatRange)))
				yield return false;
			Utils.Log("Registering field prefabs.");
			// Register prefabs. This needs to be done for every version of the assembly. (the types might be called the same, but they aren't the same)
			controller.fieldPrefabs.Add(UIPartActionIntRange.CreateTemplate());
			fieldPrefabTypes.Add(typeof(UI_IntRange));

			isRunning = false;
		}
	}
}

