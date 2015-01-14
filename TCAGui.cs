/* The GUI class of THCA.
 * Author: Quinten Feys & Willem van Vliet
 * License: BY: Attribution-ShareAlike 3.0 Unported (CC BY-SA 3.0): http://creativecommons.org/licenses/by-sa/3.0/
 */
using UnityEngine;
using KSPPluginFramework;

namespace ThrottleControlledAvionics
{
	public class TCAGui : MonoBehaviourWindow
	{
		private ThrottleControlledAvionics TCA;
		private Vessel vessel;

		#region GUI
		bool showEngines;
		bool showHelp;
		bool showAdvanced;
		bool showDebugLines;
		bool showDebugLines2;

		Vector2 positionScrollViewEngines;
		public const int controlsWidth = 400, controlsHeight = 220;
		public const int helpWidth = 500, helpHeight = 100;
		public const int advancedWidth = 200, extendedHeight = 400;
		const string ICON_ON  = "ThrottleControlledAvionics/Icons/icon_button_on";
		const string ICON_OFF = "ThrottleControlledAvionics/Icons/icon_button_off";
		const string ICON_NC  = "ThrottleControlledAvionics/Icons/icon_button_noCharge";

		IButton TCAToolbarButton;
		ApplicationLauncherButton TCAButton;

		Texture textureOn;
		Texture textureOff;
		Texture textureNoCharge;

		private Material diffuseMat;
		private LineRenderer upLine;
		private LineRenderer vecLine;
		private LineRenderer engLine;
		private LineRenderer dirLine;
		private LineRenderer gravLine;
		#endregion

		internal override void Awake()
		{
			base.Awake ();
			TCA = this.gameObject.GetComponent<ThrottleControlledAvionics> ();
			if(ToolbarManager.ToolbarAvailable)
			{
				TCAToolbarButton = ToolbarManager.Instance.add("ThrottleControlledAvionics", "ThrottleControlledAvionicsButton");
				TCAToolbarButton.TexturePath = ICON_OFF;
				TCAToolbarButton.ToolTip     = "Throttle Controlled Avionics";
				TCAToolbarButton.Visibility  = new GameScenesVisibility(GameScenes.FLIGHT);
				TCAToolbarButton.Visible     = true;
				TCAToolbarButton.OnClick += e => onGUIToggle ();
			}
			else 
			{
				textureOn = GameDatabase.Instance.GetTexture(ICON_ON, false);
				textureOff = GameDatabase.Instance.GetTexture(ICON_OFF, false);
				textureNoCharge = GameDatabase.Instance.GetTexture(ICON_NC, false);
				GameEvents.onGUIApplicationLauncherReady.Add(OnGUIAppLauncherReady);
			}
			GameEvents.onHideUI.Add(onHideUI);
			GameEvents.onShowUI.Add(onShowUI);

			WindowCaption = "Throttle Controlled Avionics";
			TCAConfiguration.Globals.ControlsPos = new Rect(50, 100, controlsWidth, controlsHeight); //this is rather a hack. it should be saved in the vessel. but it's like a browser's cahche.
			WindowRect = TCAConfiguration.Globals.ControlsPos;
			Visible = false;
			DragEnabled = true;
			TooltipsEnabled = true;

			diffuseMat = new Material (Shader.Find ("Particles/Additive")); //Unlit/Texture
		}

		//when the main class gets a vessel, we read the vessel's config to see if we need to be visible or not.
		public void onVessel(Vessel _vessel) {
			Visible = TCA.CFG.GUIVisible;
			vessel = _vessel;
		}

		#region GUI methods
		internal override void DrawWindow(int id)
		{
			Debug.Log ("[TCA] rect " + WindowRect.ToString ());
			if (GUI.Button(new Rect(10f, 5f, 18f, 20f), "?")) showHelp = !showHelp;
			if (GUI.Button(new Rect(WindowRect.width - 30f, 5f, 18f, 20f), "D")) showAdvanced = !showAdvanced;

			GUILayout.BeginHorizontal();
			GUILayout.BeginVertical(GUILayout.Width(controlsWidth-20));
			GUILayout.BeginHorizontal();
			TCA.ActivateTCA(GUILayout.Toggle(TCA.CFG.Enabled, "Toggle TCA"));
			if(!TCA.haveEC)	GUILayout.Label("WARNING! no electric charge!", GUILayout.ExpandWidth(false));
			GUILayout.EndHorizontal();

			GUILayout.BeginHorizontal();
			GUILayout.Label("Smoothness: ", GUILayout.ExpandWidth(false));
			GUILayout.Label(TCA.CFG.StabilityCurve.ToString("F1"), GUILayout.ExpandWidth(false));
			TCA.CFG.StabilityCurve = GUILayout.HorizontalSlider(TCA.CFG.StabilityCurve, 0f, 2f);
			GUILayout.EndHorizontal();

			TCA.CFG.Torque.DrawPIControls("Torque");
			TCA.CFG.Steering.DrawPIControls("Steering");
			TCA.CFG.Engines.DrawPIControls("Engines");

			GUILayout.BeginHorizontal();
			GUILayout.Label("Vertical Speed Limit: ", GUILayout.ExpandWidth(false));
			GUILayout.Label(TCA.CFG.VerticalCutoff >= TCAConfiguration.Globals.MaxCutoff? "inf." :
				TCA.CFG.VerticalCutoff.ToString("F1"), GUILayout.ExpandWidth(false));
			TCA.CFG.VerticalCutoff = GUILayout.HorizontalSlider(TCA.CFG.VerticalCutoff, 
				-TCAConfiguration.Globals.MaxCutoff, 
				TCAConfiguration.Globals.MaxCutoff);
			GUILayout.EndHorizontal();
			//engines info
			showEngines = GUILayout.Toggle(showEngines, "Show/Hide Engines Information");
			if (showEngines)
			{
				positionScrollViewEngines = GUILayout.BeginScrollView (positionScrollViewEngines, GUILayout.Height (extendedHeight));
				foreach (var e in TCA.engines) {
					if (!e.Valid)
						continue;
					GUILayout.Label (e.getName () + "\n" +
						string.Format (
							"Torque: {0}\n" +
							"Thrust Dir: {1}\n" +
							"Efficiency: {2:P1}\n" +
							"Thrust: {3:F1}%",
							e.currentTorque, e.thrustDirection,
							e.efficiency, e.thrustPercentage));
				}
				GUILayout.EndScrollView ();
			} 

			GUILayout.EndVertical();


			if (showAdvanced) {
				GUILayout.BeginVertical(GUILayout.Width(advancedWidth));
				showDebugLines = GUILayout.Toggle(showDebugLines, "Show/Hide Debug Lines");
				showDebugLines2 = GUILayout.Toggle(showDebugLines2, "Show/Hide Debug Lines 2");
				if (GUILayout.Button ("Kill Lines"))
					killLines ();
				GUILayout.EndVertical();
			}
			GUILayout.EndHorizontal();
			GUI.DragWindow();
			ResetWindowSize();


			//show helpwindow
			if(showHelp) 
			{
				TCAConfiguration.Globals.HelpPos = 
					GUILayout.Window(2, 
						TCAConfiguration.Globals.HelpPos, 
						windowHelp, 
						"Instructions",
						GUILayout.Width(helpWidth),
						GUILayout.Height(helpHeight));
				Utils.CheckRect(ref TCAConfiguration.Globals.HelpPos);
			}

			if (showDebugLines) {
				//DrawDebugLines ();
				TCA.DrawDirections();
			}
			if (showDebugLines2) {
				DrawDebugLines ();
				//TCA.DrawDirections();
			}
		}

		internal void ResetWindowSize()
		{
			WindowRect = TCAConfiguration.Globals.ControlsPos;
			if (showEngines)
				WindowRect.height += extendedHeight;
			if (showAdvanced)
				WindowRect.width += advancedWidth;
		}

		public void UpdateToolbarIcon() 
		{
			if(TCAToolbarButton != null)
			{
				if(TCA.CFG.Enabled) TCAToolbarButton.TexturePath = TCA.haveEC ? ICON_ON : ICON_NC;
				else TCAToolbarButton.TexturePath = ICON_OFF;
			}
			if(TCAButton != null) 
			{
				if(TCA.CFG.Enabled) TCAButton.SetTexture(TCA.haveEC? textureOn : textureNoCharge);
				else TCAButton.SetTexture(textureOff);
			}
		}

		void OnGUIAppLauncherReady()
		{
			if (ApplicationLauncher.Ready)
			{
				TCAButton = ApplicationLauncher.Instance.AddModApplication(
					onShowUI,
					onHideUI,
					DummyVoid, DummyVoid, DummyVoid, DummyVoid,
					ApplicationLauncher.AppScenes.FLIGHT,
					GameDatabase.Instance.GetTexture(ICON_OFF, false));
				if(TCA.CFG.GUIVisible) TCAButton.SetTrue();
				else TCAButton.SetFalse();
			}
		}
		void DummyVoid() {}

		void onGUIToggle() {
			if (Visible)
				onShowUI ();
			else  onHideUI ();
		}
		void onShowUI() { TCA.CFG.GUIVisible = Visible = true; }
		void onHideUI() { TCA.CFG.GUIVisible = Visible = false; }

		internal override void OnDestroy() 
		{ 
			GameEvents.onGUIApplicationLauncherReady.Remove(OnGUIAppLauncherReady);
			if(TCAButton != null)
				ApplicationLauncher.Instance.RemoveModApplication(TCAButton);
			if(TCAToolbarButton != null)
				TCAToolbarButton.Destroy();
			GameEvents.onHideUI.Remove(onHideUI);
			GameEvents.onShowUI.Remove(onShowUI);
		}

		void windowHelp(int windowID)
		{
			GUILayout.BeginVertical();
			GUILayout.Label(TCAConfiguration.Globals.Instructions, GUILayout.MaxWidth(helpWidth));
			if(GUILayout.Button("Close")) showHelp = !showHelp;
			GUILayout.EndVertical();
			GUI.DragWindow();
		}
		#endregion

		#region Debug Lines
		public void DrawDebugLines()
		{
			if (vessel != null){
				GameObject go;
				if(engLine == null){
					go = new GameObject();
					go.transform.parent = vessel.transform;
					go.transform.localPosition = Vector3.zero;
					go.transform.localRotation = Quaternion.identity;
					engLine = go.AddComponent<LineRenderer> ();
					engLine.SetColors(Color.red, new Color(1f,0f,0f,0f));
					engLine.SetWidth (-0.1f, 0.1f);
					engLine.useWorldSpace = false;
					engLine.material = diffuseMat;
				}
				if(dirLine == null){
					go = new GameObject();
					go.transform.parent = vessel.transform;
					go.transform.localPosition = Vector3.zero;
					go.transform.localRotation = Quaternion.identity;
					dirLine = go.AddComponent<LineRenderer> ();
					dirLine.SetColors(new Color(0.5f, 0f, 0f, 1f), new Color(0.5f,0f,0f,0f));
					dirLine.SetWidth (-0.1f, 0.1f);
					dirLine.useWorldSpace = false;
					dirLine.material = diffuseMat;
				}
				if(upLine == null){
					go = new GameObject();
					go.transform.parent = vessel.transform;
					go.transform.localPosition = Vector3.zero;
					go.transform.localRotation = Quaternion.identity;
					upLine = go.AddComponent<LineRenderer> ();
					upLine.SetColors(Color.white, new Color(1f,1f,1f,0f));
					upLine.SetWidth (-0.1f, 0.1f);
					upLine.useWorldSpace = false;
					upLine.material = diffuseMat;
				}
				if(gravLine == null){
					go = new GameObject();
					go.transform.parent = vessel.transform;
					go.transform.localPosition = Vector3.zero;
					go.transform.localRotation = Quaternion.identity;
					gravLine = go.AddComponent<LineRenderer> ();
					gravLine.SetColors(new Color(0.6f,0.4f,0.2f,1f), new Color(0.6f,0.4f,0.2f,0f));
					gravLine.SetWidth (-0.1f, 0.1f);
					gravLine.useWorldSpace = false;
					gravLine.material = diffuseMat;
					gravLine.SetPosition(0, Vector3.zero);
					gravLine.SetPosition(1, Vector3.forward * 10);
				}
				if(vecLine == null){
					go = new GameObject();
					vecLine = go.AddComponent<LineRenderer> ();
					vecLine.SetColors(new Color(0.7f,0.4f,0.8f,1f), new Color(0.7f,0.4f,0.8f,0f));
					vecLine.SetWidth (-0.1f, 0.1f);
					vecLine.useWorldSpace = true;
					vecLine.material = diffuseMat;
				}
				engLine.SetPosition (0, Vector3.zero);
				dirLine.SetPosition (0, Vector3.zero);
				upLine.SetPosition (0, Vector3.zero);
				vecLine.SetPosition (0, vessel.findWorldCenterOfMass());

				//set line targets:
				/*dirLine.SetPosition (1, mainThrustAxis.normalized * 10);
				Vector3 sum = new Vector3(0, 0, 0);
				foreach (EngineWrapper eng in TCA.engines)
				{
					sum += eng.maxThrust * eng.thrustVector;
				}
				engLine.SetPosition (1, sum.normalized * 10);
				*/
				upLine.SetPosition (1, Vector3.up * 10);
				vecLine.SetPosition (1, vessel.findWorldCenterOfMass() + ((vessel.situation == Vessel.Situations.FLYING)?vessel.GetSrfVelocity().normalized : vessel.GetObtVelocity().normalized) * 10); //GetSrfVelocity
				Vector3 gee = FlightGlobals.getGeeForceAtPosition( vessel.transform.position ); 
				gravLine.transform.rotation = Quaternion.LookRotation( gee.normalized );				
			}
		}
		private void killLines() {
			if(engLine != null){
				Destroy (engLine.gameObject);
			}
			if(dirLine != null){
				Destroy (dirLine.gameObject);
			}
			if(upLine != null){
				Destroy (upLine.gameObject);
			}
			if(gravLine != null){
				Destroy (gravLine.gameObject);
			}
			if(vecLine != null){
				Destroy (vecLine.gameObject);
			}
			TCA.KillLines ();
		}
		#endregion
	}
}

