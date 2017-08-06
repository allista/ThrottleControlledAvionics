//  Author:
//       Allis Tauri <allista@gmail.com>
//
//  Copyright (c) 2016 Allis Tauri
//
// This work is licensed under the Creative Commons Attribution-ShareAlike 4.0 International License. 
// To view a copy of this license, visit http://creativecommons.org/licenses/by-sa/4.0/ 
// or send a letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.
//

using System;
using System.Collections.Generic;
using AT_Utils;
using UnityEngine;

namespace ThrottleControlledAvionics
{
	public class AttitudeControlWindow : ControlWindow
	{
		static readonly Dictionary<Attitude,string> cues_long = new Dictionary<Attitude,string>
		{
			{Attitude.None,         ""},
			{Attitude.KillRotation, "Kill Rotation"},
			{Attitude.HoldAttitude, "Hold Attitude"},
			{Attitude.ManeuverNode, "Maneuver Node"},
			{Attitude.Prograde,     "Prograde"},
			{Attitude.Retrograde,   "Retrograde"},
			{Attitude.Radial,       "Radial"},
			{Attitude.AntiRadial,   "Anti Radial"},
			{Attitude.Normal,       "Normal"},
			{Attitude.AntiNormal,   "Anti Normal"},
			{Attitude.Target,       "To Target"},
			{Attitude.AntiTarget,   "From Target"},
			{Attitude.RelVel,       "Relative Velocity"},
			{Attitude.AntiRelVel,   "Against Relative Velocity"},
			{Attitude.TargetCorrected, "To Target, correcting lateral velocity"},
			{Attitude.Custom,       "Auto"},
		};
		static readonly Dictionary<Attitude,string> cues_short = new Dictionary<Attitude,string>
		{
			{Attitude.None,         ""},
			{Attitude.KillRotation, "Kill"},
			{Attitude.HoldAttitude, "Hold"},
			{Attitude.ManeuverNode, "Maneuver"},
			{Attitude.Prograde,     "PG"},
			{Attitude.Retrograde,   "RG"},
			{Attitude.Radial,       "R+"},
			{Attitude.AntiRadial,   "R-"},
			{Attitude.Normal,       "N+"},
			{Attitude.AntiNormal,   "N-"},
			{Attitude.Target,       "T+"},
			{Attitude.AntiTarget,   "T-"},
			{Attitude.RelVel,       "rV+"},
			{Attitude.AntiRelVel,   "rV-"},
			{Attitude.TargetCorrected, "T+ rV-"},
			{Attitude.Custom,       "Auto"},
		};

		class Cues : ControlSubwindow
		{
            #if DEBUG
            AttitudeControl ATC;
            #endif

			protected override void MainWindow(int windowID)
			{
				GUILayout.BeginVertical();
                #if DEBUG
                ATC.DrawDebugLines();
                Utils.ButtonSwitch("Mouse", ref ATC.FollowMouse, "Follow mouse", GUILayout.ExpandWidth(false));
                Utils.ButtonSwitch("Gimball", ref AttitudeControlBase.UseGimball, "Use gimball", GUILayout.ExpandWidth(false));
                #endif
				GUILayout.BeginHorizontal();
				if(Utils.ButtonSwitch("Kill", CFG.AT[Attitude.KillRotation], "Kill rotation", GUILayout.ExpandWidth(false)))
					CFG.AT.XToggle(Attitude.KillRotation);
				if(Utils.ButtonSwitch("Hold", CFG.AT[Attitude.HoldAttitude], "Hold current attitude", GUILayout.ExpandWidth(false)))
					CFG.AT.XToggle(Attitude.HoldAttitude);
				GUILayout.EndHorizontal();
				if(Utils.ButtonSwitch("Maneuver", CFG.AT[Attitude.ManeuverNode], "Maneuver Node", GUILayout.ExpandWidth(false)))
					CFG.AT.XToggle(Attitude.ManeuverNode);
				GUILayout.BeginHorizontal();
				if(Utils.ButtonSwitch("PG", CFG.AT[Attitude.Prograde], "Prograde", GUILayout.ExpandWidth(false)))
					CFG.AT.XToggle(Attitude.Prograde);
				if(Utils.ButtonSwitch("RG", CFG.AT[Attitude.Retrograde], "Retrograde", GUILayout.ExpandWidth(false)))
					CFG.AT.XToggle(Attitude.Retrograde);
				GUILayout.EndHorizontal();
				GUILayout.BeginHorizontal();
				if(Utils.ButtonSwitch("R+", CFG.AT[Attitude.Radial], "Radial", GUILayout.ExpandWidth(false)))
					CFG.AT.XToggle(Attitude.Radial);
				if(Utils.ButtonSwitch("R-", CFG.AT[Attitude.AntiRadial], "Anti Radial", GUILayout.ExpandWidth(false)))
					CFG.AT.XToggle(Attitude.AntiRadial);
				GUILayout.EndHorizontal();
				GUILayout.BeginHorizontal();
				if(Utils.ButtonSwitch("N+", CFG.AT[Attitude.Normal], "Normal", GUILayout.ExpandWidth(false)))
					CFG.AT.XToggle(Attitude.Normal);
				if(Utils.ButtonSwitch("N-", CFG.AT[Attitude.AntiNormal], "Anti Normal", GUILayout.ExpandWidth(false)))
					CFG.AT.XToggle(Attitude.AntiNormal);
				GUILayout.EndHorizontal();
				GUILayout.BeginHorizontal();
				if(Utils.ButtonSwitch("T+", CFG.AT[Attitude.Target], "To Target", GUILayout.ExpandWidth(false)))
					CFG.AT.XToggle(Attitude.Target);
				if(Utils.ButtonSwitch("T-", CFG.AT[Attitude.AntiTarget], "From Target", GUILayout.ExpandWidth(false)))
					CFG.AT.XToggle(Attitude.AntiTarget);
				GUILayout.EndHorizontal();
				GUILayout.BeginHorizontal();
				if(Utils.ButtonSwitch("rV+", CFG.AT[Attitude.RelVel], "Relative Velocity", GUILayout.ExpandWidth(false)))
					CFG.AT.XToggle(Attitude.RelVel);
				if(Utils.ButtonSwitch("rV-", CFG.AT[Attitude.AntiRelVel], "Against Relative Velocity", GUILayout.ExpandWidth(false)))
					CFG.AT.XToggle(Attitude.AntiRelVel);
				GUILayout.EndHorizontal();
				if(Utils.ButtonSwitch("T+ rV-", CFG.AT[Attitude.TargetCorrected], "To Target, correcting lateral velocity", GUILayout.ExpandWidth(true)))
					CFG.AT.XToggle(Attitude.TargetCorrected);
				GUILayout.EndVertical();
				TooltipManager.GetTooltip();
			}
		}

		class CurrentCue : ControlSubwindow
		{
			protected override bool can_draw() { return CFG.AT; }

			protected override void MainWindow(int windowID)
			{
				GUILayout.BeginHorizontal();
				if(GUILayout.Button(new GUIContent(cues_short[CFG.AT.state], cues_long[CFG.AT.state]+". Push to disable."), 
				                    Styles.green, GUILayout.ExpandWidth(false)))
					CFG.AT.Off();
				//attitude error display
				var err = "OFF";
				if(VSL.AutopilotDisabled) err = "USER";
				else if(CFG.AT) err = string.Format("Err: {0:F1}°", VSL.Controls.AttitudeError);
				GUILayout.Label(err, VSL.Controls.Aligned? Styles.green : Styles.white, GUILayout.Width(100));
				GUILayout.EndHorizontal();
				TooltipManager.GetTooltip();
			}
		}

        #pragma warning disable 169
        AttitudeControl ATC;

		[SubwindowSpec(AnchorPosition.BottomRight, -0.1f, -1, xRelative = true, yRelative = true)]
		Cues cues;

		[SubwindowSpec(AnchorPosition.TopRight, -1, -1, xRelative = true, yRelative = true)]
		CurrentCue current_cue;
		#pragma warning restore 169

		public AttitudeControlWindow() { Anchor = AnchorPosition.BottomRight; }

		protected override bool can_draw() { return base.can_draw() && ATC != null; }

		protected override void DrawContent()
		{
            #if DEBUG
            ATC.DrawDebugLines();
            Utils.GLVec(VSL.refT.position, VSL.WorldDir(VSL.OnPlanetParams.MaxAeroForceL)*10, Color.magenta);
            #endif
			if(GUILayout.Button(new GUIContent("T-SAS", "Push to toggle attitude controls"), 
			                    CFG.AT && !VSL.AutopilotDisabled? Styles.cyan : Styles.white, GUILayout.ExpandWidth(false)))
				cues.Toggle();
		}
	}
}
