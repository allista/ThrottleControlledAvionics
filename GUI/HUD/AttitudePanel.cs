//   AttitudePanel.cs
//
//  Author:
//       Allis Tauri <allista@gmail.com>
//
//  Copyright (c) 2019 Allis Tauri

using System.Collections.Generic;
using AT_Utils.UI;
using TCA.UI;

namespace ThrottleControlledAvionics
{
    public class AttitudePanel : ControlPanel<AttitudeUI>
    {
        private static readonly Dictionary<Attitude, string> cues_long = new Dictionary<Attitude, string>
        {
            { Attitude.None, "" },
            { Attitude.KillRotation, "Kill Rotation" },
            { Attitude.HoldAttitude, "Hold Attitude" },
            { Attitude.ManeuverNode, "Maneuver Node" },
            { Attitude.Prograde, "Prograde" },
            { Attitude.Retrograde, "Retrograde" },
            { Attitude.Radial, "Radial" },
            { Attitude.AntiRadial, "Anti Radial" },
            { Attitude.Normal, "Normal" },
            { Attitude.AntiNormal, "Anti Normal" },
            { Attitude.Target, "To Target" },
            { Attitude.AntiTarget, "From Target" },
            { Attitude.RelVel, "Relative Velocity" },
            { Attitude.AntiRelVel, "Against Relative Velocity" },
            { Attitude.TargetCorrected, "To Target, correcting lateral velocity" },
            { Attitude.Custom, "Attitude is controlled by autopilot" },
        };

        private static readonly Dictionary<Attitude, string> cues_short = new Dictionary<Attitude, string>
        {
            { Attitude.None, "" },
            { Attitude.KillRotation, "Kill" },
            { Attitude.HoldAttitude, "Hold" },
            { Attitude.ManeuverNode, "Maneuver" },
            { Attitude.Prograde, "PG" },
            { Attitude.Retrograde, "RG" },
            { Attitude.Radial, "R+" },
            { Attitude.AntiRadial, "R-" },
            { Attitude.Normal, "N+" },
            { Attitude.AntiNormal, "N-" },
            { Attitude.Target, "T+" },
            { Attitude.AntiTarget, "T-" },
            { Attitude.RelVel, "rV+" },
            { Attitude.AntiRelVel, "rV-" },
            { Attitude.TargetCorrected, "T+ rV-" },
            { Attitude.Custom, "Auto" },
        };

        private AttitudeControl ATC;

        protected override bool shouldShow => base.shouldShow && ATC != null;

        protected override void init_controller()
        {
            Controller.CurrentCue.CurrentCueSwitch.onClick.AddListener(disableCurrentCue);
            Controller.CuesPanel.Kill.onValueChanged.AddListener(state => onCueChange(Attitude.KillRotation, state));
            Controller.CuesPanel.Hold.onValueChanged.AddListener(state => onCueChange(Attitude.HoldAttitude, state));
            Controller.CuesPanel.Maneuver.onValueChanged.AddListener(state =>
                onCueChange(Attitude.ManeuverNode, state));
            Controller.CuesPanel.PG.onValueChanged.AddListener(state => onCueChange(Attitude.Prograde, state));
            Controller.CuesPanel.RG.onValueChanged.AddListener(state => onCueChange(Attitude.Retrograde, state));
            Controller.CuesPanel.Rp.onValueChanged.AddListener(state => onCueChange(Attitude.Radial, state));
            Controller.CuesPanel.Rm.onValueChanged.AddListener(state => onCueChange(Attitude.AntiRadial, state));
            Controller.CuesPanel.Np.onValueChanged.AddListener(state => onCueChange(Attitude.Normal, state));
            Controller.CuesPanel.Nm.onValueChanged.AddListener(state => onCueChange(Attitude.AntiNormal, state));
            Controller.CuesPanel.Tp.onValueChanged.AddListener(state => onCueChange(Attitude.Target, state));
            Controller.CuesPanel.Tm.onValueChanged.AddListener(state => onCueChange(Attitude.AntiTarget, state));
            Controller.CuesPanel.rVp.onValueChanged.AddListener(state => onCueChange(Attitude.RelVel, state));
            Controller.CuesPanel.rVm.onValueChanged.AddListener(state => onCueChange(Attitude.AntiRelVel, state));
            Controller.CuesPanel.Tp_rVm.onValueChanged.AddListener(
                state => onCueChange(Attitude.TargetCorrected, state));
            base.init_controller();
        }

        private void onCueChange(Attitude cue, bool state)
        {
            if(state)
                CFG.AT.XOnIfNot(cue);
            else
                CFG.AT.XOffIfOn(cue);
        }

        private void disableCurrentCue() => CFG.AT.XOff();

        protected override void OnLateUpdate()
        {
            base.OnLateUpdate();
            if(!IsShown)
                return;
            Controller.SetState(CFG.AT && !VSL.AutopilotDisabled);
            //current cue
            if(CFG.AT)
            {
                Controller.CurrentCue.SetActive(true);
                Controller.CurrentCue.AttitudeError.text = VSL.AutopilotDisabled
                    ? "USER"
                    : $"Err: {VSL.Controls.AttitudeError:F1}°";
                Controller.CurrentCue.AttitudeError.color = VSL.Controls.Aligned
                    ? Colors.Enabled
                    : Colors.Neutral;
                Controller.CurrentCue.CurrentCueText.text = cues_short[CFG.AT.state];
                Controller.CurrentCue.CurrentCueTooltip.text = $"{cues_long[CFG.AT.state]}. Click to disable.";
            }
            else
                Controller.CurrentCue.SetActive(false);
            //cues
            var controllable = TCA.IsControllable;
            Controller.TSASToggle.SetInteractable(controllable);
            if(!controllable && Controller.TSASToggle.isOn)
                Controller.TSASToggle.isOn = false;
            Controller.CurrentCue.CurrentCueSwitch.SetInteractable(controllable);
            Controller.CuesPanel.SetState(
                CFG.AT[Attitude.KillRotation],
                CFG.AT[Attitude.HoldAttitude],
                CFG.AT[Attitude.ManeuverNode],
                CFG.AT[Attitude.Prograde],
                CFG.AT[Attitude.Retrograde],
                CFG.AT[Attitude.Radial],
                CFG.AT[Attitude.AntiRadial],
                CFG.AT[Attitude.Normal],
                CFG.AT[Attitude.AntiNormal],
                CFG.AT[Attitude.Target],
                CFG.AT[Attitude.AntiTarget],
                CFG.AT[Attitude.RelVel],
                CFG.AT[Attitude.AntiRelVel],
                CFG.AT[Attitude.TargetCorrected]
            );
        }
    }
}
