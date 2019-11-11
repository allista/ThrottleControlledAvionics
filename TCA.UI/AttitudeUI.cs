//   AttitudeUI.cs
//
//  Author:
//       Allis Tauri <allista@gmail.com>
//
//  Copyright (c) 2019 Allis Tauri
using AT_Utils.UI;
using UnityEngine;
using UnityEngine.UI;

namespace TCA.UI
{
    public class AttitudeUI : ScreenBoundRect
    {
        public CuesUI CuesPanel;
        public CurrentCueUI CurrentCue;

        public Text TSASText;
        public Toggle TSASToggle;
        bool TSASActive;

        protected override void Awake()
        {
            base.Awake();
            TSASToggle.onValueChanged.AddListener(CuesPanel.SetActive);
            TSASToggle.isOn = true;
            SetState(false, true);
        }

        void onTSASColorChange(Color color) => TSASText.color = color;

        public void SetState(bool active, bool force = false)
        {
            if(force || TSASActive != active)
            {
                TSASActive = active;
                if(active)
                {
                    Colors.Neutral.removeOnColorChangeListner(onTSASColorChange);
                    Colors.Selected1.addOnColorChangeListner(onTSASColorChange);
                    onTSASColorChange(Colors.Selected1);
                }
                else
                {
                    Colors.Selected1.removeOnColorChangeListner(onTSASColorChange);
                    Colors.Neutral.addOnColorChangeListner(onTSASColorChange);
                    onTSASColorChange(Colors.Neutral);
                }
            }
        }
    }

    public class CuesUI : PanelledUI
    {
        public Toggle Kill, Hold, Maneuver, PG, RG, Rp, Rm, Np, Nm, Tp, Tm, rVp, rVm, Tp_rVm;

        public void SetState(bool Kill, bool Hold,
                             bool Maneuver,
                             bool PG, bool RG,
                             bool Rp, bool Rm,
                             bool Np, bool Nm,
                             bool Tp, bool Tm,
                             bool rVp, bool rVm,
                             bool Tp_rVm)
        {
            this.Kill.SetIsOnAndColorWithoutNotify(Kill);
            this.Hold.SetIsOnAndColorWithoutNotify(Hold);
            this.Maneuver.SetIsOnAndColorWithoutNotify(Maneuver);
            this.PG.SetIsOnAndColorWithoutNotify(PG);
            this.RG.SetIsOnAndColorWithoutNotify(RG);
            this.Rp.SetIsOnAndColorWithoutNotify(Rp);
            this.Rm.SetIsOnAndColorWithoutNotify(Rm);
            this.Np.SetIsOnAndColorWithoutNotify(Np);
            this.Nm.SetIsOnAndColorWithoutNotify(Nm);
            this.Tp.SetIsOnAndColorWithoutNotify(Tp);
            this.Tm.SetIsOnAndColorWithoutNotify(Tm);
            this.rVp.SetIsOnAndColorWithoutNotify(rVp);
            this.rVm.SetIsOnAndColorWithoutNotify(rVm);
            this.Tp_rVm.SetIsOnAndColorWithoutNotify(Tp_rVm);
        }
    }

    public class CurrentCueUI : PanelledUI
    {
        public Text AttitudeError;

        public Button CurrentCueSwitch;
        public Text CurrentCueText;
        public TooltipTrigger CurrentCueTooltip;
    }
}
