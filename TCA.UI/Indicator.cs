using System.Collections.Generic;
using AT_Utils.UI;
using UnityEngine;
using UnityEngine.UI;

namespace TCA.UI
{
    public class Indicator : MonoBehaviour
    {
        public CanvasGroup canvasGroup;
        public Image foreground;
        public Image background;
        public TooltipTrigger tooltip;

        public float blink;
        private Coroutine blinker;

        public static bool soundEnabled = true;
        public AudioClip signal;
        private AudioSource audio;

        private Color foreground_color_ON, foreground_color_OFF;
        [SerializeField] private bool m_IsOn;
        [SerializeField] private bool visible;

        private void Awake()
        {
            if(signal != null)
            {
                audio = gameObject.AddComponent<AudioSource>();
                audio.clip = signal;
                audio.loop = true;
                audio.playOnAwake = false;
            }
            var c = foreground.color;
            foreground_color_ON = new Color(c.r, c.g, c.b, 0.9f);
            foreground_color_OFF = new Color(c.r, c.g, c.b, 0.6f);
            visible = !canvasGroup.alpha.Equals(0);
        }

        private void Start()
        {
            playEffect();
        }

        void visualOn()
        {
            background.enabled = true;
            foreground.color = foreground_color_ON;
        }

        void visualOff()
        {
            background.enabled = false;
            foreground.color = foreground_color_OFF;
        }

        public void EnableSound(bool enable)
        {
            if(audio == null)
                return;
            if(!enable)
                audio.Stop();
            else if(soundEnabled && m_IsOn)
                audio.Play();
        }

        private void chaneOpacity(float alpha)
        {
            if(canvasGroup.alpha.Equals(alpha))
                return;
            canvasGroup.alpha = alpha;
            visible = !alpha.Equals(0);
            canvasGroup.interactable = visible;
            canvasGroup.blocksRaycasts = visible;
            if(!visible && m_IsOn)
            {
                m_IsOn = false;
                playEffect();
            }
        }

        public void Show(bool show) => chaneOpacity(show ? 1 : 0);

        public void SetActive(bool active)
        {
            if(visible)
                chaneOpacity(active ? 1 : 0.3f);
        }

        IEnumerator<YieldInstruction> doBlink()
        {
            var halfPeriod = blink / 2;
            while(m_IsOn)
            {
                visualOn();
                yield return new WaitForSeconds(halfPeriod);
                visualOff();
                yield return new WaitForSeconds(halfPeriod);
            }
        }

        private void playEffect()
        {
            if(m_IsOn)
            {
                if(audio != null && soundEnabled)
                    audio.Play();
                if(blink > 0 && blinker == null)
                    blinker = StartCoroutine(doBlink());
                else
                    visualOn();
            }
            else
            {
                if(audio != null)
                    audio.Stop();
                if(blinker != null)
                {
                    StopCoroutine(blinker);
                    blinker = null;
                }
                visualOff();
            }
        }

        public bool isOn
        {
            get => m_IsOn;
            set
            {
                if(!visible)
                    return;
                if(m_IsOn == value)
                    return;
                if(!isActiveAndEnabled)
                    return;
                m_IsOn = value;
                playEffect();
            }
        }
    }
}
