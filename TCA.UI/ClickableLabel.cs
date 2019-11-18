using System;
using UnityEngine;
using UnityEngine.Events;
using UnityEngine.EventSystems;
using UnityEngine.UI;

namespace TCA.UI
{
    public class ClickableLabel : MonoBehaviour, IPointerClickHandler
    {
        [Serializable]
        public class LabelClickEvent : UnityEvent
        {}
        
        public Text text;
        public LabelClickEvent onLabelClicked = new LabelClickEvent();
        
        public void OnPointerClick(PointerEventData eventData)
        {
            onLabelClicked.Invoke();
        }
    }
}
