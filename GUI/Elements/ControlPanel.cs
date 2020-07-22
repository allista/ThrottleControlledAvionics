//   ControlPanel.cs
//
//  Author:
//       Allis Tauri <allista@gmail.com>
//
//  Copyright (c) 2019 Allis Tauri

using System.Collections.Generic;
using AT_Utils;
using AT_Utils.UI;


namespace ThrottleControlledAvionics
{
    public interface IControlPanel
    {
        void Init(ModuleTCA tca);
        void Reset();
        void Open();
        void Close();
        void LateUpdate();
        void OnRenderObject();
    }

    public abstract class ControlPanel<T> : UIWindowBase<T>, IControlPanel where T : ScreenBoundRect
    {
        public ModuleTCA TCA;
        public VesselWrapper VSL => TCA.VSL;
        internal static Globals GLB => Globals.Instance;
        public VesselConfig CFG => TCA.VSL.CFG;
        public bool Connected => TCA != null && TCA.VSL != null && TCA.VSL.CFG != null;

        protected List<TCAModule> AllModules = new List<TCAModule>();

        protected virtual bool shouldShow => Connected && TCA.IsControllable && CFG.GUIVisible;

        protected override void init_controller()
        {
            base.init_controller();
            if(Connected)
                OnLateUpdate();
        }

        protected ControlPanel() : base(GLB.AssetBundle) { }

        public virtual void Reset()
        {
            TCA = null;
            AllModules.Clear();
            ModuleTCA.ResetModuleFields(this);
            Close();
        }

        public virtual void Init(ModuleTCA tca)
        {
            TCA = tca;
            TCA.InitModuleFields(this);
            AllModules = TCAModulesDatabase.GetAllModules(this);
            if(shouldShow)
                Open();
        }

        public void Open() => Show(TCA);

        protected virtual void OnLateUpdate() { }

        public void LateUpdate()
        {
            if(!Connected)
                return;
            if(IsShown)
            {
                if(shouldShow)
                    OnLateUpdate();
                else
                    Close();
            }
            else if(shouldShow)
                Open();
        }

        protected virtual void OnRender() { }

        public void OnRenderObject()
        {
            if(!Connected || !IsShown || !shouldShow)
                return;
            OnRender();
        }
    }
}
