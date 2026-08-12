(function () {
  const MODES = ['motorbridge', 'ros'];
  const STORAGE_KEY = 'rebot-rs-control-mode-v2';
  const tabs = Array.from(document.querySelectorAll('[data-control-mode]'));
  const panels = Array.from(document.querySelectorAll('[data-control-mode-panel]'));
  const listeners = new Set();

  if (!tabs.length || !panels.length) return;

  let activeMode = readStoredMode();

  function readStoredMode() {
    try {
      const stored = window.localStorage.getItem(STORAGE_KEY);
      return MODES.includes(stored) ? stored : 'ros';
    } catch (error) {
      return 'ros';
    }
  }

  function applyMode(mode, options) {
    const nextMode = MODES.includes(mode) ? mode : 'ros';
    const previousMode = activeMode;
    activeMode = nextMode;

    document.documentElement.dataset.controlMode = nextMode;

    tabs.forEach((tab) => {
      const selected = tab.dataset.controlMode === nextMode;
      tab.classList.toggle('is-active', selected);
      tab.setAttribute('aria-selected', String(selected));
      tab.tabIndex = selected ? 0 : -1;
    });

    panels.forEach((panel) => {
      const selected = panel.dataset.controlModePanel === nextMode;
      panel.classList.toggle('is-active', selected);
      panel.hidden = !selected;
    });

    if (
      nextMode === 'motorbridge' &&
      window.reBotSim &&
      typeof window.reBotSim.stopMotion === 'function'
    ) {
      window.reBotSim.stopMotion();
    }

    try {
      window.localStorage.setItem(STORAGE_KEY, nextMode);
    } catch (error) {
      // Private browsing or a locked-down WebView may reject storage.
    }

    if (!options || !options.silent) {
      const detail = { mode: nextMode, previousMode };
      listeners.forEach((listener) => listener(detail));
      window.dispatchEvent(new CustomEvent('rebot-control-mode-change', { detail }));
    }
  }

  function setMode(mode) {
    if (!MODES.includes(mode) || mode === activeMode) return;
    applyMode(mode);
  }

  tabs.forEach((tab, index) => {
    tab.addEventListener('click', () => setMode(tab.dataset.controlMode));
    tab.addEventListener('keydown', (event) => {
      let nextIndex = null;
      if (event.key === 'ArrowRight') nextIndex = (index + 1) % tabs.length;
      if (event.key === 'ArrowLeft') nextIndex = (index - 1 + tabs.length) % tabs.length;
      if (event.key === 'Home') nextIndex = 0;
      if (event.key === 'End') nextIndex = tabs.length - 1;
      if (nextIndex === null) return;
      event.preventDefault();
      tabs[nextIndex].focus();
      setMode(tabs[nextIndex].dataset.controlMode);
    });
  });

  window.reBotControlMode = {
    get: () => activeMode,
    is: (mode) => activeMode === mode,
    set: setMode,
    onChange(listener) {
      if (typeof listener !== 'function') return function () {};
      listeners.add(listener);
      return () => listeners.delete(listener);
    }
  };

  applyMode(activeMode, { silent: true });
})();
