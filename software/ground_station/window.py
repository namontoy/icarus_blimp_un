"""The ground station as its own window, through pywebview.

The page and the back end do not change: pywebview opens a native window
(WebView2 on Windows, WebKitGTK or Qt on Linux, WebKit on macOS) showing the
same address Tornado serves, with no tabs, address bar or browser menus, so
the whole screen is the ground station. The page can ask the window to go
full screen through window.pywebview.api.toggle_fullscreen() (F11 or the
button in the header).

Optional: without pywebview installed the back end opens the system browser
instead, as before (`pip install pywebview` adds the window).
"""
TITLE = 'Icarus Ground Station'
BACKGROUND = '#0C0F0D'      # --bg in app.css, so the window never flashes white

_window = None              # the one window; see _Api for why it lives here


def available():
    try:
        import webview  # noqa: F401
    except ImportError:
        return False
    return True


class _Api:
    """What the page reaches as window.pywebview.api. pywebview walks every
    public attribute of this object to build that bridge, so it must hold
    nothing but methods: a reference to the window here recursed through the
    native form's object graph until Python gave up."""

    def toggle_fullscreen(self):
        if _window is not None:
            _window.toggle_fullscreen()


def run(url, on_ready=None):
    """Open the window on this thread and block until it is closed."""
    global _window
    import webview
    _window = webview.create_window(TITLE, url, js_api=_Api(), width=1600, height=980,
                                    min_size=(1100, 700), background_color=BACKGROUND,
                                    text_select=False)
    if on_ready is not None:
        w = _window
        w.events.loaded += lambda: on_ready(w)
    try:
        webview.start()
    finally:
        _window = None
