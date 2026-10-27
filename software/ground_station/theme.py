"""The ground station's look: colour tokens, fonts and the one Qt style sheet.

The tokens carry the names the page's app.css had (--panel is PANEL,
--raised-2 is RAISED_2), so the design in docs/ground-station-plan.md §3
still reads against this file: the Universidad Nacional's green and red,
McLaren's papaya only as the detail colour, Titillium Web for display and UI,
IBM Plex Mono for data. New UI keeps to these names.

The style sheet covers the stock widgets (buttons, inputs, slider, table,
scroll bars, tool tips). The custom-painted pieces in widgets.py read the
same constants. Letter spacing and upper case are font properties in Qt, not
style-sheet ones, so labels get them through font() and the style sheet
never sets a font on a whole widget class that also uses font(): a sheet's
font properties win over setFont() for every attribute they name.
"""
import os
import sys

from PyQt6.QtGui import QColor, QFont, QFontDatabase, QPalette

BG = '#0C0F0D'
PANEL = '#141A16'
RAISED = '#1C2420'
RAISED_2 = '#243029'
BORDER = '#26302A'
FG = '#ECEFEA'
MUTED = '#A9B3A6'
FAINT = '#6F7A71'
UN_GREEN = '#94B43B'
UN_GREEN_INK = '#0B0F08'
UN_GREEN_DEEP = '#5E7D1E'
UN_RED = '#DC313A'
AMBER = '#F2C14E'
PAPAYA = '#FF8000'
PAPAYA_INK = '#1A0D00'
S_X = '#789E24'
S_Y = '#2E9ED0'
S_Z = '#D96D00'
SERIES = (S_X, S_Y, S_Z)
GRID = '#1E2622'
# One-off tints the page wrote inline
CRIT_BG = '#2A1416'         # a crit pill
EV_CRIT_BG = '#221214'      # a crit row in the event list
TARGET_BG = '#19261A'       # the TARGET verdict tile
CAM_BG = '#050706'          # behind the camera frame

CHAMFER = 14                # the cut top-right corner of every panel

FONTS = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'fonts')
FONT_FILES = ('TitilliumWeb-Regular', 'TitilliumWeb-SemiBold', 'TitilliumWeb-Bold', 'TitilliumWeb-Black',
              'IBMPlexMono-Regular', 'IBMPlexMono-Medium', 'IBMPlexMono-SemiBold')

# Resolved by load_fonts(): the bundled families, or the system fallbacks.
UI_FAMILY = 'Titillium Web'
MONO_FAMILY = 'IBM Plex Mono'


def mix(a, b, t):
    """a blended toward b by t, as a #rrggbb string: the page's opacity on a
    known background, which a style sheet cannot do to a whole widget."""
    ca, cb = QColor(a), QColor(b)
    return QColor(round(ca.red() + (cb.red() - ca.red()) * t), round(ca.green() + (cb.green() - ca.green()) * t),
                  round(ca.blue() + (cb.blue() - ca.blue()) * t)).name()


def load_fonts():
    """Register the bundled TTFs. A missing file is no error: Qt then uses the
    fallback family, Segoe UI and Consolas on Windows, DejaVu elsewhere."""
    global UI_FAMILY, MONO_FAMILY
    for name in FONT_FILES:
        path = os.path.join(FONTS, name + '.ttf')
        if os.path.exists(path):
            QFontDatabase.addApplicationFont(path)
    have = set(QFontDatabase.families())
    win = sys.platform == 'win32'
    if 'Titillium Web' not in have:
        UI_FAMILY = 'Segoe UI' if win else 'DejaVu Sans'
    if 'IBM Plex Mono' not in have:
        MONO_FAMILY = 'Consolas' if win else 'DejaVu Sans Mono'


def font(px, weight=400, mono=False, spacing=0.0, caps=False):
    """A font at a pixel size, as the page's CSS sized everything. spacing is
    the letter spacing in em, as in the CSS."""
    f = QFont(MONO_FAMILY if mono else UI_FAMILY)
    f.setPixelSize(px)
    f.setWeight(QFont.Weight(weight))
    if spacing:
        f.setLetterSpacing(QFont.SpacingType.AbsoluteSpacing, spacing * px)
    if caps:
        f.setCapitalization(QFont.Capitalization.AllUppercase)
    if mono:
        f.setStyleHint(QFont.StyleHint.Monospace)
    return f


def palette():
    p = QPalette()
    for role, c in ((QPalette.ColorRole.Window, BG), (QPalette.ColorRole.WindowText, FG),
                    (QPalette.ColorRole.Base, BG), (QPalette.ColorRole.AlternateBase, PANEL),
                    (QPalette.ColorRole.Text, FG), (QPalette.ColorRole.Button, RAISED),
                    (QPalette.ColorRole.ButtonText, FG), (QPalette.ColorRole.Highlight, UN_GREEN_DEEP),
                    (QPalette.ColorRole.HighlightedText, FG), (QPalette.ColorRole.ToolTipBase, RAISED_2),
                    (QPalette.ColorRole.ToolTipText, FG), (QPalette.ColorRole.PlaceholderText, FAINT)):
        p.setColor(role, QColor(c))
    return p


def _triangle(direction, color, w=4, h=5):
    """A CSS border triangle, the one way a style sheet draws an arrow
    without an image file."""
    side = 'border-left: %dpx solid transparent; border-right: %dpx solid transparent;' % (w, w)
    tip = 'border-%s: %dpx solid %s;' % ('bottom' if direction == 'up' else 'top', h, color)
    return 'image: none; width: 0; height: 0; %s %s' % (side, tip)


def stylesheet():
    t = dict(BG=BG, PANEL=PANEL, RAISED=RAISED, RAISED_2=RAISED_2, BORDER=BORDER, FG=FG, MUTED=MUTED,
             FAINT=FAINT, GREEN=UN_GREEN, GREEN_INK=UN_GREEN_INK, GREEN_DEEP=UN_GREEN_DEEP, PAPAYA=PAPAYA,
             PAPAYA_INK=PAPAYA_INK, UI=UI_FAMILY, MONO=MONO_FAMILY,
             # .btn:disabled was opacity 0.45 over the panel
             DIS_FG=mix(PANEL, FG, 0.45), DIS_BG=mix(PANEL, RAISED, 0.45), DIS_BORDER=mix(PANEL, BORDER, 0.45),
             DIS_GREEN=mix(PANEL, UN_GREEN, 0.45),
             UP=_triangle('up', MUTED), DOWN=_triangle('down', MUTED))
    return '''
QToolTip { background: %(RAISED_2)s; color: %(FG)s; border: 1px solid %(BORDER)s; padding: 4px 8px;
           font-family: "%(MONO)s"; font-size: 11px; }

QPushButton { background: %(RAISED)s; border: 1px solid %(BORDER)s; color: %(FG)s; padding: 6px 12px;
              font-family: "%(UI)s"; font-size: 12px; font-weight: 600; }
QPushButton:hover { background: %(RAISED_2)s; }
QPushButton:focus { border-color: %(PAPAYA)s; }
QPushButton:disabled { background: %(DIS_BG)s; border-color: %(DIS_BORDER)s; color: %(DIS_FG)s; }
QPushButton[kind="primary"] { background: %(GREEN)s; color: %(GREEN_INK)s; border-color: %(GREEN)s; }
QPushButton[kind="primary"]:disabled { background: %(DIS_GREEN)s; border-color: %(DIS_GREEN)s; }
QPushButton[kind="toggle"]:checked { background: %(PAPAYA)s; color: %(PAPAYA_INK)s; border-color: %(PAPAYA)s; }
QPushButton[state="up"] { border-color: %(GREEN)s; color: %(GREEN)s; }
QPushButton[state="busy"] { border-color: %(PAPAYA)s; color: %(PAPAYA)s; }
QPushButton[kind="full"] { padding: 3px 9px; font-size: 15px; }
QPushButton[kind="chip"] { color: %(MUTED)s; padding: 2px 8px; font-family: "%(MONO)s"; font-size: 11px;
                           font-weight: 400; }
QPushButton[kind="chip"]:hover { color: %(FG)s; border-color: %(GREEN)s; background: %(RAISED)s; }
QPushButton[kind="dock-toggle"] { background: transparent; color: %(MUTED)s; padding: 0; font-size: 13px;
                                  min-width: 26px; max-width: 26px; min-height: 22px; max-height: 22px; }
QPushButton[kind="dock-toggle"]:hover { color: %(FG)s; border-color: %(GREEN)s; }

QLineEdit, QPlainTextEdit { background: %(BG)s; border: 1px solid %(BORDER)s; color: %(FG)s;
                            selection-background-color: %(GREEN_DEEP)s; font-family: "%(MONO)s"; }
QLineEdit { padding: 5px 8px; font-size: 13px; }
QLineEdit:focus { border-color: %(PAPAYA)s; }
QPlainTextEdit { padding: 4px 6px; font-size: 12px; }

QSpinBox, QComboBox { background: %(BG)s; border: 1px solid %(BORDER)s; color: %(FG)s; padding: 3px 6px;
                      font-family: "%(MONO)s"; font-size: 14px; }
QSpinBox:focus, QComboBox:focus { border-color: %(PAPAYA)s; }
QSpinBox::up-button, QSpinBox::down-button { width: 14px; background: %(RAISED)s; border: 0;
                                             border-left: 1px solid %(BORDER)s; }
QSpinBox::up-button:hover, QSpinBox::down-button:hover { background: %(RAISED_2)s; }
QSpinBox::up-arrow { %(UP)s }
QSpinBox::down-arrow { %(DOWN)s }
QComboBox::drop-down { border: 0; width: 16px; }
QComboBox::down-arrow { %(DOWN)s }
QComboBox QAbstractItemView { background: %(RAISED)s; color: %(FG)s; border: 1px solid %(BORDER)s;
                              selection-background-color: %(RAISED_2)s; outline: 0; }

QSlider { min-height: 22px; }
QSlider::groove:horizontal { height: 4px; background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
    stop:0 %(RAISED_2)s, stop:0.5 %(BORDER)s, stop:1 %(RAISED_2)s); }
QSlider::handle:horizontal { width: 14px; height: 18px; margin: -7px 0; background: %(FG)s; }
QSlider::handle:horizontal:disabled { background: %(FAINT)s; }

QTableWidget { background: transparent; border: 0; gridline-color: transparent; color: %(FG)s;
               font-family: "%(MONO)s"; font-size: 12px; outline: 0; }
QTableWidget::item { padding: 0 4px; border: 0; }
QHeaderView { background: transparent; }
QHeaderView::section { background: transparent; color: %(FAINT)s; border: 0; border-bottom: 1px solid %(BORDER)s;
                       padding: 2px 4px; font-family: "%(MONO)s"; font-size: 12px; font-weight: 500; }

QScrollArea { background: transparent; border: 0; }
QScrollArea > QWidget > QWidget { background: transparent; }
QScrollBar:vertical { background: transparent; width: 8px; margin: 0; }
QScrollBar:horizontal { background: transparent; height: 8px; margin: 0; }
QScrollBar::handle { background: %(RAISED_2)s; min-height: 24px; min-width: 24px; }
QScrollBar::handle:hover { background: %(FAINT)s; }
QScrollBar::add-line, QScrollBar::sub-line { width: 0; height: 0; }
QScrollBar::add-page, QScrollBar::sub-page { background: none; }

QLabel[role="notice"] { background: %(RAISED)s; border-left: 3px solid #F2C14E; padding: 8px 10px;
                        color: %(MUTED)s; }
QFrame#verdict { background: %(RAISED)s; border-left: 4px solid %(FAINT)s; }
QFrame#verdict[state="target"] { background: #19261A; border-left-color: %(GREEN)s; }
QFrame#verdict[state="none"] { border-left-color: #F2C14E; }
''' % t


def apply(app):
    """Fonts, palette and the sheet, on the QApplication before any window."""
    load_fonts()
    app.setPalette(palette())
    app.setFont(font(14))
    app.setStyleSheet(stylesheet())


def restyle(widget, **props):
    """Set dynamic properties the sheet selects on, and re-polish so the new
    rule takes effect: the Qt form of swapping a CSS class."""
    for k, v in props.items():
        widget.setProperty(k, v)
    st = widget.style()
    st.unpolish(widget)
    st.polish(widget)
    widget.update()
