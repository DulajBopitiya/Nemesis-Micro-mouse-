"""
theme.py - shared look & feel for the Nemisis debug app.

One place for:
  * apply_theme(app)  - Fusion base + a dark QPalette + a polished stylesheet,
                        so every window/tab/dialog is cohesive in one call.
  * FlowLayout        - a wrapping layout so toolbars reflow instead of clipping
                        controls off-screen when a panel is narrow.
  * chip(...)         - bundle a label + its field into one widget so a
                        FlowLayout never wraps between them.
  * accent(btn, kind) - tag a button as a coloured primary action
                        ("cyan" | "green" | "amber" | "red").

Palette anchors (kept in step with the maze canvas in maze_view.py):
  bg #0c0f12 · panel #11161c · line #2a333d · text #e4eaf1
  cyan #33dddd · green #46d17a · amber #ffbf47 · red #ff5f6d
"""

from __future__ import annotations

from PySide6 import QtCore, QtGui, QtWidgets
from PySide6.QtCore import Qt


# --- colour anchors ---------------------------------------------------------
BG      = "#0c0f12"
PANEL   = "#11161c"
INPUT   = "#12171d"
LINE    = "#2a333d"
LINE_HI = "#3b4a5a"
TEXT    = "#e4eaf1"
MUTED   = "#8b97a4"
FAINT   = "#63707d"
CYAN    = "#33dddd"
GREEN   = "#46d17a"
AMBER   = "#ffbf47"
RED     = "#ff5f6d"


APP_QSS = f"""
QToolTip {{
    background:{PANEL}; color:{TEXT};
    border:1px solid {LINE}; border-radius:5px; padding:5px 7px;
}}

QGroupBox {{
    border:1px solid #1c232b; border-radius:9px;
    margin-top:9px; padding:9px 8px 7px 8px; background:{PANEL};
}}
QGroupBox::title {{
    subcontrol-origin:margin; left:11px; padding:0 5px;
    color:{FAINT}; font-size:10px; font-weight:700;
}}

QLabel {{ color:{MUTED}; }}
QLabel#mazeStatus {{ color:#7b8794; padding:2px; }}

QPushButton {{
    background:#1b222b; color:#d3dbe4;
    border:1px solid {LINE}; border-radius:7px;
    padding:5px 12px;
}}
QPushButton:hover    {{ background:#232d38; border-color:{LINE_HI}; }}
QPushButton:pressed  {{ background:#161c24; }}
QPushButton:disabled {{ color:#5b6673; background:#151a20; border-color:#20262e; }}

/* accented primary actions — set via accent(btn, kind) */
QPushButton[accent="cyan"]  {{ color:{BG}; background:{CYAN}; border:none; font-weight:700; }}
QPushButton[accent="cyan"]:hover   {{ background:#5be9e9; }}
QPushButton[accent="cyan"]:pressed {{ background:#27b6b6; }}
QPushButton[accent="green"] {{ color:{BG}; background:{GREEN}; border:none; font-weight:700; }}
QPushButton[accent="green"]:hover  {{ background:#63e094; }}
QPushButton[accent="green"]:pressed{{ background:#38ac64; }}
QPushButton[accent="amber"] {{ color:{BG}; background:{AMBER}; border:none; font-weight:700; }}
QPushButton[accent="amber"]:hover  {{ background:#ffcf74; }}
QPushButton[accent="amber"]:pressed{{ background:#e0a531; }}
QPushButton[accent="red"]   {{ color:#0c0f12; background:{RED}; border:none; font-weight:700; }}
QPushButton[accent="red"]:hover    {{ background:#ff7a85; }}
QPushButton[accent="red"]:pressed  {{ background:#e04653; }}

QLineEdit, QSpinBox, QDoubleSpinBox, QComboBox {{
    background:{INPUT}; color:{TEXT};
    border:1px solid {LINE}; border-radius:6px; padding:3px 6px;
    selection-background-color:{CYAN}; selection-color:{BG};
}}
QLineEdit:hover, QSpinBox:hover, QDoubleSpinBox:hover, QComboBox:hover {{
    border-color:{LINE_HI};
}}
QLineEdit:focus, QSpinBox:focus, QDoubleSpinBox:focus, QComboBox:focus {{
    border-color:{CYAN};
}}
QComboBox::drop-down {{ border:none; width:16px; }}
QComboBox QAbstractItemView {{
    background:{INPUT}; color:{TEXT}; border:1px solid {LINE};
    selection-background-color:{CYAN}; selection-color:{BG};
}}
QSpinBox::up-button, QSpinBox::down-button,
QDoubleSpinBox::up-button, QDoubleSpinBox::down-button {{
    width:14px; border:none; background:#1b222b;
}}
QSpinBox::up-button:hover, QSpinBox::down-button:hover,
QDoubleSpinBox::up-button:hover, QDoubleSpinBox::down-button:hover {{
    background:#2a333d;
}}

QCheckBox {{ color:{MUTED}; spacing:6px; }}
QCheckBox::indicator {{
    width:15px; height:15px; border-radius:4px;
    border:1px solid {LINE_HI}; background:{INPUT};
}}
QCheckBox::indicator:hover {{ border-color:{CYAN}; }}
QCheckBox::indicator:checked {{ background:{CYAN}; border-color:{CYAN}; image:none; }}

QSlider::groove:horizontal {{
    height:5px; border-radius:3px; background:#1b222b;
}}
QSlider::sub-page:horizontal {{ background:{CYAN}; border-radius:3px; }}
QSlider::handle:horizontal {{
    width:14px; margin:-6px 0; border-radius:7px;
    background:{TEXT}; border:2px solid {CYAN};
}}

QTabWidget::pane {{ border:1px solid #1c232b; border-radius:8px; top:-1px; }}
QTabBar::tab {{
    background:transparent; color:{MUTED};
    padding:6px 16px; margin-right:2px;
    border:1px solid transparent;
    border-top-left-radius:7px; border-top-right-radius:7px;
}}
QTabBar::tab:hover    {{ color:{TEXT}; }}
QTabBar::tab:selected {{
    color:{BG}; background:{CYAN}; font-weight:700;
}}

QSplitter::handle {{ background:#1c232b; }}
QSplitter::handle:horizontal {{ width:4px; }}
QSplitter::handle:vertical   {{ height:4px; }}

QScrollBar:vertical   {{ background:transparent; width:11px; margin:0; }}
QScrollBar:horizontal {{ background:transparent; height:11px; margin:0; }}
QScrollBar::handle:vertical, QScrollBar::handle:horizontal {{
    background:#2c353f; border-radius:5px; min-height:24px; min-width:24px;
}}
QScrollBar::handle:hover {{ background:{LINE_HI}; }}
QScrollBar::add-line, QScrollBar::sub-line {{ height:0; width:0; }}
QScrollBar::add-page, QScrollBar::sub-page {{ background:transparent; }}

QTableWidget {{
    background:{INPUT}; alternate-background-color:#161c24;
    gridline-color:#20262e; color:{TEXT};
    border:1px solid {LINE}; border-radius:6px;
}}
QHeaderView::section {{
    background:#1b222b; color:{MUTED};
    padding:4px 6px; border:none; border-right:1px solid #20262e;
    font-weight:600;
}}
QTableWidget::item:selected {{ background:{CYAN}; color:{BG}; }}

QPlainTextEdit {{
    background:#0b0e11; color:#cfe;
    border:1px solid {LINE}; border-radius:6px;
    selection-background-color:{CYAN}; selection-color:{BG};
}}
"""


def apply_theme(app: QtWidgets.QApplication) -> None:
    """Make the whole app cohesive: Fusion base + dark palette + APP_QSS."""
    app.setStyle("Fusion")

    pal = QtGui.QPalette()
    pal.setColor(QtGui.QPalette.Window, QtGui.QColor("#12171d"))
    pal.setColor(QtGui.QPalette.WindowText, QtGui.QColor(TEXT))
    pal.setColor(QtGui.QPalette.Base, QtGui.QColor(INPUT))
    pal.setColor(QtGui.QPalette.AlternateBase, QtGui.QColor("#161c24"))
    pal.setColor(QtGui.QPalette.Text, QtGui.QColor(TEXT))
    pal.setColor(QtGui.QPalette.Button, QtGui.QColor("#1b222b"))
    pal.setColor(QtGui.QPalette.ButtonText, QtGui.QColor("#d3dbe4"))
    pal.setColor(QtGui.QPalette.ToolTipBase, QtGui.QColor(PANEL))
    pal.setColor(QtGui.QPalette.ToolTipText, QtGui.QColor(TEXT))
    pal.setColor(QtGui.QPalette.PlaceholderText, QtGui.QColor("#6b7783"))
    pal.setColor(QtGui.QPalette.Highlight, QtGui.QColor(CYAN))
    pal.setColor(QtGui.QPalette.HighlightedText, QtGui.QColor(BG))
    pal.setColor(QtGui.QPalette.Link, QtGui.QColor(CYAN))
    for grp in (QtGui.QPalette.Disabled,):
        pal.setColor(grp, QtGui.QPalette.WindowText, QtGui.QColor("#5b6673"))
        pal.setColor(grp, QtGui.QPalette.Text, QtGui.QColor("#5b6673"))
        pal.setColor(grp, QtGui.QPalette.ButtonText, QtGui.QColor("#5b6673"))
    app.setPalette(pal)

    app.setStyleSheet(APP_QSS)


def accent(btn: QtWidgets.QAbstractButton, kind: str) -> QtWidgets.QAbstractButton:
    """Tag a button as a coloured primary action and refresh its style.
    kind: 'cyan' | 'green' | 'amber' | 'red'. Returns the button for chaining."""
    btn.setProperty("accent", kind)
    btn.style().unpolish(btn)
    btn.style().polish(btn)
    return btn


class FlowLayout(QtWidgets.QLayout):
    """Lays widgets left-to-right and wraps to the next line when it runs out of
    horizontal room — so a toolbar can never clip a control off-screen (it just
    flows down instead)."""

    def __init__(self, parent=None, margin=0, hspacing=6, vspacing=6):
        super().__init__(parent)
        self._items: list[QtWidgets.QLayoutItem] = []
        self._hspace = hspacing
        self._vspace = vspacing
        self.setContentsMargins(margin, margin, margin, margin)

    def __del__(self):
        while self._items:
            self._items.pop()

    def addItem(self, item):
        self._items.append(item)

    def count(self):
        return len(self._items)

    def itemAt(self, i):
        return self._items[i] if 0 <= i < len(self._items) else None

    def takeAt(self, i):
        return self._items.pop(i) if 0 <= i < len(self._items) else None

    def expandingDirections(self):
        return Qt.Orientations(Qt.Orientation(0))

    def hasHeightForWidth(self):
        return True

    def heightForWidth(self, width):
        return self._do_layout(QtCore.QRect(0, 0, width, 0), test_only=True)

    def setGeometry(self, rect):
        super().setGeometry(rect)
        self._do_layout(rect, test_only=False)

    def sizeHint(self):
        return self.minimumSize()

    def minimumSize(self):
        size = QtCore.QSize()
        for item in self._items:
            size = size.expandedTo(item.minimumSize())
        m = self.contentsMargins()
        size += QtCore.QSize(m.left() + m.right(), m.top() + m.bottom())
        return size

    def _do_layout(self, rect, test_only):
        m = self.contentsMargins()
        x = rect.x() + m.left()
        y = rect.y() + m.top()
        right = rect.right() - m.right()
        line_h = 0
        for item in self._items:
            hint = item.sizeHint()
            nx = x + hint.width()
            if nx - 1 > right and line_h > 0:      # wrap to next row
                x = rect.x() + m.left()
                y = y + line_h + self._vspace
                nx = x + hint.width()
                line_h = 0
            if not test_only:
                item.setGeometry(QtCore.QRect(QtCore.QPoint(x, y), hint))
            x = nx + self._hspace
            line_h = max(line_h, hint.height())
        return y + line_h - rect.y() + m.bottom()


def chip(*widgets, spacing=5) -> QtWidgets.QWidget:
    """Bundle a label + its control(s) into one widget so a FlowLayout keeps the
    pair together instead of wrapping between the label and its field. Strings
    become QLabels."""
    w = QtWidgets.QWidget()
    h = QtWidgets.QHBoxLayout(w)
    h.setContentsMargins(0, 0, 0, 0)
    h.setSpacing(spacing)
    for x in widgets:
        if isinstance(x, str):
            x = QtWidgets.QLabel(x)
        h.addWidget(x)
    return w
