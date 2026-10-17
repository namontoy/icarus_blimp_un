"""Shared widgets of the redesigned viewer (docs/GUI_REDESIGN.md §4).

PixelImageView (§4.5) is not here yet: it is built in Phase 3 together with
the Image & calibration tab, its first user, where the choice of §13.2
(pyqtgraph ViewBox in a scroll area, or a hand-painted widget) is made."""
from .controls import Segment, SegmentedButtons, ToggleSwitch
from .copy_button import CopyButton
from .pills import Banner, StatusPill
from .readout import Bar, FlowLayout, ReadoutCard, ReadoutStrip, ReadoutTable, value_label
from .section import CollapsibleSection, ElidedLabel, HeaderButton
from .splitter import GripSplitter

__all__ = ["Banner", "Bar", "CollapsibleSection", "CopyButton", "ElidedLabel",
           "FlowLayout", "GripSplitter", "HeaderButton", "ReadoutCard", "ReadoutStrip",
           "ReadoutTable", "Segment", "SegmentedButtons", "StatusPill", "ToggleSwitch",
           "value_label"]
