"""M5 lift card, retaining the desktop console's lift-panel entry point."""
from tkinter import messagebox
from .aux_panel import AuxAxisPanel


class LiftPanel(AuxAxisPanel):
    """M5 specialization of the shared auxiliary card; inherits its limits."""
    def __init__(self, app, parent, settings):
        super().__init__(app, parent, 5, settings)
