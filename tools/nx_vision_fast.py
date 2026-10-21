"""Compatibility entry for previous NX fast-detector imports.

The October 17 detector already shares intermediate images and supports scaled
work frames. Route legacy callers through it so they do not retain the old OCR
engine result contract or omit edge/concentric detection. No resources open on
import; camera ownership remains with the caller.
"""
from black_ring import make_masks, rings_from_mask, find_smallest_ring
from ring_digit import extract_ring_interior
from digit_core import recognize
