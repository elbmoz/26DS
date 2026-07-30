"""Small hardware-independent helpers for deterministic loop scheduling."""


def periodic_due(now_ms, next_due_ms, period_ms):
    """Return ``(due, next_deadline)`` without accumulating loop drift.

    Missed periods are skipped rather than emitted in a burst. Keeping the
    deadline on the original time grid avoids frame-rate quantization such as
    a requested 50 Hz output collapsing to half the detector frame rate.
    """
    now_ms = int(now_ms)
    next_due_ms = int(next_due_ms)
    period_ms = max(1, int(period_ms))
    if now_ms < next_due_ms:
        return False, next_due_ms
    missed = (now_ms - next_due_ms) // period_ms
    return True, next_due_ms + (missed + 1) * period_ms
