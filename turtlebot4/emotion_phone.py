"""Explicit phone-image selection and guarded one-shot recognition."""
import math


def normalized_roi(start, end, width, height):
    x0, x1 = sorted((max(0, min(width, start[0])), max(0, min(width, end[0]))))
    y0, y1 = sorted((max(0, min(height, start[1])), max(0, min(height, end[1]))))
    if x1-x0 < 50 or y1-y0 < 50:
        return None
    return (x0/width, y0/height, x1/width, y1/height)


def score_margin(scores):
    values = sorted(scores.values(), reverse=True)
    if len(values) < 2 or not all(math.isfinite(v) and 0 <= v <= 1 for v in values):
        return 0.
    return values[0]-values[1]


def qualification_reason(label, score, scores, count, threshold, frames, margin):
    if label == 'None':
        return 'No single supported face inside selected region'
    if score < threshold:
        return f'Confidence {score:.2f} < {threshold:.2f}'
    gap = score_margin(scores)
    if gap < margin:
        return f'Ambiguous prediction: margin {gap:.2f} < {margin:.2f}'
    if count < frames:
        return f'Stabilizing {count}/{frames}'
    return ''
