"""Camera-independent expression selection and neutral-rearmed event filtering."""
import math

LABELS = ('Happy', 'Sad', 'Neutral', 'Angry', 'Surprise', 'Fear')
BEHAVIORS = dict(Happy='left curve', Sad='right curve', Neutral='stop',
                 Angry='short reverse', Surprise='one full rotation', Fear='short forward')


def primary_face(faces):
    """Reject multiple faces: no identity switches while controlling motion."""
    if len(faces) != 1:
        return None
    face = faces[0]
    box, scores = face['box'], face['emotions']
    if len(box) != 4 or not all(math.isfinite(float(v)) for v in box) or min(box[2:]) <= 0:
        return None
    if not scores or not all(math.isfinite(float(v)) and 0 <= v <= 1 for v in scores.values()):
        return None
    # Do not renormalize six scores or turn dominant Disgust into another label.
    label = max(scores, key=scores.get).capitalize()
    return (label, float(scores[label.lower()]), box) if label in LABELS else None


class EmotionFilter:
    def __init__(self, confidence=.65, stable_frames=4, cooldown=1., no_face_timeout=.7):
        if (not math.isfinite(confidence) or not 0 < confidence <= 1
                or type(stable_frames) is not int or stable_frames < 2
                or not math.isfinite(cooldown) or cooldown < 0
                or not math.isfinite(no_face_timeout) or not .1 <= no_face_timeout <= 2):
            raise ValueError('Invalid expression filter settings')
        self.confidence, self.stable_frames = confidence, stable_frames
        self.cooldown, self.no_face_timeout = cooldown, no_face_timeout
        self.candidate, self.count = None, 0
        self.armed, self.last_event, self.last_face = False, -math.inf, -math.inf
        self.neutral_sent = None

    def update(self, label, score, now, idle=True, camera_ok=True, primary=True):
        visible = (camera_ok and primary and label in LABELS
                   and math.isfinite(score) and 0 <= score <= 1)
        if visible:
            self.last_face = now
        valid = visible and score >= self.confidence
        if not valid:
            self.candidate, self.count = None, 0
            self.neutral_sent = None
            if not camera_ok or not primary or now-self.last_face >= self.no_face_timeout:
                self.armed = False
            return dict(label='None', event=False, control_ok=(camera_ok and primary
                        and now-self.last_face < self.no_face_timeout))
        self.count = self.count+1 if self.candidate == label else 1
        self.candidate = label
        stable = self.count >= self.stable_frames
        event = False
        if stable and label == 'Neutral':
            # Neutral always stops, even during cooldown or execution. Rearm only
            # after the controller has confirmed its final stationary state.
            self.armed = idle
            event = self.neutral_sent != idle
            self.neutral_sent = idle
        elif stable and self.armed and idle and now-self.last_event >= self.cooldown:
            event, self.armed, self.last_event = True, False, now
        if label != 'Neutral':
            self.neutral_sent = None
        return dict(label=label, event=event, control_ok=True)
