"""Camera-independent temporal recognition and edge-triggered gesture events."""
from collections import deque
import math


LABELS = ('Wave', 'Pointing_Up', 'Victory', 'Thumb_Up', 'Open_Palm', 'Closed_Fist')


class GestureFilter:
    def __init__(self, confidence=.65, stable_frames=3, cooldown=.7,
                 rearm_frames=4, lost_interval=.3, wave_window=2.4,
                 wave_amplitude=.045, wave_reversals=3, smoothing=.8):
        if (not 0 < confidence <= 1 or stable_frames < 1 or rearm_frames < 1
                or cooldown < 0 or lost_interval <= 0 or wave_window <= 0
                or wave_amplitude <= 0 or wave_reversals < 2 or not 0 < smoothing <= 1):
            raise ValueError('Invalid gesture filtering limits')
        self.confidence, self.stable_frames = confidence, stable_frames
        self.cooldown, self.rearm_frames = cooldown, rearm_frames
        self.lost_interval, self.wave_window = lost_interval, wave_window
        self.amplitude, self.reversals, self.smoothing = wave_amplitude, wave_reversals, smoothing
        self.samples = deque(maxlen=120)
        self.candidate, self.count, self.neutral = 'None', 0, 0
        self.latched, self.last_event, self.last_frame = None, -math.inf, -math.inf
        self.previous_wrist = self.corrected_wrist = None
        self.last_wave_hand = -math.inf

    def update(self, label, score, wrist, now, camera_shift=0., open_hand=False, hand_confidence=0.):
        if now - self.last_frame > self.lost_interval:
            self.samples.clear()
            self.count = 0
        self.last_frame = now
        label = label if label in LABELS and math.isfinite(score) and score >= self.confidence else 'None'
        # Wrist displacement is relative to background optical flow, so a camera
        # translation alone cannot satisfy the alternating-hand-motion test.
        wave_hand = label == 'Open_Palm' or (open_hand and hand_confidence >= self.confidence)
        if wave_hand and wrist is not None and abs(camera_shift) < .015:
            self.last_wave_hand = now
            raw = float(wrist) if self.previous_wrist is None else self.corrected_wrist + float(wrist)-self.previous_wrist-camera_shift
            self.previous_wrist, self.corrected_wrist = float(wrist), raw
            x = raw if not self.samples else self.smoothing * raw + (1-self.smoothing) * self.samples[-1][1]
            self.samples.append((now, x))
            while self.samples and now - self.samples[0][0] > self.wave_window:
                self.samples.popleft()
            if self.is_wave():
                label = 'Wave'
        elif wrist is not None or now-self.last_wave_hand > .15 or abs(camera_shift) >= .015:
            self.samples.clear()
            self.previous_wrist = self.corrected_wrist = None
        if label == 'None':
            self.neutral += 1
            if self.neutral >= self.rearm_frames:
                self.latched = None
        else:
            self.neutral = 0
        self.count = self.count + 1 if label == self.candidate else 1
        self.candidate = label
        event = (label != 'None' and self.count >= self.stable_frames
                 and label != self.latched and now - self.last_event >= self.cooldown)
        if event:
            self.latched, self.last_event = label, now
        return label, event

    def is_wave(self):
        if len(self.samples) < 8 or self.samples[-1][0] - self.samples[0][0] < .35:
            return False
        return self.wave_progress() >= self.reversals

    def wave_progress(self):
        if not self.samples:
            return 0
        anchor = self.samples[0][1]
        direction, reversals = 0, 0
        for _, x in self.samples:
            delta = x - anchor
            if abs(delta) < self.amplitude:
                continue
            new_direction = 1 if delta > 0 else -1
            if direction and new_direction != direction:
                reversals += 1
            direction, anchor = new_direction, x
        return reversals
