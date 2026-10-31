"""Deterministic Vietnamese grammar with explicit ambiguity and unknown errors."""

from dataclasses import dataclass
import re
import unicodedata

from turtlebot4.semantic_regions import DEFINITIONS


def normalize(text):
    text = unicodedata.normalize(
        "NFD", text.lower().replace("đ", "d").replace("Đ", "d")
    )
    text = "".join(c for c in text if unicodedata.category(c) != "Mn")
    return " ".join(re.sub(r"[^a-z0-9\s]", " ", text).split())


@dataclass(frozen=True)
class Command:
    regions: tuple = ()
    cancel: bool = False
    error: str = ""


class VoiceCommandParser:
    def __init__(self):
        self.aliases = {}
        for ident, name, _, aliases, _ in DEFINITIONS:
            for alias in [ident, name, *aliases, f"khu vuc {ident}"]:
                self.aliases.setdefault(normalize(alias), set()).add(ident)
        # Only unambiguous spoken letters, requiring explicit khu/khu vực context.
        for spoken, ident in [
            ("a", "A"),
            ("bi", "B"),
            ("xi", "C"),
            ("ep", "F"),
            ("hat", "H"),
        ]:
            for prefix in ("khu ", "khu vuc "):
                self.aliases.setdefault(prefix + spoken, set()).add(ident)

    def destination(self, text):
        text = normalize(text)
        matched = self.aliases.get(text, set())
        if len(matched) != 1:
            return None
        return next(iter(matched))

    def parse(self, text):
        text = normalize(text)
        if text in ("dung lai", "dung", "huy", "huy nhiem vu", "dung robot"):
            return Command(cancel=True)
        match = re.fullmatch(r"(?:di|di chuyen) (?:den|toi) (.+)", text)
        if match:
            parts = [match[1]]
        else:
            match = re.fullmatch(r"(?:di|di chuyen) tu (.+)", text)
            if not match:
                return Command(error="Không nhận ra cấu trúc lệnh")
            parts = re.split(r" (?:roi den|den|qua) ", match[1])
            if len(parts) < 2:
                return Command(error="Tuyến cần ít nhất hai vùng")
        regions = tuple(self.destination(part) for part in parts)
        if any(item is None for item in regions):
            return Command(error="Vùng không tồn tại hoặc câu nói mơ hồ; hãy nói lại")
        return Command(regions=regions)


class TranscriptGate:
    """Bounded cooldown for identical transcripts; cancellations always pass."""

    def __init__(self, cooldown=4.0):
        self.cooldown = cooldown
        self.last = None
        self.when = float("-inf")

    def accept(self, text, now):
        key = normalize(text)
        if not key or (key == self.last and now - self.when < self.cooldown):
            return False
        self.last, self.when = key, now
        return True
