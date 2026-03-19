"""Simple vocal assistance utility.

Provides text-to-speech output to assist visually impaired users. The module
wraps `pyttsx3` if installed, otherwise falls back to printing to console.

:author: BLDC Control Team
:version: 0.8.0
"""

from typing import Optional

try:
    import pyttsx3
except ImportError:  # pragma: no cover - optional dependency
    pyttsx3 = None


_AUDIO_ASSISTANCE_ENABLED = True


def set_audio_assistance_enabled(enabled: bool) -> None:
    """Enable or disable text-to-speech output globally."""
    global _AUDIO_ASSISTANCE_ENABLED
    _AUDIO_ASSISTANCE_ENABLED = bool(enabled)


def is_audio_assistance_enabled() -> bool:
    """Return current global audio-assistance state."""
    return _AUDIO_ASSISTANCE_ENABLED


def speak(message: str, rate: Optional[int] = None) -> None:
    """Announce a message vocally.

    :param message: Text to speak
    :param rate: Optional speech rate (words per minute)
    """
    if not _AUDIO_ASSISTANCE_ENABLED:
        return

    if pyttsx3 is None:
        # fallback - print to console for debugging
        print(f"[Speech] {message}")
        return

    engine = pyttsx3.init()
    if rate is not None:
        engine.setProperty("rate", rate)
    engine.say(message)
    engine.runAndWait()
