"""Simple vocal assistance utility.

Provides text-to-speech output to assist visually impaired users. The module
wraps `pyttsx3` if installed, otherwise falls back to printing to console.

:author: BLDC Control Team
:version: 1.0.0
"""

from typing import Optional

try:
    import pyttsx3
except ImportError:  # pragma: no cover - optional dependency
    pyttsx3 = None


def speak(message: str, rate: Optional[int] = None) -> None:
    """Announce a message vocally.

    :param message: Text to speak
    :param rate: Optional speech rate (words per minute)
    """
    if pyttsx3 is None:
        # fallback - print to console for debugging
        print(f"[Speech] {message}")
        return

    engine = pyttsx3.init()
    if rate is not None:
        engine.setProperty("rate", rate)
    engine.say(message)
    engine.runAndWait()
