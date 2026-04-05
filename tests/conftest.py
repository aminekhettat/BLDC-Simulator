"""Global pytest configuration and shared fixtures for the SPINOTOR test suite."""

import pytest


@pytest.fixture(autouse=True)
def disable_tts(monkeypatch):
    """Prevent real pyttsx3 TTS calls during tests.

    On Linux CI environments (Ubuntu, GitHub Actions), ``pyttsx3.init()`` can
    hang for 80-90 seconds before timing out when no audio daemon or dbus
    session is available, causing the entire test step to appear to crash.

    This fixture patches ``pyttsx3`` to ``None`` in *src.utils.speech* for
    every test function, so ``speak()`` falls back to a console print instead
    of attempting real speech synthesis.

    Tests that specifically exercise pyttsx3 interaction (e.g.
    *test_feature_utils_misc.py*) override this by applying their own
    ``monkeypatch.setattr(speech, "pyttsx3", ...)`` within the test body,
    which takes precedence because both patches share the same function-scoped
    monkeypatch instance and pytest reverts them in reverse-application order.
    """
    import src.utils.speech as _speech_module

    monkeypatch.setattr(_speech_module, "pyttsx3", None)
