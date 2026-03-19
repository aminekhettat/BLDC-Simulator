"""
Atomic features tested in this module:
- ListMotorProfiles
- SaveMotorProfile
- LoadMotorProfile
- NormalizeMotorParamsValidation
- MotorParametersLdLq
"""
from __future__ import annotations

import json
from pathlib import Path

import pytest

from src.utils.motor_profiles import (
    PROFILE_SCHEMA,
    list_motor_profiles,
    load_motor_profile,
    save_motor_profile,
)
from src.core.motor_model import MotorParameters


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------


@pytest.fixture()
def tmp_profiles_dir(tmp_path: Path) -> Path:
    """Return a temporary directory for motor profile JSON files."""
    d = tmp_path / "profiles"
    d.mkdir()
    return d


@pytest.fixture()
def minimal_motor_params() -> dict:
    """Smallest valid set of motor parameters."""
    return {
        "nominal_voltage": 48.0,
        "phase_resistance": 2.5,
        "phase_inductance": 0.005,
        "back_emf_constant": 0.1,
        "torque_constant": 0.1,
        "rotor_inertia": 0.0005,
        "friction_coefficient": 0.001,
        "num_poles": 8,
    }


@pytest.fixture()
def saved_profile(tmp_profiles_dir: Path, minimal_motor_params: dict) -> Path:
    """Write one profile to disk and return its path."""
    fp = tmp_profiles_dir / "test_motor.json"
    save_motor_profile(fp, minimal_motor_params, profile_name="Test Motor")
    return fp


# ---------------------------------------------------------------------------
# list_motor_profiles
# ---------------------------------------------------------------------------


class TestListMotorProfiles:
    def test_empty_directory_returns_empty_list(self, tmp_profiles_dir: Path) -> None:
        assert list_motor_profiles(tmp_profiles_dir) == []

    def test_only_json_files_are_returned(self, tmp_profiles_dir: Path) -> None:
        (tmp_profiles_dir / "motor.json").write_text("{}")
        (tmp_profiles_dir / "readme.txt").write_text("ignore me")
        paths = list_motor_profiles(tmp_profiles_dir)
        assert len(paths) == 1
        assert paths[0].suffix == ".json"

    def test_multiple_profiles_are_sorted(self, tmp_profiles_dir: Path) -> None:
        names = ["c_motor.json", "a_motor.json", "b_motor.json"]
        for n in names:
            (tmp_profiles_dir / n).write_text("{}")
        paths = list_motor_profiles(tmp_profiles_dir)
        assert [p.name for p in paths] == sorted(names)

    def test_creates_directory_if_missing(self, tmp_path: Path) -> None:
        new_dir = tmp_path / "nonexistent"
        assert not new_dir.exists()
        list_motor_profiles(new_dir)
        assert new_dir.exists()


# ---------------------------------------------------------------------------
# save_motor_profile
# ---------------------------------------------------------------------------


class TestSaveMotorProfile:
    def test_file_is_created(
        self, tmp_profiles_dir: Path, minimal_motor_params: dict
    ) -> None:
        fp = tmp_profiles_dir / "saved.json"
        result = save_motor_profile(fp, minimal_motor_params, "My Motor")
        assert fp.exists()
        assert result == fp

    def test_saved_json_has_correct_schema(
        self, tmp_profiles_dir: Path, minimal_motor_params: dict
    ) -> None:
        fp = tmp_profiles_dir / "schema_check.json"
        save_motor_profile(fp, minimal_motor_params, "Schema Motor")
        payload = json.loads(fp.read_text(encoding="utf-8"))
        assert payload["schema"] == PROFILE_SCHEMA
        assert payload["profile_name"] == "Schema Motor"
        assert "motor_params" in payload

    def test_rated_info_and_source_are_stored(
        self, tmp_profiles_dir: Path, minimal_motor_params: dict
    ) -> None:
        fp = tmp_profiles_dir / "extra_info.json"
        save_motor_profile(
            fp,
            minimal_motor_params,
            "Motor With Extra",
            rated_info={"rated_speed_rpm": 3000},
            source_info={"manufacturer": "ACME"},
        )
        payload = json.loads(fp.read_text(encoding="utf-8"))
        assert payload["rated_info"]["rated_speed_rpm"] == 3000
        assert payload["source"]["manufacturer"] == "ACME"

    def test_parent_directories_are_created(
        self, tmp_path: Path, minimal_motor_params: dict
    ) -> None:
        nested = tmp_path / "deep" / "nested" / "motor.json"
        save_motor_profile(nested, minimal_motor_params, "Nested Motor")
        assert nested.exists()

    def test_poles_pairs_is_computed_and_stored(
        self, tmp_profiles_dir: Path, minimal_motor_params: dict
    ) -> None:
        fp = tmp_profiles_dir / "poles.json"
        save_motor_profile(fp, minimal_motor_params, "Poles Motor")
        payload = json.loads(fp.read_text(encoding="utf-8"))
        assert (
            payload["motor_params"]["poles_pairs"]
            == minimal_motor_params["num_poles"] // 2
        )


# ---------------------------------------------------------------------------
# load_motor_profile
# ---------------------------------------------------------------------------


class TestLoadMotorProfile:
    def test_round_trip_preserves_profile_name(self, saved_profile: Path) -> None:
        profile = load_motor_profile(saved_profile)
        assert profile["profile_name"] == "Test Motor"

    def test_round_trip_preserves_motor_params(
        self, saved_profile: Path, minimal_motor_params: dict
    ) -> None:
        profile = load_motor_profile(saved_profile)
        params = profile["motor_params"]
        assert params["nominal_voltage"] == pytest.approx(
            minimal_motor_params["nominal_voltage"]
        )
        assert params["phase_resistance"] == pytest.approx(
            minimal_motor_params["phase_resistance"]
        )

    def test_schema_field_is_returned(self, saved_profile: Path) -> None:
        profile = load_motor_profile(saved_profile)
        assert profile["schema"] == PROFILE_SCHEMA

    def test_rated_info_is_returned_as_dict(self, saved_profile: Path) -> None:
        profile = load_motor_profile(saved_profile)
        assert isinstance(profile["rated_info"], dict)

    def test_wrong_schema_raises_value_error(
        self, tmp_profiles_dir: Path, minimal_motor_params: dict
    ) -> None:
        fp = tmp_profiles_dir / "wrong_schema.json"
        payload = {
            "schema": "bldc.motor_profile.v99",
            "profile_name": "Bad Schema",
            "motor_params": minimal_motor_params,
        }
        fp.write_text(json.dumps(payload), encoding="utf-8")
        with pytest.raises(ValueError, match="Unsupported motor profile schema"):
            load_motor_profile(fp)

    def test_profile_name_falls_back_to_stem(
        self, tmp_profiles_dir: Path, minimal_motor_params: dict
    ) -> None:
        fp = tmp_profiles_dir / "my_motor_stem.json"
        payload = {
            "schema": PROFILE_SCHEMA,
            "profile_name": None,
            "motor_params": minimal_motor_params,
        }
        fp.write_text(json.dumps(payload), encoding="utf-8")
        profile = load_motor_profile(fp)
        assert profile["profile_name"] == "my_motor_stem"

    def test_ld_lq_default_to_phase_inductance_on_load(
        self, saved_profile: Path, minimal_motor_params: dict
    ) -> None:
        profile = load_motor_profile(saved_profile)
        params = profile["motor_params"]
        assert params["ld"] == pytest.approx(minimal_motor_params["phase_inductance"])
        assert params["lq"] == pytest.approx(minimal_motor_params["phase_inductance"])

    def test_explicit_ld_lq_are_preserved_on_load(
        self, tmp_profiles_dir: Path, minimal_motor_params: dict
    ) -> None:
        params_with_salient = dict(minimal_motor_params)
        params_with_salient["ld"] = 0.003
        params_with_salient["lq"] = 0.006
        fp = tmp_profiles_dir / "salient.json"
        save_motor_profile(fp, params_with_salient, "Salient Motor")
        profile = load_motor_profile(fp)
        assert profile["motor_params"]["ld"] == pytest.approx(0.003)
        assert profile["motor_params"]["lq"] == pytest.approx(0.006)


# ---------------------------------------------------------------------------
# _normalize_motor_params validation (exercised via save + load)
# ---------------------------------------------------------------------------


class TestNormalizeMotorParamsValidation:
    def test_missing_optional_key_falls_back_to_default(
        self, tmp_profiles_dir: Path, minimal_motor_params: dict
    ) -> None:
        # _normalize_motor_params seeds from DEFAULT_MOTOR_PARAMS, so omitted
        # params receive default values rather than raising an error.
        incomplete = dict(minimal_motor_params)
        del incomplete["phase_resistance"]
        fp = tmp_profiles_dir / "incomplete_ok.json"
        save_motor_profile(fp, incomplete, "Incomplete Motor")
        profile = load_motor_profile(fp)
        # phase_resistance should be filled by the default (2.5 Î©)
        from src.utils.config import DEFAULT_MOTOR_PARAMS

        assert profile["motor_params"]["phase_resistance"] == pytest.approx(
            DEFAULT_MOTOR_PARAMS["phase_resistance"]
        )

    def test_num_poles_less_than_2_raises_value_error(
        self, tmp_profiles_dir: Path, minimal_motor_params: dict
    ) -> None:
        bad = dict(minimal_motor_params)
        bad["num_poles"] = 1
        fp = tmp_profiles_dir / "bad_poles.json"
        with pytest.raises(ValueError, match="num_poles must be >= 2"):
            save_motor_profile(fp, bad, "Bad Poles Motor")

    def test_odd_num_poles_raises_value_error(
        self, tmp_profiles_dir: Path, minimal_motor_params: dict
    ) -> None:
        bad = dict(minimal_motor_params)
        bad["num_poles"] = 7
        fp = tmp_profiles_dir / "odd_poles.json"
        with pytest.raises(ValueError, match="num_poles must be even"):
            save_motor_profile(fp, bad, "Odd Poles Motor")

    def test_model_type_defaults_to_dq(self, saved_profile: Path) -> None:
        profile = load_motor_profile(saved_profile)
        assert profile["motor_params"]["model_type"] == "dq"

    def test_emf_shape_defaults_to_sinusoidal(self, saved_profile: Path) -> None:
        profile = load_motor_profile(saved_profile)
        assert profile["motor_params"]["emf_shape"] == "sinusoidal"


# ---------------------------------------------------------------------------
# MotorParameters â€” ld/lq defaulting (unit-level)
# ---------------------------------------------------------------------------


class TestMotorParametersLdLq:
    def test_ld_lq_default_to_phase_inductance_when_not_set(self) -> None:
        params = MotorParameters(phase_inductance=0.007)
        assert params.ld == pytest.approx(0.007)
        assert params.lq == pytest.approx(0.007)

    def test_ld_lq_respect_explicit_values(self) -> None:
        params = MotorParameters(
            phase_inductance=0.005,
            ld=0.003,
            lq=0.008,
        )
        assert params.ld == pytest.approx(0.003)
        assert params.lq == pytest.approx(0.008)

    def test_ld_equals_lq_for_non_salient_motor(self) -> None:
        params = MotorParameters(phase_inductance=0.004)
        assert params.ld == params.lq

    def test_ld_lq_differ_for_salient_pole_motor(self) -> None:
        params = MotorParameters(
            phase_inductance=0.005,
            ld=0.003,
            lq=0.009,
        )
        assert params.ld != params.lq
        assert params.lq > params.ld  # typical salient-pole relationship






