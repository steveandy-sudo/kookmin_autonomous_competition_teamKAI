from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Iterable, Optional


POLICIES = ("drive", "cone", "overtake")
MODEL_TYPES = ("pilotnet", "mobilenet_v3_small", "resnet18", "vit_tiny")
PHASE_MODES = ("auto", "on", "off")

DEFAULT_MODEL_BY_POLICY: Dict[str, str] = {
    "drive": "resnet18",
    "cone": "pilotnet",
    "overtake": "pilotnet",
}

EXPERIMENTAL_MODELS = {"vit_tiny"}


@dataclass(frozen=True)
class PolicyModelSpec:
    policy: str
    model_type: str
    phase_enabled: bool
    model_variant: str
    experimental: bool


def resolve_policy_model(
    policy: str,
    model_type: Optional[str] = None,
    phase_mode: str = "auto",
) -> PolicyModelSpec:
    policy = normalize_choice(policy, POLICIES, "policy")
    phase_mode = normalize_choice(phase_mode, PHASE_MODES, "phase_mode")
    resolved_model = model_type or DEFAULT_MODEL_BY_POLICY[policy]
    resolved_model = normalize_model_type(resolved_model)

    phase_enabled = phase_mode == "on" or (phase_mode == "auto" and policy == "overtake")
    model_variant = f"{resolved_model}_phase" if phase_enabled else resolved_model
    return PolicyModelSpec(
        policy=policy,
        model_type=resolved_model,
        phase_enabled=phase_enabled,
        model_variant=model_variant,
        experimental=resolved_model in EXPERIMENTAL_MODELS,
    )


def normalize_model_type(value: str) -> str:
    cleaned = str(value).strip().lower().replace("-", "_")
    if cleaned.endswith("_phase"):
        cleaned = cleaned[: -len("_phase")]
    return normalize_choice(cleaned, MODEL_TYPES, "model_type")


def normalize_choice(value: str, choices: Iterable[str], name: str) -> str:
    cleaned = str(value).strip().lower().replace("-", "_")
    allowed = tuple(choices)
    if cleaned not in allowed:
        raise ValueError(f"{name} must be one of {', '.join(allowed)}; got {value!r}")
    return cleaned


def default_model_summary() -> Dict[str, str]:
    return {
        "drive": "resnet18",
        "cone": "pilotnet",
        "overtake": "pilotnet_phase",
    }
