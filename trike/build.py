"""Build the whole trike. build_all(overrides) -> (reg, K, P)."""
import importlib, copy
from .core import Registry
from .keypoints import load_params, keypoints

MODS = ["frame", "front", "rear", "drive", "seat", "canopy", "electrics"]

def build_all(path="params.yaml", param_overrides=None, mods=MODS):
    P = load_params(path) if path else None
    if param_overrides:
        for (sec, key), val in param_overrides.items():
            P[sec][key]["value"] = val
    K = keypoints(P); reg = Registry()
    for m in mods:
        importlib.import_module("trike." + m).build(reg, K, P)
    return reg, K, P
