import copy
import os


FULL_CONFIG_KEYS = ("shared", "02_train", "03_eval")

LOCAL_IIT_ROOT = "C:/Github/IIT"
SERVER_IIT_ROOT = "/home/chanyoungko/IIT"


def deep_update(target, update):
    for key, value in update.items():
        if isinstance(value, dict) and isinstance(target.get(key), dict):
            deep_update(target[key], value)
        else:
            target[key] = copy.deepcopy(value)
    return target


def is_full_experiment_config(cfg):
    return isinstance(cfg, dict) and all(key in cfg for key in FULL_CONFIG_KEYS)


def get_shared_config(cfg):
    return cfg.get("shared", {}) if isinstance(cfg, dict) else {}


def get_train_config(cfg):
    return cfg.get("02_train", {}) if isinstance(cfg, dict) else {}


def get_eval_config(cfg):
    return cfg.get("03_eval", {}) if isinstance(cfg, dict) else {}


def get_train_data_config(cfg):
    train_cfg = get_train_config(cfg)
    if isinstance(train_cfg.get("data"), dict):
        return train_cfg["data"]
    return {}


def get_shared_data_config(cfg):
    shared_cfg = get_shared_config(cfg)
    if isinstance(shared_cfg.get("data"), dict):
        return shared_cfg["data"]
    return {}


def get_model_config(cfg):
    train_cfg = get_train_config(cfg)
    if isinstance(train_cfg.get("model"), dict):
        return train_cfg["model"]
    shared_cfg = get_shared_config(cfg)
    if isinstance(shared_cfg.get("model"), dict):
        return shared_cfg["model"]
    model_cfg = cfg.get("model", {}) if isinstance(cfg, dict) else {}
    return model_cfg if isinstance(model_cfg, dict) else {}


def get_data_sources(cfg):
    shared_cfg = get_shared_config(cfg)
    raw_sources = shared_cfg.get("data_sources", {}) or cfg.get("data_sources", {})
    if not isinstance(raw_sources, dict):
        return {}

    resolved = {}
    for name, source in raw_sources.items():
        if not isinstance(source, dict):
            continue
        source_copy = copy.deepcopy(source)
        resolved_path = resolve_data_source_path(source_copy)
        if resolved_path:
            source_copy["path"] = resolved_path
        resolved[name] = source_copy
    return resolved


def _translate_known_iit_path(path_value):
    if not path_value:
        return None

    normalized = str(path_value).replace("\\", "/")

    if os.name == "nt" and normalized.startswith(f"{SERVER_IIT_ROOT}/"):
        tail = normalized[len(SERVER_IIT_ROOT) + 1 :]
        return f"{LOCAL_IIT_ROOT}/{tail}"

    if os.name != "nt" and normalized.startswith(f"{LOCAL_IIT_ROOT}/"):
        tail = normalized[len(LOCAL_IIT_ROOT) + 1 :]
        return f"{SERVER_IIT_ROOT}/{tail}"

    return None


def _fallback_local_iit_candidates(path_value):
    if not path_value or os.name != "nt":
        return []

    normalized = str(path_value).replace("\\", "/")
    basename = os.path.basename(normalized)
    candidates = []

    if basename:
        candidates.append(f"{LOCAL_IIT_ROOT}/{basename}")

    if normalized.endswith("/combined_data.h5"):
        candidates.append(f"{LOCAL_IIT_ROOT}/combined_data.h5")

    deduped = []
    for candidate in candidates:
        if candidate not in deduped:
            deduped.append(candidate)
    return deduped


def resolve_data_source_path(source):
    if isinstance(source, dict):
        if os.name == "nt":
            preferred_keys = ("local_path", "windows_path", "path", "server_path", "linux_path")
        else:
            preferred_keys = ("path", "server_path", "linux_path", "local_path", "windows_path")

        candidates = []
        for key in preferred_keys:
            value = source.get(key)
            if value:
                candidates.append(str(value))
                translated = _translate_known_iit_path(value)
                if translated:
                    candidates.append(translated)
                candidates.extend(_fallback_local_iit_candidates(value))
    else:
        candidates = [str(source)] if source else []
        translated = _translate_known_iit_path(source)
        if translated:
            candidates.append(translated)
        candidates.extend(_fallback_local_iit_candidates(source))

    deduped = []
    for candidate in candidates:
        if candidate and candidate not in deduped:
            deduped.append(candidate)

    for candidate in deduped:
        if os.path.exists(candidate):
            return candidate

    return deduped[0] if deduped else None


def get_primary_data_path(cfg):
    if not isinstance(cfg, dict):
        return None
    if cfg.get("data_path"):
        return resolve_data_source_path(cfg["data_path"])
    construction_cfg = cfg.get("01_construction", {})
    if isinstance(construction_cfg, dict) and construction_cfg.get("src_h5"):
        return resolve_data_source_path(construction_cfg["src_h5"])
    shared_cfg = get_shared_config(cfg)
    if shared_cfg.get("src_h5"):
        return resolve_data_source_path(shared_cfg["src_h5"])
    data_sources = get_data_sources(cfg)
    for source in data_sources.values():
        if isinstance(source, dict) and source.get("path"):
            return source["path"]
    return None


def get_config_subjects(cfg):
    if not isinstance(cfg, dict):
        return []
    if cfg.get("subjects"):
        return copy.deepcopy(cfg["subjects"])
    construction_cfg = cfg.get("01_construction", {})
    if isinstance(construction_cfg, dict) and construction_cfg.get("include_subjects"):
        return copy.deepcopy(construction_cfg["include_subjects"])
    shared_cfg = get_shared_config(cfg)
    return copy.deepcopy(shared_cfg.get("subjects", []))


def get_config_conditions(cfg):
    if not isinstance(cfg, dict):
        return []
    if cfg.get("conditions"):
        return copy.deepcopy(cfg["conditions"])
    construction_cfg = cfg.get("01_construction", {})
    if isinstance(construction_cfg, dict) and construction_cfg.get("include_conditions"):
        return copy.deepcopy(construction_cfg["include_conditions"])
    shared_cfg = get_shared_config(cfg)
    return copy.deepcopy(shared_cfg.get("conditions", []))


def get_config_input_vars(cfg):
    if not isinstance(cfg, dict):
        return []
    if cfg.get("input_vars"):
        return copy.deepcopy(cfg["input_vars"])
    construction_cfg = cfg.get("01_construction", {})
    if isinstance(construction_cfg, dict) and construction_cfg.get("inputs"):
        return copy.deepcopy(construction_cfg["inputs"])
    shared_cfg = get_shared_config(cfg)
    return copy.deepcopy(shared_cfg.get("input_vars", []))


def get_config_output_vars(cfg):
    if not isinstance(cfg, dict):
        return []
    if cfg.get("output_vars"):
        return copy.deepcopy(cfg["output_vars"])
    construction_cfg = cfg.get("01_construction", {})
    if isinstance(construction_cfg, dict) and construction_cfg.get("outputs"):
        return copy.deepcopy(construction_cfg["outputs"])
    shared_cfg = get_shared_config(cfg)
    return copy.deepcopy(shared_cfg.get("output_vars", []))


def get_config_level_selection(cfg):
    if not isinstance(cfg, dict):
        return None
    if cfg.get("level_selection"):
        return copy.deepcopy(cfg["level_selection"])
    shared_cfg = get_shared_config(cfg)
    return copy.deepcopy(shared_cfg.get("level_selection"))


def get_window_size(cfg, default=200):
    if not isinstance(cfg, dict):
        return default
    if cfg.get("time_window_input"):
        return cfg["time_window_input"]
    data_cfg = get_train_data_config(cfg)
    if data_cfg.get("window_size") is not None:
        return data_cfg["window_size"]
    shared_data_cfg = get_shared_data_config(cfg)
    if shared_data_cfg.get("window_size") is not None:
        return shared_data_cfg["window_size"]
    return default


def get_est_tick_ranges(cfg, default=None):
    if not isinstance(cfg, dict):
        return default
    if cfg.get("est_tick_ranges") is not None:
        return copy.deepcopy(cfg["est_tick_ranges"])
    for data_cfg in (get_train_data_config(cfg), get_shared_data_config(cfg)):
        if data_cfg.get("est_tick_ranges") is not None:
            return copy.deepcopy(data_cfg["est_tick_ranges"])
        if data_cfg.get("y_delay") is not None:
            y_delay = data_cfg["y_delay"]
            return [y_delay] if isinstance(y_delay, int) else copy.deepcopy(y_delay)
    return copy.deepcopy(default)


def get_input_lpf_cutoff(cfg):
    if not isinstance(cfg, dict):
        return None
    if cfg.get("input_lpf_cutoff_hz") is not None:
        return cfg["input_lpf_cutoff_hz"]
    shared_cfg = get_shared_config(cfg)
    if shared_cfg.get("input_lpf_cutoff_hz") is not None:
        return shared_cfg["input_lpf_cutoff_hz"]
    data_cfg = get_train_data_config(cfg)
    return data_cfg.get("input_lpf_cutoff_hz")


def get_input_lpf_order(cfg, default=4):
    if not isinstance(cfg, dict):
        return default
    if cfg.get("input_lpf_order") is not None:
        return cfg["input_lpf_order"]
    shared_cfg = get_shared_config(cfg)
    if shared_cfg.get("input_lpf_order") is not None:
        return shared_cfg["input_lpf_order"]
    data_cfg = get_train_data_config(cfg)
    return data_cfg.get("input_lpf_order", default)


def get_tcn_channels(cfg, fallback_layers=3, fallback_hidden=64):
    if not isinstance(cfg, dict):
        return [fallback_hidden] * fallback_layers
    if cfg.get("tcn_channels"):
        return copy.deepcopy(cfg["tcn_channels"])
    model_cfg = get_model_config(cfg)
    if model_cfg.get("channels"):
        return copy.deepcopy(model_cfg["channels"])
    data_cfg = get_train_data_config(cfg)
    if data_cfg.get("channels"):
        return copy.deepcopy(data_cfg["channels"])
    n_layers = cfg.get("tcn_layers", fallback_layers)
    n_hidden = cfg.get("tcn_hidden", fallback_hidden)
    return [n_hidden] * n_layers


def normalize_experiment_config(cfg, base_config=None, exp_name=None):
    if not isinstance(cfg, dict):
        raise TypeError("Config must be a dictionary")

    if is_full_experiment_config(cfg):
        merged = copy.deepcopy(cfg)
    else:
        merged = copy.deepcopy(base_config) if isinstance(base_config, dict) else {}
        deep_update(merged, cfg)

    if exp_name:
        merged["exp_name"] = cfg.get("exp_name", exp_name)

    merged["subjects"] = get_config_subjects(merged)
    merged["conditions"] = get_config_conditions(merged)

    if get_config_input_vars(merged):
        merged["input_vars"] = get_config_input_vars(merged)
    if get_config_output_vars(merged):
        merged["output_vars"] = get_config_output_vars(merged)

    level_selection = get_config_level_selection(merged)
    if level_selection:
        merged["level_selection"] = level_selection

    data_path = get_primary_data_path(merged)
    if data_path:
        merged["data_path"] = data_path

    est_tick_ranges = get_est_tick_ranges(merged)
    if est_tick_ranges is not None:
        merged["est_tick_ranges"] = est_tick_ranges

    return merged
