
# 실행 코드 : streamlit run GUI_yaml_export.py


import streamlit as st
import yaml
import itertools
import math
import re
from pathlib import Path
import io
import zipfile
from datetime import datetime

st.set_page_config(page_title="YAML Grid Generator", layout="wide")

def safe_load_yaml_bytes(b: bytes):
    return yaml.safe_load(b.decode("utf-8"))

def dump_yaml(obj) -> str:
    return yaml.safe_dump(obj, sort_keys=False, allow_unicode=True)

def parse_csv_values(text: str, cast_fn):
    vals = []
    for part in [p.strip() for p in text.split(",") if p.strip()]:
        vals.append(cast_fn(part))
    return vals

def linspace(start, stop, num):
    if num <= 1:
        return [start]
    step = (stop - start) / (num - 1)
    return [start + i * step for i in range(num)]

def logspace(start, stop, num):
    # start/stop are exponents (base10)
    exps = linspace(start, stop, num)
    return [10 ** e for e in exps]

def infer_type(value):
    if isinstance(value, bool):
        return "bool"
    if isinstance(value, int) and not isinstance(value, bool):
        return "int"
    if isinstance(value, float):
        return "float"
    if isinstance(value, str):
        return "str"
    if isinstance(value, list):
        return "list"
    if isinstance(value, dict):
        return "dict"
    return "other"

def yaml_literal_list_parser(text: str):
    # Options separated by lines containing only ---
    blocks = re.split(r"(?m)^\s*---\s*$", text.strip())
    opts = []
    for b in blocks:
        if not b.strip():
            continue
        opts.append(yaml.safe_load(b))
    return opts

def make_exp_name(base_exp_name: str, combo_idx: int):
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    return f"{base_exp_name}_grid_{ts}_{combo_idx:04d}"

st.title("YAML Grid Generator")

with st.sidebar:
    st.header("Base YAML")
    uploaded = st.file_uploader("base yaml 업로드", type=["yaml", "yml"])
    default_path = st.text_input("또는 경로", value="configs/base_config.yaml")
    st.markdown("업로드가 있으면 업로드가 우선입니다.")

base = None
base_bytes = None

if uploaded is not None:
    base_bytes = uploaded.getvalue()
    try:
        base = safe_load_yaml_bytes(base_bytes)
    except Exception as e:
        st.error(f"yaml 파싱 실패: {e}")
else:
    p = Path(default_path)
    if p.exists():
        base_bytes = p.read_bytes()
        try:
            base = safe_load_yaml_bytes(base_bytes)
        except Exception as e:
            st.error(f"yaml 파싱 실패: {e}")
    else:
        st.info("좌측에서 base yaml을 업로드하거나 올바른 경로를 입력하세요.")

if base is None:
    st.stop()

if not isinstance(base, dict):
    st.error("base yaml 최상위가 dict(map) 형태여야 합니다.")
    st.stop()

st.subheader("Base Config 미리보기")
st.code(dump_yaml(base), language="yaml")

st.divider()
st.subheader("Grid 설정")

top_keys = list(base.keys())

# Exclude keys that should be fixed/hidden
excluded_keys = ["lpf_cutoff", "lpf_order", "tcn_channels"] # tcn_channels is replaced by tcn_layers
top_keys = [k for k in top_keys if k not in excluded_keys]

if "sweeps" not in st.session_state:
    st.session_state["sweeps"] = []  # list of dicts: {key, mode, values}

colA, colB = st.columns([1, 2], gap="large")

with colA:
    st.markdown("추가할 파라미터")
    sel_key = st.selectbox("key", options=top_keys)
    cur_val = base.get(sel_key)
    cur_type = infer_type(cur_val)
    st.write("현재 값:", cur_val)
    st.write("타입:", cur_type)

    mode = st.selectbox("설정 방식", options=["직접 값 리스트", "linspace", "logspace(10^x)", "bool 토글", "yaml literal 옵션(list/dict)"])

    values = None

    if mode == "직접 값 리스트":
        if cur_type in ["int", "float"]:
            txt = st.text_input("값들 (comma)", value=str(cur_val))
            if cur_type == "int":
                values = parse_csv_values(txt, int)
            else:
                values = parse_csv_values(txt, float)
        elif cur_type == "str":
            txt = st.text_input("값들 (comma)", value=str(cur_val))
            values = [p.strip() for p in txt.split(",") if p.strip()]
        else:
            st.info("이 모드는 int/float/str에 권장됩니다. list/dict는 yaml literal 옵션을 쓰세요.")
            txt = st.text_input("값들 (comma)", value="")
            values = [p.strip() for p in txt.split(",") if p.strip()]

    elif mode == "linspace":
        if cur_type not in ["int", "float"]:
            st.warning("linspace는 숫자에만 사용하세요.")
        start = st.number_input("start", value=float(cur_val) if cur_type in ["int","float"] else 0.0)
        stop = st.number_input("stop", value=float(cur_val) if cur_type in ["int","float"] else 1.0)
        num = st.number_input("num", min_value=1, value=5, step=1)
        vals = linspace(float(start), float(stop), int(num))
        if cur_type == "int":
            values = [int(round(v)) for v in vals]
        else:
            values = vals

    elif mode == "logspace(10^x)":
        if cur_type not in ["int", "float"]:
            st.warning("logspace는 숫자에만 사용하세요.")
        st.caption("입력은 지수(exponent)입니다. 예: -5 ~ -3 -> 1e-5 ~ 1e-3")
        exp_start = st.number_input("exp start", value=-5.0)
        exp_stop = st.number_input("exp stop", value=-3.0)
        num = st.number_input("num", min_value=1, value=5, step=1)
        vals = logspace(float(exp_start), float(exp_stop), int(num))
        if cur_type == "int":
            values = [int(round(v)) for v in vals]
        else:
            values = vals

    elif mode == "bool 토글":
        values = [True, False]

    elif mode == "yaml literal 옵션(list/dict)":
        st.caption("옵션을 yaml로 각각 쓰고, 옵션 구분은 단독 줄 '---' 로 하세요.")
        placeholder = ""
        if cur_type in ["list", "dict"]:
            placeholder = dump_yaml(cur_val).strip()
        txt = st.text_area("옵션들", value=placeholder, height=200)
        if txt.strip():
            try:
                values = yaml_literal_list_parser(txt)
            except Exception as e:
                st.error(f"옵션 파싱 실패: {e}")
                values = None

    add_btn = st.button("이 key를 sweep에 추가", type="primary", use_container_width=True, disabled=(values is None or len(values) == 0))

    if add_btn and values is not None and len(values) > 0:
        st.session_state["sweeps"].append({"key": sel_key, "values": values})
        st.success(f"추가됨: {sel_key} ({len(values)}개)")

with colB:
    st.markdown("현재 sweep 목록")
    if len(st.session_state["sweeps"]) == 0:
        st.info("좌측에서 key를 추가하세요.")
    else:
        for i, s in enumerate(st.session_state["sweeps"]):
            c1, c2, c3 = st.columns([2, 6, 1])
            with c1:
                st.write(s["key"])
            with c2:
                st.write(s["values"])
            with c3:
                if st.button("삭제", key=f"del_{i}"):
                    st.session_state["sweeps"].pop(i)
                    st.experimental_rerun()

st.divider()
st.subheader("생성")

sweeps = st.session_state["sweeps"]
keys = [s["key"] for s in sweeps]
value_lists = [s["values"] for s in sweeps]

if len(sweeps) == 0:
    st.warning("sweep가 비어있습니다. 최소 1개 key를 추가하세요.")
    st.stop()

n_combo = 1
for vs in value_lists:
    n_combo *= len(vs)

st.write("총 조합 수:", n_combo)

out_dir_name = st.text_input("output 폴더명", value="generated_yamls")
keep_base_exp_name = st.checkbox("exp_name을 유지", value=False)

pattern_help = "파일명 규칙: {idx}는 0부터 시작, {exp}는 exp_name, {ts}는 생성 시각"
file_pattern = st.text_input("파일명 패턴", value="{exp}_{idx:04d}.yaml", help=pattern_help)

gen_btn = st.button("YAML 생성 + ZIP 만들기", type="primary", use_container_width=True)

if gen_btn:
    out_dir = Path(out_dir_name)
    out_dir.mkdir(parents=True, exist_ok=True)

    combos = list(itertools.product(*value_lists))

    zip_buf = io.BytesIO()
    with zipfile.ZipFile(zip_buf, "w", compression=zipfile.ZIP_DEFLATED) as zf:
        for idx, combo in enumerate(combos):
            cfg = dict(base)  # shallow copy is ok for top-level replacements
            for k, v in zip(keys, combo):
                cfg[k] = v

            if not keep_base_exp_name and "exp_name" in cfg:
                cfg["exp_name"] = make_exp_name(str(cfg["exp_name"]), idx)

            exp = str(cfg.get("exp_name", "exp"))
            ts = datetime.now().strftime("%Y%m%d_%H%M%S")
            fname = file_pattern.format(idx=idx, exp=exp, ts=ts)
            if not fname.endswith((".yaml", ".yml")):
                fname += ".yaml"

            path = out_dir / fname
            path.write_text(dump_yaml(cfg), encoding="utf-8")

            zf.writestr(fname, dump_yaml(cfg).encode("utf-8"))

    zip_buf.seek(0)
    st.success(f"생성 완료: {out_dir.resolve()} (파일 {n_combo}개)")
    st.download_button(
        label="ZIP 다운로드",
        data=zip_buf.getvalue(),
        file_name="generated_yamls.zip",
        mime="application/zip",
        use_container_width=True
    )
