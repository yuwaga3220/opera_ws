"""
i-PRO / ONVIF 用の認証情報を .env から読み込む（リポジトリにコミットしない）。

環境変数（既に設定されていれば .env より優先）:
  IPRO_CAMERA_IP
  IPRO_CAMERA_USER
  IPRO_CAMERA_PASSWORD

任意で .env の場所を固定:
  IPRO_ENV_FILE=/absolute/path/to/.env
"""
import os
from pathlib import Path
from typing import Optional, Tuple

_ENV_LOADED = False


def _parse_env_line(line: str) -> Tuple[Optional[str], Optional[str]]:
    s = line.strip()
    if not s or s.startswith("#"):
        return None, None
    if "=" not in s:
        return None, None
    key, _, val = s.partition("=")
    key = key.strip()
    if not key:
        return None, None
    val = val.strip()
    if len(val) >= 2 and val[0] == val[-1] and val[0] in "\"'":
        val = val[1:-1]
    return key, val


def _first_dotenv_path() -> Optional[Path]:
    exp = os.environ.get("IPRO_ENV_FILE", "").strip()
    if exp:
        p = Path(exp).expanduser().resolve()
        if p.is_file():
            return p
    cur = Path.cwd().resolve()
    for _ in range(12):
        cand = cur / ".env"
        if cand.is_file():
            return cand
        if cur.parent == cur:
            break
        cur = cur.parent
    here = Path(__file__).resolve().parent
    for p in [here, *here.parents]:
        cand = p / ".env"
        if cand.is_file():
            return cand
    return None


def load_ipro_dotenv() -> None:
    """最初に見つかった .env を読み、未設定の環境変数だけ埋める。"""
    global _ENV_LOADED
    if _ENV_LOADED:
        return
    path = _first_dotenv_path()
    if path is None:
        _ENV_LOADED = True
        return
    try:
        text = path.read_text(encoding="utf-8")
    except OSError:
        _ENV_LOADED = True
        return
    for line in text.splitlines():
        k, v = _parse_env_line(line)
        if k is None:
            continue
        if k not in os.environ:
            os.environ[k] = v
    _ENV_LOADED = True


def camera_ip(default: str = "192.168.11.200") -> str:
    load_ipro_dotenv()
    return os.environ.get("IPRO_CAMERA_IP", default)


def camera_user(default: str = "") -> str:
    load_ipro_dotenv()
    return os.environ.get("IPRO_CAMERA_USER", default)


def camera_password(default: str = "") -> str:
    load_ipro_dotenv()
    return os.environ.get("IPRO_CAMERA_PASSWORD", default)
