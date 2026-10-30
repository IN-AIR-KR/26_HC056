"""
sitl_procs.py — SITL 프로세스 식별/정리 도우미 (다른 SITL 세션을 죽이지 않기 위한 규칙)  [arms_sim]

예전 정리 방식은 `pkill -9 -f px4_sitl_default/bin/px4` / `pkill -9 -f "gz sim"` 였다.
같은 PC 에서 다른 사람이(또는 다른 런치가) 돌리던 PX4·Gazebo 까지 같이 죽였다.

규칙
  - '내 것' = 같은 PX4 인스턴스 번호(-i N) 이면서 같은 GZ_PARTITION 환경변수.
  - 기동 전 정리(stale)는 '내 것' 중 **고아(init/systemd 로 입양된) 프로세스**만 죽인다.
    살아 있는 런치·셸·make 의 자식(누군가 돌리고 있는 것)은 건드리지 않는다.
  - 종료 정리는 '내 것' 중 이 런치 프로세스의 자손이거나 고아인 것만 죽인다.
  - 다른 세션이 같은 인스턴스·파티션을 쓰고 있으면 포트/월드가 겹치므로 기동을 거부한다.
"""

import os
import signal

PX4_PATTERN = "px4_sitl_default/bin/px4"
GZ_PATTERN = "gz sim"


def _read(path):
    try:
        with open(path, "rb") as f:
            return f.read()
    except OSError:
        return b""


def cmdline(pid):
    return _read(f"/proc/{pid}/cmdline").replace(b"\0", b" ").decode(errors="replace").strip()


def environ(pid):
    out = {}
    for item in _read(f"/proc/{pid}/environ").split(b"\0"):
        if b"=" in item:
            k, v = item.split(b"=", 1)
            out[k.decode(errors="replace")] = v.decode(errors="replace")
    return out


def ppid(pid):
    stat = _read(f"/proc/{pid}/stat").decode(errors="replace")
    try:
        return int(stat.rsplit(")", 1)[1].split()[1])
    except (IndexError, ValueError):
        return 0


def ancestors(pid):
    chain, seen = [], set()
    p = ppid(pid)
    while p > 1 and p not in seen:
        seen.add(p)
        chain.append(p)
        p = ppid(p)
    return chain


def is_orphan(pid):
    """원래 부모가 죽어 init/systemd 로 입양된 프로세스 = 이전 실행의 잔재.

    살아 있는 셸·런치·make 등의 자식이면(누가 돌리고 있는 것) 고아가 아니다.
    """
    p = ppid(pid)
    if p <= 1:
        return True
    parent = cmdline(p)
    return parent.startswith("/sbin/init") or parent.startswith("/lib/systemd/systemd") \
        or parent.startswith("/usr/lib/systemd/systemd") or parent.split(" ")[0].endswith("/systemd")


def px4_instance(pid):
    parts = cmdline(pid).split()
    for i, tok in enumerate(parts):
        if tok == "-i" and i + 1 < len(parts):
            try:
                return int(parts[i + 1])
            except ValueError:
                return 0
    return 0


def list_pids(pattern):
    me = os.getpid()
    pids = []
    for name in os.listdir("/proc"):
        if not name.isdigit():
            continue
        pid = int(name)
        if pid == me:
            continue
        c = cmdline(pid)
        if pattern in c and "pgrep" not in c and "sitl_procs" not in c:
            pids.append(pid)
    return pids


def same_scope(pid, instance, partition):
    """PX4 는 인스턴스+파티션, gz 는 파티션으로 '같은 세션 영역'인지 본다."""
    if environ(pid).get("GZ_PARTITION", "") != (partition or ""):
        return False
    if PX4_PATTERN in cmdline(pid):
        return px4_instance(pid) == instance
    return True


def scan(instance, partition):
    """(내 영역의 PX4/gz, 그중 살아 있는 세션 소속, 다른 영역에서 도는 것).

    gz sim 은 PX4 rcS 가 '&' 로 띄우고 빠지므로 PX4 가 살아 있어도 systemd 로 입양돼
    있다. 그래서 gz 의 생사는 '같은 영역에 살아 있는(고아 아닌) PX4 가 있는가'로 판단한다.
    """
    mine, live, foreign = [], [], []
    for pattern in (PX4_PATTERN, GZ_PATTERN):
        for pid in list_pids(pattern):
            (mine if same_scope(pid, instance, partition) else foreign).append(pid)
    live_px4 = [p for p in mine if PX4_PATTERN in cmdline(p) and not is_orphan(p)]
    if live_px4:
        live = list(mine)   # 살아 있는 PX4 가 있으면 그 영역의 gz 도 그 세션 것이다
    return mine, live, foreign


def kill(pids, sig=signal.SIGKILL):
    for pid in pids:
        try:
            os.kill(pid, sig)
        except (ProcessLookupError, PermissionError):
            pass


def kill_stale(instance, partition):
    """기동 전: 내 영역에 살아 있는 세션이 있으면 아무것도 안 죽이고 (False, 설명).
    없으면 내 영역의 잔재(고아 PX4, 주인 없는 gz)만 죽인다. 다른 영역은 건드리지 않는다."""
    mine, live, foreign = scan(instance, partition)
    if live:
        desc = "; ".join(f"{p}: {cmdline(p)[:80]}" for p in live)
        return False, (f"같은 PX4 인스턴스({instance})/GZ_PARTITION('{partition}') 을 쓰는 "
                       f"다른 SITL 세션이 실행 중 — 아무것도 종료하지 않음: {desc}")
    kill(mine)
    return True, f"잔재 {len(mine)}개 정리, 다른 영역 프로세스 {len(foreign)}개는 유지"


def kill_own(instance, partition, launch_pid):
    """종료 시: 이 런치의 자손 PX4 와, 내 영역에 다른 살아 있는 PX4 가 없을 때의 gz 만 죽인다."""
    targets, others_alive = [], False
    for pid in list_pids(PX4_PATTERN):
        if not same_scope(pid, instance, partition):
            continue
        if launch_pid in ancestors(pid) or is_orphan(pid):
            targets.append(pid)
        else:
            others_alive = True
    if not others_alive:
        targets += [p for p in list_pids(GZ_PATTERN) if same_scope(p, instance, partition)]
    kill(targets)
    return targets


def own_px4(instance, partition):
    """내 영역의 PX4 pid 목록 (RESET 이 재기동할 대상)."""
    return [p for p in list_pids(PX4_PATTERN) if same_scope(p, instance, partition)]
