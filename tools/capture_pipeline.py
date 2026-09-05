"""MTC capture-dataset pipeline (per-environment self-contained layout).

Root = MTC_CAPTURE_ROOT (default E:\\MTC_data\\capture). One folder per env:

    <env_id>/
      <env_id>.yaml        geometry (export_to_yaml) -- the editable scene contract
      gen_meta.json        provenance: seed, tier, generator_commit, difficulty
      mesh/<env_id>_simplified.fbx
      raw/                 long capture CSVs, as recorded (never edited)
      trimmed/             clips produced by the Unity trimmer

Subcommands:
    generate     --easy N --medium N --hard N     build folders + yaml + FBX
    import-unity <env_id> [...]                     copy env FBX into Unity for capture
    stow         <env_id> --from <file|dir>         move a long capture into raw/
    stow-batch   --from <adb-pull dir>              route a whole atomic session by
                                                    its manifest: each take to the env
                                                    it was recorded in, paired with its
                                                    own collision log, rejects aside
    trim-stow    <env_id> --from <file|dir>         move trimmed clips into trimmed/
    index                                           rebuild _index/{envs,captures}.csv

Reproducibility: env_id encodes tier+seed; gen_meta.json records the generator
commit. Regenerate any env with generate_corridor(seed, tier) at that commit,
or just reuse the saved FBX (authoritative geometry).
"""

from __future__ import annotations

import argparse
import csv
import json
import os
import re
import shutil
import subprocess
import sys
from datetime import datetime
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
# The generator package is vendored under envgen/ in this handoff repo.
sys.path.insert(0, str(REPO / "envgen"))

from mtc_corridor import export_to_yaml, generate_corridor  # noqa: E402
from scripts.export_to_unity import UNITY_FBX_DIR, build_one  # noqa: E402

# Capture root. Point MTC_CAPTURE_ROOT at your data drive; the repo-local
# default keeps a fresh clone working with no configuration.
ROOT = Path(os.environ.get("MTC_CAPTURE_ROOT") or REPO / "captures")
# Fresh, non-overlapping seed bases (old collected envs used 101-110 / 201-240 /
# 301-310; these ranges keep the large batch clearly separate and reproducible).
SEED0 = {"easy": 10001, "medium": 20001, "hard": 30001}


def git_info() -> tuple[str, bool]:
    try:
        commit = subprocess.check_output(
            ["git", "-C", str(REPO), "rev-parse", "HEAD"], text=True
        ).strip()
        dirty = bool(
            subprocess.check_output(
                ["git", "-C", str(REPO), "status", "--porcelain"], text=True
            ).strip()
        )
        return commit, dirty
    except Exception:  # noqa: BLE001
        return "unknown", False


# Captured envs are filed under a MM-DD-YYYY folder for the day they were
# recorded, so a day's output -- and whoever ran that session -- is one folder.
# Envs with no data yet stay directly under ROOT. Both layouts are read.
DATE_DIR = re.compile(r"^\d{2}-\d{2}-\d{4}$")
DAY_FMT = "%m-%d-%Y"


def env_dirs() -> dict[str, Path]:
    """env_id -> its folder, wherever it currently lives."""
    out: dict[str, Path] = {}
    if not ROOT.exists():
        return out
    for p in sorted(ROOT.iterdir()):
        if not p.is_dir() or p.name.startswith("_"):
            continue
        if DATE_DIR.match(p.name):
            for q in sorted(p.iterdir()):
                if q.is_dir():
                    out.setdefault(q.name, q)
        else:
            out.setdefault(p.name, p)
    return out


def env_dir(env_id: str) -> Path:
    return env_dirs().get(env_id, ROOT / env_id)


def file_under_day(env_id: str, day: str, dry: bool = False) -> Path:
    """Move a never-captured env under its capture day. An env that already sits
    in a day folder stays put -- a second take on another day joins the first,
    and the take filenames carry their own dates anyway.

    `dry` answers where the env WOULD land without moving anything, so a caller
    previewing a run cannot reshape the capture root as a side effect."""
    cur = env_dir(env_id)
    if DATE_DIR.match(cur.parent.name):
        return cur
    dst = ROOT / day / env_id
    if dry:
        return dst
    dst.parent.mkdir(parents=True, exist_ok=True)
    if cur.exists() and cur != dst:
        shutil.move(str(cur), str(dst))
    dst.mkdir(parents=True, exist_ok=True)
    return dst


def _write_meta(env, seed: int, tier: str, commit: str, dirty: bool) -> None:
    meta = {
        "env_id": env.env_id,
        "seed": int(seed),
        "tier": tier,
        "length": round(float(env.length), 4),
        "width": round(float(env.width), 4),
        "wall_height": round(float(env.wall_height), 4),
        "difficulty": round(float(env.env_difficulty), 4),
        "n_obstacles": len(env.obstacles),
        "obstacle_types": sorted({o.obstacle_type for o in env.obstacles}),
        "has_dog_hole": any(o.obstacle_type == "dog_hole" for o in env.obstacles),
        "has_leaning": any(o.obstacle_type == "leaning_columns" for o in env.obstacles),
        "generator_commit": commit,
        "generator_dirty": dirty,
        "generated_at": datetime.now().isoformat(timespec="seconds"),
    }
    (env_dir(env.env_id) / "gen_meta.json").write_text(json.dumps(meta, indent=2), encoding="utf-8")


def cmd_generate(args) -> None:
    ROOT.mkdir(parents=True, exist_ok=True)
    commit, dirty = git_info()
    plan = [("easy", args.easy), ("medium", args.medium), ("hard", args.hard)]
    built, skipped, failed = 0, 0, []
    for tier, n in plan:
        for i in range(n):
            seed = SEED0[tier] + i
            try:
                env = generate_corridor(seed=seed, tier=tier)
            except Exception as e:  # noqa: BLE001
                failed.append(f"{tier}:{seed} gen:{e}")
                continue
            d = env_dir(env.env_id)
            for sub in ("mesh", "raw", "trimmed"):
                (d / sub).mkdir(parents=True, exist_ok=True)
            export_to_yaml(env, d / f"{env.env_id}.yaml")
            _write_meta(env, seed, tier, commit, dirty)
            fbx = d / "mesh" / f"{env.env_id}_simplified.fbx"
            if fbx.exists() and not args.rebuild:
                skipped += 1
            else:
                try:
                    build_one(env, d / "mesh")
                    built += 1
                except Exception as e:  # noqa: BLE001
                    failed.append(f"{env.env_id} fbx:{e}")
            print(f"[{env.env_id}] mesh={'ok' if fbx.exists() else 'MISSING'}", flush=True)
    print(f"\nbuilt={built} skipped={skipped} failed={len(failed)}", flush=True)
    for f in failed[:30]:
        print("  FAIL", f, flush=True)
    build_index()


# Capture stamp: MM-DD-YYYY then HH_MM. The YEAR is not decoration -- takes are
# grouped by the day they were recorded so a session can be traced back to whoever
# ran it, and a bare MM_DD becomes ambiguous the moment capture spans a new year.
CAPTURE_STAMP = "%m-%d-%Y__%H_%M"


def _take_time(row: dict, f: Path) -> datetime:
    """When the take was RECORDED.

    `adb pull` stamps the local copy with the pull time, so the file mtime is the
    day the data was fetched, not the day it was captured -- filing by it would put
    a session in the wrong day folder and flatten several takes onto one minute.
    The session manifest carries the real clock, so it wins; mtime is the fallback
    for a take the manifest does not list.
    """
    for k in ("started_at", "ended_at"):
        v = row.get(k)
        if not v:
            continue
        for fmt in ("%Y-%m-%d %H:%M:%S.%f", "%Y-%m-%d %H:%M:%S"):
            try:
                return datetime.strptime(str(v), fmt)
            except ValueError:
                pass
    return datetime.fromtimestamp(f.stat().st_mtime)


def _stamp(epoch: float) -> str:
    return datetime.fromtimestamp(epoch).strftime(CAPTURE_STAMP)


def _newest_or_file(src: Path) -> Path | None:
    if src.is_file():
        return src
    if src.is_dir():
        csvs = sorted(src.glob("*.csv"), key=lambda p: p.stat().st_mtime, reverse=True)
        return csvs[0] if csvs else None
    return None


def cmd_stow(args) -> None:
    """Move a long capture CSV (and its paired collision log, if any) into
    <env_id>/raw/ with a timestamped name."""
    d = env_dir(args.env_id)
    raw = d / "raw"
    if not raw.exists():
        sys.exit(f"unknown env {args.env_id} (folder missing)")
    src = _newest_or_file(Path(args.frm))
    if src is None:
        sys.exit(f"no CSV found at {args.frm}")
    n = len([p for p in raw.glob("*.csv") if not p.name.endswith(".collisions.csv")]) + 1
    ts = _stamp(src.stat().st_mtime)
    dst = raw / f"{args.env_id}__raw__{ts}__{n:02d}.csv"
    mv = shutil.copy2 if args.copy else shutil.move
    mv(str(src), str(dst))
    print(f"stowed motion -> {dst}")

    # Pair the newest collision log from the same folder (Body24Collision writes
    # collisions_<ts>.csv on the same F9). Renamed to <raw_stem>.collisions.csv.
    search = Path(args.frm) if Path(args.frm).is_dir() else Path(args.frm).parent
    logs = sorted(search.glob("collisions_*.csv"), key=lambda p: p.stat().st_mtime, reverse=True)
    if logs:
        cl_dst = raw / f"{dst.stem}.collisions.csv"
        mv(str(logs[0]), str(cl_dst))
        print(f"stowed collisions -> {cl_dst}")
    else:
        print("  (no collision log found alongside -- ok if you didn't run Body24Collision)")
    build_index()


MANIFEST_NAME = "mtc_session_manifest.jsonl"


def _read_manifest(path: Path) -> list[dict]:
    """Parse MTC_AtomicSession's append-only jsonl into one row per take.

    The file is append-only by design (nothing is deleted on the headset), so a
    take can be followed by a `{"op": "reject"}` row naming the same motion CSV.
    Last op wins.
    """
    takes: dict[str, dict] = {}
    # utf-8-SIG: Unity writes the file with a BOM, which otherwise makes the
    # FIRST take of every session unparseable -- silently dropping it.
    for i, line in enumerate(path.read_text(encoding="utf-8-sig").splitlines(), 1):
        line = line.strip()
        if not line:
            continue
        try:
            row = json.loads(line)
        except json.JSONDecodeError:
            print(f"  manifest line {i}: unparseable, skipped")
            continue
        name = row.get("motion_csv") or ""
        if not name:
            continue
        if row.get("op") == "reject":
            if name in takes:
                takes[name]["status"] = "reject"
            else:
                takes[name] = {**row, "status": "reject"}
        else:
            takes[name] = row
    return list(takes.values())


def _inject_meta(raw: bytes, meta: dict[str, str]) -> bytes:
    """Add `# key=value` lines to a pxr24 CSV, after the existing meta block.

    The recorder cannot write these itself: room_yaw_deg is only known to the
    session controller, and PXR_24PointMapper is off-limits (it is the verified
    capture/coord pipeline). Stamping them into the STOWED COPY keeps the file
    self-describing without touching what is on the headset.
    """
    if not meta:
        return raw
    lines = raw.splitlines(keepends=True)
    cut = len(lines)
    for i, ln in enumerate(lines):
        if not ln.lstrip(b"\xef\xbb\xbf").lstrip().startswith(b"#"):
            cut = i
            break
    term = b"\r\n" if cut and lines[cut - 1].endswith(b"\r\n") else b"\n"
    add = [f"# {k}={v}".encode() + term for k, v in meta.items()]
    return b"".join(lines[:cut] + add + lines[cut:])


def _env_of(csv_path: Path, known: set[str]) -> str | None:
    """env_id for a take: `# scene_prefix=` if it names a known env, else the
    longest known env_id the file name starts with."""
    try:
        with csv_path.open(encoding="utf-8", errors="replace") as fh:
            for line in fh:
                if not line.startswith("#"):
                    break
                if line.startswith("# scene_prefix="):
                    p = line.split("=", 1)[1].strip()
                    if p in known:
                        return p
    except OSError:
        pass
    hits = [e for e in known if csv_path.name.startswith(e)]
    return max(hits, key=len) if hits else None


def cmd_stow_batch(args) -> None:
    """Route a whole capture session into per-env folders in one pass.

    `stow` handles ONE file and pairs the collision log by picking the newest in
    the folder -- correct for a single long take, wrong the moment a session
    holds dozens of takes across many envs. This reads the session manifest
    instead, so every motion CSV goes to the env it was actually recorded in and
    is paired with ITS collision log by name.
    """
    src = Path(args.frm)
    if not src.is_dir():
        sys.exit(f"--from must be the adb-pull directory, got {src}")
    known = set(env_dirs())

    manifest = src / (args.manifest or MANIFEST_NAME)
    rows = _read_manifest(manifest) if manifest.exists() else []
    if rows:
        print(f"manifest: {len(rows)} take(s) from {manifest}")
    else:
        print(f"no manifest at {manifest} -- falling back to file names")

    by_name = {r["motion_csv"]: r for r in rows}
    listed = set(by_name)
    on_disk = {
        p.name
        for p in src.glob("*.csv")
        if not p.name.startswith("collisions_") and not p.name.endswith(".collisions.csv")
    }
    unlisted = sorted(on_disk - listed)
    if unlisted:
        what = "stowing" if args.include_unlisted else "SKIPPING (use --include-unlisted)"
        print(
            f"{len(unlisted)} CSV(s) not in the manifest -- {what}: {', '.join(unlisted[:6])}"
            + (" ..." if len(unlisted) > 6 else "")
        )
    todo = sorted(listed & on_disk) + (unlisted if args.include_unlisted else [])

    take_log = shutil.move if args.move else shutil.copy2
    counts: dict[str, int] = {}
    n_ok = n_rej = n_skip = 0
    for name in todo:
        f = src / name
        row = by_name.get(name, {})
        env_id = row.get("env_id") or _env_of(f, known)
        if env_id not in known:
            print(f"  SKIP {name}: no env folder for {env_id!r}")
            n_skip += 1
            continue
        rejected = row.get("status") == "reject"
        sub = "rejected" if rejected else "raw"
        # File the env under the day it was recorded. A never-captured env moves
        # under that day; one already in a day folder stays put and the new take
        # joins it -- the take filename carries its own date either way.
        when = _take_time(row, f)
        day = when.strftime(DAY_FMT)
        out = file_under_day(env_id, day, dry=args.dry_run) / sub
        if not args.dry_run:
            out.mkdir(parents=True, exist_ok=True)
        counts[env_id] = counts.get(env_id, 0) + 1
        ts = when.strftime(CAPTURE_STAMP)
        n = len([p for p in out.glob("*.csv") if not p.name.endswith(".collisions.csv")]) + 1
        dst = out / f"{env_id}__take__{ts}__{n:02d}.csv"

        meta = {
            k: row[k]
            for k in ("room_yaw_deg", "world_scale", "duration_s", "start_back_off_m")
            if k in row
        }
        if meta:
            meta["capture_mode"] = "atomic_session"
        if args.dry_run:
            print(f"  [dry] {name} -> {dst.relative_to(ROOT)}" + (" (reject)" if rejected else ""))
            continue
        dst.write_bytes(_inject_meta(f.read_bytes(), meta))
        if args.move:
            f.unlink()

        col_name = row.get("collision_csv") or ""
        col = src / col_name if col_name else None
        if col is not None and col.exists():
            take_log(str(col), str(out / f"{dst.stem}.collisions.csv"))
        if rejected:
            n_rej += 1
        else:
            n_ok += 1

    if args.dry_run:
        print("dry run -- nothing written")
        return
    print(f"stowed {n_ok} take(s), {n_rej} rejected, {n_skip} skipped")
    for env_id in sorted(counts):
        print(f"  {env_id}: {counts[env_id]}")
    if rows and not args.move:
        print("(originals left in the staging dir -- pass --move once you have verified them)")
    build_index()


def _frame_col(csv_path: Path) -> list[int]:
    """Read the integer `frame` column (Time.frameCount) from a pxr24 CSV."""
    frames = []
    with csv_path.open(encoding="utf-8", errors="replace") as fh:
        header_seen = False
        for line in fh:
            if line.startswith("#"):
                continue
            if not header_seen:
                header_seen = True  # first non-# line is the column header
                continue
            tok = line.split(",", 1)[0].strip()
            if tok.isdigit():
                frames.append(int(tok))
    return frames


def cmd_flag_collisions(args) -> None:
    """Join trimmed clips to their raw collision log on `frame` and report, per
    clip, which frames were in collision (bad data for open-sourcing)."""
    d = env_dir(args.env_id)
    raw, trimmed = d / "raw", d / "trimmed"
    logs: dict[str, dict[int, tuple[bool, str]]] = {}
    for cl in raw.glob("*.collisions.csv"):
        stem = cl.name[: -len(".collisions.csv")]
        fr: dict[int, tuple[bool, str]] = {}
        with cl.open(encoding="utf-8", errors="replace") as fh:
            rd = csv.DictReader(fh)
            for row in rd:
                try:
                    f = int(row["frame"])
                except (KeyError, ValueError, TypeError):
                    continue
                fr[f] = (row.get("hit", "0") == "1", row.get("joints", ""))
        logs[stem] = fr
    if not logs:
        print(f"no collision logs in {raw} -- nothing to flag")
        return

    rows = []
    for clip in sorted(trimmed.glob("*.csv")):
        cl_stem = next((s for s in logs if clip.stem.startswith(s)), None)
        frames = _frame_col(clip)
        if cl_stem is None or not frames:
            rows.append(
                {
                    "clip": clip.name,
                    "frames": len(frames),
                    "collision_log": "MISSING",
                    "bad_frames": "",
                    "bad_fraction": "",
                    "bad_ranges": "",
                    "joints": "",
                }
            )
            continue
        flags = logs[cl_stem]
        bad = [f for f in frames if flags.get(f, (False, ""))[0]]
        joints = sorted({j for f in bad for j in flags[f][1].split("|") if j})
        # contiguous runs over the clip's own frame sequence
        ranges, run = [], []
        badset = set(bad)
        for f in frames:
            if f in badset:
                run.append(f)
            elif run:
                ranges.append((run[0], run[-1]))
                run = []
        if run:
            ranges.append((run[0], run[-1]))
        rows.append(
            {
                "clip": clip.name,
                "frames": len(frames),
                "collision_log": cl_stem,
                "bad_frames": len(bad),
                "bad_fraction": round(len(bad) / max(len(frames), 1), 4),
                "bad_ranges": ";".join(f"{a}-{b}" for a, b in ranges),
                "joints": ",".join(joints),
            }
        )
    out = d / "collisions_report.csv"
    _write_csv(
        out,
        rows,
        ["clip", "frames", "collision_log", "bad_frames", "bad_fraction", "bad_ranges", "joints"],
    )
    n_bad = sum(1 for r in rows if r.get("bad_frames"))
    print(f"flagged {len(rows)} clips ({n_bad} with collisions) -> {out}")


def cmd_trim_stow(args) -> None:
    """Move trimmed clips into <env_id>/trimmed/."""
    d = env_dir(args.env_id)
    trimmed = d / "trimmed"
    if not trimmed.exists():
        sys.exit(f"unknown env {args.env_id} (folder missing)")
    src = Path(args.frm)
    files = [src] if src.is_file() else sorted(src.glob("*.csv"))
    moved = 0
    for f in files:
        dst = trimmed / f.name
        (shutil.copy2 if args.copy else shutil.move)(str(f), str(dst))
        moved += 1
    print(f"stowed {moved} clip(s) -> {trimmed}")
    build_index()


def cmd_import_unity(args) -> None:
    """Copy env FBX into the Unity capture project (one at a time as needed)."""
    dst_dir = Path(args.unity_dir or UNITY_FBX_DIR)
    dst_dir.mkdir(parents=True, exist_ok=True)
    for eid in args.env_ids:
        fbx = env_dir(eid) / "mesh" / f"{eid}_simplified.fbx"
        if not fbx.exists():
            print(f"  MISSING {fbx}")
            continue
        shutil.copy2(fbx, dst_dir / fbx.name)
        print(f"  imported {eid} -> {dst_dir / fbx.name}")


def build_index() -> None:
    idx = ROOT / "_index"
    idx.mkdir(parents=True, exist_ok=True)
    env_rows, cap_rows = [], []
    for name, d in sorted(env_dirs().items()):
        meta_f = d / "gen_meta.json"
        meta = (
            json.loads(meta_f.read_text(encoding="utf-8")) if meta_f.exists() else {"env_id": name}
        )
        meta["capture_day"] = d.parent.name if DATE_DIR.match(d.parent.name) else ""
        fbx = d / "mesh" / f"{name}_simplified.fbx"
        meta["mesh_present"] = int(fbx.exists())
        env_rows.append(meta)
        # takes only -- each has a paired .collisions.csv sitting next to it
        n_raw = (
            len([q for q in (d / "raw").glob("*.csv") if not q.name.endswith(".collisions.csv")])
            if (d / "raw").exists()
            else 0
        )
        n_trim = len(list((d / "trimmed").glob("*.csv"))) if (d / "trimmed").exists() else 0
        status = "done" if n_trim else ("captured" if n_raw else "pending")
        cap_rows.append(
            {
                "env_id": name,
                "capture_day": meta["capture_day"],
                "tier": meta.get("tier", ""),
                "difficulty": meta.get("difficulty", ""),
                "mesh": meta["mesh_present"],
                "n_raw": n_raw,
                "n_trimmed": n_trim,
                "status": status,
            }
        )
    _write_csv(
        idx / "envs.csv",
        env_rows,
        [
            "env_id",
            "tier",
            "seed",
            "length",
            "width",
            "difficulty",
            "n_obstacles",
            "has_dog_hole",
            "has_leaning",
            "mesh_present",
            "generator_commit",
            "generated_at",
        ],
    )
    _write_csv(
        idx / "captures.csv",
        cap_rows,
        ["env_id", "capture_day", "tier", "difficulty", "mesh", "n_raw", "n_trimmed", "status"],
    )
    pend = sum(1 for r in cap_rows if r["status"] == "pending")
    print(f"index: {len(env_rows)} envs  ({pend} pending capture)  -> {idx}")


def _write_csv(path: Path, rows: list[dict], fields: list[str]) -> None:
    with path.open("w", newline="", encoding="utf-8") as fh:
        w = csv.DictWriter(fh, fieldnames=fields, extrasaction="ignore")
        w.writeheader()
        for r in rows:
            w.writerow(r)


def main() -> None:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    sub = ap.add_subparsers(dest="cmd", required=True)

    g = sub.add_parser("generate")
    g.add_argument("--easy", type=int, default=0)
    g.add_argument("--medium", type=int, default=0)
    g.add_argument("--hard", type=int, default=0)
    g.add_argument("--rebuild", action="store_true", help="rebuild FBX even if present")
    g.set_defaults(func=cmd_generate)

    s = sub.add_parser("stow")
    s.add_argument("env_id")
    s.add_argument("--from", dest="frm", required=True, help="capture CSV file or dir")
    s.add_argument("--copy", action="store_true")
    s.set_defaults(func=cmd_stow)

    sb = sub.add_parser("stow-batch", help="route a whole adb-pull session by its manifest")
    sb.add_argument("--from", dest="frm", required=True, help="adb-pull staging directory")
    sb.add_argument("--manifest", default=None, help=f"manifest name (default {MANIFEST_NAME})")
    sb.add_argument("--move", action="store_true", help="move instead of copy (default: copy)")
    sb.add_argument(
        "--include-unlisted",
        action="store_true",
        help="also stow CSVs absent from the manifest, routed by file name",
    )
    sb.add_argument("--dry-run", action="store_true")
    sb.set_defaults(func=cmd_stow_batch)

    t = sub.add_parser("trim-stow")
    t.add_argument("env_id")
    t.add_argument("--from", dest="frm", required=True)
    t.add_argument("--copy", action="store_true")
    t.set_defaults(func=cmd_trim_stow)

    im = sub.add_parser("import-unity")
    im.add_argument("env_ids", nargs="+")
    im.add_argument("--unity-dir", default=None)
    im.set_defaults(func=cmd_import_unity)

    fc = sub.add_parser("flag-collisions")
    fc.add_argument("env_id")
    fc.set_defaults(func=cmd_flag_collisions)

    ix = sub.add_parser("index")
    ix.set_defaults(func=lambda a: build_index())

    args = ap.parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
