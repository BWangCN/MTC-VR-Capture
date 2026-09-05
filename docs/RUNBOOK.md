# Runbook — one capture session, start to finish

The four stages, in the order you actually run them. Every command assumes the
repo root as the working directory unless stated otherwise.

```
1. generate      envgen/scripts/make_standard_envs.py   ->  captures/<env_id>/{yaml, mesh/*.fbx}
2. load + build  Unity: MTC > Fix Capture Session       ->  APK on the headset
3. capture       on-headset, hands-free or button       ->  CSV in the app's files/ dir
4. ingest        tools/capture_pipeline.py stow-batch   ->  captures/<MM-DD-YYYY>/<env_id>/raw/
5. trim          Unity CsvReplayTrimmer (or tools/)     ->  captures/<...>/trimmed/
```

## 0. One-time configuration

Two environment variables decide where everything lives. Without them the repo
falls back to `captures/` and `envgen/fbx_out/` inside the clone, which works but
puts capture data on your system drive.

```bash
export MTC_CAPTURE_ROOT="D:/MTC_data/capture"          # env library + captured takes
export MTC_UNITY_FBX_DIR="D:/Unity/MyCapture/Assets/FBX_Scenes"
export MTC_BLENDER_EXE="C:/Program Files/Blender Foundation/Blender 4.2/blender.exe"
```

`MTC_CAPTURE_ROOT` must be the **same value** for the generator and for
`capture_pipeline.py` — the generator writes the env library there and the
ingest step files takes into it.

```bash
python -m pip install -r envgen/requirements.txt
```

Blender is only needed to build FBX (`--no-fbx` skips it for a fast layout check).

## 1. Generate the environments

Environments are deterministic in `(seed, tier)`; the same spec always yields the
same corridor. Named sets live in `SPECS` in `make_standard_envs.py`.

```bash
cd envgen
python scripts/make_standard_envs.py --version atomic4 --no-fbx   # layout check, seconds
python scripts/make_standard_envs.py --version atomic4            # + FBX via Blender
```

Each env lands in `$MTC_CAPTURE_ROOT/<env_id>/` with `<env_id>.yaml`,
`gen_meta.json` (seed + layout version, for provenance) and
`mesh/<env_id>_simplified.fbx`.

**Adding a new obstacle family.** Add a builder to `scripts/new_obstacles.py`,
register it in the `builders()` dispatch in `make_standard_envs.py`, then add a
spec entry. Follow `_atomic4_spec()` — it is the smallest complete example.
Thread all randomness through the seeded `rng` argument; determinism is relied on
downstream and is easy to break with module-level `random`.

**Verify before you capture.** The YAML is intent; the FBX is what the person
actually sees. Read the built FBX and check the dimension that matters for the
behaviour you are after (opening height for crawls, top height for step-overs,
free gap for squeezes). A stray sub-mesh in the FBX has bitten this pipeline
before — see TROUBLESHOOTING.

## 2. Load into Unity and build

`session_filter.txt` selects which behaviours this session carries. Without it,
Unity loads *every* uncaptured env, which drags in leftovers from earlier sets.

```
$MTC_CAPTURE_ROOT/_index/session_filter.txt
```
```
# one behaviour per line = env_id minus the corridor_atomic_ prefix and the _NN suffix
highcrawl
prone
```

Then in Unity, with `Assets/Scenes/<your capture scene>` open:

**`MTC > Fix Capture Session (hmd + envs)`**

which wires `PXR_SessionAutoFlow.hmd`, imports the filtered envs via
`AtomicEnvSet.ImportAll()`, retires the previous session's meshes, and saves the
scene. Check the console: it prints how many envs the session carries. **Zero
envs means the build would record nothing** — usually the filter does not match
any pending env.

Then **rebuild the APK**. A scene change does not reach the headset any other way.

## 3. Capture

`MTC_AtomicSession` walks the env list on the headset. Controller buttons switch
env and start/stop recording, so one donning covers the whole set. Each take
writes `<env_id>_<NN>.csv` plus a matching `collisions_<env_id>_<stamp>.csv`, and
appends a row to `mtc_session_manifest.jsonl` — that manifest is what makes
ingest unambiguous, so never delete it from the device.

Walk the same behaviour several times per env. Segmentation downstream keys on
the person crossing the obstacle, and more passes simply means more clips.

## 4. Pull and file

```bash
adb pull /storage/emulated/0/Android/data/<bundle.id>/files/mtc_session_manifest.jsonl <staging>
# then pull the CSVs named by the manifest rows for today's date
python tools/capture_pipeline.py stow-batch --from <staging> --dry-run
python tools/capture_pipeline.py stow-batch --from <staging>            # copies
python tools/capture_pipeline.py stow-batch --from <staging> --move     # once verified
```

`stow-batch` reads the manifest and routes every take to the env it was actually
recorded in, pairing it with **its own** collision log by name. A never-captured
env moves under its capture day; an env already in a day folder stays put and the
new take joins it — the take filename carries its own date either way.

Always run `--dry-run` first. Note that the duplicate destination names it may
show are an artifact: the per-env counter reads files on disk, and a dry run
writes none. The real run disambiguates them.

Verify before `--move`: compare device-side and local byte counts, and compare
the CSV *body* hash (the header differs, ingest injects metadata).

## 5. Trim

Manual trimming in the Unity Editor is the reference path — see README §8. Set
`outputNamePrefix` to the `env_id`, or the clips cannot be traced back to their
environment.

`tools/experimental/segment_turning_point.py` is a reference implementation of an
automatic first pass, not a turnkey tool -- read its header before using it. It cuts on the
turning points of the pelvis trajectory: the person oscillates along the
corridor, each rising limb that crosses the obstacle is one pass, and the
turnaround is a natural boundary because velocity goes to zero there. It exists
because stand-based segmentation under-counts whenever the person does not fully
stop between passes.

Treat its output as candidates, not as the dataset. Check the count against an
independent crossing count before trusting it.
