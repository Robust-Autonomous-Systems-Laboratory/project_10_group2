# EE5531 — Introduction to 3D LiDAR Mapping with MOLA

In this lab you will build a 3D point cloud map of your environment from a LiDAR bag file,
inspect it interactively, and publish it as an interactive web viewer on GitHub Pages.
The goal is to produce the **best-looking, most interesting point cloud you can in under 10 MB**.

**Full pipeline:**
```
bag.mcap
  │
  ▼  (MOLA SLAM — lidar odometry + mapping)
bag.mcap.mm       ← metric map with multiple point cloud layers
  │
  ▼  (inspect with mm-viewer and mm-info)
  │
  ▼  (crop to region of interest with mm-filter)
*_cropped.mm
  │
  ▼  (export + decimate + convert)
*.laz             ← compressed point cloud
  │
  ▼  (PotreeConverter — spatial index for web streaming)
pointclouds/*/    ← tiles ready for GitHub Pages
```

---

## What is MOLA?

**MOLA** (Modular Optimization framework for LiDAR-based Odometry and mapping) is a modern
open-source robotics framework for building 3D maps from LiDAR sensors. It processes the raw
sensor data from your bag file through several stages:

1. **Odometry** — estimates where the robot was at every moment using LiDAR scan matching
2. **Simple map** — stores all the raw scans paired with the estimated robot poses
3. **Metric map** — replays the simple map through a filter pipeline to produce a clean,
   structured point cloud organized into *layers*

The key insight of MOLA's approach: it separates *when* data was collected from *how* it is
processed. You can re-run the metric map step with different parameters without re-running
the slow odometry step.

---

## Understanding MOLA Output Files

After Step 1 you will have three files:

| File | Size | Description |
|------|------|-------------|
| `bag.mcap.simplemap` | Large | Raw data: every LiDAR scan + robot pose. Think of it as a "recording" of the mapping run. You won't edit this. |
| `bag.mcap.mm` | Medium | The processed metric map. Contains multiple named **layers** of points. This is what you work with. |
| `trajectory.tum` | Small | The robot's path as a list of timestamped poses. Used for visualization. |

### Metric map layers

The `.mm` file contains several layers, each a separate point cloud:

| Layer | Description |
|-------|-------------|
| `map` | **All** points accumulated over the entire run, after motion deskewing and range filtering. The most complete view. |
| `static_map` | Points that appear consistently across multiple passes — walls, trees, ground. Good for structure. |
| `dynamic_map` | Points that moved between passes — people, vehicles. Often noisy but interesting. |
| `voxelmap` | A volumetric occupancy grid used internally to classify static vs. dynamic. Not a point cloud. |

The `ply_to_laz.py` script uses the **`map_cropped`** layer (the cropped version of `map`).
You can experiment with exporting `static_map_cropped` instead for a cleaner result.

---

## Prerequisites

### Software (lab machine has this; personal machines need it installed)

```bash
# Python libraries for LAZ compression
sudo apt install python3-laspy python3-laszip

# MOLA + tools (personal machines only — already on the lab machine)
sudo apt install ros-jazzy-mola-lidar-odometry
```

Verify the installation:
```bash
source /opt/ros/jazzy/setup.bash
mola-lidar-odometry-cli --version
mm-info --help
```

### Create your GitHub repository

1. On GitHub, click **+** (top-right) → **New repository**.
2. Set the owner to **Robust-Autonomous-Systems-Laboratory** and give your repo a descriptive name
   (e.g. `point-cloud-husky-plaza`).
3. Under **Configuration**, select **ee5531_proj10** as the repository template.
4. **Important:** Set visibility to **Public** — GitHub Pages requires a public repository.
5. Click **Create repository**.
6. Clone it to the lab machine:
   ```bash
   git clone https://github.com/Robust-Autonomous-Systems-Laboratory/<your-repo>.git
   cd <your-repo>
   ```
7. Enable GitHub Pages: **Settings → Pages → Deploy from branch → `main`, root `/`**.
   Your viewer will be at `https://robust-autonomous-systems-laboratory.github.io/<your-repo>/`.

---

## Step 1 — Run MOLA SLAM

This step takes 5–20 minutes. It does the heavy work: replaying your bag file, matching
successive LiDAR scans to estimate the robot's trajectory, and building a 3D map.

```bash
source /opt/ros/jazzy/setup.bash
bash scripts/1_run_slam.sh <path/to/your_bag.mcap>
```

**What's happening inside:**

- `mola-lidar-odometry-cli` processes the bag scan-by-scan. It aligns each new scan to the
  previous one (a technique called *point cloud registration*) to estimate how the robot moved.
  This is LiDAR odometry — dead reckoning with LiDAR instead of wheel encoders.

- `sm2mm` replays the simple map through the pipeline in
  `scripts/sm2mm_voxels_static_dynamic_points.yaml`. For each scan it:
  - Corrects for motion distortion (*deskewing* — the robot keeps moving while the LiDAR spins)
  - Filters points that are too close (<5 m, usually the robot itself) or too far (>100 m)
  - Accumulates all points into the `map` layer
  - Builds a voxel occupancy grid to distinguish static from dynamic objects

At the end, the script prints the **bounding box** of your map — note the X, Y, and Z ranges.

**Output files:**

```
your_bag.mcap.simplemap   -- raw scans + trajectory (large, ~same size as bag)
your_bag.mcap.mm          -- the processed metric map
trajectory.tum            -- robot path (timestamp x y z qx qy qz qw)
```

---

## Step 2 — Inspect the Map with mm-viewer

Before doing anything else, take a look at what you built.

```bash
source /opt/ros/jazzy/setup.bash
mm-viewer -l libmola_metric_maps.so \
          -t trajectory.tum \
          your_bag.mcap.mm
```

The `-t trajectory.tum` flag overlays the robot's path so you can see where it went.

**Things to look for:**
- The overall shape and extent of the map — does it match where the robot drove?
- Noisy or incomplete regions (usually at the start/end of the run, or where the robot turned quickly)
- The Z range — most outdoor maps are fairly flat (< 10 m vertical extent)
- Interesting features you want to keep: buildings, trees, vehicles, open spaces

Also run `mm-info` to get precise numbers:
```bash
mm-info your_bag.mcap.mm
```

The output will look like:
```
"map": ... bounding box: [-104.9 -92.8 -6.0]-[114.7 123.8 27.6]
```
This tells you the map spans about 220 m × 217 m × 34 m — far too large to fit under 10 MB.
You will need to crop it down to a region of interest.

---

## Step 3 — Choose and Set Your Crop Region

Point cloud maps get large fast. A typical 20-minute outdoor LiDAR run produces 5–10 million
points. Exporting everything would be tens of megabytes; streaming it on the web would be slow.

The solution is to **crop** — keep only the region you care about. This is done by
`mm-filter`, which applies a spatial bounding box to each layer in the `.mm` file.

### Edit `scripts/crop_filter.yaml`

```yaml
filters:
  - class_name: mp2p_icp_filters::FilterBoundingBox
    params:
      input_pointcloud_layer: "map"
      inside_pointcloud_layer: "map_cropped"   # points inside the box go here
      bounding_box_min: [-50.0, -50.0, -50.0]  # ← X_min, Y_min, Z_min (metres)
      bounding_box_max: [ 50.0,  50.0,  50.0]  # ← X_max, Y_max, Z_max (metres)
```

These coordinates are in the **SLAM frame** — the origin (0, 0, 0) is where the robot
started. The filter keeps every point whose (x, y, z) falls inside the box.

**Strategy:** Start with a ±50 m cube (100 m on each side). Then look at your `mm-info`
output and shrink/shift the box to frame the most interesting part of your map.
For the Z axis, your map is probably only a few metres tall — tighten those bounds
to remove ground noise below and sky noise above.

**Example** — if mm-info shows Z range [-2.1, 5.3]:
```yaml
      bounding_box_min: [-50.0, -50.0, -3.0]
      bounding_box_max: [ 50.0,  50.0,  6.0]
```

> **Tip:** The `crop_filter.yaml` is used in Step 4 by `mm-filter`. You can run Step 4
> multiple times with different crop settings — it only takes a few seconds. Experiment!

---

## Step 4 — Export, Decimate, and Convert

Now run the full export pipeline. This uses your `crop_filter.yaml` settings.

```bash
bash scripts/2_process_map.sh <path/to/your_bag.mcap.mm>
```

You can optionally pass a **voxel size** (default `0.04` m):
```bash
bash scripts/2_process_map.sh your_bag.mcap.mm 0.04   # ~10 MB, good detail
bash scripts/2_process_map.sh your_bag.mcap.mm 0.06   # ~6 MB, less detail
bash scripts/2_process_map.sh your_bag.mcap.mm 0.02   # ~25 MB, very dense
```

### What each sub-step does

**1. `mm-filter` — crop to your bounding box**

Reads your `crop_filter.yaml` and writes a new `.mm` file containing only the points
inside your specified box. This is the first chance to see how much of the map you're keeping.

**2. `mm2ply` — export to PLY format**

PLY (Polygon File Format) is a simple, universal 3D file format. The cropped map is
exported as a binary PLY with `x, y, z, intensity` fields per point.
*(You may see a warning about an empty `raw` layer — this is normal and can be ignored.)*

**3. `ply_to_laz.py` — decimate and compress**

Even after cropping, the point cloud may have millions of points clustered densely in some
areas and sparsely in others. **Voxel decimation** divides space into a 3D grid of cells
(voxels) and keeps only one representative point per cell — giving you uniform spatial
coverage without redundant points.

The voxel size controls the trade-off:
| Voxel size | Typical output | Visual quality |
|---|---|---|
| 0.02 m | 25–40 MB | Near-original detail |
| 0.04 m | 8–12 MB | Good — recommended starting point |
| 0.06 m | 4–7 MB | Acceptable for large areas |
| 0.10 m | 1–3 MB | Noticeably coarser |

After decimation the result is saved as **LAZ** — a lossless compressed version of the
standard LAS LiDAR format. Compression typically achieves 3–4× size reduction.

**4. `PotreeConverter` — build a spatial index for web streaming**

A raw LAZ file can't be streamed efficiently in a browser — loading 10 million points at
once would be too slow. PotreeConverter organizes the points into an **octree**: a 3D tree
structure that lets the browser load coarse detail first and add fine detail as you zoom in.
It produces a folder of small tile files (`cloud.js` + numbered bin files) that Potree
streams on demand.

---

## Step 5 — Inspect the Cropped Map

Before publishing, view your cropped map in mm-viewer to check the result:

```bash
source /opt/ros/jazzy/setup.bash
mm-viewer -l libmola_metric_maps.so your_bag.mcap_cropped.mm
```

Does it look the way you want? If not, go back and adjust `crop_filter.yaml` or the
voxel size and re-run Step 4. It only takes a few seconds.

---

## Step 6 — Tune for Quality (The Challenge)

**Goal: produce the best-looking, most interesting point cloud you can in under 10 MB.**

There is no single right answer. Here are the parameters you can vary:

### Crop region (`scripts/crop_filter.yaml`)
- Frame a specific feature: a building entrance, a courtyard, a dense tree canopy
- Tighter bounds = more detail in less space
- Shift the origin to include the most interesting part of the trajectory

### Voxel size (`2_process_map.sh` second argument)
- Smaller = more points = richer detail = larger file
- Try 0.03 m for a tight crop of a small area, 0.06 m for a wide outdoor scene

### Point cloud layer
By default the `map` layer (all points) is exported. To use a different layer, edit
`scripts/crop_filter.yaml` and change `input_pointcloud_layer`:
```yaml
      input_pointcloud_layer: "static_map"    # cleaner — removes moving objects
      inside_pointcloud_layer: "static_map_cropped"
```
Then in `scripts/2_process_map.sh`, change the PLY filename on the `ply_to_laz.py` line:
```bash
python3 scripts/ply_to_laz.py "${NAME}_static_map_cropped.ply" ...
```

### Elevation color range (`index.html`)
The viewer colors points by height. Tune this to your Z range for maximum contrast:
```javascript
material.elevationRange = [-2, 8];   // tight range = vivid colors
```

### Point budget (`index.html`)
Increase this to render more points at once (may be slower on older machines):
```javascript
viewer.setPointBudget(2_000_000);
```

---

## Step 7 — Update the Viewer and Publish

### Edit `index.html`

Replace `YOUR_MAP_NAME` with the folder name printed at the end of Step 4:
```javascript
// Change this line:
Potree.loadPointCloud("./pointclouds/YOUR_MAP_NAME/cloud.js", "map", e => {

// To (example):
Potree.loadPointCloud("./pointclouds/rosbag2_2026_04_09-19_50_35_0.mcap/cloud.js", "map", e => {
```

Update the description:
```javascript
viewer.setDescription("EE5531 — Your Name — Location");
```

### Push to GitHub

```bash
git add pointclouds/ index.html
git commit -m "Add point cloud map"
git push
```

Wait ~60 seconds, then visit `https://robust-autonomous-systems-laboratory.github.io/<your-repo>/`.

---

## Troubleshooting

| Problem | Fix |
|---|---|
| `mola-lidar-odometry-cli: command not found` | Run `source /opt/ros/jazzy/setup.bash` first |
| `No module named 'laspy'` | `sudo apt install python3-laspy python3-laszip` |
| `mm2ply` error about `raw` layer | Normal — ignore it. The `map_cropped.ply` file is written before the error. |
| `PotreeConverter: cannot find liblaszip.so` | Run the script from the repo root — the script sets `LD_LIBRARY_PATH` for you |
| Blank page on GitHub Pages | Check that `pointclouds/<name>/cloud.js` exists and the name in `index.html` matches exactly |
| Point cloud loads but colors look washed out | Tighten `elevationRange` in `index.html` to match your actual Z bounds from `mm-info` |
| Map looks correct in mm-viewer but PLY is empty | Check that `map_cropped` layer exists: run `mm-info *_cropped.mm` |

---

## File Reference

```
scripts/
  1_run_slam.sh         bag → .simplemap → .mm (SLAM pipeline)
  2_process_map.sh      .mm → crop → PLY → LAZ → Potree tiles
  ply_to_laz.py         PLY → voxel decimated LAZ (edit --layer to change source layer)
  crop_filter.yaml      ← EDIT: set your spatial crop region and source layer
  sm2mm_voxels_static_dynamic_points.yaml   map generation config (advanced, do not edit)

index.html              Potree viewer ← EDIT: YOUR_MAP_NAME and description
PotreeConverter         Linux binary (included — no install needed)
liblaszip.so            Required by PotreeConverter (included)
resources/              Potree viewer libraries (do not edit)
```
