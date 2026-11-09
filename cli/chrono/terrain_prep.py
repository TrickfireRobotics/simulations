"""Turning survey data into terrain the simulator can load.

URC does not guarantee a DEM release - the 2027 Q&A says one may appear, probably as
GeoTIFF, but neither release nor timing is promised. So this reads whatever is available
(GeoTIFF via rasterio, ESRI ASCII grids and XYZ point clouds with no dependencies at
all) and writes two things: a greyscale height map for SCM, and a metadata sidecar
recording the source CRS, transform, resolution, vertical scale, crop and content hash.

The sidecar is the point. A height map on its own is an image; with the sidecar it is a
traceable piece of terrain whose provenance survives into every run manifest that uses it.
"""

from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path
from typing import Any

import numpy as np
import yaml

from ..output import die, info, warn
from ..paths import CHRONO_TERRAIN_DATA_DIR, REPO_DIR
from . import configs


class TerrainPrepError(Exception):
    pass


def build_heightmap(
    source: Path,
    name: str,
    crop: list[float] | None = None,
    resolution: float = 0.05,
    level: bool = True,
    vertical_scale: float = 1.0,
    force: bool = False,
) -> Path:
    """Read a DEM, crop and resample it, and write a height map plus its metadata."""
    directory = CHRONO_TERRAIN_DATA_DIR / name
    if directory.exists() and not force:
        die(f"{directory} already exists. Pass --force to replace it.")
    directory.mkdir(parents=True, exist_ok=True)

    info(f"reading {source.name}")
    elevation, transform, crs = _read_source(source)
    info(f"source grid {elevation.shape[1]} x {elevation.shape[0]}")

    if crop:
        elevation, transform = _crop(elevation, transform, crop)
        info(f"cropped to {elevation.shape[1]} x {elevation.shape[0]}")

    elevation = _fill_gaps(elevation)

    size_x = abs(transform["pixel_width"]) * elevation.shape[1]
    size_y = abs(transform["pixel_height"]) * elevation.shape[0]

    resampled = _resample(elevation, size_x, size_y, resolution)
    info(f"resampled to {resampled.shape[1]} x {resampled.shape[0]} at {resolution} m")

    plane_removed = None
    if level:
        resampled, plane_removed = _remove_mean_plane(resampled, resolution)
        info(
            f"removed a mean plane of {math.degrees(plane_removed['slope_rad']):.2f} deg; "
            "set the terrain's pitch_deg/roll_deg if you wanted to keep the slope"
        )

    if vertical_scale != 1.0:
        resampled *= vertical_scale
        warn(
            f"relief scaled by {vertical_scale}x. This is for visualisation and debugging; "
            "a scaled terrain is not the real terrain and must not be used for a "
            "capability claim."
        )

    height_min = float(np.nanmin(resampled))
    height_max = float(np.nanmax(resampled))
    if height_max - height_min < 1e-6:
        raise TerrainPrepError(
            "the cropped region is perfectly flat, so there is no relief to simulate. "
            "Check the crop window and the source's vertical units."
        )

    image_path = directory / "heightmap.png"
    _write_greyscale(resampled, height_min, height_max, image_path)
    info(f"wrote {_relative(image_path)}")

    mesh_path = directory / "surface.obj"
    _write_mesh(resampled, size_x, size_y, mesh_path)
    info(f"wrote {_relative(mesh_path)} (for rigid-terrain use)")

    metadata = {
        "name": name,
        "source": {
            "file": str(source),
            "hash": _file_hash(source),
            "crs": crs or "unknown",
            "pixel_width": transform["pixel_width"],
            "pixel_height": transform["pixel_height"],
            "origin": [transform["origin_x"], transform["origin_y"]],
            "crop": crop,
        },
        "output": {
            "heightmap": image_path.name,
            "mesh": mesh_path.name,
            "size_x": round(size_x, 4),
            "size_y": round(size_y, 4),
            "height_min": round(height_min, 4),
            "height_max": round(height_max, 4),
            "resolution_m": resolution,
            "vertical_scale": vertical_scale,
            "grid": [resampled.shape[1], resampled.shape[0]],
        },
        "processing": {
            "levelled": level,
            "plane_removed": plane_removed,
            "gaps_filled": True,
        },
    }
    metadata_path = directory / "metadata.json"
    metadata_path.write_text(json.dumps(metadata, indent=2) + "\n")
    info(f"wrote {_relative(metadata_path)}")

    _print_terrain_stanza(name, metadata)
    return image_path


def _print_terrain_stanza(name: str, metadata: dict[str, Any]) -> None:
    """Print the terrain config to paste in, filled from the metadata."""
    output = metadata["output"]
    source = metadata["source"]

    stanza = {
        "name": name,
        "version": 1,
        "description": f"Tier 2 - SCM over {Path(source['file']).name}",
        "type": "scm_heightmap",
        "heightmap_file": f"data/terrain/{name}/{output['heightmap']}",
        "size_x": output["size_x"],
        "size_y": output["size_y"],
        "height_min": output["height_min"],
        "height_max": output["height_max"],
        "grid_delta": max(0.02, output["resolution_m"]),
        "soils": ["REPLACE_WITH_MEASURED_SOIL_PROFILE"],
        "default_soil": "REPLACE_WITH_MEASURED_SOIL_PROFILE",
        "bulldozing": True,
        "active_domains": True,
        "active_domain_margin": 0.3,
        "source": {
            "file": source["file"],
            "hash": source["hash"],
            "crs": source["crs"],
            "resolution_m": output["resolution_m"],
            "vertical_scale": output["vertical_scale"],
        },
    }

    print(f"\n# paste into {configs.kind_dir('terrain').relative_to(REPO_DIR)}/{name}.yaml\n")
    print(yaml.safe_dump(stanza, sort_keys=False, default_flow_style=None))
    print(
        "# The soil is left as a placeholder on purpose: geometry from a DEM says nothing\n"
        "# about terramechanics, and the simulator will refuse to run until a measured\n"
        "# soil profile is named here.\n"
    )


# ---------------------------------------------------------------------------- #
#                                 source readers                               #
# ---------------------------------------------------------------------------- #


def _read_source(source: Path) -> tuple[np.ndarray, dict[str, float], str]:
    if not source.is_file():
        raise TerrainPrepError(f"no such file: {source}")

    suffix = source.suffix.lower()
    if suffix in {".tif", ".tiff"}:
        return _read_geotiff(source)
    if suffix in {".asc", ".grd"}:
        return _read_ascii_grid(source)
    if suffix in {".xyz", ".txt", ".csv", ".pts"}:
        return _read_point_cloud(source)
    raise TerrainPrepError(
        f"unsupported source '{suffix}'. Handled: .tif/.tiff (GeoTIFF, needs rasterio), "
        ".asc/.grd (ESRI ASCII grid), .xyz/.txt/.csv/.pts (point cloud)."
    )


def _read_geotiff(source: Path) -> tuple[np.ndarray, dict[str, float], str]:
    try:
        import rasterio
    except ImportError as exc:
        raise TerrainPrepError(
            "reading GeoTIFF needs rasterio, which is an optional dependency:\n"
            "  pixi add rasterio   (or: pip install rasterio)\n"
            "Alternatively convert it once with GDAL:\n"
            "  gdal_translate -of AAIGrid input.tif output.asc\n"
            "and import the ASCII grid, which needs no extra packages."
        ) from exc

    with rasterio.open(source) as dataset:
        band = dataset.read(1, masked=True)
        elevation = np.ma.filled(band.astype(float), np.nan)
        transform = {
            "pixel_width": float(dataset.transform.a),
            "pixel_height": float(dataset.transform.e),
            "origin_x": float(dataset.transform.c),
            "origin_y": float(dataset.transform.f),
        }
        crs = str(dataset.crs) if dataset.crs else ""
    return elevation, transform, crs


def _read_ascii_grid(source: Path) -> tuple[np.ndarray, dict[str, float], str]:
    """ESRI ASCII grid. Header keys, then rows of values, top row first."""
    header: dict[str, float] = {}
    values: list[float] = []

    with source.open() as handle:
        for line in handle:
            parts = line.split()
            if not parts:
                continue
            key = parts[0].lower()
            if (
                key
                in {
                    "ncols",
                    "nrows",
                    "xllcorner",
                    "yllcorner",
                    "cellsize",
                    "nodata_value",
                    "xllcenter",
                    "yllcenter",
                }
                and len(parts) == 2
            ):
                header[key] = float(parts[1])
            else:
                values += [float(p) for p in parts]

    required = {"ncols", "nrows", "cellsize"}
    missing = required - set(header)
    if missing:
        raise TerrainPrepError(f"{source.name} is missing header keys: {', '.join(missing)}")

    cols, rows = int(header["ncols"]), int(header["nrows"])
    if len(values) != cols * rows:
        raise TerrainPrepError(
            f"{source.name} declares {cols} x {rows} = {cols * rows} cells but holds {len(values)}"
        )

    elevation = np.array(values, dtype=float).reshape(rows, cols)
    nodata = header.get("nodata_value")
    if nodata is not None:
        elevation[elevation == nodata] = np.nan

    cell = header["cellsize"]
    origin_x = header.get("xllcorner", header.get("xllcenter", 0.0))
    origin_y = header.get("yllcorner", header.get("yllcenter", 0.0))

    return (
        elevation,
        {
            "pixel_width": cell,
            # Negative because an ASCII grid's first row is its northernmost, matching
            # the GeoTIFF convention this returns.
            "pixel_height": -cell,
            "origin_x": origin_x,
            "origin_y": origin_y + rows * cell,
        },
        "",
    )


def _read_point_cloud(source: Path) -> tuple[np.ndarray, dict[str, float], str]:
    """XYZ points, binned onto a regular grid by taking the mean height per cell."""
    points = []
    with source.open() as handle:
        for line in handle:
            if not line.strip() or line.lstrip().startswith("#"):
                continue
            parts = line.replace(",", " ").split()
            if len(parts) < 3:
                continue
            try:
                points.append((float(parts[0]), float(parts[1]), float(parts[2])))
            except ValueError:
                continue  # a header row

    if len(points) < 16:
        raise TerrainPrepError(f"{source.name} holds only {len(points)} usable points")

    data = np.array(points)
    x, y, z = data[:, 0], data[:, 1], data[:, 2]

    # Cell size from the point density, so a dense scan is not thrown away and a sparse
    # one does not produce a grid that is mostly holes.
    area = (x.max() - x.min()) * (y.max() - y.min())
    cell = math.sqrt(area / len(points)) if area > 0 else 1.0
    cols = max(2, int((x.max() - x.min()) / cell) + 1)
    rows = max(2, int((y.max() - y.min()) / cell) + 1)

    totals = np.zeros((rows, cols))
    counts = np.zeros((rows, cols))

    col_index = np.clip(((x - x.min()) / cell).astype(int), 0, cols - 1)
    row_index = np.clip(((y.max() - y) / cell).astype(int), 0, rows - 1)
    np.add.at(totals, (row_index, col_index), z)
    np.add.at(counts, (row_index, col_index), 1)

    with np.errstate(invalid="ignore"):
        elevation = np.where(counts > 0, totals / np.maximum(counts, 1), np.nan)

    filled = int(np.count_nonzero(counts))
    info(f"binned {len(points)} points into {cols} x {rows} cells ({filled} occupied)")

    return (
        elevation,
        {
            "pixel_width": cell,
            "pixel_height": -cell,
            "origin_x": float(x.min()),
            "origin_y": float(y.max()),
        },
        "",
    )


# ---------------------------------------------------------------------------- #
#                                  processing                                  #
# ---------------------------------------------------------------------------- #


def _crop(
    elevation: np.ndarray, transform: dict[str, float], crop: list[float]
) -> tuple[np.ndarray, dict[str, float]]:
    min_x, min_y, max_x, max_y = crop
    width, height = transform["pixel_width"], transform["pixel_height"]

    left = int((min_x - transform["origin_x"]) / width)
    right = int((max_x - transform["origin_x"]) / width)
    # pixel_height is negative, so the row order flips relative to the y coordinates.
    top = int((max_y - transform["origin_y"]) / height)
    bottom = int((min_y - transform["origin_y"]) / height)

    left, right = sorted((max(0, left), min(elevation.shape[1], right)))
    top, bottom = sorted((max(0, top), min(elevation.shape[0], bottom)))

    if right - left < 2 or bottom - top < 2:
        raise TerrainPrepError(
            f"the crop window {crop} selects {right - left} x {bottom - top} cells. "
            "Check that it is in the source's own coordinates, not metres from a corner."
        )

    return elevation[top:bottom, left:right], {
        **transform,
        "origin_x": transform["origin_x"] + left * width,
        "origin_y": transform["origin_y"] + top * height,
    }


def _fill_gaps(elevation: np.ndarray) -> np.ndarray:
    """Fill nodata cells by nearest-neighbour growth.

    A DEM with holes - voids behind vegetation, scan shadows - produces an SCM patch with
    pits the rover drives into. Filling them is a repair, so the count is reported: a
    source that is mostly holes should be rejected by the person, not patched over.
    """
    holes = np.isnan(elevation)
    count = int(np.count_nonzero(holes))
    if count == 0:
        return elevation
    if count == elevation.size:
        raise TerrainPrepError("every cell in the cropped region is nodata")

    fraction = count / elevation.size
    if fraction > 0.3:
        warn(
            f"{100 * fraction:.0f}% of the cropped region is nodata and is being filled by "
            "interpolation. That is a lot of invented terrain - consider a different crop."
        )
    else:
        info(f"filled {count} nodata cell(s) ({100 * fraction:.2f}%)")

    filled = elevation.copy()
    # Iterative mean of valid neighbours: a few passes close typical scan gaps, and it
    # needs no scipy.
    for _ in range(64):
        holes = np.isnan(filled)
        if not holes.any():
            break
        padded = np.pad(filled, 1, mode="edge")
        neighbours = np.stack(
            [
                padded[:-2, 1:-1],
                padded[2:, 1:-1],
                padded[1:-1, :-2],
                padded[1:-1, 2:],
            ]
        )
        with np.errstate(invalid="ignore"):
            mean = np.nanmean(neighbours, axis=0)
        filled = np.where(holes & ~np.isnan(mean), mean, filled)

    # Anything still open is surrounded entirely by holes; flatten it to the mean.
    remaining = np.isnan(filled)
    if remaining.any():
        filled[remaining] = float(np.nanmean(filled))
    return filled


def _resample(elevation: np.ndarray, size_x: float, size_y: float, resolution: float) -> np.ndarray:
    """Bilinear resample onto a regular grid at the requested spacing."""
    cols = max(4, round(size_x / resolution) + 1)
    rows = max(4, round(size_y / resolution) + 1)

    if cols * rows > 16_000_000:
        raise TerrainPrepError(
            f"{cols} x {rows} = {cols * rows / 1e6:.1f} M cells at {resolution} m. "
            "Coarsen --resolution or crop a smaller region: SCM's cost grows with the "
            "grid, and a patch this size will not run at a useful speed anyway."
        )

    source_rows, source_cols = elevation.shape
    y = np.linspace(0, source_rows - 1, rows)
    x = np.linspace(0, source_cols - 1, cols)

    y0 = np.clip(np.floor(y).astype(int), 0, source_rows - 1)
    y1 = np.clip(y0 + 1, 0, source_rows - 1)
    x0 = np.clip(np.floor(x).astype(int), 0, source_cols - 1)
    x1 = np.clip(x0 + 1, 0, source_cols - 1)

    wy = (y - y0)[:, None]
    wx = (x - x0)[None, :]

    top = elevation[np.ix_(y0, x0)] * (1 - wx) + elevation[np.ix_(y0, x1)] * wx
    bottom = elevation[np.ix_(y1, x0)] * (1 - wx) + elevation[np.ix_(y1, x1)] * wx
    return top * (1 - wy) + bottom * wy


def _remove_mean_plane(
    elevation: np.ndarray, resolution: float
) -> tuple[np.ndarray, dict[str, float]]:
    """Subtract the least-squares plane, leaving relief about a level datum.

    A cropped hillside is mostly a tilted slab, and leaving the tilt in the height map
    wastes the image's dynamic range on it. The slope is reported and recorded so it can
    be reinstated as the terrain's pitch_deg and roll_deg, where it belongs.
    """
    rows, cols = elevation.shape
    y, x = np.mgrid[0:rows, 0:cols]
    x = x.ravel() * resolution
    y = y.ravel() * resolution
    z = elevation.ravel()

    design = np.column_stack([x, y, np.ones_like(x)])
    coefficients, *_ = np.linalg.lstsq(design, z, rcond=None)
    gradient_x, gradient_y, offset = coefficients

    plane = (design @ coefficients).reshape(elevation.shape)
    slope = math.atan(math.hypot(gradient_x, gradient_y))

    return elevation - plane, {
        "slope_rad": round(float(slope), 6),
        "slope_deg": round(math.degrees(slope), 4),
        "gradient_x": round(float(gradient_x), 8),
        "gradient_y": round(float(gradient_y), 8),
        "offset_m": round(float(offset), 4),
        "suggested_pitch_deg": round(math.degrees(math.atan(gradient_x)), 4),
        "suggested_roll_deg": round(math.degrees(math.atan(gradient_y)), 4),
    }


def _write_greyscale(
    elevation: np.ndarray, height_min: float, height_max: float, path: Path
) -> None:
    """Write a 16-bit greyscale PNG.

    16 bits rather than 8: over a 10 m relief range, 8 bits quantises to 4 cm steps,
    which SCM renders as visible terraces and which a sinkage measurement cannot
    distinguish from real terrain.
    """
    normalised = (elevation - height_min) / (height_max - height_min)
    pixels = np.clip(np.round(normalised * 65535), 0, 65535).astype(np.uint16)

    try:
        from PIL import Image

        Image.fromarray(pixels, mode="I;16").save(path)
        return
    except ImportError:
        pass

    _write_png_16bit(pixels, path)


def _write_png_16bit(pixels: np.ndarray, path: Path) -> None:
    """Minimal 16-bit greyscale PNG writer, so no image library is required."""
    import struct
    import zlib

    rows, cols = pixels.shape
    raw = bytearray()
    big_endian = pixels.astype(">u2")
    for row in range(rows):
        raw.append(0)  # filter type: none
        raw += big_endian[row].tobytes()

    def chunk(kind: bytes, payload: bytes) -> bytes:
        header = struct.pack(">I", len(payload)) + kind
        return header + payload + struct.pack(">I", zlib.crc32(kind + payload) & 0xFFFFFFFF)

    header = struct.pack(">IIBBBBB", cols, rows, 16, 0, 0, 0, 0)
    path.write_bytes(
        b"\x89PNG\r\n\x1a\n"
        + chunk(b"IHDR", header)
        + chunk(b"IDAT", zlib.compress(bytes(raw), 6))
        + chunk(b"IEND", b"")
    )


def _write_mesh(elevation: np.ndarray, size_x: float, size_y: float, path: Path) -> None:
    """Write the same surface as an OBJ, for rigid-terrain use.

    The plan asks for both a height map for SCM and a mesh for rigid terrain from one
    source, so the two tiers describe the same ground rather than two similar grounds.
    """
    rows, cols = elevation.shape

    # Cap the mesh: a 2 M-vertex OBJ is slow to load and Bullet will not thank you for it.
    step = max(1, math.ceil(math.sqrt(rows * cols / 250_000)))
    sampled = elevation[::step, ::step]
    rows, cols = sampled.shape

    dx = size_x / max(1, cols - 1)
    dy = size_y / max(1, rows - 1)

    lines = [
        "# Rigid-terrain surface, generated by `sim chrono terrain heightmap`.",
        f"# {cols} x {rows} vertices over {size_x:.3f} x {size_y:.3f} m",
    ]
    for row in range(rows):
        for col in range(cols):
            x = -size_x / 2 + col * dx
            y = size_y / 2 - row * dy
            lines.append(f"v {x:.5f} {y:.5f} {sampled[row][col]:.5f}")

    for row in range(rows - 1):
        for col in range(cols - 1):
            a = row * cols + col + 1
            b = a + 1
            c = a + cols
            d = c + 1
            lines.append(f"f {a} {c} {b}")
            lines.append(f"f {b} {c} {d}")

    path.write_text("\n".join(lines) + "\n")


# ---------------------------------------------------------------------------- #
#                                  rock fields                                 #
# ---------------------------------------------------------------------------- #


def print_rock_field(terrain_name: str, seed: int | None) -> None:
    """Expand a terrain's rock_field and print the obstacles it produces.

    The expansion itself happens when a run is prepared, so the manifest carries the
    explicit rocks. This command exists to inspect a layout before committing to it.
    """
    from . import runner

    try:
        terrain, path, _ = configs.load("terrain", terrain_name)
    except configs.ConfigError as exc:
        die(str(exc))

    if not terrain.get("rock_field"):
        die(f"{path.name} has no rock_field section")

    if seed is not None:
        terrain["rock_field"]["seed"] = seed

    expanded = runner._expand_rock_field(
        {"terrain": terrain, "run": {"seed": seed or terrain["rock_field"].get("seed", 0)}}
    )
    obstacles = expanded["terrain"].get("obstacles", [])

    fixed = sum(1 for o in obstacles if o.get("fixed", True))
    print(f"\n{len(obstacles)} obstacle(s): {fixed} fixed, {len(obstacles) - fixed} loose\n")
    print(yaml.safe_dump({"obstacles": obstacles}, sort_keys=False, default_flow_style=None))


def _file_hash(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1 << 20), b""):
            digest.update(chunk)
    return f"sha256:{digest.hexdigest()}"


def _relative(path: Path) -> str:
    try:
        return str(path.resolve().relative_to(REPO_DIR))
    except ValueError:
        return str(path)
