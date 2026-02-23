import re
import sqlite3
from pathlib import Path
from typing import List, Optional, Tuple

import numpy as np
import pandas as pd

# ---------- USER SETTINGS ----------
IMAGE_EXTS = {"jpg", "jpeg", "png", "tif", "tiff", "bmp"}

# Depth prior strength (tight)
Z_SIGMA = 0.01  # meters

# Planar prior strength (very loose)
XY_SIGMA = 1e6  # meters

# Depth sign convention:
# If your CSV depth is "meters below surface" (positive down), set True to map world z = -depth (z-up).
DEPTH_IS_POSITIVE_DOWN = True

# Use median over this many rows starting at the image_count increment row
MEDIAN_WINDOW = 1  # 1 = use only the increment row

# Coordinate system for COLMAP pose priors:
# Newer COLMAP DBs store this either as TEXT (e.g., 'CARTESIAN') or INTEGER enum.
# We'll detect the column type and write appropriately.
COORD_SYS_TEXT = "CARTESIAN"
COORD_SYS_INT_FALLBACK = 2  # fallback enum code if INTEGER is required (will be used only if needed)


# ---------- HELPERS ----------
def natural_key(s: str):
    return [int(t) if t.isdigit() else t.lower() for t in re.split(r"(\d+)", s)]


def list_images(folder: Path) -> List[Path]:
    imgs = [p for p in folder.iterdir()
            if p.is_file() and p.suffix.lower().lstrip(".") in IMAGE_EXTS]
    imgs.sort(key=lambda p: natural_key(p.name))
    return imgs


def find_collect_csv(folder: Path) -> Optional[Path]:
    cands = sorted(folder.glob("collect_*.csv"), key=lambda p: p.name)
    return cands[0] if cands else None


def first_rows_where_image_count_increments(df: pd.DataFrame) -> pd.DataFrame:
    ic = df["image_count"].astype(int).to_numpy()
    inc = np.zeros_like(ic, dtype=bool)
    inc[1:] = ic[1:] > ic[:-1]
    return df.loc[inc].copy()


def choose_project_folder() -> Path:
    import tkinter as tk
    from tkinter import filedialog

    root = tk.Tk()
    root.withdraw()
    root.attributes("-topmost", True)
    folder = filedialog.askdirectory(
        title="Select folder containing database.db and robot image subfolders"
    )
    root.destroy()

    if not folder:
        raise SystemExit("No folder selected. Exiting.")
    return Path(folder)


def find_database(project_folder: Path) -> Path:
    preferred = project_folder / "database.db"
    if preferred.exists():
        return preferred

    dbs = sorted(project_folder.glob("*.db"), key=lambda p: p.name.lower())
    if not dbs:
        raise FileNotFoundError(
            f"No .db file found in {project_folder}\n"
            "Expected database.db (or any *.db) in the selected folder."
        )
    return dbs[0]


def ensure_pose_priors_schema(conn: sqlite3.Connection) -> Tuple[List[str], str]:
    """
    Returns (columns, coord_system_declared_type).
    coord_system_declared_type is whatever PRAGMA table_info reports (e.g., 'TEXT', 'INTEGER', etc.).
    """
    cur = conn.cursor()
    cur.execute("SELECT name FROM sqlite_master WHERE type='table' AND name='pose_priors';")
    if cur.fetchone() is None:
        raise RuntimeError("Database has no 'pose_priors' table. Use pose_prior_mapper-capable COLMAP DB.")

    cur.execute("PRAGMA table_info(pose_priors);")
    info = cur.fetchall()  # (cid, name, type, notnull, dflt_value, pk)
    cols = [r[1] for r in info]

    expected = {"image_id", "position", "coordinate_system", "position_covariance"}
    if not expected.issubset(set(cols)):
        raise RuntimeError(
            f"pose_priors schema unexpected.\n"
            f"Have columns: {cols}\n"
            f"Need at least: {sorted(expected)}"
        )

    coord_type = None
    for cid, name, ctype, notnull, dflt, pk in info:
        if name == "coordinate_system":
            coord_type = (ctype or "").upper().strip()
            break
    if coord_type is None:
        coord_type = ""

    return cols, coord_type


def get_image_id_by_name(conn: sqlite3.Connection, rel_name: str) -> Optional[int]:
    cur = conn.cursor()
    cur.execute("SELECT image_id FROM images WHERE name = ?;", (rel_name,))
    r = cur.fetchone()
    return int(r[0]) if r else None


def pack_vec3_f64(v: np.ndarray) -> bytes:
    v = np.asarray(v, dtype=np.float64).reshape(3,)
    return v.tobytes(order="C")


def pack_mat3_f64(M: np.ndarray) -> bytes:
    M = np.asarray(M, dtype=np.float64).reshape(3, 3)
    return M.tobytes(order="C")


def coord_system_value(coord_type: str):
    """
    If coordinate_system column is declared as TEXT, write 'CARTESIAN'.
    If INTEGER, write an int code (fallback = 2).
    If blank/unknown, prefer TEXT; SQLite will often accept it anyway, but we keep it consistent.
    """
    if "INT" in coord_type:
        return COORD_SYS_INT_FALLBACK
    if "CHAR" in coord_type or "TEXT" in coord_type or "CLOB" in coord_type:
        return COORD_SYS_TEXT
    # Unknown type: default to text (most human-readable)
    return COORD_SYS_TEXT


def insert_or_replace_pose_prior(conn: sqlite3.Connection,
                                image_id: int,
                                position_xyz: np.ndarray,
                                covariance_3x3: np.ndarray,
                                coord_sys):
    """
    Works with schema:
      pose_priors(image_id, position, coordinate_system, position_covariance)
    with position and covariance packed as float64 blobs.
    """
    cur = conn.cursor()
    cur.execute("DELETE FROM pose_priors WHERE image_id = ?;", (image_id,))
    cur.execute(
        "INSERT INTO pose_priors (image_id, position, coordinate_system, position_covariance) "
        "VALUES (?, ?, ?, ?);",
        (
            image_id,
            sqlite3.Binary(pack_vec3_f64(position_xyz)),
            coord_sys,
            sqlite3.Binary(pack_mat3_f64(covariance_3x3)),
        )
    )


def main():
    project = choose_project_folder()
    db_path = find_database(project)

    print(f"Project folder: {project}")
    print(f"Database:       {db_path}")

    conn = sqlite3.connect(str(db_path))
    conn.execute("PRAGMA foreign_keys=ON;")

    cols, coord_type = ensure_pose_priors_schema(conn)
    print("pose_priors columns:", cols)
    print("coordinate_system declared type:", coord_type if coord_type else "(unknown)")

    coord_sys = coord_system_value(coord_type)
    print("Using coordinate_system value:", coord_sys)

    # Robot subfolders = directories under project that contain images + collect_*.csv
    robot_folders = [p for p in project.iterdir() if p.is_dir()]
    robot_folders.sort(key=lambda p: natural_key(p.name))

    total_written = 0
    warnings = 0

    for rf in robot_folders:
        csv_path = find_collect_csv(rf)
        images = list_images(rf)

        if csv_path is None or not images:
            continue  # ignore non-robot folders

        print(f"\n--- {rf.name} ---")
        print(f"CSV:    {csv_path.name}")
        print(f"Images: {len(images)}")

        df = pd.read_csv(csv_path, on_bad_lines="skip")

        required = {"depth", "image_count"}
        if not required.issubset(df.columns):
            print(f"[WARN] Missing required columns in {csv_path}: have={list(df.columns)} need={sorted(required)}")
            warnings += 1
            continue

        inc_rows = first_rows_where_image_count_increments(df)
        if inc_rows.empty:
            print(f"[WARN] No image_count increments found in {csv_path}")
            warnings += 1
            continue

        # Covariance in the chosen world frame: [x,y,z] with z = vertical axis by convention
        cov = np.diag([XY_SIGMA ** 2, XY_SIGMA ** 2, Z_SIGMA ** 2]).astype(np.float64)

        for _, row in inc_rows.iterrows():
            k = int(row["image_count"])
            if k <= 0:
                continue
            if k > len(images):
                print(f"[WARN] image_count={k} but only {len(images)} images. Skipping.")
                warnings += 1
                continue

            img_path = images[k - 1]
            rel_name = str(img_path.relative_to(project)).replace("\\", "/")

            # median window starting at increment row
            idx = int(row.name)
            win = df.iloc[idx: idx + max(1, MEDIAN_WINDOW)]
            depth = float(np.nanmedian(win["depth"].to_numpy()))

            # Define world vertical axis as z. If depth is positive-down, map to z-up by negating.
            z_world = -depth if DEPTH_IS_POSITIVE_DOWN else depth

            image_id = get_image_id_by_name(conn, rel_name)
            if image_id is None:
                print(f"[WARN] Image not found in DB: {rel_name}")
                warnings += 1
                continue

            # Position prior: only vertical is meaningful; x/y arbitrary but covariance makes them irrelevant
            pos = np.array([0.0, 0.0, z_world], dtype=np.float64)

            insert_or_replace_pose_prior(conn, image_id, pos, cov, coord_sys)
            total_written += 1

    conn.commit()
    conn.close()

    print(f"\nDone. Wrote depth-based pose priors for {total_written} images. Warnings: {warnings}")
    print("Pipeline reminder:")
    print("  1) feature_extractor (adds images to DB)")
    print("  2) run THIS script (writes pose_priors)")
    print("  3) matcher")
    print("  4) pose_prior_mapper")


if __name__ == "__main__":
    main()