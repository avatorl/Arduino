import argparse
import csv
import math
import re
import sys
from collections import defaultdict
from pathlib import Path


CSV_HEADER = [
    "capture_label",
    "elapsed_ms",
    "raw_r",
    "raw_g",
    "raw_b",
    "raw_c",
    "balanced_r",
    "balanced_g",
    "balanced_b",
    "normalized_r",
    "normalized_g",
    "normalized_b",
]
REQUIRED_LABELS = ("white", "black", "nothing")
MAX_CLUSTERS_PER_COLOR = 3
MIN_CLUSTER_SAMPLES_PER_COLOR = 3
CLUSTER_IMPROVEMENT_THRESHOLD = 0.20
CLUSTER_RADIUS_SAFETY_MARGIN = 0
TRAIN_MARKER_CLASS_BY_LABEL = {
    "white": "MarkerWhite",
    "brown": "MarkerBrown",
    "cyan": "MarkerCyan",
    "green": "MarkerGreen",
    "grey": "MarkerGrey",
    "magenta": "MarkerMagenta",
    "orange": "MarkerOrange",
    "red": "MarkerRed",
    "yellow": "MarkerYellow",
}


def canonical_label(label: str) -> str:
    return label.strip().lower()


def sanitize_label(label: str) -> str:
    slug = re.sub(r"[^0-9A-Za-z]+", "_", label.strip().lower()).strip("_")
    return slug or "capture"


def parse_csv_line(line: str):
    rows = list(csv.reader([line]))
    if not rows:
        return []
    return rows[0]


def to_int(value: str) -> int:
    return int(value.strip())


def mean(values):
    return sum(values) / len(values) if values else 0.0


def sample_stddev(values):
    if len(values) < 2:
        return 0.0
    avg = mean(values)
    variance = sum((value - avg) ** 2 for value in values) / (len(values) - 1)
    return math.sqrt(variance)


def normalize_rgb(r: float, g: float, b: float):
    total = r + g + b
    if total <= 0:
        return (-1, -1, -1)
    return (
        int(round(r * 1000.0 / total)),
        int(round(g * 1000.0 / total)),
        int(round(b * 1000.0 / total)),
    )


def apply_white_balance(row, gains):
    return (
        int(round(row["raw_r"] * gains[0])),
        int(round(row["raw_g"] * gains[1])),
        int(round(row["raw_b"] * gains[2])),
    )


def get_label_rows(input_dir: Path):
    rows_by_label = {}
    for csv_path in sorted(input_dir.glob("*.csv")):
        if csv_path.name == "all_captures.csv":
            continue

        with csv_path.open("r", newline="", encoding="utf-8") as handle:
            reader = csv.DictReader(handle)
            rows = []
            for raw_row in reader:
                row = {"capture_label": raw_row["capture_label"]}
                for key in CSV_HEADER[1:]:
                    row[key] = to_int(raw_row[key])
                rows.append(row)

        if rows:
            rows_by_label[canonical_label(rows[0]["capture_label"])] = rows

    return rows_by_label


def ensure_required_labels(rows_by_label):
    missing = [label for label in REQUIRED_LABELS if label not in rows_by_label]
    if missing:
        joined = ", ".join(missing)
        raise SystemExit(f"Missing required calibration labels: {joined}")

    if len([label for label in rows_by_label if label not in ("black", "nothing", "white")]) == 0:
        raise SystemExit("Missing target color captures. Record at least one named color in addition to white, black, and nothing.")


def compute_white_balance_gains(white_rows):
    white_r = mean([row["raw_r"] for row in white_rows])
    white_g = mean([row["raw_g"] for row in white_rows])
    white_b = mean([row["raw_b"] for row in white_rows])

    if white_r <= 0 or white_g <= 0 or white_b <= 0:
        raise SystemExit("White samples must contain non-zero raw RGB means to derive white-balance gains.")

    return (1.0, white_r / white_g, white_r / white_b)


def enrich_rows(rows_by_label, gains):
    enriched = {}
    for label, rows in rows_by_label.items():
        enriched_rows = []
        for row in rows:
            balanced_r, balanced_g, balanced_b = apply_white_balance(row, gains)
            normalized_r, normalized_g, normalized_b = normalize_rgb(balanced_r, balanced_g, balanced_b)
            max_channel = max(balanced_r, balanced_g, balanced_b)
            min_channel = min(balanced_r, balanced_g, balanced_b)
            spread_pct = int(round((max_channel - min_channel) * 100.0 / max_channel)) if max_channel > 0 else 100
            enriched_row = dict(row)
            enriched_row["balanced_r"] = balanced_r
            enriched_row["balanced_g"] = balanced_g
            enriched_row["balanced_b"] = balanced_b
            enriched_row["normalized_r"] = normalized_r
            enriched_row["normalized_g"] = normalized_g
            enriched_row["normalized_b"] = normalized_b
            enriched_row["spread_pct"] = spread_pct
            enriched_rows.append(enriched_row)
        enriched[label] = enriched_rows
    return enriched


def ordered_profile_labels(rows_by_label):
    labels = [label for label in rows_by_label.keys() if label != "nothing" and label != "black"]
    labels.sort()
    if "white" in labels:
        labels.remove("white")
        labels.insert(0, "white")
    return labels


def get_prototype(rows):
    normalized_rows = [row for row in rows if row["normalized_r"] >= 0]
    return (
        int(round(mean([row["normalized_r"] for row in normalized_rows]))),
        int(round(mean([row["normalized_g"] for row in normalized_rows]))),
        int(round(mean([row["normalized_b"] for row in normalized_rows]))),
    )


def prototype_distance(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1]) + abs(a[2] - b[2])


def percentile_95(values):
    if not values:
        return 0

    ordered = sorted(values)
    index = int(math.ceil(len(ordered) * 0.95)) - 1
    if index < 0:
        index = 0
    if index >= len(ordered):
        index = len(ordered) - 1
    return ordered[index]


def compute_cluster_center(points):
    return (
        int(round(mean([point[0] for point in points]))),
        int(round(mean([point[1] for point in points]))),
        int(round(mean([point[2] for point in points]))),
    )


def choose_initial_centers(points, cluster_count):
    unique_points = sorted(set(points))
    if len(unique_points) < cluster_count:
        raise SystemExit(
            f"Not enough distinct samples to build {cluster_count} clusters. "
            f"Only {len(unique_points)} distinct points are available."
        )

    overall_center = compute_cluster_center(unique_points)
    first_center = min(unique_points, key=lambda point: (prototype_distance(point, overall_center), point))
    centers = [first_center]

    while len(centers) < cluster_count:
        candidate = max(
            (point for point in unique_points if point not in centers),
            key=lambda point: (
                min(prototype_distance(point, center) for center in centers),
                point,
            ),
        )
        centers.append(candidate)

    return centers


def assign_points_to_centers(points, centers):
    assignments = [[] for _ in centers]
    for point in points:
        best_index = 0
        best_distance = prototype_distance(point, centers[0])
        for index in range(1, len(centers)):
            distance = prototype_distance(point, centers[index])
            if distance < best_distance:
                best_distance = distance
                best_index = index
        assignments[best_index].append(point)
    return assignments


def fit_clusters(points, cluster_count):
    centers = choose_initial_centers(points, cluster_count)

    for _ in range(20):
        assignments = assign_points_to_centers(points, centers)
        if any(len(cluster_points) == 0 for cluster_points in assignments):
            raise SystemExit(f"Unable to build {cluster_count} stable clusters because one cluster received no samples.")

        next_centers = [compute_cluster_center(cluster_points) for cluster_points in assignments]
        if next_centers == centers:
            break
        centers = next_centers

    assignments = assign_points_to_centers(points, centers)
    clusters = []
    total_distance = 0

    for center, cluster_points in zip(centers, assignments):
        distances = [prototype_distance(point, center) for point in cluster_points]
        total_distance += sum(distances)
        clusters.append(
            {
                "center": center,
                "radius": percentile_95(distances) + CLUSTER_RADIUS_SAFETY_MARGIN,
                "sample_count": len(cluster_points),
                "mean_distance": mean(distances),
            }
        )

    score = total_distance / len(points)
    clusters.sort(key=lambda cluster: (cluster["center"][0], cluster["center"][1], cluster["center"][2]))
    return {"clusters": clusters, "score": score}


def extract_cluster_points(rows):
    points = []
    for row in rows:
        if row["normalized_r"] < 0 or row["normalized_g"] < 0 or row["normalized_b"] < 0:
            continue
        points.append((row["normalized_r"], row["normalized_g"], row["normalized_b"]))
    return points


def build_clusters_for_label(label, rows):
    points = extract_cluster_points(rows)
    if len(points) < MIN_CLUSTER_SAMPLES_PER_COLOR:
        raise SystemExit(
            f"Color '{rows[0]['capture_label']}' has only {len(points)} usable samples; "
            f"at least {MIN_CLUSTER_SAMPLES_PER_COLOR} are required for cluster generation."
        )

    best_fit = fit_clusters(points, 1)
    best_cluster_count = 1
    previous_score = best_fit["score"]
    max_candidate_count = min(MAX_CLUSTERS_PER_COLOR, len(set(points)))

    for candidate_count in range(2, max_candidate_count + 1):
        candidate_fit = fit_clusters(points, candidate_count)
        improvement = 0.0
        if previous_score > 0:
            improvement = (previous_score - candidate_fit["score"]) / previous_score

        if improvement >= CLUSTER_IMPROVEMENT_THRESHOLD:
            best_fit = candidate_fit
            best_cluster_count = candidate_count
            previous_score = candidate_fit["score"]
        else:
            break

    return {
        "label": label,
        "display_name": rows[0]["capture_label"],
        "cluster_count": best_cluster_count,
        "score": best_fit["score"],
        "clusters": best_fit["clusters"],
    }


def derive_thresholds(enriched_rows, profile_labels):
    white_rows = enriched_rows["white"]
    nothing_rows = enriched_rows["nothing"]
    black_rows = enriched_rows["black"]

    nothing_max_clear = max(row["raw_c"] for row in nothing_rows)
    nothing_mean_clear = mean([row["raw_c"] for row in nothing_rows])
    nothing_stddev_clear = sample_stddev([row["raw_c"] for row in nothing_rows])
    color_clear_min_threshold = max(50, int(math.ceil(max(nothing_max_clear + 1, nothing_mean_clear + 2.0 * nothing_stddev_clear))))

    white_mean_clear = mean([row["raw_c"] for row in white_rows])
    white_balanced_clear_threshold = max(1200, int(round(white_mean_clear * 0.7)))
    white_channel_threshold = max(
        200,
        int(
            round(
                min(
                    mean([row["balanced_r"] for row in white_rows]),
                    mean([row["balanced_g"] for row in white_rows]),
                    mean([row["balanced_b"] for row in white_rows]),
                )
                * 0.7
            )
        ),
    )
    color_clear_white_threshold = max(white_balanced_clear_threshold, int(round(white_mean_clear * 0.9)))
    white_balanced_max_spread_pct = min(25, max(6, max(row["spread_pct"] for row in white_rows) + 2))

    non_white_profile_rows = [row for label in profile_labels if label != "white" for row in enriched_rows[label]]
    if not non_white_profile_rows:
        raise SystemExit("Need at least one non-white color capture to derive profile thresholds.")

    color_match_clear_threshold = max(
        color_clear_min_threshold,
        int(round(min(row["raw_c"] for row in non_white_profile_rows) * 0.85)),
    )

    prototypes = {label: get_prototype(enriched_rows[label]) for label in profile_labels}
    compared_labels = [label for label in profile_labels if label != "white"]
    nearest_distance = None
    for left_index, left_label in enumerate(compared_labels):
        for right_label in compared_labels[left_index + 1:]:
            distance = prototype_distance(prototypes[left_label], prototypes[right_label])
            nearest_distance = distance if nearest_distance is None else min(nearest_distance, distance)

    if nearest_distance is None:
        nearest_distance = 110

    prototype_distance_threshold = min(160, max(50, int(round(nearest_distance * 0.6))))

    black_max_clear = max(row["raw_c"] for row in black_rows)

    return {
        "color_clear_min_threshold": color_clear_min_threshold,
        "color_clear_white_threshold": color_clear_white_threshold,
        "color_white_channel_threshold": white_channel_threshold,
        "white_balanced_clear_threshold": white_balanced_clear_threshold,
        "white_balanced_max_spread_pct": white_balanced_max_spread_pct,
        "color_match_clear_threshold": color_match_clear_threshold,
        "prototype_distance_threshold": prototype_distance_threshold,
        "black_reference_max_clear": black_max_clear,
        "nothing_reference_max_clear": nothing_max_clear,
    }, prototypes


def build_calibration_block(rows_by_label):
    ensure_required_labels(rows_by_label)
    gains = compute_white_balance_gains(rows_by_label["white"])
    enriched_rows = enrich_rows(rows_by_label, gains)
    profile_labels = ordered_profile_labels(rows_by_label)
    thresholds, prototypes = derive_thresholds(enriched_rows, profile_labels)

    output_lines = []
    output_lines.append("// Generated by rgb-color-sensor/calibration_tool.py")
    output_lines.append("// 'black' and 'nothing' captures are used to derive low-light thresholds.")
    output_lines.append(f"const uint8_t neutralSampleIndex = 0;")
    output_lines.append(f"const uint16_t colorClearMinThreshold = {thresholds['color_clear_min_threshold']};")
    output_lines.append(f"const uint16_t colorClearWhiteThreshold = {thresholds['color_clear_white_threshold']};")
    output_lines.append(f"const uint16_t colorWhiteChannelThreshold = {thresholds['color_white_channel_threshold']};")
    output_lines.append(f"const float whiteBalanceRedGain = {gains[0]:.3f}f;")
    output_lines.append(f"const float whiteBalanceGreenGain = {gains[1]:.3f}f;")
    output_lines.append(f"const float whiteBalanceBlueGain = {gains[2]:.3f}f;")
    output_lines.append(f"const uint16_t whiteBalancedClearThreshold = {thresholds['white_balanced_clear_threshold']};")
    output_lines.append(f"const uint8_t whiteBalancedMaxSpreadPct = {thresholds['white_balanced_max_spread_pct']};")
    output_lines.append(f"const uint16_t prototypeDistanceThreshold = {thresholds['prototype_distance_threshold']};")
    output_lines.append(f"const uint16_t colorMatchClearThreshold = {thresholds['color_match_clear_threshold']};")
    output_lines.append("")
    output_lines.append("const CalibrationSampleDefinition calibrationSamples[] = {")
    for label in profile_labels:
        source_label = enriched_rows[label][0]["capture_label"]
        prototype = prototypes[label]
        output_lines.append(f'  {{ "{source_label}", {{ {prototype[0]}, {prototype[1]}, {prototype[2]} }} }},')
    output_lines.append("};")
    output_lines.append("")
    output_lines.append(f"// Reference only: black max clear={thresholds['black_reference_max_clear']}, nothing max clear={thresholds['nothing_reference_max_clear']}")
    return "\n".join(output_lines) + "\n"


def build_cluster_calibration_block(rows_by_label):
    ensure_required_labels(rows_by_label)
    gains = compute_white_balance_gains(rows_by_label["white"])
    enriched_rows = enrich_rows(rows_by_label, gains)
    profile_labels = ordered_profile_labels(rows_by_label)
    thresholds, _ = derive_thresholds(enriched_rows, profile_labels)

    label_profiles = []
    for label in profile_labels:
        label_profiles.append(build_clusters_for_label(label, enriched_rows[label]))

    output_lines = []
    output_lines.append("// Generated by rgb-color-sensor/calibration_tool.py analyze-clusters")
    output_lines.append("// Paste this block into rgb-color-sensor-recognition-clusters.ino")
    output_lines.append(f"const uint16_t colorClearMinThreshold = {thresholds['color_clear_min_threshold']};")
    output_lines.append(f"const uint16_t colorMatchClearThreshold = {thresholds['color_match_clear_threshold']};")
    output_lines.append(f"const float whiteBalanceRedGain = {gains[0]:.3f}f;")
    output_lines.append(f"const float whiteBalanceGreenGain = {gains[1]:.3f}f;")
    output_lines.append(f"const float whiteBalanceBlueGain = {gains[2]:.3f}f;")
    output_lines.append("")
    output_lines.append("const char* colorLabels[] = {")
    for profile in label_profiles:
        output_lines.append(f'  "{profile["display_name"]}",')
    output_lines.append("};")
    output_lines.append("")
    output_lines.append("const ClusterDefinition clusterDefinitions[] = {")
    for label_index, profile in enumerate(label_profiles):
        output_lines.append(
            f"  // {profile['display_name']}: {profile['cluster_count']} cluster(s), mean_distance={profile['score']:.1f}"
        )
        for cluster in profile["clusters"]:
            center = cluster["center"]
            output_lines.append(
                f"  {{ {label_index}, {{ {center[0]}, {center[1]}, {center[2]} }}, {cluster['radius']} }},"
            )
    output_lines.append("};")
    output_lines.append("")
    output_lines.append("const uint8_t colorLabelCount = sizeof(colorLabels) / sizeof(colorLabels[0]);")
    output_lines.append("const uint8_t clusterDefinitionCount = sizeof(clusterDefinitions) / sizeof(clusterDefinitions[0]);")
    output_lines.append("")
    output_lines.append(
        f"// Reference only: black max clear={thresholds['black_reference_max_clear']}, nothing max clear={thresholds['nothing_reference_max_clear']}"
    )
    return "\n".join(output_lines) + "\n"


def build_train_cluster_calibration_block(rows_by_label):
    ensure_required_labels(rows_by_label)
    gains = compute_white_balance_gains(rows_by_label["white"])
    enriched_rows = enrich_rows(rows_by_label, gains)
    profile_labels = ordered_profile_labels(rows_by_label)
    thresholds, _ = derive_thresholds(enriched_rows, profile_labels)

    label_profiles = []
    for label in profile_labels:
        label_profiles.append(build_clusters_for_label(label, enriched_rows[label]))

    output_lines = []
    output_lines.append("// Generated by rgb-color-sensor/calibration_tool.py analyze-clusters --target train")
    output_lines.append(f"const uint16_t colorClearMinThreshold = {thresholds['color_clear_min_threshold']};")
    output_lines.append(f"const uint16_t colorMatchClearThreshold = {thresholds['color_match_clear_threshold']};")
    output_lines.append(f"const float whiteBalanceRedGain = {gains[0]:.3f}f;")
    output_lines.append(f"const float whiteBalanceGreenGain = {gains[1]:.3f}f;")
    output_lines.append(f"const float whiteBalanceBlueGain = {gains[2]:.3f}f;")
    output_lines.append("")
    output_lines.append("const MarkerClusterDefinition markerClusters[] = {")
    for profile in label_profiles:
        marker_class = TRAIN_MARKER_CLASS_BY_LABEL.get(profile["label"])
        if marker_class is None:
            raise SystemExit(f"Unsupported train marker label: {profile['display_name']}")

        output_lines.append(
            f"  // {profile['display_name']}: {profile['cluster_count']} cluster(s), mean_distance={profile['score']:.1f}"
        )
        for cluster in profile["clusters"]:
            center = cluster["center"]
            output_lines.append(
                f"  {{ {marker_class}, {{ {center[0]}, {center[1]}, {center[2]} }}, {cluster['radius']} }},"
            )
    output_lines.append("};")
    output_lines.append("")
    output_lines.append("const uint8_t markerClusterCount = sizeof(markerClusters) / sizeof(markerClusters[0]);")
    output_lines.append("")
    output_lines.append(
        f"// Reference only: black max clear={thresholds['black_reference_max_clear']}, nothing max clear={thresholds['nothing_reference_max_clear']}"
    )
    return "\n".join(output_lines) + "\n"


class CaptureWriter:
    def __init__(self, output_dir: Path):
        self.output_dir = output_dir
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.combined_path = self.output_dir / "all_captures.csv"
        self._ensure_header(self.combined_path)

    def _ensure_header(self, path: Path):
        if path.exists() and path.stat().st_size > 0:
            return
        with path.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.writer(handle)
            writer.writerow(CSV_HEADER)

    def write_sample(self, record):
        label = record["capture_label"]
        filename = sanitize_label(label) + ".csv"
        target_path = self.output_dir / filename
        self._ensure_header(target_path)

        row = [record[column] for column in CSV_HEADER]
        for path in (target_path, self.combined_path):
            with path.open("a", newline="", encoding="utf-8") as handle:
                writer = csv.writer(handle)
                writer.writerow(row)


def parse_capture_sample(row):
    if len(row) != 13:
        raise ValueError(f"Expected 13 CAPTURE_SAMPLE columns, got {len(row)}")

    return {
        "capture_label": row[2],
        "elapsed_ms": to_int(row[1]),
        "raw_r": to_int(row[3]),
        "raw_g": to_int(row[4]),
        "raw_b": to_int(row[5]),
        "raw_c": to_int(row[6]),
        "balanced_r": to_int(row[7]),
        "balanced_g": to_int(row[8]),
        "balanced_b": to_int(row[9]),
        "normalized_r": to_int(row[10]),
        "normalized_g": to_int(row[11]),
        "normalized_b": to_int(row[12]),
    }


def open_serial(port: str, baud: int, timeout: float):
    try:
        import serial
    except ImportError as exc:
        raise SystemExit("pyserial is required for capture mode. Install it with: pip install pyserial") from exc

    return serial.Serial(port=port, baudrate=baud, timeout=timeout)


def import_log_file(log_path: Path, output_dir: Path, echo: bool):
    writer = CaptureWriter(output_dir)

    with log_path.open("r", encoding="utf-8", errors="replace") as handle:
        for raw_line in handle:
            line = raw_line.strip()
            if not line:
                continue

            if echo:
                print(line)

            row = parse_csv_line(line)
            if not row:
                continue

            if row[0] == "CAPTURE_SAMPLE":
                record = parse_capture_sample(row)
                writer.write_sample(record)


def run_capture(args):
    writer = CaptureWriter(Path(args.out))
    active_labels = defaultdict(int)

    with open_serial(args.port, args.baud, args.timeout) as serial_port:
        if args.command:
            serial_port.write((args.command + "\n").encode("utf-8"))

        while True:
            raw_line = serial_port.readline()
            if not raw_line:
                continue

            line = raw_line.decode("utf-8", errors="replace").strip()
            if not line:
                continue

            if not args.quiet:
                print(line)

            row = parse_csv_line(line)
            if not row:
                continue

            record_type = row[0]
            if record_type == "CAPTURE_BEGIN" and len(row) == 3:
                active_labels[canonical_label(row[1])] += 1
                continue

            if record_type == "CAPTURE_SAMPLE":
                record = parse_capture_sample(row)
                writer.write_sample(record)
                continue

            if record_type == "CAPTURE_END" and len(row) == 4:
                label_key = canonical_label(row[1])
                if active_labels[label_key] > 0:
                    active_labels[label_key] -= 1

                if args.command and not any(active_labels.values()):
                    break


def run_import_log(args):
    import_log_file(Path(args.log), Path(args.out), not args.quiet)


def run_analyze(args):
    input_dir = Path(args.input)
    rows_by_label = get_label_rows(input_dir)
    calibration_block = build_calibration_block(rows_by_label)

    if args.output:
        output_path = Path(args.output)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_text(calibration_block, encoding="utf-8")

    sys.stdout.write(calibration_block)


def run_analyze_clusters(args):
    input_dir = Path(args.input)
    rows_by_label = get_label_rows(input_dir)
    if args.target == "train":
        calibration_block = build_train_cluster_calibration_block(rows_by_label)
    else:
        calibration_block = build_cluster_calibration_block(rows_by_label)

    if args.output:
        output_path = Path(args.output)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_text(calibration_block, encoding="utf-8")

    sys.stdout.write(calibration_block)


def build_parser():
    parser = argparse.ArgumentParser(description="Capture and analyze RGB color sensor calibration data.")
    subparsers = parser.add_subparsers(dest="command_name", required=True)

    capture_parser = subparsers.add_parser("capture", help="Listen to Arduino Serial output and save timed-capture CSV files.")
    capture_parser.add_argument("--port", required=True, help="Serial port, for example COM3")
    capture_parser.add_argument("--baud", type=int, default=9600, help="Serial baud rate")
    capture_parser.add_argument("--timeout", type=float, default=1.0, help="Serial read timeout in seconds")
    capture_parser.add_argument("--out", required=True, help="Directory for per-color and combined CSV output")
    capture_parser.add_argument("--command", help='Optional command to send immediately, for example cal "Maroon Red" 15')
    capture_parser.add_argument("--quiet", action="store_true", help="Do not echo Serial lines to stdout")
    capture_parser.set_defaults(func=run_capture)

    import_parser = subparsers.add_parser("import-log", help="Import saved Serial Monitor text output and extract timed-capture CSV rows.")
    import_parser.add_argument("--log", required=True, help="Path to a saved Serial Monitor text file")
    import_parser.add_argument("--out", required=True, help="Directory for per-color and combined CSV output")
    import_parser.add_argument("--quiet", action="store_true", help="Do not echo imported log lines to stdout")
    import_parser.set_defaults(func=run_import_log)

    analyze_parser = subparsers.add_parser("analyze", help="Analyze captured CSV files and emit a sketch-ready calibration block.")
    analyze_parser.add_argument("--input", required=True, help="Directory containing per-color CSV files")
    analyze_parser.add_argument("--output", help="Optional file path for the generated calibration block")
    analyze_parser.set_defaults(func=run_analyze)

    analyze_clusters_parser = subparsers.add_parser("analyze-clusters", help="Analyze captured CSV files and emit a multi-cluster recognition block.")
    analyze_clusters_parser.add_argument("--input", required=True, help="Directory containing per-color CSV files")
    analyze_clusters_parser.add_argument("--output", help="Optional file path for the generated cluster block")
    analyze_clusters_parser.add_argument("--target", choices=("generic", "train"), default="generic", help="Output target; generic is for the standalone recognition sketch")
    analyze_clusters_parser.set_defaults(func=run_analyze_clusters)

    return parser


def main():
    parser = build_parser()
    args = parser.parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
