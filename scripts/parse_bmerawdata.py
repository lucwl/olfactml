"""Parse .bmerawdata JSON files into flat CSV files."""

import json
import csv
import sys
from pathlib import Path


def parse_bmerawdata(input_path: str, output_path: str | None = None) -> str:
    input_path = Path(input_path)
    if output_path is None:
        output_path = input_path.with_suffix(".csv")
    else:
        output_path = Path(output_path)

    with open(input_path, "r", encoding="utf-8") as f:
        data = json.load(f)

    # Extract timestamp from the filename (everything before "_Board")
    # e.g. "2024_07_30_13_05_Board_..." -> "2024_07_30_13_05"
    stem = input_path.stem
    board_idx = stem.find("_Board")
    if board_idx != -1:
        file_timestamp = stem[:board_idx]
    else:
        # Fallback: use dateCreated_ISO from the header
        file_timestamp = data["rawDataHeader"].get("dateCreated_ISO", stem)

    columns = data["rawDataBody"]["dataColumns"]
    col_keys = [col["key"] for col in columns]
    data_block = data["rawDataBody"]["dataBlock"]

    header = ["id_timestamp", "id_index"] + col_keys

    with open(output_path, "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(header)
        for idx, row in enumerate(data_block):
            writer.writerow([file_timestamp, idx] + row)

    print(f"Wrote {len(data_block)} rows to {output_path}")
    return str(output_path)


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python parse_bmerawdata.py <file.bmerawdata> [output.csv]")
        sys.exit(1)

    in_file = sys.argv[1]
    out_file = sys.argv[2] if len(sys.argv) > 2 else None
    parse_bmerawdata(in_file, out_file)
