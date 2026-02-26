"""
Parallel-Mode Fingerprint Analysis
===================================
Loads a CSV of BME688 measurements collected in parallel scan mode
(multiple sensors operating simultaneously).

Each *fingerprint* is one complete heating-profile cycle; each *position*
is a single heater step within that cycle.

Expected CSV columns:
    sensor_index, fingerprint_index, position,
    plate_temperature, heater_duration,
    temperature, pressure, humidity, gas_resistance, label

Usage:
    python analyse_fingerprints.py path/to/data.csv
"""

import sys
from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker

# ---------------------------------------------------------------------------
# Config
# ---------------------------------------------------------------------------

FEATURES = ["temperature", "pressure", "humidity", "gas_resistance"]
FEATURE_LABELS = ["Temperature (°C)", "Pressure (hPa)", "Humidity (%)", "Gas Resistance (Ω)"]

plt.rcParams.update({
    "figure.dpi": 120,
    "axes.spines.top": False,
    "axes.spines.right": False,
})

_large_fmt = ticker.FuncFormatter(
    lambda v, _: f"{v:,.0f}" if abs(v) >= 1000 else f"{v:.2f}"
)


# ---------------------------------------------------------------------------
# Load & split
# ---------------------------------------------------------------------------

def load(csv_path: str | Path) -> pd.DataFrame:
    """Load the CSV and return the raw DataFrame."""
    df = pd.read_csv(csv_path)
    print(f"Loaded {len(df):,} rows — columns: {list(df.columns)}")
    return df


def split_sensors(df: pd.DataFrame) -> dict[int, pd.DataFrame]:
    """Split into per-sensor DataFrames sorted into correct temporal order.

    Sort key: (fingerprint_index, position) — fingerprint_index is the cycle
    counter; position is the heater step within each cycle.
    """
    sensors: dict[int, pd.DataFrame] = {}
    for sid, group in df.groupby("sensor_index"):
        sensors[sid] = (
            group
            .sort_values(["fingerprint_index", "position"])
            .reset_index(drop=True)
        )
    for sid, sdf in sensors.items():
        n_fp  = sdf["fingerprint_index"].nunique()
        n_pos = sdf["position"].nunique()
        print(f"  Sensor {sid}: {len(sdf):>5} rows | {n_fp} fingerprints × {n_pos} positions")
    return sensors


# ---------------------------------------------------------------------------
# Plot 1 — time series per sensor
# ---------------------------------------------------------------------------

def plot_time_series(df_sensor: pd.DataFrame, title: str | None = None) -> None:
    """Scatter each measurement feature against time for a single sensor.

    One subplot per feature. The x-axis is the sequential measurement index
    (temporal order after sorting). Faint vertical lines mark fingerprint
    boundaries.

    Parameters
    ----------
    df_sensor:
        Pre-sorted DataFrame for one sensor (output of split_sensors).
    title:
        Figure title. Defaults to 'Sensor <id> — time series'.
    """
    df = df_sensor.reset_index(drop=True)  # row number = time index

    # Rows where fingerprint_index changes → fingerprint boundaries
    fp_starts = df.index[df["fingerprint_index"].diff().ne(0)].tolist()

    sid = df["sensor_index"].iloc[0]
    fig, axes = plt.subplots(
        len(FEATURES), 1,
        figsize=(14, 3 * len(FEATURES)),
        sharex=True,
    )
    fig.suptitle(title or f"Sensor {sid} — time series", fontsize=13, y=1.01)

    for ax, feat, label in zip(axes, FEATURES, FEATURE_LABELS):
        ax.scatter(df.index, df[feat], s=5, alpha=0.6, linewidths=0)
        for x in fp_starts:
            ax.axvline(x=x, color="grey", alpha=0.15, linewidth=0.8)
        ax.set_ylabel(label, fontsize=9)
        ax.yaxis.set_major_formatter(_large_fmt)
        ax.grid(axis="y", alpha=0.2)

    axes[-1].set_xlabel("Measurement index (temporal order)", fontsize=9)
    plt.tight_layout()
    plt.show()


# ---------------------------------------------------------------------------
# Plot 2 — features vs plate temperature
# ---------------------------------------------------------------------------

def plot_vs_plate_temp(df: pd.DataFrame, title: str | None = None) -> None:
    """Scatter each measurement feature against plate temperature.

    Sensors are colour-coded when multiple are present in `df`.

    Parameters
    ----------
    df:
        One or more sensors' data (sorting not required).
    title:
        Figure title.
    """
    ncols = 2
    nrows = (len(FEATURES) + 1) // ncols

    fig, axes = plt.subplots(nrows, ncols, figsize=(12, 4 * nrows))
    axes = np.array(axes).flatten()
    fig.suptitle(title or "Features vs plate temperature", fontsize=13)

    unique_sensors = sorted(df["sensor_index"].unique())
    palette = plt.cm.tab10(np.linspace(0, 0.9, max(len(unique_sensors), 1)))
    colour_map = dict(zip(unique_sensors, palette))

    for ax, feat, label in zip(axes, FEATURES, FEATURE_LABELS):
        for sid in unique_sensors:
            sub = df[df["sensor_index"] == sid]
            ax.scatter(
                sub["plate_temperature"], sub[feat],
                s=5, alpha=0.5, linewidths=0,
                color=colour_map[sid], label=f"Sensor {sid}",
            )
        ax.set_xlabel("Plate temperature (°C)", fontsize=9)
        ax.set_ylabel(label, fontsize=9)
        ax.yaxis.set_major_formatter(_large_fmt)
        ax.grid(alpha=0.2)

    if len(unique_sensors) > 1:
        handles, labels_ = axes[0].get_legend_handles_labels()
        seen: set = set()
        dedup = [(h, l) for h, l in zip(handles, labels_)
                 if not (l in seen or seen.add(l))]
        fig.legend(*zip(*dedup), loc="upper right", fontsize=8, markerscale=2)

    for ax in axes[len(FEATURES):]:
        ax.set_visible(False)

    plt.tight_layout()
    plt.show()


# ---------------------------------------------------------------------------
# Plot 3 — average fingerprint
# ---------------------------------------------------------------------------

def plot_average_fingerprint(df: pd.DataFrame, title: str | None = None) -> None:
    """Plot the mean (±1 std) feature value at each heater position across all fingerprints.

    For each position, the feature is averaged over every fingerprint cycle,
    giving the characteristic response shape of the heating profile. The
    shaded band shows ±1 std.

    Multiple sensors in `df` are overlaid as separate coloured lines.

    Parameters
    ----------
    df:
        One or more sensors' data.
    title:
        Figure title.
    """
    ncols = 2
    nrows = (len(FEATURES) + 1) // ncols

    fig, axes = plt.subplots(nrows, ncols, figsize=(12, 4 * nrows))
    axes = np.array(axes).flatten()
    fig.suptitle(title or "Average fingerprint", fontsize=13)

    unique_sensors = sorted(df["sensor_index"].unique())
    palette = plt.cm.tab10(np.linspace(0, 0.9, max(len(unique_sensors), 1)))
    colour_map = dict(zip(unique_sensors, palette))

    for ax, feat, label in zip(axes, FEATURES, FEATURE_LABELS):
        for sid in unique_sensors:
            sub     = df[df["sensor_index"] == sid]
            grouped = sub.groupby("position")[feat]
            avg     = grouped.mean()
            std     = grouped.std()
            pos     = avg.index
            c       = colour_map[sid]

            ax.plot(pos, avg, marker="o", markersize=4, color=c, label=f"Sensor {sid}")
            ax.fill_between(pos, avg - std, avg + std, alpha=0.15, color=c)

        ax.set_xlabel("Position (heater step)", fontsize=9)
        ax.set_ylabel(label, fontsize=9)
        ax.xaxis.set_major_locator(ticker.MaxNLocator(integer=True))
        ax.yaxis.set_major_formatter(_large_fmt)
        ax.grid(alpha=0.2)

    if len(unique_sensors) > 1:
        handles, labels_ = axes[0].get_legend_handles_labels()
        seen: set = set()
        dedup = [(h, l) for h, l in zip(handles, labels_)
                 if not (l in seen or seen.add(l))]
        fig.legend(*zip(*dedup), loc="upper right", fontsize=8)

    for ax in axes[len(FEATURES):]:
        ax.set_visible(False)

    plt.tight_layout()
    plt.show()


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    csv_path = sys.argv[1] if len(sys.argv) > 1 else "../data/custom/open_room.csv"

    df_raw  = load(csv_path)
    print()
    print(df_raw.describe().to_string())
    print()

    sensors = split_sensors(df_raw)
    print()

    # Time series — one plot per sensor
    for sid in sorted(sensors):
        plot_time_series(sensors[sid])

    # Features vs plate temperature
    first_sid = sorted(sensors)[0]
    plot_vs_plate_temp(sensors[first_sid],
                       title=f"Sensor {first_sid} — features vs plate temperature")
    plot_vs_plate_temp(df_raw, title="All sensors — features vs plate temperature")

    # Average fingerprint
    plot_average_fingerprint(sensors[first_sid],
                             title=f"Sensor {first_sid} — average fingerprint")
    plot_average_fingerprint(df_raw, title="All sensors — average fingerprint")
