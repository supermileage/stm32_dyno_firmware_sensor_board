"""
Sensor board UART data parser.

Firmware config assumed:
    #define PRINT_DATA_USING_SPRINTF 1   -> data is ASCII space-separated
    #define BYPASS_HANDSHAKES 1          -> no command handshake, sensors run freely

Packet format (5 space-separated uint32 values, no newline):
    <optical_count_posedges> <optical_timer_counter_value> <force_sensor_raw_value> <task_id> <error_id>

USAGE
-----
Basic:
    python parse_sensor_data.py COM3

Custom baud rate:
    python parse_sensor_data.py COM3 --baud 115200

Print errors only (console), but still log everything to file:
    python parse_sensor_data.py COM3 --errors-only

Custom log directory:
    python parse_sensor_data.py COM3 --log-dir D:\\logs

All options:
    python parse_sensor_data.py COM3 --baud 115200 --errors-only --log-dir logs

Output columns (console & log):
    Force (N)  |  Angular velocity (rad/s)  |  RPM  |  Status / error
"""

import argparse
import logging
import math
import os
import serial
import sys
from datetime import datetime

# ---------------------------------------------------------------------------
# Physics constants  (mirror firmware / hardware values)
# ---------------------------------------------------------------------------

ADS1115_MV_PER_COUNT       = 0.1875   # ADS1115_GetMvPerCount(ADS1115_PGA_6P144)
ADS1115_FORCESENSOR_VOLTAGE = 5.0     # Supply voltage to force sensor (V) — adjust if different
MAX_FORCE_LBF              = 100.0   # Maximum force in lbf — adjust to match sensor spec
LBF_TO_NEWTONS             = 4.44822 # 1 lbf = 4.44822 N
NUM_APERTURES              = 64      # Must match #define NUM_APERTURES in firmware


def calculate_force_newtons(raw_value: int) -> float:
    """
    Mirror of SensorBoardController::GetForce().
    mv_per_count * raw / 1000 -> volts / supply_voltage -> ratio * max_lbf * lbf_to_N
    """
    return (ADS1115_MV_PER_COUNT * raw_value / 1000.0
            / ADS1115_FORCESENSOR_VOLTAGE
            * MAX_FORCE_LBF
            * LBF_TO_NEWTONS)


def calculate_angular_velocity(num_posedges: int, timer_counter_value: int) -> float:
    """
    Mirror of SensorBoardController::GetAngularVelocity().
    Returns radians per second.
    timer_counter_value is in microseconds.
    """
    if timer_counter_value == 0:
        return 0.0
    revolutions = num_posedges / NUM_APERTURES
    elapsed_s   = timer_counter_value / 1_000_000.0
    return (revolutions / elapsed_s) * 2.0 * math.pi


def calculate_rpm(num_posedges: int, timer_counter_value: int) -> float:
    omega = calculate_angular_velocity(num_posedges, timer_counter_value)
    return omega * 60.0 / (2.0 * math.pi)


# ---------------------------------------------------------------------------
# Mirror of messages_public.h enums
# ---------------------------------------------------------------------------

TASK_IDS = {
    0:   "INVALID_TASK_ID",
    100: "OPTICAL_SENSOR",
    101: "FORCE_SENSOR_ADC",
    102: "FORCE_SENSOR_ADS1115",
}

# Per-task error lookup: task_id -> {error_id -> name}
ERROR_IDS = {
    # INVALID / USB errors  (task_id = 0)
    0: {
        0: "ERROR_USB_RX_INVALID_TASK_ID",
        1: "ERROR_USB_RX_CB_START_FAILURE",
    },
    # Optical sensor errors  (task_id = 100)
    100: {
        0: "ERROR_OPTICAL_SENSOR_TIMER_START_FAIL",
        1: "ERROR_OPTICAL_SENSOR_TIMER_POSEDGES_COUNTER_START_FAIL",
        2: "ERROR_OPTICAL_SENSOR_TIMER_STOP_FAIL",
        3: "ERROR_OPTICAL_SENSOR_TIMER_POSEDGES_COUNTER_STOP_FAIL",
    },
    # Force sensor ADC errors  (task_id = 101)
    101: {
        0: "ERROR_FORCE_SENSOR_ADC_START_FAILURE",
    },
    # Force sensor ADS1115 errors / warnings  (task_id = 102)
    102: {
        0:     "ERROR_FORCE_SENSOR_ADS1115_NULL_SETTINGS_PTR",
        1:     "ERROR_FORCE_SENSOR_ADS1115_CONSTR_FAIL",
        2:     "ERROR_FORCE_SENSOR_ADS1115_SET_MULTIPLEXER_FAIL",
        3:     "ERROR_FORCE_SENSOR_ADS1115_SET_COMPARATOR_FAIL",
        4:     "ERROR_FORCE_SENSOR_ADS1115_SET_COMPARATOR_POLARITY_FAIL",
        5:     "ERROR_FORCE_SENSOR_ADS1115_SET_COMPARATOR_LATCH_FAIL",
        6:     "ERROR_FORCE_SENSOR_ADS1115_SET_COMPARATOR_QUEUE_MODE_FAIL",
        7:     "ERROR_FORCE_SENSOR_ADS1115_SET_MODE_FAIL",
        8:     "ERROR_FORCE_SENSOR_ADS1115_SET_RATE_FAIL",
        9:     "ERROR_FORCE_SENSOR_ADS1115_SET_GAIN_FAIL",
        10:    "ERROR_FORCE_SENSOR_ADS1115_SET_CONVERSION_READY_PIN_MODE_FAIL",
        10000: "WARNING_FORCE_SENSOR_ADS1115_TRIGGER_CONVERSION_FAILURE",
        10001: "WARNING_FORCE_SENSOR_ADS1115_GET_CONVERSION_FAILURE",
    },
}

NO_ERROR_TASK_ID  = 0   # task_id == 0 AND error_id == 0 means no error
NO_ERROR_ERROR_ID = 0

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def resolve_task(task_id: int) -> str:
    return TASK_IDS.get(task_id, f"UNKNOWN_TASK({task_id})")


def resolve_error(task_id: int, error_id: int) -> str:
    task_errors = ERROR_IDS.get(task_id, {})
    return task_errors.get(error_id, f"UNKNOWN_ERROR({error_id})")


def is_error(task_id: int, error_id: int) -> bool:
    """No error is represented by task_id=0, error_id=0 (zero-initialised struct)."""
    return not (task_id == NO_ERROR_TASK_ID and error_id == NO_ERROR_ERROR_ID)


def parse_packet(raw: str):
    """
    Parse a single whitespace-separated packet.
    Returns a dict or raises ValueError on bad input.
    """
    parts = raw.split()
    if len(parts) != 5:
        raise ValueError(f"Expected 5 fields, got {len(parts)}: {raw!r}")

    optical_count    = int(parts[0])
    optical_timer    = int(parts[1])
    force_raw        = int(parts[2])
    task_id          = int(parts[3])
    error_id         = int(parts[4])

    return {
        "optical_count_posedges":      optical_count,
        "optical_timer_counter_value": optical_timer,
        "force_sensor_raw_value":      force_raw,
        "task_id":                     task_id,
        "error_id":                    error_id,
    }


def setup_logger(log_dir: str) -> logging.Logger:
    os.makedirs(log_dir, exist_ok=True)
    session_ts = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    log_path = os.path.join(log_dir, f"sensor_{session_ts}.log")

    logger = logging.getLogger("sensor")
    logger.setLevel(logging.DEBUG)

    # File handler — always logs everything
    fh = logging.FileHandler(log_path, encoding="utf-8")
    fh.setLevel(logging.DEBUG)
    fh.setFormatter(logging.Formatter("%(asctime)s  %(levelname)-8s  %(message)s",
                                      datefmt="%H:%M:%S.%f"))
    logger.addHandler(fh)

    print(f"Logging to: {log_path}")
    return logger


def format_packet_lines(data: dict, timestamp: str) -> list[str]:
    task_id   = data["task_id"]
    error_id  = data["error_id"]
    has_err   = is_error(task_id, error_id)

    force_n   = calculate_force_newtons(data["force_sensor_raw_value"])
    omega     = calculate_angular_velocity(data["optical_count_posedges"],
                                          data["optical_timer_counter_value"])
    rpm       = calculate_rpm(data["optical_count_posedges"],
                              data["optical_timer_counter_value"])

    lines = [
        f"[{timestamp}]",
        f"  Force                 : {force_n:.4f} N",
        f"  Angular velocity      : {omega:.4f} rad/s",
        f"  RPM                   : {rpm:.2f}",
    ]
    if has_err:
        task_name  = resolve_task(task_id)
        error_name = resolve_error(task_id, error_id)
        lines.append(f"  *** ERROR  task={task_name}  error={error_name} ***")
    else:
        lines.append(f"  Status                : OK")
    return lines


def print_and_log_packet(data: dict, timestamp: str, logger: logging.Logger) -> None:
    lines = format_packet_lines(data, timestamp)
    has_err = is_error(data["task_id"], data["error_id"])

    for line in lines:
        print(line)
    print()

    # Build a single log record per packet
    log_msg = "  |  ".join(lines)
    if has_err:
        logger.error(log_msg)
    else:
        logger.info(log_msg)


# ---------------------------------------------------------------------------
# UART packet reader
# ---------------------------------------------------------------------------

# The firmware transmits strlen(buffer) bytes with no newline.
# Each packet is exactly the serialised string; we accumulate bytes until we
# have received a full packet's worth of characters.
#
# Strategy: read until we have collected 5 whitespace-separated tokens.
# Because the firmware sends back-to-back packets we use a partial-line
# buffer and split on runs of whitespace, collecting tokens five at a time.

def stream_packets(port: serial.Serial):
    """Generator that yields raw 5-token strings from the serial stream."""
    leftover = ""
    while True:
        chunk = port.read(64).decode("ascii", errors="replace")
        if not chunk:
            continue
        leftover += chunk
        tokens = leftover.split()
        while len(tokens) >= 5:
            yield " ".join(tokens[:5])
            tokens = tokens[5:]
        # Keep any partial token for the next iteration
        leftover = tokens[0] if tokens else ""


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="Parse STM32 dyno sensor board UART output."
    )
    parser.add_argument(
        "port",
        help="COM port to open (e.g. COM3 or /dev/ttyUSB0)",
    )
    parser.add_argument(
        "--baud", "-b",
        type=int,
        default=115200,
        help="Baud rate (default: 115200)",
    )
    parser.add_argument(
        "--errors-only", "-e",
        action="store_true",
        help="Only print packets that contain an error",
    )
    parser.add_argument(
        "--log-dir", "-l",
        default="logs",
        help="Directory to write log files into (default: logs/)",
    )
    args = parser.parse_args()

    print(f"Opening {args.port} at {args.baud} baud …")
    logger = setup_logger(args.log_dir)

    try:
        with serial.Serial(args.port, args.baud, timeout=1) as port:
            print(f"Connected. Waiting for data (Ctrl+C to stop).\n")
            for raw in stream_packets(port):
                try:
                    data = parse_packet(raw)
                except ValueError as exc:
                    msg = f"[PARSE ERROR] {exc}"
                    print(msg, file=sys.stderr)
                    logger.warning(msg)
                    continue

                ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]

                if args.errors_only and not is_error(data["task_id"], data["error_id"]):
                    # Still log everything to file even when console is filtered
                    logger.info("  |  ".join(format_packet_lines(data, ts)))
                    continue

                print_and_log_packet(data, ts, logger)

    except serial.SerialException as exc:
        print(f"Serial error: {exc}", file=sys.stderr)
        sys.exit(1)
    except KeyboardInterrupt:
        print("\nStopped.")


if __name__ == "__main__":
    main()
