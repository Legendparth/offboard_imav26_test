"""
thermal_reader.py

Standalone, robust MLX90640 acquisition module.
- Handles transient I2C errors (common on Jetson + vibration/movement) via retry+backoff
- Timestamps every successful frame
- Tracks running max temperature + pixel location for the session
- Exposes a clean class interface so thermaldetection.py can just import and call
  get_latest_frame() / get_running_max() without worrying about I2C plumbing

Run directly for a standalone test:
    python3 thermal_reader.py
"""

import time
import threading
import logging
from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np
import board
import busio
import adafruit_mlx90640

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
)
log = logging.getLogger("thermal_reader")


@dataclass
class ThermalFrame:
    timestamp: float          # time.time() at successful read
    grid: np.ndarray          # 24x32 float array, degrees C
    max_temp: float
    max_pixel: Tuple[int, int]  # (row, col) within the 24x32 grid


class MLX90640Reader:
    """
    Wraps the MLX90640 with retry logic and a background polling thread.
    Call start() to begin continuous acquisition, stop() to cleanly shut down.
    """

    def __init__(
        self,
        refresh_rate=adafruit_mlx90640.RefreshRate.REFRESH_2_HZ,
        max_consecutive_errors: int = 10,
        retry_backoff_s: float = 0.05,
    ):
        self._i2c = busio.I2C(board.SCL, board.SDA, frequency=400000)
        self._mlx = adafruit_mlx90640.MLX90640(self._i2c)
        self._mlx.refresh_rate = refresh_rate

        self._frame_buf = np.zeros((24 * 32,))
        self._lock = threading.Lock()
        self._latest_frame: Optional[ThermalFrame] = None

        self._running_max_temp: float = float("-inf")
        self._running_max_pixel: Optional[Tuple[int, int]] = None
        self._running_max_timestamp: Optional[float] = None

        self._max_consecutive_errors = max_consecutive_errors
        self._retry_backoff_s = retry_backoff_s

        self._stop_event = threading.Event()
        self._thread: Optional[threading.Thread] = None

    # ---------- public API ----------

    def start(self):
        """Start background acquisition thread."""
        if self._thread is not None:
            log.warning("Reader already started.")
            return
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._poll_loop, daemon=True)
        self._thread.start()
        log.info("MLX90640 acquisition thread started.")

    def stop(self):
        """Stop background acquisition thread cleanly."""
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        log.info("MLX90640 acquisition thread stopped.")

    def get_latest_frame(self) -> Optional[ThermalFrame]:
        """Thread-safe getter for the most recent successfully read frame."""
        with self._lock:
            return self._latest_frame

    def get_running_max(self) -> Tuple[float, Optional[Tuple[int, int]], Optional[float]]:
        """Returns (max_temp, max_pixel, timestamp_of_max) since start() was called
        or since reset_running_max() was last invoked."""
        with self._lock:
            return self._running_max_temp, self._running_max_pixel, self._running_max_timestamp

    def reset_running_max(self):
        """Call this at the start of each lawnmower pass / mission if you want
        a fresh max rather than carrying over from a previous run."""
        with self._lock:
            self._running_max_temp = float("-inf")
            self._running_max_pixel = None
            self._running_max_timestamp = None
        log.info("Running max reset.")

    # ---------- internals ----------

    def _read_frame_with_retry(self) -> Optional[np.ndarray]:
        """
        Attempts a single frame read, retrying on transient I2C errors
        (ValueError from bad CRC, OSError 121 Remote I/O error from
        connection glitches -- common under vibration/movement).
        Returns the 24x32 grid, or None if all retries in this call failed.
        """
        consecutive_errors = 0
        while consecutive_errors < self._max_consecutive_errors:
            try:
                self._mlx.getFrame(self._frame_buf)
                return np.reshape(self._frame_buf, (24, 32)).copy()
            except (ValueError, OSError) as e:
                consecutive_errors += 1
                log.debug(f"Frame read error ({consecutive_errors}/"
                          f"{self._max_consecutive_errors}): {e}")
                time.sleep(self._retry_backoff_s)
        log.warning(
            f"Exceeded {self._max_consecutive_errors} consecutive I2C errors. "
            "Check wiring/connector -- this indicates a physical connection "
            "issue, not a transient glitch."
        )
        return None

    def _poll_loop(self):
        error_streak = 0
        while not self._stop_event.is_set():
            grid = self._read_frame_with_retry()

            if grid is None:
                error_streak += 1
                # Back off harder if we're in a sustained bad patch,
                # so we don't spam the bus / log.
                time.sleep(min(1.0, 0.1 * error_streak))
                continue

            error_streak = 0
            ts = time.time()
            max_temp = float(np.max(grid))
            max_pixel = tuple(int(v) for v in np.unravel_index(np.argmax(grid), grid.shape))

            frame = ThermalFrame(
                timestamp=ts,
                grid=grid,
                max_temp=max_temp,
                max_pixel=max_pixel,
            )

            with self._lock:
                self._latest_frame = frame
                if max_temp > self._running_max_temp:
                    self._running_max_temp = max_temp
                    self._running_max_pixel = max_pixel
                    self._running_max_timestamp = ts

    def close(self):
        """Release I2C bus. Call this before another process needs the bus
        (e.g. if goto_and_drop.py or another script needs I2C access)."""
        self.stop()
        try:
            self._i2c.deinit()
        except Exception as e:
            log.warning(f"Error during I2C deinit: {e}")


# ---------------- standalone test ----------------
# ---------------- standalone test ----------------
if __name__ == "__main__":
    import cv2
    import numpy as np
    
    reader = MLX90640Reader()
    reader.start()
    
    try:
        print("Starting fused thermal feed. Press 'q' in the window to quit.")
        while True:
            
            frame = reader.get_latest_frame()
            if frame is not None:

                hot_y_thermal, hot_x_thermal = frame.max_pixel
                print(f"max Temp: {frame.max_temp}")

                grid = frame.grid
                grid_min, grid_max = np.min(grid), np.max(grid)
                if grid_max == grid_min: grid_max += 0.1 
                
                norm_grid = np.uint8((grid - grid_min) * 255 / (grid_max - grid_min))
                heatmap = cv2.applyColorMap(norm_grid, cv2.COLORMAP_INFERNO)
                heatmap_resized = cv2.resize(heatmap, (640, 480), interpolation=cv2.INTER_CUBIC)

                cv2.imshow('MLX90640 Thermal Feed', heatmap_resized)
                
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
                    
            time.sleep(0.05)
            
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        cv2.destroyAllWindows()
        reader.close()