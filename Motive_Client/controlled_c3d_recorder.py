import argparse
import datetime
import socket
import threading
from pathlib import Path

import ezc3d
import numpy as np


class ControlledC3DRecorder:
    def __init__(
        self,
        mocap_port: int,
        control_port: int,
        output_directory: Path,
        frame_rate: float,
        input_units: str,
    ) -> None:
        self.mocap_port = mocap_port
        self.control_port = control_port
        self.output_directory = output_directory
        self.frame_rate = frame_rate
        self.input_units = input_units

        self.recording = False
        self.frames: list[dict[int, np.ndarray]] = []
        self.frame_numbers: list[int] = []

        self.lock = threading.Lock()
        self.running = True

    @staticmethod
    def parse_mocap_packet(
        message: str,
    ) -> tuple[int, dict[int, np.ndarray]]:
        fields = message.strip().split(";")

        if len(fields) < 2:
            raise ValueError("Packet has fewer than two fields")

        frame_number = int(fields[0])
        point_count = int(fields[1])

        if len(fields) < point_count + 2:
            raise ValueError(
                "Packet does not contain all declared points"
            )

        points: dict[int, np.ndarray] = {}

        for index in range(point_count):
            values = fields[index + 2].split(",")

            if len(values) == 4:
                marker_id = int(values[0])
                x, y, z = map(float, values[1:])
            elif len(values) == 3:
                marker_id = index
                x, y, z = map(float, values)
            else:
                raise ValueError(
                    f"Invalid point field: {fields[index + 2]}"
                )

            points[marker_id] = np.array(
                [x, y, z],
                dtype=np.float64,
            )

        return frame_number, points

    def control_loop(self) -> None:
        sock = socket.socket(
            socket.AF_INET,
            socket.SOCK_DGRAM,
        )

        sock.bind(
            ("0.0.0.0", self.control_port)
        )

        print(
            f"Listening for START/STOP on UDP "
            f"{self.control_port}"
        )

        while self.running:
            try:
                payload, address = sock.recvfrom(1024)
                command = (
                    payload.decode("utf-8")
                    .strip()
                    .upper()
                )

                if command == "START":
                    self.start_recording()

                elif command == "STOP":
                    self.stop_recording()

                else:
                    print(
                        f"Unknown control command "
                        f"from {address}: {command}"
                    )

            except OSError:
                break
            except Exception as exception:
                print(
                    f"Control error: {exception}"
                )

        sock.close()

    def start_recording(self) -> None:
        with self.lock:
            if self.recording:
                print("Already recording")
                return

            self.frames.clear()
            self.frame_numbers.clear()
            self.recording = True

        print("Recording started")

    def stop_recording(self) -> None:
        with self.lock:
            if not self.recording:
                print("Not currently recording")
                return

            self.recording = False

            frames_copy = list(self.frames)
            frame_numbers_copy = list(
                self.frame_numbers
            )

        if not frames_copy:
            print(
                "Recording stopped, but no frames "
                "were captured"
            )
            return

        timestamp = datetime.datetime.now().strftime(
            "%Y%m%d_%H%M%S"
        )

        output_path = (
            self.output_directory /
            f"mocap_{timestamp}.c3d"
        )

        self.write_c3d(
            output_path,
            frames_copy,
            frame_numbers_copy,
        )

    def mocap_loop(self) -> None:
        sock = socket.socket(
            socket.AF_INET,
            socket.SOCK_DGRAM,
        )

        sock.bind(
            ("0.0.0.0", self.mocap_port)
        )

        print(
            f"Listening for mocap frames on UDP "
            f"{self.mocap_port}"
        )

        last_frame_number = None

        while self.running:
            try:
                payload, _ = sock.recvfrom(65535)

                try:
                    frame_number, points = (
                        self.parse_mocap_packet(
                            payload.decode("utf-8")
                        )
                    )
                except Exception as exception:
                    print(
                        f"Dropped packet: {exception}"
                    )
                    continue

                if frame_number == last_frame_number:
                    continue

                last_frame_number = frame_number

                with self.lock:
                    if self.recording:
                        self.frames.append(points)
                        self.frame_numbers.append(
                            frame_number
                        )

                        frame_count = len(self.frames)

                        if frame_count % 100 == 0:
                            print(
                                f"Recorded {frame_count} frames"
                            )

            except OSError:
                break
            except Exception as exception:
                print(
                    f"Mocap receive error: {exception}"
                )

        sock.close()

    def write_c3d(
        self,
        output_path: Path,
        frames: list[dict[int, np.ndarray]],
        frame_numbers: list[int],
    ) -> None:
        marker_ids = sorted(
            {
                marker_id
                for frame in frames
                for marker_id in frame
            }
        )

        marker_to_index = {
            marker_id: index
            for index, marker_id
            in enumerate(marker_ids)
        }

        labels = [
            f"M{marker_id:04d}"
            for marker_id in marker_ids
        ]

        point_count = len(marker_ids)
        frame_count = len(frames)

        points = np.zeros(
            (4, point_count, frame_count),
            dtype=np.float64,
        )

        # Negative residual means missing/invalid.
        points[3, :, :] = -1.0

        scale_to_mm = (
            1000.0
            if self.input_units == "m"
            else 1.0
        )

        for frame_index, frame in enumerate(frames):
            for marker_id, position in frame.items():
                marker_index = (
                    marker_to_index[marker_id]
                )

                points[
                    0:3,
                    marker_index,
                    frame_index,
                ] = position * scale_to_mm

                points[
                    3,
                    marker_index,
                    frame_index,
                ] = 0.0

        c3d = ezc3d.c3d()

        c3d["parameters"]["POINT"]["RATE"]["value"] = [
            float(self.frame_rate)
        ]

        c3d["parameters"]["POINT"]["UNITS"]["value"] = [
            "mm"
        ]

        c3d["parameters"]["POINT"]["LABELS"]["value"] = (
            labels
        )

        c3d["parameters"]["POINT"][
            "DESCRIPTIONS"
        ]["value"] = [
            f"Raw marker ID {marker_id}"
            for marker_id in marker_ids
        ]

        c3d["data"]["points"] = points

        self.output_directory.mkdir(
            parents=True,
            exist_ok=True,
        )

        c3d.write(str(output_path))

        duration = (
            frame_count /
            self.frame_rate
        )

        print(
            f"Saved: {output_path}\n"
            f"Frames: {frame_count}\n"
            f"Markers: {point_count}\n"
            f"Duration: {duration:.2f} seconds\n"
            f"First source frame: {frame_numbers[0]}\n"
            f"Last source frame: {frame_numbers[-1]}"
        )

    def run(self) -> None:
        control_thread = threading.Thread(
            target=self.control_loop,
            daemon=True,
        )

        control_thread.start()

        try:
            self.mocap_loop()
        except KeyboardInterrupt:
            print("\nShutting down...")
        finally:
            self.running = False

            with self.lock:
                was_recording = self.recording

            if was_recording:
                self.stop_recording()


def main() -> None:
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--mocap-port",
        type=int,
        default=5005,
    )

    parser.add_argument(
        "--control-port",
        type=int,
        default=5010,
    )

    parser.add_argument(
        "--fps",
        type=float,
        default=100.0,
    )

    parser.add_argument(
        "--input-units",
        choices=("m", "mm"),
        default="m",
    )

    parser.add_argument(
        "--output-directory",
        type=Path,
        default=Path("recordings"),
    )

    args = parser.parse_args()

    recorder = ControlledC3DRecorder(
        mocap_port=args.mocap_port,
        control_port=args.control_port,
        output_directory=args.output_directory,
        frame_rate=args.fps,
        input_units=args.input_units,
    )

    recorder.run()


if __name__ == "__main__":
    main()