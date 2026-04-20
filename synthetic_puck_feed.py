import argparse
import json
import socket
import time


def parse_args():
    parser = argparse.ArgumentParser(
        description="Send synthetic puck packets over UDP for estimator testing."
    )
    parser.add_argument("--host", default="127.0.0.1", help="Destination UDP host.")
    parser.add_argument("--port", type=int, default=5005, help="Destination UDP port.")
    parser.add_argument("--fps", type=float, default=30.0, help="Send rate in frames per second.")
    parser.add_argument("--seconds", type=float, default=5.0, help="How long to send packets.")
    parser.add_argument("--width", type=int, default=640, help="Frame width for packet metadata.")
    parser.add_argument("--height", type=int, default=480, help="Frame height for packet metadata.")
    parser.add_argument(
        "--mode",
        choices=["showcase", "toward", "away", "zigzag", "attack"],
        default="showcase",
        help="Synthetic puck trajectory mode.",
    )
    return parser.parse_args()


def _clamp_point(x, y, width, height):
    x = max(0, min(width - 1, int(x)))
    y = max(0, min(height - 1, int(y)))
    return x, y


def _showcase_point(i, total_packets, width, height):
    section = max(1, total_packets // 4)
    local_i = i % section
    phase = min(3, i // section)

    if phase == 0:
        x = width - 80 - (4.5 * local_i)
        y = height * 0.30 + (1.1 * local_i)
    elif phase == 1:
        x = width * 0.27 + (0.7 * local_i)
        y = height * 0.64 - (0.2 * local_i)
    elif phase == 2:
        x = width * 0.36 - (1.0 * local_i)
        y = height * 0.42 + (22 if (local_i // 10) % 2 == 0 else -22)
    else:
        x = width * 0.70 - (3.1 * local_i)
        y = height * 0.72 - (1.4 * local_i)
    return _clamp_point(x, y, width, height)


def generate_point(i, width, height, mode, total_packets):
    if mode == "showcase":
        return _showcase_point(i, total_packets, width, height)
    if mode == "toward":
        x = width - 110 - (3.5 * i)
        y = height * 0.28 + (1.8 * i)
    elif mode == "away":
        x = width * 0.20 + (2.4 * i)
        y = height * 0.55 - (0.8 * i)
    elif mode == "attack":
        x = width * 0.24 + (0.35 * i)
        y = height * 0.52 + (0.20 * i)
    else:  # zigzag
        x = width * 0.55 - (2.0 * i)
        y = height * 0.35 + (38 if (i // 12) % 2 == 0 else -38)

    return _clamp_point(x, y, width, height)


def main():
    args = parse_args()

    interval = 1.0 / max(1.0, args.fps)
    total_packets = max(1, int(args.seconds * args.fps))

    udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    print(
        f"Sending {total_packets} synthetic packets to {args.host}:{args.port} "
        f"at {args.fps:.1f} FPS (mode={args.mode})"
    )

    for i in range(total_packets):
        x, y = generate_point(i, args.width, args.height, args.mode, total_packets)
        packet = {
            "timestamp": time.time(),
            "detected": True,
            "x": x,
            "y": y,
            "x_norm": x / args.width,
            "y_norm": y / args.height,
            "radius": 18.0,
            "frame_width": args.width,
            "frame_height": args.height,
        }
        payload = json.dumps(packet).encode("utf-8")
        udp.sendto(payload, (args.host, args.port))
        time.sleep(interval)

    udp.close()
    print("Done.")


if __name__ == "__main__":
    main()
