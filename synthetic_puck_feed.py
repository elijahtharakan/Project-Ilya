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
        choices=["toward", "away", "zigzag"],
        default="toward",
        help="Synthetic puck trajectory mode.",
    )
    return parser.parse_args()


def generate_point(i, width, height, mode):
    if mode == "toward":
        x = 320 - (2 * i)
        y = 120 + (3 * i)
    elif mode == "away":
        x = 220 + i
        y = 420 - (3 * i)
    else:  # zigzag
        x = 320 + (120 if (i // 15) % 2 == 0 else -120)
        y = 100 + (2 * i)

    x = max(0, min(width - 1, int(x)))
    y = max(0, min(height - 1, int(y)))
    return x, y


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
        x, y = generate_point(i, args.width, args.height, args.mode)
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
