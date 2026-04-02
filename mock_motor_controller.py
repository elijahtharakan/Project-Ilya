import argparse
import json
import socket
import time


def parse_args():
    parser = argparse.ArgumentParser(
        description="Mock motor controller that receives paddle targets over UDP."
    )
    parser.add_argument("--listen-host", default="0.0.0.0", help="UDP host to bind.")
    parser.add_argument("--listen-port", type=int, default=5007, help="UDP port to bind.")
    parser.add_argument("--timeout", type=float, default=0.0, help="Optional runtime timeout in seconds.")
    return parser.parse_args()


def main():
    args = parse_args()
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((args.listen_host, args.listen_port))
    sock.settimeout(1.0)

    print(f"Mock motor controller listening on {args.listen_host}:{args.listen_port}")
    start = time.time()

    while True:
        if args.timeout > 0 and (time.time() - start) > args.timeout:
            print("Mock motor controller timeout reached.")
            break

        try:
            raw, addr = sock.recvfrom(65535)
        except socket.timeout:
            continue

        try:
            packet = json.loads(raw.decode("utf-8"))
        except json.JSONDecodeError:
            print("Received invalid JSON packet.")
            continue

        mode = packet.get("mode", "UNKNOWN")
        target_x = packet.get("target_x_m", None)
        target_y = packet.get("target_y_m", None)
        reason = packet.get("reason", "")

        print(
            f"From {addr[0]}:{addr[1]} | mode={mode} "
            f"target=({target_x}, {target_y}) reason={reason}"
        )
        print(json.dumps(packet), flush=True)

    sock.close()


if __name__ == "__main__":
    main()
