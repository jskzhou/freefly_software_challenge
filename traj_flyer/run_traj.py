import argparse
import anyio
import numpy as np

from offboard import fly_traj

def main():
    parser = argparse.ArgumentParser(description="Fly pattern from trajectory CSV")
    parser.add_argument("traj", help="Path to trajectory CSV")
    parser.add_argument(
        "--url",
        default="udp://:14540",
        help="MAVSDK connection URL (default: udp://:14540)",
    )

    args = parser.parse_args()
    anyio.run(
        fly_traj,
        args.traj,
        args.url,
    )


if __name__ == "__main__":
    main()