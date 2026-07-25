"""Diagnostic command-line interface."""

import argparse

from soarm101_motion import SOARM101
from soarm101_motion.backends import MockSOARM101Backend


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="soarm101")
    subparsers = parser.add_subparsers(dest="command", required=True)
    subparsers.add_parser("info", help="show package and backend information")
    diagnose = subparsers.add_parser("diagnose", help="run non-motion diagnostics")
    diagnose.add_argument("--backend", choices=("mock",), default="mock")
    return parser


def main() -> int:
    args = build_parser().parse_args()
    if args.command == "info":
        print("SO-ARM101 Motion SDK (early development)")
        return 0
    arm = SOARM101(backend=MockSOARM101Backend())
    arm.connect()
    print(arm.run_diagnostics())
    arm.disconnect()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
