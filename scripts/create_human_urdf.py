import sys
import argparse
import xacro

TPL = "../urdf/human-tpl.xacro"

parser = argparse.ArgumentParser(description="Generate the URDF of a human.")
parser.add_argument(
    "-i", "--id", type=str, default="", help="the person's unique ID (default: empty)"
)
parser.add_argument(
    "-u",
    "--upperarm-length",
    type=float,
    default=20.0,
    help="length of the upperarm, in cm (default: 20)",
)
parser.add_argument(
    "-f",
    "--forearm-length",
    type=float,
    default=30.0,
    help="length of the forearm, in cm (default: 30)",
)
parser.add_argument(
    "-t",
    "--torso-height",
    type=float,
    default=30.0,
    help="torso height, in cm (default: 30)",
)
parser.add_argument(
    "-s",
    "--shoulder-to-shoulder",
    type=float,
    default=50.0,
    help="distance between the two shoulders, in cm (default: 50)",
)

args = parser.parse_args()

params = {
    "id": args.id,
    "upperarm_length": str(args.upperarm_length / 100.0),
    "forearm_length": str(args.forearm_length / 100.0),
    "torso_height": str(args.torso_height / 100.0),
    "neck_shoulder_length": str((args.shoulder_to_shoulder / 100.0) / 2),
}

print(xacro.process_file(TPL, mappings=params).toprettyxml(indent="   "))
