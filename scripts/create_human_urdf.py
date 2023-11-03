# Copyright (c) 2023 PAL Robotics S.L. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

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
