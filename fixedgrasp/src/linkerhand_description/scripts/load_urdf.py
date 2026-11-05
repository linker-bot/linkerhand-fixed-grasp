#!/usr/bin/env python3
"""Load the pinned upstream L10 model and resolve its relative mesh paths."""
import argparse
from pathlib import Path
import sys
import xml.etree.ElementTree as ET


def load_urdf(package_path, side):
    package_path = Path(package_path).resolve()
    models = package_path / "models"
    if not models.exists():
        models = package_path.parents[2] / "vendor" / "linkerhand-urdf"
    model = models / "L20lite" / side / ("linkerhand_l20lite_" + side + ".urdf")
    text = model.read_text().replace(r"\r\n", "\n")
    robot = ET.fromstring(text)
    for mesh in robot.findall(".//mesh"):
        resource = mesh.attrib["filename"]
        if "://" in resource or Path(resource).is_absolute():
            raise ValueError("Expected an upstream relative mesh path: " + resource)
        asset = (model.parent / resource).resolve()
        if not asset.is_file() or model.parent.resolve() not in asset.parents:
            raise ValueError("Missing or invalid mesh: " + str(asset))
        mesh.set("filename", asset.as_uri())
    return ET.tostring(robot, encoding="unicode")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--package-path", required=True)
    parser.add_argument("--side", choices=("left", "right"), default="left")
    args = parser.parse_args()
    try:
        print(load_urdf(args.package_path, args.side))
    except (OSError, ValueError, ET.ParseError) as exc:
        sys.exit(str(exc))
