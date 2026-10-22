#!/usr/bin/env python3
"""Render Usage graphs, then center styled label boxes on their connections.

Requires Graphviz's dot and Python's standard library. With no arguments,
renders every DOT file beside this script; otherwise renders the given files.
"""

import argparse
import math
from pathlib import Path
import re
import subprocess
import xml.etree.ElementTree as ET


SVG = "http://www.w3.org/2000/svg"
ET.register_namespace("", SVG)
ET.register_namespace("xlink", "http://www.w3.org/1999/xlink")
NUMBER = r"[-+]?(?:\d*\.\d+|\d+)(?:[eE][-+]?\d+)?"


def tag(name):
    return f"{{{SVG}}}{name}"


def coordinates(value):
    numbers = [float(n) for n in re.findall(NUMBER, value)]
    return list(zip(numbers[::2], numbers[1::2]))


def bounds(points):
    xs, ys = zip(*points)
    return min(xs), min(ys), max(xs), max(ys)


def overlap(a, b, gap=4):
    return (
        a[0] < b[2] + gap and b[0] < a[2] + gap
        and a[1] < b[3] + gap and b[1] < a[3] + gap
    )


def polyline(path):
    """Read dot's axis-aligned M/L/C path; simplify straight Bezier segments."""
    points = []
    for command, values in re.findall(r"([MLC])([^MLC]*)", path):
        pairs = coordinates(values)
        if command == "C":
            # Ortho splines are cubic curves with collinear control points.
            pairs = pairs[2::3]
        for point in pairs:
            if points and point == points[-1]:
                continue
            while len(points) > 1:
                a, b = points[-2:]
                if not (a[0] == b[0] == point[0] or a[1] == b[1] == point[1]):
                    break
                points.pop()
            points.append(point)
    if len(points) < 2:
        raise ValueError(f"Expected an orthogonal connection path: {path}")
    return points


def label_positions(points, width, height):
    """Prefer the path midpoint, using nearby straight sections when needed."""
    lengths = [math.dist(a, b) for a, b in zip(points, points[1:])]
    halfway = sum(lengths) / 2
    distance = 0
    candidates = []
    for a, b, length in zip(points, points[1:], lengths):
        horizontal = abs(a[1] - b[1]) < 0.01
        extent = width if horizontal else height
        clearance = extent / 2 + 4
        if length >= 2 * clearance:
            near = max(clearance, min(length - clearance, halfway - distance))
            offsets = [near, length / 2, clearance, length - clearance]
            offsets += [length * fraction for fraction in (0.25, 0.75)]
            offsets += [clearance + step * 6
                        for step in range(int((length - 2 * clearance) / 6) + 1)]
            for offset in offsets:
                if not clearance <= offset <= length - clearance:
                    continue
                fraction = offset / length
                x = a[0] + fraction * (b[0] - a[0])
                y = a[1] + fraction * (b[1] - a[1])
                box = (x - width / 2, y - height / 2,
                       x + width / 2, y + height / 2)
                candidates.append((abs(distance + offset - halfway), x, y, box))
        distance += length
    return candidates


def render(source):
    result = subprocess.run(
        ["dot", "-Tsvg", str(source)], capture_output=True, check=True
    )
    if result.stderr:
        raise RuntimeError(result.stderr.decode())
    root = ET.fromstring(result.stdout)
    graph = root.find(tag("g"))
    _, _, canvas_width, canvas_height = map(float, root.get("viewBox").split())
    translation = re.search(r"translate\(([^)]+)\)", graph.get("transform"))
    tx, ty = map(float, translation[1].split())
    canvas = (-tx, -ty, canvas_width - tx, canvas_height - ty)
    obstacles = []
    for node in graph.findall(f"{tag('g')}[@class='node']"):
        for shape in node:
            if shape.tag == tag("path"):
                obstacles.append(bounds(coordinates(shape.get("d"))))
            elif shape.tag == tag("polygon"):
                obstacles.append(bounds(coordinates(shape.get("points"))))

    labels = []
    for edge in graph.findall(f"{tag('g')}[@class='edge']"):
        texts = edge.findall(tag("text"))
        if not texts:
            continue
        background = next(
            polygon for polygon in edge.findall(tag("polygon"))
            if polygon.get("fill") == "white"
        )
        box = bounds(coordinates(background.get("points")))
        path = edge.find(tag("path"))
        width, height = box[2] - box[0], box[3] - box[1]
        labels.append((width * height, edge, texts, background, box, path))

    choices = {}
    for index, (_, edge, texts, background, original, path) in enumerate(labels):
        width, height = original[2] - original[0], original[3] - original[1]
        candidates = label_positions(polyline(path.get("d")), width, height)
        choices[index] = sorted(
            p for p in candidates
            if canvas[0] <= p[3][0] and canvas[1] <= p[3][1]
            and p[3][2] <= canvas[2] and p[3][3] <= canvas[3]
            and not any(overlap(p[3], o) for o in obstacles)
        )
        if not choices[index]:
            title = edge.find(tag("title")).text
            raise ValueError(f"{source.name}: no clear straight segment for {title}")

    def place(remaining, occupied):
        if not remaining:
            return {}
        viable = {i: [p for p in choices[i]
                      if not any(overlap(p[3], box) for box in occupied)]
                  for i in remaining}
        index = min(remaining, key=lambda i: (len(viable[i]), -labels[i][0]))
        for candidate in viable[index]:
            result = place(remaining - {index}, occupied + [candidate[3]])
            if result is not None:
                result[index] = candidate
                return result
        return None

    positions = place(set(choices), [])
    if positions is None:
        raise ValueError(f"{source.name}: label boxes overlap; increase DOT spacing")
    for index, (_, edge, texts, background, original, path) in enumerate(labels):
        _, x, y, box = positions[index]
        width, height = original[2] - original[0], original[3] - original[1]
        dx = x - (original[0] + original[2]) / 2
        dy = y - (original[1] + original[3]) / 2
        label = ET.SubElement(graph, tag("g"), {
            "class": "edge-label",
            "data-edge": edge.find(tag("title")).text,
            "transform": f"translate({dx:.3f} {dy:.3f})",
        })
        border = {
            "x": str(original[0]), "y": str(original[1]),
            "width": str(width), "height": str(height),
            "fill": "white", "fill-opacity": "1",
            "stroke": path.get("stroke"),
            "stroke-width": path.get("stroke-width", "1"),
        }
        if path.get("stroke-dasharray"):
            border["stroke-dasharray"] = path.get("stroke-dasharray")
        ET.SubElement(label, tag("rect"), border)
        edge.remove(background)
        for text in texts:
            edge.remove(text)
            label.append(text)
        obstacles.append(box)

    # Appended label groups paint above all connection strokes and arrowheads.
    destination = source.with_suffix(".svg")
    ET.ElementTree(root).write(destination, encoding="utf-8", xml_declaration=True)
    print(f"{destination.name}: {len(labels)} centered connection labels")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("sources", nargs="*", type=Path)
    args = parser.parse_args()
    sources = args.sources or sorted(Path(__file__).resolve().parent.glob("*.dot"))
    for source in sources:
        render(source)


if __name__ == "__main__":
    main()
