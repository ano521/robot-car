#!/usr/bin/env python3

import math
import sys
import unittest
from pathlib import Path


SCRIPTS_DIR = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS_DIR))

from vision_common import (  # noqa: E402
    angular_velocity_for_error,
    normalized_bearing,
    normalized_pixel_offset,
    parse_template_detections,
    select_template_detection,
)


class VisionCommonTest(unittest.TestCase):
    def test_identity_homography_center_and_area(self):
        detections = parse_template_detections(
            [7, 100, 50, 1, 0, 0, 0, 1, 0, 0, 0, 1]
        )

        self.assertEqual(len(detections), 1)
        detection = detections[0]
        self.assertEqual(detection.object_id, 7)
        self.assertAlmostEqual(detection.center_x, 50.0)
        self.assertAlmostEqual(detection.center_y, 25.0)
        self.assertAlmostEqual(detection.area, 5000.0)

    def test_translated_homography(self):
        detections = parse_template_detections(
            [3, 40, 20, 1, 0, 0, 0, 1, 0, 15, 8, 1]
        )

        detection = detections[0]
        self.assertAlmostEqual(detection.center_x, 35.0)
        self.assertAlmostEqual(detection.center_y, 18.0)

    def test_malformed_payload_is_ignored(self):
        self.assertEqual(parse_template_detections([1, 10, 10, 1]), [])

    def test_largest_detection_or_requested_id_is_selected(self):
        detections = parse_template_detections(
            [
                1,
                10,
                10,
                1,
                0,
                0,
                0,
                1,
                0,
                0,
                0,
                1,
                2,
                20,
                20,
                1,
                0,
                0,
                0,
                1,
                0,
                0,
                0,
                1,
            ]
        )

        self.assertEqual(select_template_detection(detections).object_id, 2)
        self.assertEqual(select_template_detection(detections, 1).object_id, 1)

    def test_pixel_offset_and_bearing_are_normalized(self):
        self.assertAlmostEqual(normalized_pixel_offset(320, 640), 0.0)
        self.assertAlmostEqual(normalized_pixel_offset(0, 640), -1.0)
        self.assertAlmostEqual(normalized_pixel_offset(640, 640), 1.0)
        self.assertAlmostEqual(
            normalized_bearing(math.radians(15.5), 62.0), 0.5
        )

    def test_controller_deadband_and_direction(self):
        self.assertEqual(angular_velocity_for_error(0.05, 0.06, 0.8, 0.35), 0.0)
        self.assertLess(
            angular_velocity_for_error(0.5, 0.06, 0.8, 0.35), 0.0
        )
        self.assertGreater(
            angular_velocity_for_error(-0.5, 0.06, 0.8, 0.35), 0.0
        )
        self.assertAlmostEqual(
            angular_velocity_for_error(1.0, 0.06, 0.8, 0.35), -0.35
        )


if __name__ == "__main__":
    unittest.main()
