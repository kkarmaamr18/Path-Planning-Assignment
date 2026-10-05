from __future__ import annotations

import math
from typing import List

from src.models import CarPose, Cone, Path2D


class PathPlanning:

    def __init__(self, car_pose: CarPose, cones: List[Cone]):
        self.car_pose = car_pose
        self.cones = cones

    def generatePath(self) -> Path2D:

        step = 0.5
        desired_path_length = 8.0
        half_track_width = 1.0

        cx = self.car_pose.x
        cy = self.car_pose.y
        yaw = self.car_pose.yaw

        forward_x = math.cos(yaw)
        forward_y = math.sin(yaw)

        blue = [cone for cone in self.cones if cone.color == 1]
        yellow = [cone for cone in self.cones if cone.color == 0]

        def forward_distance(cone: Cone) -> float:
            return (
                (cone.x - cx) * forward_x
                + (cone.y - cy) * forward_y
            )

        blue.sort(key=forward_distance)
        yellow.sort(key=forward_distance)

        target_points: Path2D = []

        if blue and yellow:
            pair_count = min(len(blue), len(yellow))

            for i in range(pair_count):
                blue_cone = blue[i]
                yellow_cone = yellow[i]

                center_x = (blue_cone.x + yellow_cone.x) / 2.0
                center_y = (blue_cone.y + yellow_cone.y) / 2.0

                target_points.append((center_x, center_y))


        elif blue or yellow:
            if blue:
                visible_cones = blue
                is_blue = True
            else:
                visible_cones = yellow
                is_blue = False


            if len(visible_cones) >= 2:
                first_cone = visible_cones[0]
                last_cone = visible_cones[-1]

                direction_x = last_cone.x - first_cone.x
                direction_y = last_cone.y - first_cone.y

                direction_length = math.hypot(
                    direction_x,
                    direction_y
                )

                if direction_length > 1e-6:
                    direction_x /= direction_length
                    direction_y /= direction_length
                else:
                    direction_x = forward_x
                    direction_y = forward_y


                dot_product = (
                    direction_x * forward_x
                    + direction_y * forward_y
                )

                if dot_product < 0:
                    direction_x = -direction_x
                    direction_y = -direction_y

            else:

                direction_x = forward_x
                direction_y = forward_y

            normal_x = -direction_y
            normal_y = direction_x

            for cone in visible_cones:
                if is_blue:

                    target_x = (
                        cone.x
                        - normal_x * half_track_width
                    )

                    target_y = (
                        cone.y
                        - normal_y * half_track_width
                    )

                else:

                    target_x = (
                        cone.x
                        + normal_x * half_track_width
                    )

                    target_y = (
                        cone.y
                        + normal_y * half_track_width
                    )

                target_points.append(
                    (target_x, target_y)
                )


        control_points: Path2D = [(cx, cy)]

        for point in target_points:
            px, py = point

            projection = (
                (px - cx) * forward_x
                + (py - cy) * forward_y
            )

            if projection >= -0.5:
                control_points.append((px, py))

        if len(control_points) >= 2:
            last_x, last_y = control_points[-1]

            if len(control_points) >= 3:
                previous_x, previous_y = control_points[-2]

                end_dx = last_x - previous_x
                end_dy = last_y - previous_y

            else:
                end_dx = last_x - cx
                end_dy = last_y - cy

            end_length = math.hypot(end_dx, end_dy)

            if end_length > 1e-6:
                end_dx /= end_length
                end_dy /= end_length
            else:
                end_dx = forward_x
                end_dy = forward_y

            dot_product = (
                end_dx * forward_x
                + end_dy * forward_y
            )

            if dot_product < 0:
                end_dx = -end_dx
                end_dy = -end_dy

        else:
            
            last_x = cx
            last_y = cy

            end_dx = forward_x
            end_dy = forward_y


        current_length = 0.0

        for i in range(len(control_points) - 1):
            x1, y1 = control_points[i]
            x2, y2 = control_points[i + 1]

            current_length += math.hypot(
                x2 - x1,
                y2 - y1
            )

        remaining_length = max(
            0.0,
            desired_path_length - current_length
        )

        if remaining_length > 0.0:
            extension_x = (
                last_x
                + end_dx * remaining_length
            )

            extension_y = (
                last_y
                + end_dy * remaining_length
            )

            control_points.append(
                (extension_x, extension_y)
            )


        path: Path2D = []

        for i in range(len(control_points) - 1):
            x1, y1 = control_points[i]
            x2, y2 = control_points[i + 1]

            dx = x2 - x1
            dy = y2 - y1

            distance = math.hypot(dx, dy)

            if distance < 1e-6:
                continue

            number_of_segments = max(
                1,
                math.ceil(distance / step)
            )

            for j in range(1, number_of_segments + 1):
                t = j / number_of_segments

                x = x1 + dx * t
                y = y1 + dy * t

                path.append((x, y))

        return path