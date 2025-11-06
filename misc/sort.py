import cv2
import numpy as np

# Initialize video
cap = cv2.VideoCapture("./two_bouncing_squares.mp4")

# Initialize linear filter variable
tracks = []


def bbox_chw(pts):
    """
    Returns
    (cx, cy, w, h)
        cx, cy : centre coordinates
        w, h   : width and height
    """
    # min / max over the 4-point axis (-2)
    xy_min = pts.min(axis=-2)  # (..., 2)
    xy_max = pts.max(axis=-2)

    wh = xy_max - xy_min  # (..., 2)
    centre = (xy_min + xy_max) / 2  # (..., 2)

    cx, cy = centre[..., 0], centre[..., 1]
    w, h = wh[..., 0], wh[..., 1]
    return cx.item(), cy.item(), w.item(), h.item()


def detect_square():
    """
    Returns bbox
        np.array([
            [x1, y1],
            [x2, y2],
            [x3, y3],
            [x4, y4],
        ])
    """
    global tracks

    while True:
        ret, frame = cap.read()

        if not ret:  # If frame is not read correctly, break the loop (end of video)
            break

        # Get bounding box
        lower_white = np.array([200, 200, 200])  # Adjust these values as needed
        upper_white = np.array([255, 255, 255])  # Adjust these values as needed
        mask = cv2.inRange(frame, lower_white, upper_white)
        contours, hierarchy = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        detections = []
        for contour in contours:
            if len(contour) < 4:
                # Invalid detection
                continue

            bbox = np.array([contour[i][0] for i in range(4)])
            cx, cy, w, h = bbox_chw(bbox)

            detections.append((cx, cy, w, h))

        # Initialize tracks if empty
        if len(tracks) == 0:
            tracks.append(
                {"id": 1, "state": np.array([cx, cy, w, 1, 0, 0]), "missed_frames": 0}
            )

        # Make predictions of existing tracks
        for track in tracks:
            center = np.array([track["state"][0], track["state"][1]])
            velocity = np.array([track["state"][4], track["state"][5]])
            track["predicted"] = make_prediction(center, velocity)

        # Use overlap to assign detections to existing tracks
        assigned_detections = set()
        for track in tracks:
            best_overlap = 0
            best_det = None
            for i, det in enumerate(detections):
                if i in assigned_detections:
                    continue
                cx, cy, w, h = det
                w_p = w * 1.6
                overlap = bbox_overlap(
                    np.array(
                        [
                            track["predicted"][0] - w_p / 2,
                            track["predicted"][1] - w_p / 2,
                            track["predicted"][0] + w_p / 2,
                            track["predicted"][1] + w_p / 2,
                        ]
                    ),
                    np.array([cx - w / 2, cy - h / 2, cx + w / 2, cy + h / 2]),
                )
                if overlap > best_overlap:
                    best_overlap = overlap
                    best_det = (i, det)

            # Update if match found
            if best_det and best_overlap > 0:
                i, (cx, cy, w, h) = best_det
                assigned_detections.add(i)
                new_vel_x = cx - track["state"][0]
                new_vel_y = cy - track["state"][1]
                track["state"] = np.array([cx, cy, w, 1, new_vel_x, new_vel_y])
                track["missed_frames"] = 0
            else:
                # No matches were found
                track["missed_frames"] += 1

        tracks = [t for t in tracks if t["missed_frames"] < 10]

        # Add new tracks for unmatched detections
        for i, det in enumerate(detections):
            if i not in assigned_detections:
                cx, cy, w, h = det
                new_track = {
                    "id": len(tracks) + 1,
                    "state": np.array([cx, cy, w, 1, 0, 0]),
                    "missed_frames": 0,
                }
                tracks.append(new_track)

        # Draw all detections
        for track in tracks:
            cx, cy, w = track["state"][0], track["state"][1], track["state"][2]
            frame = cv2.rectangle(
                frame,
                (int(cx - w / 2), int(cy - w / 2)),
                (int(cx + w / 2), int(cy + w / 2)),
                (0, 255, 0),
                2,
            )
            # Define text properties
            text = str(track["id"])
            org = (int(cx), int(cy))  # Bottom-left corner of the text
            fontFace = cv2.FONT_HERSHEY_SIMPLEX
            fontScale = 1
            color = (0, 0, 0)  # Green color (BGR)
            thickness = 2
            frame = cv2.putText(
                frame, text, org, fontFace, fontScale, color, thickness, cv2.LINE_AA
            )

        cv2.imshow("Video Frame", frame)

        # Exit the loop if 'q' is pressed
        if cv2.waitKey(25) & 0xFF == ord("q"):
            break

    # Release capture
    cap.release()
    cv2.destroyAllWindows()


def make_prediction(center, velocity):
    """
    Prediction of contour in the next iteration

    Returns: (x, y)
    """
    return center + velocity


def bbox_overlap(boxA, boxB):
    """
    Calculate overlap area (in pixels) between two bounding boxes.

    Each box is defined as a tuple: (x_min, y_min, x_max, y_max)
    """
    # Unpack coordinates
    xA1, yA1, xA2, yA2 = boxA
    xB1, yB1, xB2, yB2 = boxB

    # Compute intersection rectangle
    x_left = max(xA1, xB1)
    y_top = max(yA1, yB1)
    x_right = min(xA2, xB2)
    y_bottom = min(yA2, yB2)

    # Check for no overlap
    if x_right <= x_left or y_bottom <= y_top:
        return 0  # No overlap

    # Compute overlapping area
    overlap_area = (x_right - x_left) * (y_bottom - y_top)
    return overlap_area


def boxes_overlap(boxA, boxB):
    """Return True if boxes overlap, otherwise False."""
    return bbox_overlap(boxA, boxB) > 0


detect_square()
