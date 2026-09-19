#!/usr/bin/env python3
"""
Shared MLX90640 frame maths: the blob finder both the sensor node and the
flight node run.

It lives here, and not in either node, so the boxes drawn on the video stream
are the SAME boxes the flight node is flying to. A second implementation for
the overlay would eventually disagree with the one that matters, and the
overlay is the only thing anyone actually watches.

Deliberately free of ROS and of the I2C driver: it takes a 24x32 numpy array
of degrees C and gives back blobs, so it can be exercised on the bench.
"""

import numpy as np

H, W = 24, 32


def find_blobs(grid, min_contrast, max_blobs=6, min_pixels=2):
    """Warm blobs in a 24x32 grid of degrees C, hottest first.

    Each blob is a dict:
        peak      hottest pixel in it, deg C
        row, col  temperature-weighted centroid (floats), far steadier than
                  argmax -- this is what the flight node aims at
        pixels    how many pixels it covers
        r0,r1,c0,c1   bounding box, inclusive, for drawing

    min_pixels rejects single-pixel blobs: one noisy pixel reading high is
    exactly what must not be allowed to look like a hot box.
    """
    ambient = float(np.median(grid))
    used = np.zeros(grid.shape, dtype=bool)
    blobs = []
    for _ in range(max_blobs):
        masked = np.where(used, -np.inf, grid)
        r0, c0 = np.unravel_index(int(np.argmax(masked)), grid.shape)
        peak = float(grid[r0, c0])
        if peak < ambient + min_contrast:
            break
        # Half-way between ambient and the peak: the blob's edge.
        thresh = max(ambient + 0.5 * min_contrast, ambient + 0.5 * (peak - ambient))
        stack = [(r0, c0)]
        pix = []
        seen = {(r0, c0)}
        while stack:
            r, c = stack.pop()
            if used[r, c] or grid[r, c] < thresh:
                continue
            pix.append((r, c))
            for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                rr, cc = r + dr, c + dc
                if 0 <= rr < H and 0 <= cc < W and (rr, cc) not in seen:
                    seen.add((rr, cc))
                    stack.append((rr, cc))
        if len(pix) < min_pixels:
            # A single warm pixel is sensor noise, not a box. This is the first
            # of the three filters that stop one bad pixel being "the hot box";
            # the others are min_cluster_frames in the survey and the
            # hottest-in-frame check before the descent commits.
            for r, c in pix:
                used[r, c] = True
            continue
        rows = np.array([p[0] for p in pix], dtype=float)
        cols = np.array([p[1] for p in pix], dtype=float)
        wts = np.array([grid[p] - thresh for p in pix], dtype=float) + 0.05
        blobs.append({
            'peak': peak,
            'row': float(np.sum(rows * wts) / np.sum(wts)),
            'col': float(np.sum(cols * wts) / np.sum(wts)),
            'pixels': len(pix),
            'r0': int(rows.min()), 'r1': int(rows.max()),
            'c0': int(cols.min()), 'c1': int(cols.max()),
        })
        # Blank the blob and a one-pixel ring so its shoulder is not a new blob.
        for r, c in pix:
            used[max(0, r - 1):r + 2, max(0, c - 1):c + 2] = True
    return blobs, ambient


def annotate(grid, blobs, ambient, scale=20, note=''):
    """The thermal frame as a BGR image with the hot spot boxed.

    The HOTTEST blob -- the one the mission would fly to -- gets a thick box
    and its temperature; the rest get thin ones, so what is being rejected is
    as visible as what is being chosen. Imported lazily by the caller: this is
    the only part of this module that needs OpenCV.
    """
    import cv2

    lo, hi = float(grid.min()), float(grid.max())
    if hi - lo < 0.1:
        hi = lo + 0.1
    norm = np.uint8((grid - lo) * 255.0 / (hi - lo))
    img = cv2.applyColorMap(norm, cv2.COLORMAP_INFERNO)
    img = cv2.resize(img, (W * scale, H * scale), interpolation=cv2.INTER_CUBIC)

    # Nadir. With the lens level this is the point directly under the camera,
    # so it is also the mark the hot box has to be brought onto.
    cx, cy = img.shape[1] // 2, img.shape[0] // 2
    cv2.line(img, (cx - 12, cy), (cx + 12, cy), (255, 255, 255), 1)
    cv2.line(img, (cx, cy - 12), (cx, cy + 12), (255, 255, 255), 1)

    for i, b in enumerate(blobs):
        x0, y0 = int(b['c0'] * scale), int(b['r0'] * scale)
        x1, y1 = int((b['c1'] + 1) * scale), int((b['r1'] + 1) * scale)
        hottest = (i == 0)
        color = (0, 255, 0) if hottest else (200, 200, 200)
        cv2.rectangle(img, (x0, y0), (x1, y1), color, 3 if hottest else 1)
        label = f"{b['peak']:.1f}C" + (" TARGET" if hottest else "")
        cv2.putText(img, label, (x0, max(14, y0 - 6)), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5 if hottest else 0.4, color, 2 if hottest else 1)
        if hottest:
            # The centroid is what the flight node actually aims at, and it is
            # not the middle of the box when the blob is lopsided.
            cv2.drawMarker(img, (int((b['col'] + 0.5) * scale),
                                 int((b['row'] + 0.5) * scale)),
                           (0, 255, 255), cv2.MARKER_CROSS, 14, 2)

    # "target" is the hottest accepted BLOB; "hot px" is the hottest single
    # pixel in the frame. They differ exactly when a noise spike has been
    # rejected, which is the moment you most want to see both numbers.
    target = f"target {blobs[0]['peak']:.1f}C  " if blobs else "no target  "
    banner = (target + f"hot px {hi:.1f}C  ambient {ambient:.1f}C  "
              f"blobs {len(blobs)}" + (f"  {note}" if note else ""))
    cv2.rectangle(img, (0, img.shape[0] - 22), (img.shape[1], img.shape[0]),
                  (0, 0, 0), -1)
    cv2.putText(img, banner, (6, img.shape[0] - 7), cv2.FONT_HERSHEY_SIMPLEX,
                0.45, (255, 255, 255), 1)
    return img
