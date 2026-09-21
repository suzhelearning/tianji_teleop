import time

import pytest

from tianji_runtime.camera_stream import (
    CameraStreamValidator, ImageRecord, MetadataJsonParser, MetadataRecord, StreamFault,
)


def test_image_first_waits_for_metadata_and_preserves_receive_time():
    validator = CameraStreamValidator('top', width=2, height=1)
    received = time.monotonic_ns()
    pixels = object()
    image = ImageRecord(100, 'optical', 2, 1, 6, 'rgb8', pixels)
    assert validator.accept_image(image, received) is None
    frame = validator.accept_metadata(MetadataRecord(100, 'optical', 7), received + 1)
    assert frame.rgb is pixels
    assert frame.sequence == 7
    assert frame.received_ns == received
    assert frame.timestamp_ns >= received


def test_metadata_first_and_missing_older_image_do_not_block_complete_pair():
    validator = CameraStreamValidator('top', width=2, height=1)
    now = time.monotonic_ns()
    first = ImageRecord(100, 'optical', 2, 1, 6, 'rgb8', object())
    second = ImageRecord(200, 'optical', 2, 1, 6, 'rgb8', object())
    assert validator.accept_image(first, now) is None
    assert validator.accept_metadata(MetadataRecord(200, 'optical', 8), now) is None
    paired = validator.accept_image(second, now)
    assert paired.sequence == 8
    assert paired.rgb is second.rgb


def test_duplicate_metadata_does_not_refresh_pair_freshness():
    validator = CameraStreamValidator('top', width=2, height=1)
    now = time.monotonic_ns()
    image = ImageRecord(100, 'optical', 2, 1, 6, 'rgb8', object())
    validator.accept_image(image, now)
    frame = validator.accept_metadata(MetadataRecord(100, 'optical', 7), now)
    assert validator.accept_metadata(MetadataRecord(200, 'optical', 7), now + 1) is None
    with pytest.raises(StreamFault):
        validator.check_fresh(frame.timestamp_ns + 2_000_000_001)


def test_image_and_metadata_publishers_are_independently_pinned():
    validator = CameraStreamValidator('top', width=2, height=1)
    validator.note_image_publisher(b'image')
    validator.note_metadata_publisher(b'metadata')
    with pytest.raises(StreamFault):
        validator.note_metadata_publisher(b'restarted-metadata')


def test_metadata_requires_complete_json_and_integer_frame_number():
    parser = MetadataJsonParser('top')
    assert parser.accept('optical', 100, '{"frame_number":42}').frame_number == 42
    for payload in ('{', '{"frame_number":true}', '{}', '[]'):
        with pytest.raises(StreamFault):
            parser.accept('optical', 100, payload)


def test_metadata_first_preserves_pre_episode_receive_boundary():
    validator = CameraStreamValidator('top', width=2, height=1)
    before_start = time.monotonic_ns()
    assert validator.accept_metadata(MetadataRecord(100, 'optical', 7), before_start) is None
    after_start = before_start + 10
    frame = validator.accept_image(
        ImageRecord(100, 'optical', 2, 1, 6, 'rgb8', object()), after_start)
    assert frame.received_ns == before_start
    assert frame.received_ns < after_start


def test_late_metadata_is_dropped_but_driver_counter_restart_is_a_fault():
    validator = CameraStreamValidator('top', width=2, height=1)
    now = time.monotonic_ns()
    validator.accept_image(ImageRecord(100, 'optical', 2, 1, 6, 'rgb8', object()), now)
    validator.accept_image(ImageRecord(200, 'optical', 2, 1, 6, 'rgb8', object()), now)
    paired = validator.accept_metadata(MetadataRecord(200, 'optical', 8), now)
    assert paired.sequence == 8
    assert validator.accept_metadata(MetadataRecord(100, 'optical', 7), now) is None
    with pytest.raises(StreamFault):
        validator.accept_metadata(MetadataRecord(300, 'optical', 0), now)
    with pytest.raises(StreamFault):
        validator.check_fresh(paired.timestamp_ns + 2_000_000_001)
