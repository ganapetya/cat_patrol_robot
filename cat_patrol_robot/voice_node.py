#!/usr/bin/env python3
# =============================================================================
# voice_node.py — plays a per-cat voice line when cat_detector_cpp recognizes
# a specific cat (white/brown), instead of the onboard buzzer beep.
# =============================================================================
#
# HOW IT WORKS:
#   1. Subscribes to /cat_detector_cpp/detections (vision_msgs/Detection2DArray).
#   2. Each Detection2D carries up to two hypotheses in `results`:
#        results[0] = generic "cat" class (from YOLO)
#        results[1] = per-cat identity ("brown"/"white") + confidence,
#                     present only when the classifier fired.
#   3. On a confident identity, picks a random mp3 from ~/cats/voices/ whose
#      filename starts with the matching prefix (jente-=white, arik-=brown)
#      and plays it. paplay can't decode mp3 directly (libsndfile has no mp3
#      support), so it's decoded to WAV via ffmpeg and piped into paplay --
#      that keeps playback going through PulseAudio's default sink (the BT
#      speaker connect_bt_speaker.sh sets up), rather than through whatever
#      device a standalone mp3 player like ffplay/cvlc would pick on its own.
#   4. A per-identity cooldown stops a single lingering sighting from
#      re-triggering a new line every frame.
#
# WHY A SEPARATE NODE (not patrol_node)?
#   patrol_node is the C++ FSM driving the robot; it has no dependency on
#   cat identification today. Keeping this reactive/audio logic in its own
#   Python node means it can be started, stopped, and iterated on
#   independently of the patrol round itself.
# =============================================================================

import fcntl
import os
import random
import subprocess
import time

import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray

PREFIX_BY_LABEL = {
    'white': 'jente-',
    'brown': 'arik-',
}

# Singleton lock: only one cat_voice may run at a time. Two instances would each
# react to the same detection and play overlapping mp3s (and each keeps its own
# cooldown, so a lingering cat retriggers constantly). A stray instance is easy
# to leave behind — e.g. `ros2 run ... &` then killing the wrapper orphans the
# node. An exclusive flock is race-free and self-healing: the kernel releases it
# automatically when the holder dies, so there is no stale-lock to clean up.
_LOCK_PATH = '/tmp/cat_voice.singleton.lock'


def acquire_singleton_lock():
    """Return the held lock file object, or None if another instance holds it.

    The returned object MUST stay referenced for the process lifetime — closing
    it (or letting it be garbage-collected) releases the lock.
    """
    fd = open(_LOCK_PATH, 'w')
    try:
        fcntl.flock(fd.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
    except OSError:
        fd.close()
        return None
    fd.write(f'{os.getpid()}\n')
    fd.flush()
    return fd


class VoiceNode(Node):
    """Plays a random per-cat mp3 line when cat_detector_cpp recognizes a cat."""

    def __init__(self):
        super().__init__('cat_voice')

        self.declare_parameter('detections_topic', '/cat_detector_cpp/detections')
        self.declare_parameter('voices_dir', os.path.expanduser('~/cats/voices'))
        self.declare_parameter('min_identity_conf', 0.6)
        self.declare_parameter('cooldown_sec', 15.0)

        topic = self.get_parameter('detections_topic').get_parameter_value().string_value
        self.voices_dir = os.path.expanduser(
            self.get_parameter('voices_dir').get_parameter_value().string_value)
        self.min_conf = self.get_parameter('min_identity_conf').get_parameter_value().double_value
        self.cooldown_sec = self.get_parameter('cooldown_sec').get_parameter_value().double_value

        self._files_by_label = self._index_voice_files()
        self._last_played = {}  # label -> monotonic timestamp of last playback

        self.sub = self.create_subscription(Detection2DArray, topic, self._on_detections, 10)
        self.get_logger().info(
            f'cat_voice started. topic={topic} voices_dir={self.voices_dir} '
            f'files={ {k: len(v) for k, v in self._files_by_label.items()} }')

    def _index_voice_files(self):
        files_by_label = {label: [] for label in PREFIX_BY_LABEL}
        if not os.path.isdir(self.voices_dir):
            self.get_logger().warning(f'voices_dir does not exist: {self.voices_dir}')
            return files_by_label
        for fn in sorted(os.listdir(self.voices_dir)):
            if not fn.lower().endswith('.mp3'):
                continue
            for label, prefix in PREFIX_BY_LABEL.items():
                if fn.startswith(prefix):
                    files_by_label[label].append(os.path.join(self.voices_dir, fn))
        for label, files in files_by_label.items():
            if not files:
                self.get_logger().warning(
                    f'No mp3 files matching "{PREFIX_BY_LABEL[label]}*" in {self.voices_dir}')
        return files_by_label

    def _on_detections(self, msg: Detection2DArray):
        # Best (highest-confidence) identity across all boxes in this frame.
        best_label, best_conf = None, -1.0
        for det in msg.detections:
            if len(det.results) < 2:
                continue  # identification didn't fire for this box
            id_hyp = det.results[1].hypothesis
            if id_hyp.class_id in PREFIX_BY_LABEL and id_hyp.score > best_conf:
                best_label, best_conf = id_hyp.class_id, id_hyp.score

        if best_label is None or best_conf < self.min_conf:
            return

        now = time.monotonic()
        last = self._last_played.get(best_label, 0.0)
        if now - last < self.cooldown_sec:
            return

        files = self._files_by_label.get(best_label, [])
        if not files:
            return

        self._last_played[best_label] = now
        path = random.choice(files)
        self.get_logger().info(f'Recognized {best_label} ({best_conf:.2f}) -> playing {path}')
        self._play(path)

    @staticmethod
    def _play(path):
        # ffmpeg decodes mp3 -> WAV on stdout; paplay reads that WAV from
        # stdin and plays it on PulseAudio's current default sink.
        decode = subprocess.Popen(
            ['ffmpeg', '-loglevel', 'error', '-i', path, '-f', 'wav', '-'],
            stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
        subprocess.Popen(
            ['paplay'], stdin=decode.stdout,
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        decode.stdout.close()  # let `decode` receive SIGPIPE if paplay exits first


def main():
    rclpy.init()
    logger = rclpy.logging.get_logger('cat_voice')

    # Acquire the singleton lock BEFORE building the node, so a duplicate never
    # even creates its detections subscription (no window to play a stray line).
    lock = acquire_singleton_lock()
    if lock is None:
        logger.error(
            f'another cat_voice instance already holds {_LOCK_PATH}; exiting to '
            f'avoid overlapping playback')
        rclpy.shutdown()
        return

    node = VoiceNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        lock.close()  # release the singleton lock


if __name__ == '__main__':
    main()
