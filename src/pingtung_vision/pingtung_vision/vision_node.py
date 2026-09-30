"""ROS 2 node that runs exactly one selected vision algorithm."""

from __future__ import annotations

import gc
import os
import signal
import threading
import time
from typing import Optional

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rcl_interfaces.msg import SetParametersResult
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import Image
from std_msgs.msg import Header, String

from pingtung_vision.temporal_filters import (
    AnimalProbabilityFilter,
    FruitStabilityFilter,
    SpatialPercentageFilter,
    animal_id,
)
from pingtung_vision_interfaces.msg import (
    AnimalResult,
    FruitLineHit,
    PigShitSpatial,
)


VALID_MODES = {'off', 'animal', 'fruit_color', 'pig_shit'}


class VisionNode(Node):
    """Consume camera images and publish the selected task result."""

    def __init__(self) -> None:
        super().__init__('vision')
        package_share = get_package_share_directory('pingtung_vision')
        default_model = os.path.join(
            package_share, 'models', 'animal_mobilenet_v3_v2.pt'
        )

        self.declare_parameter('algorithm', 'off')
        self.declare_parameter('camera_topic', '/camera/image_raw')
        self.declare_parameter('processing_rate_hz', 10.0)
        self.declare_parameter('camera_timeout_s', 0.5)
        self.declare_parameter('publish_debug_image', False)
        self.declare_parameter('animal.model_path', default_model)
        self.declare_parameter('animal.device', 'auto')
        self.declare_parameter('animal.confidence_threshold', 0.65)
        self.declare_parameter('animal.ema_alpha', 0.25)
        self.declare_parameter('fruit.max_side', 384)
        self.declare_parameter('fruit.roi_width', 0.68)
        self.declare_parameter('fruit.roi_height', 0.74)
        self.declare_parameter('fruit.stable_frames', 4)
        self.declare_parameter('fruit.line_tolerance_px', 10)
        self.declare_parameter('fruit.centroid_ema_alpha', 0.30)
        self.declare_parameter('pig_shit.max_side', 384)
        self.declare_parameter('pig_shit.spatial_ema_alpha', 0.25)

        self._validate_startup_parameters()
        self._bridge = CvBridge()
        self._latest_lock = threading.Lock()
        self._latest_image: Optional[Image] = None
        self._last_camera_ns: Optional[int] = None
        self._camera_stale_reported = False
        self._pending_mode: Optional[str] = None
        self._mode = 'off'
        self._animal_classifier = None
        self._animal_filter: Optional[AnimalProbabilityFilter] = None
        self._fruit_filter: Optional[FruitStabilityFilter] = None
        self._spatial_filter: Optional[SpatialPercentageFilter] = None
        self._fps_average: Optional[float] = None

        result_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        active_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        camera_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._animal_pub = self.create_publisher(
            AnimalResult, '/vision/animal/result', result_qos
        )
        self._fruit_pub = self.create_publisher(
            FruitLineHit, '/vision/fruit/line_hit', result_qos
        )
        self._spatial_pub = self.create_publisher(
            PigShitSpatial, '/vision/pig_shit/spatial', result_qos
        )
        self._active_mode_pub = self.create_publisher(
            String, '/vision/active_mode', active_qos
        )
        self._debug_pub = self.create_publisher(
            Image, '/vision/debug_image', camera_qos
        )
        camera_topic = self.get_parameter('camera_topic').value
        self._image_sub = self.create_subscription(
            Image, camera_topic, self._on_image, camera_qos
        )
        processing_rate = float(self.get_parameter('processing_rate_hz').value)
        self._timer = self.create_timer(1.0 / processing_rate, self._on_timer)
        self.add_on_set_parameters_callback(self._on_set_parameters)

        self._switch_mode(str(self.get_parameter('algorithm').value))
        self.get_logger().info(
            f'Listening on {camera_topic} at up to {processing_rate:.1f} Hz'
        )

    def _validate_startup_parameters(self) -> None:
        mode = str(self.get_parameter('algorithm').value)
        if mode not in VALID_MODES:
            raise ValueError(f'algorithm must be one of {sorted(VALID_MODES)}')
        positive_parameters = (
            'processing_rate_hz',
            'camera_timeout_s',
            'animal.confidence_threshold',
            'animal.ema_alpha',
            'fruit.max_side',
            'fruit.roi_width',
            'fruit.roi_height',
            'fruit.stable_frames',
            'fruit.centroid_ema_alpha',
            'pig_shit.max_side',
            'pig_shit.spatial_ema_alpha',
        )
        for name in positive_parameters:
            if float(self.get_parameter(name).value) <= 0.0:
                raise ValueError(f'{name} must be greater than zero')
        for name in (
            'animal.confidence_threshold',
            'animal.ema_alpha',
            'fruit.roi_width',
            'fruit.roi_height',
            'fruit.centroid_ema_alpha',
            'pig_shit.spatial_ema_alpha',
        ):
            if float(self.get_parameter(name).value) > 1.0:
                raise ValueError(f'{name} must not exceed 1.0')
        if int(self.get_parameter('fruit.line_tolerance_px').value) < 0:
            raise ValueError('fruit.line_tolerance_px must not be negative')

    def _on_set_parameters(self, parameters) -> SetParametersResult:
        requested_mode = None
        for parameter in parameters:
            if parameter.name == 'algorithm':
                value = str(parameter.value)
                if value not in VALID_MODES:
                    return SetParametersResult(
                        successful=False,
                        reason=f'algorithm must be one of {sorted(VALID_MODES)}',
                    )
                requested_mode = value
            elif parameter.name == 'publish_debug_image':
                if parameter.type_ != Parameter.Type.BOOL:
                    return SetParametersResult(
                        successful=False,
                        reason='publish_debug_image must be a boolean',
                    )
            else:
                return SetParametersResult(
                    successful=False,
                    reason=(
                        'only algorithm and publish_debug_image can be changed '
                        'while running; restart the node for tuning changes'
                    ),
                )
        if requested_mode is not None:
            self._pending_mode = requested_mode
        return SetParametersResult(successful=True)

    def _on_image(self, message: Image) -> None:
        with self._latest_lock:
            self._latest_image = message
            self._last_camera_ns = self.get_clock().now().nanoseconds
        self._camera_stale_reported = False

    def _on_timer(self) -> None:
        if self._pending_mode is not None:
            pending = self._pending_mode
            self._pending_mode = None
            self._switch_mode(pending)

        if self._mode == 'off':
            return
        with self._latest_lock:
            image_message = self._latest_image
            self._latest_image = None
            last_camera_ns = self._last_camera_ns
        if image_message is None:
            self._publish_stale_if_needed(last_camera_ns)
            return

        start = time.perf_counter()
        try:
            frame = self._bridge.imgmsg_to_cv2(
                image_message, desired_encoding='bgr8'
            )
            if self._mode == 'animal':
                self._process_animal(frame, image_message.header)
            elif self._mode == 'fruit_color':
                self._process_fruit(frame, image_message.header)
            elif self._mode == 'pig_shit':
                self._process_pig_shit(frame, image_message.header)
        except Exception as error:  # Keep a bad frame from killing robot vision.
            self.get_logger().error(
                f'{self._mode} frame processing failed: {error}'
            )
            self._publish_invalid(self._mode)
            return

        elapsed = max(time.perf_counter() - start, 1e-6)
        fps_now = 1.0 / elapsed
        self._fps_average = (
            fps_now
            if self._fps_average is None
            else 0.90 * self._fps_average + 0.10 * fps_now
        )

    def _process_animal(self, frame: np.ndarray, header: Header) -> None:
        assert self._animal_classifier is not None
        assert self._animal_filter is not None
        probabilities = self._animal_filter.update(
            self._animal_classifier.predict(frame)
        )
        threshold = float(
            self.get_parameter('animal.confidence_threshold').value
        )
        result_id = animal_id(probabilities, threshold)
        message = AnimalResult()
        message.header = header
        message.valid = True
        message.animal_id = result_id
        message.dog_probability = float(probabilities[0])
        message.monkey_probability = float(probabilities[1])
        message.rabbit_probability = float(probabilities[2])
        message.turtle_probability = float(probabilities[3])
        self._animal_pub.publish(message)
        if bool(self.get_parameter('publish_debug_image').value):
            from pingtung_vision.debug_draw import draw_animal

            debug = draw_animal(
                frame, probabilities, result_id, self._fps_average or 0.0
            )
            self._publish_debug(debug, header)

    def _process_fruit(self, frame: np.ndarray, header: Header) -> None:
        from pingtung_vision.algorithms.fruit_color import classify, hit_line_y

        assert self._fruit_filter is not None
        roi_width = float(self.get_parameter('fruit.roi_width').value)
        roi_height = float(self.get_parameter('fruit.roi_height').value)
        result = classify(
            frame,
            max_side=int(self.get_parameter('fruit.max_side').value),
            roi_width=roi_width,
            roi_height=roi_height,
        )
        line_hit, centroid, streak = self._fruit_filter.update(
            result.class_id,
            result.centroid,
            result.fruit_mask.shape,
            frame.shape[:2],
            hit_line_y(frame.shape[1], frame.shape[0], roi_width, roi_height),
        )
        message = FruitLineHit()
        message.header = header
        message.valid = True
        message.line_hit = line_hit
        self._fruit_pub.publish(message)
        if bool(self.get_parameter('publish_debug_image').value):
            from pingtung_vision.debug_draw import draw_fruit

            debug = draw_fruit(
                frame,
                result,
                line_hit,
                centroid,
                streak,
                self._fruit_filter.stable_frames,
                self._fruit_filter.line_tolerance_px,
                roi_width,
                roi_height,
                self._fps_average or 0.0,
            )
            self._publish_debug(debug, header)

    def _process_pig_shit(self, frame: np.ndarray, header: Header) -> None:
        from pingtung_vision.algorithms.pig_shit import (
            classify,
            object_spatial_percentages,
        )

        assert self._spatial_filter is not None
        result = classify(
            frame,
            use_grabcut=False,
            max_side=int(self.get_parameter('pig_shit.max_side').value),
        )
        left_now, right_now = object_spatial_percentages(result)
        left, right = self._spatial_filter.update(
            result.label, left_now, right_now
        )
        message = PigShitSpatial()
        message.header = header
        message.valid = True
        message.left_pig = float(left[0])
        message.left_shit = float(left[1])
        message.left_empty = float(left[2])
        message.right_pig = float(right[0])
        message.right_shit = float(right[1])
        message.right_empty = float(right[2])
        self._spatial_pub.publish(message)
        if bool(self.get_parameter('publish_debug_image').value):
            from pingtung_vision.debug_draw import draw_pig_shit

            debug = draw_pig_shit(
                frame,
                result.label,
                result.confidence,
                left,
                right,
                self._fps_average or 0.0,
            )
            self._publish_debug(debug, header)

    def _switch_mode(self, mode: str) -> None:
        old_mode = self._mode
        if old_mode != 'off':
            self._publish_invalid(old_mode)
        with self._latest_lock:
            self._latest_image = None
        self._release_processors()
        self._fps_average = None
        try:
            if mode == 'animal':
                from pingtung_vision.algorithms.animal import AnimalClassifier

                model_path = str(self.get_parameter('animal.model_path').value)
                self.get_logger().info(f'Loading animal model: {model_path}')
                self._animal_classifier = AnimalClassifier(
                    model_path,
                    str(self.get_parameter('animal.device').value),
                )
                self._animal_filter = AnimalProbabilityFilter(
                    float(self.get_parameter('animal.ema_alpha').value)
                )
            elif mode == 'fruit_color':
                self._fruit_filter = FruitStabilityFilter(
                    stable_frames=int(
                        self.get_parameter('fruit.stable_frames').value
                    ),
                    line_tolerance_px=int(
                        self.get_parameter('fruit.line_tolerance_px').value
                    ),
                    centroid_alpha=float(
                        self.get_parameter('fruit.centroid_ema_alpha').value
                    ),
                )
            elif mode == 'pig_shit':
                self._spatial_filter = SpatialPercentageFilter(
                    float(
                        self.get_parameter('pig_shit.spatial_ema_alpha').value
                    )
                )
            self._mode = mode
        except Exception as error:
            self._mode = 'off'
            self._release_processors()
            self.get_logger().error(f'Cannot activate {mode}: {error}')

        active_message = String()
        active_message.data = self._mode
        self._active_mode_pub.publish(active_message)
        if self._mode != 'off':
            self._publish_invalid(self._mode)
        self.get_logger().info(f'Active vision algorithm: {self._mode}')

    def _release_processors(self) -> None:
        self._animal_classifier = None
        self._animal_filter = None
        self._fruit_filter = None
        self._spatial_filter = None
        gc.collect()

    def _publish_stale_if_needed(self, last_camera_ns: Optional[int]) -> None:
        if self._camera_stale_reported:
            return
        timeout_ns = int(
            float(self.get_parameter('camera_timeout_s').value) * 1e9
        )
        now_ns = self.get_clock().now().nanoseconds
        if last_camera_ns is None or now_ns - last_camera_ns > timeout_ns:
            self._publish_invalid(self._mode)
            self._camera_stale_reported = True
            self.get_logger().warning('Camera input is stale; result marked invalid')

    def _publish_invalid(self, mode: str) -> None:
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        if mode == 'animal':
            message = AnimalResult()
            message.header = header
            message.valid = False
            message.animal_id = AnimalResult.UNKNOWN
            self._animal_pub.publish(message)
        elif mode == 'fruit_color':
            message = FruitLineHit()
            message.header = header
            message.valid = False
            message.line_hit = FruitLineHit.NO_HIT
            self._fruit_pub.publish(message)
        elif mode == 'pig_shit':
            message = PigShitSpatial()
            message.header = header
            message.valid = False
            message.left_empty = 100.0
            message.right_empty = 100.0
            self._spatial_pub.publish(message)

    def _publish_debug(self, frame: np.ndarray, header: Header) -> None:
        message = self._bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        message.header = header
        self._debug_pub.publish(message)


def _hold_off_signals(deadline_s: float = 3.0) -> None:
    """Let the cleanup finish, but never let the process outlive Ctrl+C.

    Under ros2 launch, Ctrl+C reaches the node twice (terminal and launch), and
    the second KeyboardInterrupt would cut destroy_node() short. Ignore it,
    make SIGTERM kill at once, and exit hard if the cleanup hangs.
    """
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    signal.signal(signal.SIGTERM, signal.SIG_DFL)
    watchdog = threading.Timer(deadline_s, os._exit, (1,))
    watchdog.daemon = True
    watchdog.start()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VisionNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        _hold_off_signals()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
