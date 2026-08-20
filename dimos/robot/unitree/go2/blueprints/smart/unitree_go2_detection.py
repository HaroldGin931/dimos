#!/usr/bin/env python3
# Copyright 2025-2026 Dimensional Inc.
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

import hashlib

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.transport import LCMTransport
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.vision_msgs.Detection2DArray import Detection2DArray
from dimos.perception.detection.module3D import Detection3DModule
from dimos.perception.detection.type.detection2d.imageDetections2D import ImageDetections2D
from dimos.perception.detection.type.detection3d.imageDetections3DPC import ImageDetections3DPC
from dimos.robot.unitree.go2.blueprints.basic.unitree_go2_basic import rerun_config
from dimos.robot.unitree.go2.blueprints.smart.unitree_go2 import unitree_go2
from dimos.robot.unitree.go2.connection import GO2Connection
from dimos.utils.logging_config import setup_logger
from dimos.visualization.rerun.bridge import RerunBridgeModule, _with_graph_tab
from dimos.visualization.rerun.constants import RERUN_GRPC_PORT

logger = setup_logger()


def _topic_path(topic: object) -> str:
    topic_str = getattr(topic, "name", None) or str(topic)
    raw = getattr(topic, "topic", topic_str)
    if isinstance(raw, str):
        topic_str = raw
    topic_str = topic_str.split("#")[0]
    if topic_str.startswith("dimos/"):
        topic_str = "/" + topic_str.removeprefix("dimos/")
    elif not topic_str.startswith("/"):
        topic_str = "/" + topic_str
    return topic_str


def _detection_topic_to_entity(topic: object) -> str:
    path = _topic_path(topic)
    parts = path.strip("/").split("/")
    if len(parts) == 4 and parts[:2] == ["detector3d", "3d"]:
        return f"world/detections/3d/{parts[2]}"
    if len(parts) == 4 and parts[:2] == ["detector3d", "2d"]:
        return f"world/detections/2d/{parts[2]}"
    return f"world{path}"


detection_rerun_config = {
    **rerun_config,
    "topic_to_entity": _detection_topic_to_entity,
}


class _HashCheckingDetection3DModule(Detection3DModule):
    """Pause this demo's Rerun viewer when its 2D and 3D crop slots diverge."""

    _pause_after_publish = False
    _pause_sent = False

    @staticmethod
    def _crop_hash(image: Image) -> str:
        return hashlib.sha256(image.data.tobytes()).hexdigest()

    def process_frame(
        self,
        detections: ImageDetections2D,
        pointcloud: PointCloud2,
        transform: Transform | None,
    ) -> ImageDetections3DPC:
        detections_3d = super().process_frame(detections, pointcloud, transform)
        if self._pause_sent or self._pause_after_publish:
            return detections_3d

        for display_slot, (detection_2d, detection_3d) in enumerate(
            zip(detections[:3], detections_3d[:3], strict=False)
        ):
            hash_2d = self._crop_hash(detection_2d.cropped_image())
            hash_3d = self._crop_hash(detection_3d.cropped_image())
            if hash_2d != hash_3d:
                source_2d_slot = [d.bbox for d in detections].index(detection_3d.bbox)
                logger.error(
                    "Detection crop mismatch; pausing Rerun\n"
                    "  2D display:\n"
                    f"    slot: {display_slot}\n"
                    f"    name: {detection_2d.name}\n"
                    f"    hash: {hash_2d[:12]}...\n"
                    "  3D display:\n"
                    f"    slot: {display_slot}\n"
                    f"    name: {detection_3d.name}\n"
                    f"    hash: {hash_3d[:12]}...\n"
                    "  Source:\n"
                    f"    3D display slot {display_slot} image comes from "
                    f"2D detection slot {source_2d_slot}"
                )
                self._pause_after_publish = True
                break

        return detections_3d

    def _publish_detections(self, detections: ImageDetections3DPC) -> None:
        super()._publish_detections(detections)
        if self._pause_after_publish:
            self._pause_after_publish = False
            self._pause_sent = True
            self._pause_rerun()

    def _pause_rerun(self) -> None:
        import rerun.blueprint as rrb

        blueprint = _with_graph_tab(rerun_config["blueprint"]())
        blueprint.time_panel = rrb.TimePanel(timeline="log_time", play_state="paused")
        host = self.config.g.rerun_host or self.config.g.listen_host
        connect_url = f"rerun+http://{host}:{RERUN_GRPC_PORT}/proxy"
        blueprint.connect_grpc("dimos", url=connect_url, make_default=False)
        logger.info("Rerun pause blueprint sent", connect_url=connect_url)


unitree_go2_detection = (
    autoconnect(
        unitree_go2,
        # Replaces the RerunBridgeModule already present in unitree_go2 while
        # leaving its single vis_module bundle and websocket modules intact.
        RerunBridgeModule.blueprint(**detection_rerun_config),
        _HashCheckingDetection3DModule.blueprint(
            instance_name=Detection3DModule.name,
            camera_info=GO2Connection.camera_info_static,
        ),
    )
    .remappings(
        [
            (Detection3DModule, "pointcloud", "global_map"),
        ]
    )
    .transports(
        {
            # Detection 3D module outputs
            ("detections", Detection2DArray): LCMTransport(
                "/detector3d/detections", Detection2DArray
            ),
            ("detected_pointcloud_0", PointCloud2): LCMTransport(
                "/detector3d/3d/slot_0/pointcloud", PointCloud2
            ),
            ("detected_pointcloud_1", PointCloud2): LCMTransport(
                "/detector3d/3d/slot_1/pointcloud", PointCloud2
            ),
            ("detected_pointcloud_2", PointCloud2): LCMTransport(
                "/detector3d/3d/slot_2/pointcloud", PointCloud2
            ),
            ("detected_image_0", Image): LCMTransport("/detector3d/2d/slot_0/image", Image),
            ("detected_image_1", Image): LCMTransport("/detector3d/2d/slot_1/image", Image),
            ("detected_image_2", Image): LCMTransport("/detector3d/2d/slot_2/image", Image),
            ("detected_3d_image_0", Image): LCMTransport("/detector3d/3d/slot_0/image", Image),
            ("detected_3d_image_1", Image): LCMTransport("/detector3d/3d/slot_1/image", Image),
            ("detected_3d_image_2", Image): LCMTransport("/detector3d/3d/slot_2/image", Image),
        }
    )
)
