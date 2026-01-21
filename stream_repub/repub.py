import asyncio
import logging
import os
import threading
import uuid
import aiohttp
import cv2
import yaml
from aiohttp import ClientSession
from aiortc import MediaStreamTrack, RTCPeerConnection, RTCSessionDescription
from aiortc.contrib.media import MediaRelay

import rclpy
from cv_bridge import CvBridge
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo

ROOT = os.path.dirname(__file__)

logger = logging.getLogger("pc")
pcs = set()
relay = MediaRelay()


class ImageRepub(Node):
    def __init__(self):
        super().__init__("image_republisher")
        self.img_pub_ = self.create_publisher(Image, "image_raw", 10)
        self.cam_pub_ = self.create_publisher(CameraInfo, "camera_info", 10)
        self.declare_parameter("stereo", "left")
        stereo = self.get_parameter("stereo").get_parameter_value().string_value
        self.stereo = stereo
        try:
            with open(f"{ROOT}/camera_info/{stereo}.yaml", "r") as fp:
                d = yaml.load(fp, Loader=yaml.FullLoader)
                self.camera_info = load_caminfo(d)
        except IOError as _e:
            self.camera_info = CameraInfo()

        # timer_period = 0.05  # seconds
        # self.timer = self.create_timer(timer_period, self.timer_callback)
        # self.i = 0

    def pub_ndarray(self, img: np.ndarray, encoding="bgr8"):
        img_msg = CvBridge().cv2_to_imgmsg(img, encoding=encoding)
        img_msg.header.stamp = self.get_clock().now().to_msg()
        img_msg.header.frame_id = f"stereo_{self.stereo}_rgb_link"

        self.camera_info.header.stamp = img_msg.header.stamp
        self.camera_info.header.frame_id = img_msg.header.frame_id

        self.cam_pub_.publish(self.camera_info)
        self.img_pub_.publish(img_msg)

    async def handle_video_track(self, track: MediaStreamTrack):
        while True:
            try:
                frame = await track.recv()
                # Process the frame (e.g., convert to OpenCV format)
                if frame.key_frame:
                    # logger.info("Key frame received")
                    pass
                img = frame.to_ndarray(format="bgr24")
                self.pub_ndarray(img)
                # cv2.imshow("Received Video", img)
                # if cv2.waitKey(1) & 0xFF == ord("q"):
                #     break
            except Exception as e:
                logger.error(f"Error receiving video frame: {e}")
                break
        cv2.destroyAllWindows()

    async def rtc_read(self):
        url = f"http://192.168.11.3:1985/rtc/v1/whep/?app=sensor/camera/stereo_{self.stereo}&stream=image"
        pc = RTCPeerConnection()
        pc.addTransceiver("video", direction="recvonly")
        pc.addTransceiver("audio", direction="recvonly")
        pc_id = "PeerConnection(%s)" % uuid.uuid4()
        pcs.add(pc)

        def log_info(msg, *args):
            logger.info(pc_id + " " + msg, *args)

        offer = await pc.createOffer()
        # connector = aiohttp.TCPConnector(limit=1, ssl=False, force_close=True)
        # logger.info(f"{offer.sdp}")
        # log_info(f"url: {url}")
        async with ClientSession() as session:
            try:
                async with session.post(
                    # url="http://localhost:8080/offer",
                    url,
                    # json={"sdp": offer.sdp, "type":"offer"},
                    data=offer.sdp,
                    headers={"Content-Type": "application/sdp"},
                    # headers={"Content-Type": "application/json"},
                    # params={"app": "sensor/camera/fisheye_left", "stream": "image"},
                ) as resp:
                    sdp = await resp.text()
                    # r = await resp.json()
                    # sdp = r["sdp"]
            except aiohttp.ServerDisconnectedError as e:
                logger.error(f"Server disconnected: {e}")
                return
            except Exception as e:
                logger.error(f"Failed to get SDP from {url}: {e}")
                return
        answer = RTCSessionDescription(sdp=sdp, type="answer")
        await pc.setLocalDescription(offer)

        @pc.on("connectionstatechange")
        async def on_connectionstatechange():
            log_info("Connection state is %s", pc.connectionState)
            if pc.connectionState == "connected":
                log_info("Connection established")
            if pc.connectionState == "failed":
                await pc.close()
                pcs.discard(pc)
                cv2.destroyAllWindows()

        @pc.on("track")
        async def on_track(track: MediaStreamTrack):
            log_info("Track %s received", track.kind)
            if track.kind == "video":
                rl = relay.subscribe(track)
                await self.handle_video_track(rl)
                # t = CVTrack(rl)

            @track.on("ended")
            async def on_ended():
                log_info("Track %s ended", track.kind)

        await pc.setRemoteDescription(answer)
        # rclpy.spin(self)
        # await asyncio.sleep(30000)
        await asyncio.Event().wait()


def load_caminfo(cam_dict: dict) -> CameraInfo:
    cam_info = CameraInfo()
    cam_info.width = cam_dict["image_width"]
    cam_info.height = cam_dict["image_height"]
    cam_info.distortion_model = cam_dict["distortion_model"]
    cam_info.d = cam_dict["distortion_coefficients"]["data"]
    cam_info.k = cam_dict["camera_matrix"]["data"]
    cam_info.r = cam_dict["rectification_matrix"]["data"]
    cam_info.p = cam_dict["projection_matrix"]["data"]
    return cam_info


async def on_shutdown(app):
    # close peer connections
    coros = [pc.close() for pc in pcs]
    await asyncio.gather(*coros)
    pcs.clear()


# def rosnode(node: Node):
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()


def main(args=None, verbose=False):
    if verbose:
        logging.basicConfig(level=logging.DEBUG)
    else:
        logging.basicConfig(level=logging.INFO)

    rclpy.init(args=args)
    img_repub = ImageRepub()
    asyncio.run(img_repub.rtc_read())
    # rtc_thread = threading.Thread(target=asyncio.run, args=(img_repub.rtc_read(url),))
    # rtc_thread.start()
    # rcl_thread = threading.Thread(target=rosnode, args=(img_repub,))
    # rcl_thread.start()
    # rclpy.spin(img_repub)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    img_repub.destroy_node()
    # rtc_thread.join()
    # rcl_thread.join()
    rclpy.shutdown()
