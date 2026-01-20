import asyncio
import logging
import os
import threading
import uuid
import aiohttp
import cv2
from aiohttp import ClientSession
from aiortc import MediaStreamTrack, RTCPeerConnection, RTCSessionDescription
from aiortc.contrib.media import MediaRelay

import rclpy
from cv_bridge import CvBridge
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Image

ROOT = os.path.dirname(__file__)

logger = logging.getLogger("pc")
pcs = set()
relay = MediaRelay()


class ImageRepub(Node):
    def __init__(self):
        super().__init__("image_republisher")
        self.publisher_ = self.create_publisher(Image, "image_raw", 10)
        # timer_period = 0.05  # seconds
        # self.timer = self.create_timer(timer_period, self.timer_callback)
        # self.i = 0

    def pub_ndarray(self, img: np.ndarray, encoding="bgr24"):
        msg = CvBridge().cv2_to_imgmsg(img, encoding="passthrough")
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "stream_converted"
        self.publisher_.publish(msg)

    async def handle_video_track(self, track: MediaStreamTrack):
        while True:
            try:
                frame = await track.recv()
                # Process the frame (e.g., convert to OpenCV format)
                if frame.key_frame:
                    logger.info("Key frame received")
                img = frame.to_ndarray(format="bgr24")
                self.pub_ndarray(img)
                # cv2.imshow("Received Video", img)
                # if cv2.waitKey(1) & 0xFF == ord("q"):
                #     break
            except Exception as e:
                logger.error(f"Error receiving video frame: {e}")
                break
        cv2.destroyAllWindows()

    async def rtc_read(self, url: str):
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


async def on_shutdown(app):
    # close peer connections
    coros = [pc.close() for pc in pcs]
    await asyncio.gather(*coros)
    pcs.clear()

# def rosnode(node: Node):
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

def main(
    args=None,
    url="http://192.168.11.3:1985/rtc/v1/whep/?app=sensor/camera/stereo_left&stream=image",
    verbose=False,
):
    if verbose:
        logging.basicConfig(level=logging.DEBUG)
    else:
        logging.basicConfig(level=logging.INFO)

    rclpy.init(args=args)
    img_repub = ImageRepub()
    asyncio.run(img_repub.rtc_read(url))
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
