import aimrt_py as aimrt
from modules.http_rpc import RpcClient
from protocols.pose_pb2 import PoseStamped
from algo.transforms import quat_to_mat4
import yaml

import time
import datetime
import numpy as np



class HandEyeModule(aimrt.ModuleBase):
    def __init__(self):
        super().__init__()
        # AimRT 对象
        self.core = aimrt.CoreRef()
        self.logger = aimrt.LoggerRef()
        self.work_executor = aimrt.ExecutorRef()
        self.sub_cart_pose = aimrt.SubscriberRef()


        self.rpc = RpcClient("http://127.0.0.1:50080", 30)
        self.tcp_pose = None

        self.running = False

    def Info(self) -> aimrt.ModuleInfo:
        info = aimrt.ModuleInfo()
        info.name = "HandEyeModule"
        return info

    def Initialize(self, core: aimrt.CoreRef) -> bool:
        self.core = core
        self.logger = self.core.GetLogger()

        # yaml 初始化
        file_path = self.core.GetConfigurator().GetConfigFilePath()
        with open(file_path, "r") as f:
            config = yaml.safe_load(f)
            work_executor_name = str(config["work_executor"])
            sub_cart_pose_topic = str(config["sub_cart_pose_topic"])
            
        # 创建任务执行器
        self.work_executor = self.core.GetExecutorManager().GetExecutor(work_executor_name)
        self.sub_cart_pose = self.core.GetChannelHandle().GetSubscriber(sub_cart_pose_topic)

        aimrt.Subscribe(self.sub_cart_pose, PoseStamped, self.cart_pose_callback)

        return True

    def Start(self) -> bool:
        self.running = True
        # self.work_executor.ExecuteAfter(datetime.timedelta(milliseconds=1000), self.main_loop)
        return True

    def Shutdown(self):
        try:
            self.running = False
            aimrt.info(self.logger, "HandEyeModule shutdown.")
        except Exception as e:
            aimrt.error(self.logger, f"Error in Shutdown: {e}") 

    def cart_pose_callback(self, msg):
        if not self.running:
            return
        p, q = msg.pose.position, msg.pose.orientation
        self.tcp_pose = quat_to_mat4(p.x, p.y, p.z, q.x, q.y, q.z, q.w)
