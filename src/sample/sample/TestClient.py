import rclpy as rp
from rclpy.node import Node
from rclpy.qos import QoSProfile

from library.Constants import Service, RobotCommand, RobotParameter, Command
from message.srv import RobotService
import traceback
import datetime


class SampleNode(Node):
    DRAIN_ALL = 'DRAIN_ALL'
    DRAIN_FIT = 'DRAIN_FIT'
    DRIP = 'DRIP'
    FLATTENING = 'FLATTENING'
    GESTURE = 'GESTURE'
    HOLD = 'HOLD'
    HOME_KETTLE = 'HOME_KETTLE'
    HOME_NORMAL = 'HOME_NORMAL'
    PICKUP = 'PICKUP'
    PICKUP_CUP = 'PICKUP_CUP'
    PICKUP_DRIP = 'PICKUP_DRIP'
    PLACE = 'PLACE'
    PLACE_CUP = 'PLACE_CUP'
    PLACE_DRIP = 'PLACE_DRIP'
    UNHOLD = 'UNHOLD'
    GRIPPER_INIT = 'GRIPPER_INIT'
    RUN = 'RUN'
    STOP = 'STOP'
    GET_STATUS = 'GET_STATUS'
    CLOSE = 'CLOSE'
    UNLOCK_PROTECT = 'UNLOCK_PROTECT'
    SHUTDOWN = 'SHUTDOWN'

    def __init__(self):
        super().__init__('TestSample')
        self.qos_profile = QoSProfile(depth=25)
        self.client = self.create_client(RobotService, Service.SERVICE_ROBOT, qos_profile=self.qos_profile)

        print('@@@@@@@@@@@@')

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.pickup_index = 0
        self.robot_req = None
        self.execute_node()


    def execute_node(self):
        while self.client.service_is_ready():
            srv_req = RobotService.Request()
            button = input('Button : ')
            if button == 'home':
                srv_req = self.robot_request(RobotCommand.HOME_NORMAL, RobotParameter.ZERO)
            elif button == 'gripper':
                srv_req = self.robot_request(RobotCommand.GRIPPER_INIT, RobotParameter.ZERO)
            elif button == 'pickup_dsp':
                srv_req = self.robot_request(RobotCommand.PICKUP, RobotParameter.DSP)
            elif button == 'pickup_kettle':
                srv_req = self.robot_request(RobotCommand.PICKUP, RobotParameter.KET)
            elif button == 'place_cup':
                srv_req = self.robot_request(RobotCommand.PLACE_CUP, RobotParameter.ZON)
            elif button == 'drain_fit':
                srv_req = self.robot_request(RobotCommand.DRAIN_FIT, RobotParameter.DPO)
            elif button == 'drain_all':
                srv_req = self.robot_request(RobotCommand.DRAIN_ALL, RobotParameter.ZERO)
            elif button == 'home_kettle':
                srv_req = self.robot_request(RobotCommand.HOME_KETTLE, RobotParameter.ZERO)
            elif button == 'drip':
                srv_req = self.robot_request(RobotCommand.DRIP, RobotParameter.DPO)
            elif button == 'place_kettle':
                srv_req = self.robot_request(RobotCommand.PLACE, RobotParameter.KET)
            elif button == 'pickup_cup':
                srv_req = self.robot_request(RobotCommand.PICKUP_CUP, RobotParameter.ZON)
            elif button == 'place_pic':
                srv_req = self.robot_request(RobotCommand.PLACE, RobotParameter.PIC)
            elif button == 'gesture':
                srv_req = self.robot_request(RobotCommand.GESTURE, RobotParameter.ETC)
            elif button == 'reset':
                srv_req = self.robot_request(Command.RESET, RobotParameter.ZERO)
            else:
                continue
            response = self.call_service(self.client, srv_req)
            print(f"Response {response.status_cd}, {response.response_cd}, {response.component_cd}, {response.result}")
    def call_service(self, client, request):
        try:
            future = client.call_async(request)
            rp.spin_until_future_complete(self, future)
            return future.result()
        except Exception as error:
            print(f"call_service {error=}, {type(error)=}")
            raise error

    def robot_request(self, command, param1=None, param2=None, param3=None, param4=None, param5=None):

        srv_req = RobotService.Request()
        srv_req.seq_no = str(datetime.datetime.now())
        srv_req.cmd = command

        srv_req.par1 = str(param1) if param1 else '0'
        srv_req.par2 = str(param2) if param2 else '0'
        srv_req.par3 = str(param3) if param3 else '0'
        srv_req.par4 = str(param4) if param4 else '0'
        srv_req.par5 = str(param5) if param5 else '0'

        return srv_req

def main(args=None):
    rp.init(args=args)
    sample_node = SampleNode()
    try:
        rp.spin(sample_node)
    except Exception as error:
        print(f"main error {error=}, {type(error)=}")

if __name__ == '__main__':
    main()






