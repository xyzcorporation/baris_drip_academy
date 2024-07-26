import time

import rclpy as rp
from datetime import datetime
from robot_system.DIODispenser import DIODispenser
from library.Constants import Constants, ResponseCode, DispenseCommand


from message.srv import DispenseService
from rclpy.node import Node
from rclpy.qos import QoSProfile


class DispenserNode(Node):
    """
        커피 DP, water DP 노드 클래스
    """

    # 서비스 체크 시간
    CHECK_INTERVAL_SEC = 0.1

    def __init__(self):

        super().__init__(Constants.DISPENSER_NODE)

        # set qos profile
        qos_profile = QoSProfile(depth=Constants.QOS_DEFAULT)

        # 아카데미 서비스
        self.xyz_dispenser_service = self.create_service(DispenseService, 'XYZ_dispenser/service',
                                                         self.xyz_callback_service)

        # 디스펜서 생성
        self.dispenser = DIODispenser()

    def xyz_callback_service(self, request, response):

        if request.command == DispenseCommand.COFFEE_ON:
            response = self.dispenser.dispense(DispenseCommand.COFFEE_ON)
            time.sleep(0.05)
            response = self.dispenser.dispense(DispenseCommand.COFFEE_PIN_RESET)

            time.sleep(0.5)

            response = self.dispenser.dispense(DispenseCommand.COFFEE_OFF)
            time.sleep(0.05)
            response = self.dispenser.dispense(DispenseCommand.COFFEE_PIN_RESET)

        elif request.command == DispenseCommand.WATER_TOGGLE:
            response = self.dispenser.dispense(DispenseCommand.WATER_TOGGLE)
            time.sleep(0.05)
            response = self.dispenser.dispense(DispenseCommand.WATER_PIN_RESET)

            time.sleep(9.0)

            response = self.dispenser.dispense(DispenseCommand.WATER_TOGGLE)
            time.sleep(0.05)
            response = self.dispenser.dispense(DispenseCommand.WATER_PIN_RESET)
        else:
            response = DispenseService.Response()
            response.seq_no = ''
            response.response_cd = 9999
            

        return response


def main(args=None):

    rp.init(args=args)

    dispenserNode = DispenserNode()
    rp.spin(dispenserNode)

    dispenserNode.destroy_node()
    rp.shutdown()


if __name__ == '__main__':
    main()
