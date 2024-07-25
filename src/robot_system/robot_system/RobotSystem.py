import traceback
from library.Constants import ResponseDataList, ResponseCode, DeviceStatus, DeviceCode, RobotCommand
from message.srv.RobotService import RobotService
import datetime

class RobotSystem:
    """
        로봇 시스템 클래스
    """

    def __init__(self):

        # 로봇 객체
        self.robot = None
        self.before_pos = "HOME"
        self.request_pos= None
    def check_path(self, request):
        pass
        return self.execute(request)
    def execute(self, request):
        response = None
        response = RobotService.Response()
        try:
            request_list = [request.cmd, request.par1, request.par2, request.par3, request.par4, request.par5]
            response.component_cd = DeviceCode.ROBOT
            self.request_pos = request.cmd
            if request.cmd == RobotCommand.HOME :
                response.status_cd = DeviceStatus.STANDBY
                response.result = self.make_str(request_list)
            elif request.cmd == RobotCommand.DRAIN_ALL:
                response.status_cd = DeviceStatus.WORKING
                response.result = self.make_str(request_list)

            elif request.cmd == RobotCommand.DRIP:
                response.status_cd = DeviceStatus.WORKING
                response.result = self.make_str(request_list)

            elif request.cmd == RobotCommand.FLATTENING:
                response.status_cd = DeviceStatus.WORKING
                response.result = self.make_str(request_list)

            elif request.cmd == RobotCommand.GESTURE:
                response.status_cd = DeviceStatus.STANDBY
                response.result = self.make_str(request_list)

            elif request.cmd == RobotCommand.HOLD:
                response.status_cd = DeviceStatus.WORKING
                response.result = self.make_str(request_list)

            elif request.cmd == RobotCommand.HOME_KETTLE:
                response.status_cd = DeviceStatus.WORKING
                response.result = self.make_str(request_list)
            elif request.cmd == RobotCommand.PLACE:
                response.status_cd = DeviceStatus.WORKING
                response.result = self.make_str(request_list)

            elif request.cmd == RobotCommand.PLACE_CUP:
                response.status_cd = DeviceStatus.WORKING
                response.result = self.make_str(request_list)

            elif request.cmd == RobotCommand.PLACE_DRIP:
                response.status_cd = DeviceStatus.WORKING
                response.result = self.make_str(request_list)

            elif request.cmd == RobotCommand.UNHOLD:
                response.status_cd = DeviceStatus.WORKING
                response.result = self.make_str(request_list)

            elif request.cmd == RobotCommand.GRIPPER_INIT:
                response.status_cd = DeviceStatus.WORKING
                response.result = self.make_str(request_list)

            elif request.cmd == RobotCommand.HOME_NORMAL:
                response.status_cd = DeviceStatus.WORKING
                response.result = self.make_str(request_list)

            elif request.cmd == RobotCommand.HOME_KETTLE:
                response.status_cd = DeviceStatus.WORKING
                response.result = self.make_str(request_list)

            elif request.cmd == RobotCommand.PICKUP:
                response.status_cd = DeviceStatus.WORKING
                response.result = self.make_str(request_list)

            elif request.cmd == RobotCommand.PICKUP_CUP:
                response.status_cd = DeviceStatus.WORKING
                response.result = self.make_str(request_list)

            elif request.cmd == RobotCommand.PICKUP_DRIP:
                response.status_cd = DeviceStatus.WORKING
                response.result = self.make_str(request_list)

            elif request.cmd == RobotCommand.PICKUP_DRIP:
                response.status_cd = DeviceStatus.WORKING
                response.result = self.make_str(request_list)

            elif request.cmd == RobotCommand.SHUTDOWN:
                response.status_cd = DeviceStatus.WORKING
                response.result = self.make_str(request_list)

        except Exception as error:
            print(f"RobotSystem execute {error=}, {type(error)=}")
            print(traceback.format_exc())

        finally :
            self.before_pos = request.cmd
            return response
        
    def make_str(self, result_list):
        result_str = ""
        split_char = '//'
        result_str = ""
        try:
            for i in range(len(result_list)):
                result_str = result_str + result_list[i]
                if i < len(result_list):
                    result_str += split_char
        except Exception as error:
            print(f"RobotSystem make_str ERROR {error}")
            result_str = "ERROR"
        finally: 
            return result_str 
        
if __name__ == '__main__':

    pass