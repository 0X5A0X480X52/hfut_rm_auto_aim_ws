import serial
import threading
import time
import logging
import struct

# 配置日志
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)

# 发射状态常量
class FireState:
    NotFire = 0
    Fire = 1

class InfantryProtocolCommunicator:
    def __init__(self, port='/dev/ttyUSB0', baudrate=115200, timeout=0.1):
        """初始化步兵协议串口通信器"""
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.serial = None
        self.running = False
        self.receive_thread = None
        self.send_thread = None
        
        # 初始化串口
        self.init_serial()
        
        # 固定要发送的消息参数（可以根据需要修改）
        self.fixed_fire_advice = False  # 不发射
        self.fixed_pitch_diff = 0.0     # 俯仰角差
        self.fixed_yaw_diff = 0.0       # 偏航角差
        self.fixed_distance = 0.0       # 距离

    def init_serial(self):
        """初始化串口连接"""
        try:
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.timeout
            )
            if self.serial.is_open:
                logging.info(f"成功打开串口: {self.port}，波特率: {self.baudrate}")
                return True
            return False
        except Exception as e:
            logging.error(f"串口初始化失败: {str(e)}")
            return False

    def start(self):
        """开始串口通信"""
        if not self.serial or not self.serial.is_open:
            if not self.init_serial():
                logging.error("无法启动串口通信，串口未正确初始化")
                return
        
        self.running = True
        # 启动接收线程
        self.receive_thread = threading.Thread(target=self.receive_loop, daemon=True)
        self.receive_thread.start()
        logging.info("串口通信已启动，开始接收数据...")
        
        # 启动发送线程（定期发送固定消息）
        self.send_thread = threading.Thread(target=self.send_loop, daemon=True)
        self.send_thread.start()
        logging.info("开始发送固定消息...")

    def receive_loop(self):
        """接收并解析串口数据的循环"""
        while self.running and self.serial.is_open:
            try:
                # 读取16字节固定长度数据包
                packet = self.serial.read(16)
                if len(packet) == 16:
                    # 解析数据包（小端字节序）
                    # 格式: 1字节mode + 3个4字节float(roll, pitch, yaw) + 3字节填充
                    mode, roll, pitch, yaw = struct.unpack('<Bfffxxx', packet)
                    
                    # 打印解析结果
                    logging.info(
                        f"收到数据 - 模式: {mode}, 横滚角: {roll:.4f}rad, "
                        f"俯仰角: {pitch:.4f}rad, 偏航角: {yaw:.4f}rad"
                    )
                    logging.debug(f"原始数据包: {packet.hex()}")
                elif len(packet) > 0:
                    logging.warning(f"收到不完整数据包，长度: {len(packet)}/{16}字节")
                    
            except struct.error as e:
                logging.error(f"数据包解析错误: {str(e)}, 原始数据: {packet.hex()}")
            except Exception as e:
                logging.error(f"接收数据出错: {str(e)}")
                time.sleep(0.1)

    def send_loop(self):
        """定期发送固定消息的循环"""
        while self.running and self.serial.is_open:
            try:
                self.send_gimbal_command(
                    self.fixed_fire_advice,
                    self.fixed_pitch_diff,
                    self.fixed_yaw_diff,
                    self.fixed_distance
                )
                # 每2秒发送一次
                time.sleep(2)
            except Exception as e:
                logging.error(f"发送数据出错: {str(e)}")
                time.sleep(0.1)

    def send_gimbal_command(self, fire_advice, pitch_diff, yaw_diff, distance):
        """
        发送云台控制指令到下位机
        :param fire_advice: 布尔值，True=发射，False=不发射
        :param pitch_diff: 俯仰角差（float，弧度）
        :param yaw_diff: 偏航角差（float，弧度）
        :param distance: 目标距离（float，米）
        """
        # 处理发射状态
        fire_state = FireState.Fire if fire_advice else FireState.NotFire
        
        # 打包16字节数据包（小端字节序）
        # 格式: 1字节fire_state + 3个4字节float(pitch_diff, -yaw_diff, distance) + 3字节填充
        packet = struct.pack(
            '<Bfffxxx',  # 总长度: 1 + 4*3 + 3 = 16字节
            fire_state,
            pitch_diff,
            -yaw_diff,  # 偏航角差取负值
            distance
        )
        
        # 发送数据包
        self.serial.write(packet)
        logging.info(
            f"发送指令 - 发射: {'是' if fire_advice else '否'}, "
            f"俯仰角差: {pitch_diff:.4f}rad, 偏航角差: {yaw_diff:.4f}rad, "
            f"距离: {distance:.2f}m"
        )
        logging.debug(f"发送数据包: {packet.hex()}")

    def stop(self):
        """停止串口通信"""
        self.running = False
        if self.receive_thread:
            self.receive_thread.join()
        if self.send_thread:
            self.send_thread.join()
        if self.serial and self.serial.is_open:
            self.serial.close()
            logging.info("串口已关闭")

if __name__ == "__main__":
    # 根据实际情况修改串口参数
    communicator = InfantryProtocolCommunicator(
        port='/dev/ttyACM0',  # Windows系统可能是'COM3'等
        baudrate=115200
    )
    
    try:
        communicator.start()
        # 保持主线程运行
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        logging.info("用户中断程序")
    finally:
        communicator.stop()
    