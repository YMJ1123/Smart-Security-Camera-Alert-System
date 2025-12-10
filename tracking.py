import time

class PIDController:
    """PID ����G�t�d�p�⥭�ƪ��ץ��q"""
    def __init__(self, kp, ki, kd, output_limit=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limit = output_limit
        self.prev_error = 0.0
        self.integral = 0.0
        self.last_time = time.time()

    def compute(self, error):
        current_time = time.time()
        dt = current_time - self.last_time
        if dt <= 0: dt = 1e-16

        # P, I, D ���p��
        p_term = self.kp * error
        self.integral += error * dt
        i_term = self.ki * self.integral
        d_term = self.kd * (error - self.prev_error) / dt

        output = p_term + i_term + d_term

        # ��s���A
        self.prev_error = error
        self.last_time = current_time

        # �����X�T�� (����F������ӧ�)
        if self.output_limit:
            output = max(min(output, self.output_limit), -self.output_limit)
        
        return output

class Tracker:
    """�l�ܾ��G���X PID �P ���e���ת��A"""
    def __init__(self, width=640, height=480):
        self.center_x = width // 2
        self.center_y = height // 2
        
        # ��l���� (�]�b 90 �פ���)
        self.pan_angle = 90.0
        self.tilt_angle = 90.0

        # --- PID �Ѽ� (�o�̬O�i�H�վ㪺�a��) ---
        # Kp: �����t�� (�Ӥj�|�ݡA�Ӥp�l����)
        # Kd: ���� (����_��)
        self.pid_pan = PIDController(kp=0.06, ki=0.0, kd=0.002, output_limit=4.0)
        self.pid_tilt = PIDController(kp=0.06, ki=0.0, kd=0.002, output_limit=4.0)

        # ���� (Dead Zone)�G�~�t�p�󦹹����N���ʡA�קK���g��ݰ�
        self.dead_zone = 25

    def calculate_new_angles(self, target_x, target_y):
        """��J�ؼЮy�СA�^�Ƿs�� Servo ����"""
        if target_x is None or target_y is None:
            return self.pan_angle, self.tilt_angle

        # 1. �p��~�t
        error_x = self.center_x - target_x
        error_y = self.center_y - target_y

        # 2. ���ϹL�o
        if abs(error_x) < self.dead_zone: error_x = 0
        if abs(error_y) < self.dead_zone: error_y = 0

        # 3. PID �p��ץ��q
        delta_pan = self.pid_pan.compute(error_x)
        delta_tilt = self.pid_tilt.compute(error_y)

        # 4. ��s���� (�`�N�G�[���δ���M�󰨹F�w�ˤ�V)
        # �p�G�o�{���Ϥ�V�l�A��o�̪� + �令 - �Y�i
        self.pan_angle += delta_pan   
        self.tilt_angle -= delta_tilt # Tilt �q�`�O�ϦV��

        # 5. ����׽d�� (0~180)
        self.pan_angle = max(0, min(180, self.pan_angle))
        self.tilt_angle = max(0, min(180, self.tilt_angle))

        return self.pan_angle, self.tilt_angle