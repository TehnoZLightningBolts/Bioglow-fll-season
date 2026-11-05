from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor
from pybricks.parameters import Button, Direction, Port, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch, multitask, run_task


class MyRobot:

    def __init__(self):

        # =========================
        # HUB
        # =========================

        self.hub = PrimeHub()

        # =========================
        # DRIVE MOTORS
        # =========================

        self.left_motor = Motor(
            Port.E,
            Direction.COUNTERCLOCKWISE
        )

        self.right_motor = Motor(
            Port.A,
            Direction.CLOCKWISE
        )
        
        #WE DO NOT USE SELF.DRIVEBASE.USE_GYRO(TRUE)

        # =========================
        # DRIVEBASE
        # =========================

        self.drivebase = DriveBase(
            self.left_motor,
            self.right_motor,
            wheel_diameter=65,
            axle_track=80
        )

        # =========================
        # ATTACHMENTS
        # =========================

        self.arm_left = Motor(
            Port.B,
            Direction.CLOCKWISE
        )

        self.arm_right = Motor(
            Port.F,
            Direction.COUNTERCLOCKWISE
        )
        


        print(self.hub.battery.voltage()) # Only in practice runs
        if(self.hub.battery.voltage() <= 7500):
            print("Failed to initialize - Battery too low\n")


        # =========================
        # PID CONSTANTS
        # =========================

        self.kp = 2.0
        self.ki = 0.0
        self.kd = 0.5

        # =========================
        # MOTOR STATE
        # =========================

        self.error=0
        self.last_error=0
        self.integral=0
        self.derivative=0

    # ==================================================
    # PID LIMITS
    # ==================================================

        self.integral_limit = 5000
        self.correction_limit = 1000

    # ==================================================
    # PID SETTINGS
    # ==================================================

        self.deadzone=0.0
        self.noise_filtering=0.0


    # ==================================================
    # GYRO FILTERING
    # ==================================================

        self.filtered_heading = 0

    # ==================================================
    # MOTOR SIGNS
    # ==================================================

        self.rgsign=1
        self.lfsign=1

    # ==================================================
    # PID INITIALIZATION
    # ==================================================

    def init_pid(self):

        self.error=0
        self.last_error=0
        self.integral=0
        self.derivative=0

        self.filtered_heading = self.hub.imu.heading()

    # ==================================================
    # SET PID
    # ==================================================

    def set_pid(
        self,
        kp: float,
        ki: float,
        kd: float,
        deadzone: float = 0.0,
        noise_filtering: float = 0.0
    ) -> None:

        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.deadzone = deadzone
        self.noise_filtering = noise_filtering

    # ==================================================
    # NOISE FILTERING
    #===================================================
    def filter_gyro(self, heading: float) -> float:

       alpha = self.noise_filtering

       self.filtered_heading = (
          alpha * heading
          + (1 - alpha) * self.filtered_heading
       )

       return self.filtered_heading


    # ==================================================
    # PID CALCULATION
    # ==================================================

    def calculate_pid(
     self,
     target: float,
     current: float,
     dt: float
    ):

        self.error = target - current

        # DEADZONE
        if abs(self.error) < self.deadzone:
            self.error = 0

        self.integral += self.error * dt

        if dt > 0:
            self.derivative = (
                self.error - self.last_error
            ) / dt
        else:
            self.derivative = 0

        correction = (
            self.kp * self.error
            + self.ki * self.integral
            + self.kd * self.derivative
        )

        self.last_error = self.error

        return correction


    # ======================================================
    # STOP
    # ======================================================

    def stop_drive(self, typestop: int) -> None:

        if typestop == 0:

            self.left_motor.stop()
            self.right_motor.stop()

        elif typestop == 1:

            self.left_motor.brake()
            self.right_motor.brake()

        elif typestop == 2:

            self.left_motor.hold()
            self.right_motor.hold()

        else:

            self.left_motor.brake()
            self.right_motor.brake()

    # ======================================================
    # DRIVE
    # ======================================================        


    async def gyroforwards(
      self,
      distance: int,
      speed: int,
      typestop: int,
      lfoffset: int = 0,
      rgoffset: int = 0,
      initpid: int = 0,
      setted_speed: int = 0,
    ) -> None:

       if initpid == 1:
           self.init_pid()

       self.drivebase.reset()

    # =================================
    # TARGET HEADING
    # =================================

       target_heading = self.hub.imu.heading()

       self.filtered_heading = target_heading

    # =================================
    # TIMER
    # =================================

       timer = StopWatch()
       timer.reset()

       last_time = timer.time() / 1000

    # =================================
    # MOVE
    # =================================
       
        
       while abs(self.drivebase.distance()) < abs(distance):

           current_time = timer.time() / 1000

           dt = current_time - last_time

           last_time = current_time

           if dt <= 0:
               dt = 0.01

           raw_heading = self.hub.imu.heading()

           current_heading = self.filter_gyro(
               raw_heading
           )

           correction = self.calculate_pid(
               target_heading,
               current_heading,
               dt
           )

           remaining=distance-abs(self.drivebase.distance())

           if remaining < 100:
            if setted_speed > 0:
               current_speed = setted_speed
           else:
            current_speed=speed
 
           left_speed = (
               current_speed
               + correction
               + lfoffset
           )

           right_speed = (
               current_speed
               - correction
               + rgoffset
           )


           left_speed *= self.lfsign
           right_speed *= self.rgsign

           left_speed = max(-1100, min(1100, left_speed))
           right_speed = max(-1100, min(1100, right_speed))

           self.left_motor.run(left_speed)
           self.right_motor.run(right_speed)

     

           await wait(10)

       self.stop_drive(typestop)


    async def gyrobackwards(
        self,
        distance: int,
        speed: int,
        typestop: int,
        lfoffset: int=0,
        rgoffset: int=0,
        initpid: int = 0
    ) -> None:

        if initpid == 1:
            self.init_pid()

        self.drivebase.reset()

        target_heading = self.hub.imu.heading()

        self.filtered_heading = target_heading


        timer = StopWatch()
        timer.reset()

        last_time = timer.time() / 1000

        while abs(self.drivebase.distance()) < abs(distance):

            current_time = timer.time() / 1000
            dt = current_time - last_time
            last_time = current_time

            if dt <= 0:
                dt = 0.01


            raw_heading = self.hub.imu.heading()

            current_heading = self.filter_gyro(
               raw_heading
            )

            correction = self.calculate_pid(
                target_heading,
                current_heading,
                dt
            )

            remaining=distance-abs(self.drivebase.distance())

            if remaining < 100:
             current_speed = 300
            else:
             current_speed=speed

            left_speed = (
                -current_speed
                + correction
                + lfoffset
            )

            right_speed = (
                -current_speed
                - correction
                + rgoffset
            )


            left_speed *= self.lfsign
            right_speed *= self.rgsign

            left_speed = max(-1100 , min(1100 , left_speed))
            right_speed = max(-1100 , min(1100 , right_speed))

            self.left_motor.run(left_speed)
            self.right_motor.run(right_speed)

            await wait(10)

        self.stop_drive(typestop)

    # =====================================
    # TURN
    #======================================

    async def turnright(
       self,
       angle: int,
       speed: int,
       typestop: int,
       lfsign: int = 1,
       rgsign: int = -1,
       initpid: int = 0
    ) -> None:

        if initpid == 1:
            self.init_pid()

        start_heading = self.hub.imu.heading()
        target_heading = start_heading + angle

        timer = StopWatch()
        timer.reset()

        last_time = timer.time() / 1000

        while True:

            current_time = timer.time() / 1000
            dt = current_time - last_time
            last_time = current_time

            if dt <= 0:
              dt = 0.01

        # =========================
        # GYRO + FILTER
        # =========================

            raw_heading = self.hub.imu.heading()

            current_heading = self.filter_gyro(
                raw_heading
            )

        # =========================
        # ERROR
        # =========================

            error = target_heading - current_heading

            while error > 180:
                error -= 360

            while error < -180:
                error += 360

        # =========================
        # PID
        # =========================

            correction = self.calculate_pid(
                0,
                -error,
                dt
            )

        # =========================
        # LIMIT
        # =========================

            correction = max(
                -speed,
                min(speed, correction)
            )

        # =========================
        # MINIMUM SPEED
        # =========================

            min_speed = 15

            if abs(error) > self.deadzone:

                if abs(correction) < min_speed:
                    if correction > 0:
                        correction = min_speed
                    else:
                        correction = -min_speed

        # =========================
        # MOTORS
        # =========================

            left_speed = correction * lfsign
            right_speed = correction * rgsign

            self.left_motor.run(left_speed)
            self.right_motor.run(right_speed)

        # =========================
        # STOP
        # =========================

            if abs(error) <= self.deadzone:

                self.stop_drive(typestop)
                break

            await wait(10)


    async def turnleft(
     self,
     angle: int,
     speed: int,
     typestop: int,
     lfsign: int=1,
     rgsign: int=-1,
     initpid: int = 0
    ) -> None:

     if initpid == 1:
        self.init_pid()

     start_heading = self.hub.imu.heading()
     target_heading = start_heading - angle

     self.filtered_heading = start_heading

     timer = StopWatch()
     timer.reset()

     last_time = timer.time() / 1000

     while True:

        current_time = timer.time() / 1000
        dt = current_time - last_time
        last_time = current_time

        if dt <= 0:
            dt = 0.01

        raw_heading = self.hub.imu.heading()

        current_heading = self.filter_gyro(raw_heading)

        error = target_heading - current_heading

        while error > 180:
            error -= 360

        while error < -180:
            error += 360

        correction = self.calculate_pid(
            0,
            -error,
            dt
        )

        if correction > speed:
            correction = speed

        if correction < -speed:
            correction = -speed

        left_speed = correction * lfsign
        right_speed = correction * rgsign

        self.left_motor.run(left_speed)
        self.right_motor.run(right_speed)

        if abs(error) < 1:
            self.stop_drive(typestop)
            break

        await wait(10)
    

    async def curve(
        self,
        distance: int,
        radius: int,
        speed: int,
        typestop: int,
        direction: int,
        initpid: int = 0
    ) -> None:

        if radius <= 40:
            return

        if direction != 1 and direction != -1:
            return

        if initpid == 1:
            self.init_pid()

        self.drivebase.reset()

        # =========================
        # START HEADING
        # =========================

        start_heading = self.hub.imu.heading()

        self.filtered_heading = start_heading

        # =========================
        # TOTAL ANGLE
        # =========================
        #
        # angle = distance / radius
        #
        # rad -> deg
        # =========================

        total_angle = (
            distance / radius
        ) * 180 / 3.14159265

        total_angle *= direction

        # =========================
        # WHEEL SPEEDS
        # =========================
        #
        # axle_track = 80 mm
        # half axle_track = 40 mm
        # =========================

        if direction == 1:

            outer_speed = speed

            inner_speed = (
                speed
                * (radius - 40)
                / (radius + 40)
            )

            left_base = outer_speed
            right_base = inner_speed

        else:

            outer_speed = speed

            inner_speed = (
                speed
                * (radius - 40)
                / (radius + 40)
            )

            left_base = inner_speed
            right_base = outer_speed

        # =========================
        # TIMER
        # =========================

        timer = StopWatch()
        timer.reset()

        last_time = timer.time() / 1000

        # =========================
        # CURVE
        # =========================

        while abs(self.drivebase.distance()) < abs(distance):

            # =========================
            # TIME
            # =========================

            current_time = timer.time() / 1000

            dt = current_time - last_time

            last_time = current_time

            if dt <= 0:
                dt = 0.01

            # =========================
            # DISTANCE
            # =========================

            traveled = abs(
                self.drivebase.distance()
            )

            # =========================
            # PROGRESS
            # =========================

            progress = traveled / distance

            if progress > 1:
                progress = 1

            # =========================
            # TARGET HEADING
            # =========================
            #
            # Heading-ul se schimbă
            # progresiv pe curbă.
            # =========================

            target_heading = (
                start_heading
                + total_angle * progress
            )

            # =========================
            # GYRO FILTER
            # =========================

            raw_heading = self.hub.imu.heading()

            current_heading = self.filter_gyro(
                raw_heading
            )

            # =========================
            # HEADING ERROR
            # =========================

            error = (
                target_heading
                - current_heading
            )

            while error > 180:
                error -= 360

            while error < -180:
                error += 360

            # =========================
            # PID
            # =========================

            correction = self.calculate_pid(
                0,
                -error,
                dt
            )

            # =========================
            # MOTOR SPEEDS
            # =========================

            left_speed = (
                left_base
                + correction
            )

            right_speed = (
                right_base
                - correction
            )

            # =========================
            # SIGNS
            # =========================

            left_speed *= self.lfsign
            right_speed *= self.rgsign

            # =========================
            # LIMIT
            # =========================

            left_speed = max(
                -1100,
                min(1100, left_speed)
            )

            right_speed = max(
                -1100,
                min(1100, right_speed)
            )

            # =========================
            # MOTORS
            # =========================

            self.left_motor.run(left_speed)
            self.right_motor.run(right_speed)

            await wait(10)

        # =========================
        # STOP
        # =========================

        self.stop_drive(typestop)



    # ==================================================
    # ATTACHMENTS
    # ==================================================

    async def move_leftsys_motor(
        self,
        angle: int,
        speed: int,
        typestop: int
    ) -> None:

        if typestop == 0:
            stop_type = Stop.COAST

        elif typestop == 1:
            stop_type = Stop.BRAKE

        else:
            stop_type = Stop.HOLD

        await self.arm_left.run_angle(
            speed,
            angle,
            then=stop_type
        )

    async def move_rightsys_motor(
        self,
        angle: int,
        speed: int,
        typestop: int
    ) -> None:

        if typestop == 0:
            stop_type = Stop.COAST

        elif typestop == 1:
            stop_type = Stop.BRAKE

        else:
            stop_type = Stop.HOLD

        await self.arm_right.run_angle(
            speed,
            angle,
            then=stop_type
        )

    # ======================================================
    # RESET
    # ======================================================

    def reset_attachments(self)->None:

        self.arm_left.reset_angle(0)
        self.arm_right.reset_angle(0)


    def reset_drive(self) -> None:

        self.left_motor.stop()
        self.right_motor.stop()

        self.drivebase.reset()

        self.init_pid()


# ======================================================
# ROBOT
# ======================================================

robot = MyRobot()


# ======================================================
# RUNS
# ======================================================

async def run1():

    robot.set_pid(3.0, 0.0, 0.0, 0.5, 0.4)

    await robot.arm_left.run_angle(1000, 800)


    await robot.gyroforwards(1020, 1100, typestop=1, setted_speed=700); await wait(200)
    await robot.arm_right.run_angle(1100, 700)
    await robot.arm_right.run_angle(-1100, 600); await wait(200);
    await robot.gyrobackwards(1100, 1100, typestop=1)

    #await robot.gyrobackwards(1000, 1100, typestop=1)

    


    robot.stop_drive(1)
    



async def run2():
     robot.set_pid(4, 0, 0, 0.5, 0.4)

     await robot.gyroforwards(550, 800, 1);await wait(200)
     await robot.arm_right.run_angle(900, -200);await wait(200)
     await multitask(
      robot.gyrobackwards(150, 600, typestop=1),
      robot.arm_right.run_angle(800, -200)
     )
     await robot.gyrobackwards(600, 1100, typestop=1)

    
     robot.stop_drive(1)




async def run3():


     robot.stop_drive(1)




async def run4():

    robot.set_pid(7.0, 0.0, 2, 0.5, 1);await wait(200)

    await robot.gyroforwards(1250, 1000, typestop=1, rgoffset=30, setted_speed=500)
    await multitask(
        robot.arm_right.run_angle(700, 350),
        robot.arm_left.run_angle(200, 100)
    )
    await robot.gyrobackwards(1300, 1100, typestop=1)

    robot.stop_drive(1)



async def run5():

    robot.set_pid(8, 0, 0.5, 0.3, 1);await wait(200)

    await robot.gyroforwards(900, 1100, 1, setted_speed=500);await wait(200)
    await robot.turnleft(90, 50, typestop=1);await wait(200)
    await robot.gyroforwards(300, 900, typestop=1, setted_speed=300)

    robot.stop_drive(1)



async def run6():


    robot.stop_drive(1)



async def run7():


    robot.stop_drive(1)



async def run8():
    robot.set_pid(3.0, 0.0, 0.0, 0.5, 1);await wait(200)
    await robot.gyroforwards(600, 1100, typestop=1)
    await multitask(
        robot.arm_right.run_angle(-900, 900),
        robot.arm_left.run_angle(900, 900)
    )
    await robot.gyrobackwards(600, 1100,typestop=1) 

    robot.stop_drive(1)




async def run9():
    
    while Button.BLUETOOTH not in robot.hub.buttons.pressed():

        await robot.gyroforwards(1100, 1100, typestop=1)

        await wait(10)

    # Bluetooth apăsat

    robot.stop_drive(1)


runs = [
    run1,
    run2,
    run3,
    run4,
    run5,
    run6,
    run7,
    run8,
    run9
]


# ======================================================
# MENU
# ======================================================

async def menu():

    selected = 0

    while True:

        robot.hub.display.number(selected + 1)

        buttons = robot.hub.buttons.pressed()

        if Button.RIGHT in buttons:

            selected = (selected + 1) % len(runs)

            while Button.RIGHT in robot.hub.buttons.pressed():
                await wait(10)

        elif Button.LEFT in buttons:

            selected = (selected - 1) % len(runs)

            while Button.LEFT in robot.hub.buttons.pressed():
                await wait(10)

        elif Button.CENTER in buttons:

            await wait(300)

            await runs[selected]()

            while Button.CENTER in robot.hub.buttons.pressed():
                await wait(10)


# ======================================================
# START
# ======================================================

robot.hub.system.set_stop_button(Button.BLUETOOTH)

run_task(menu())
