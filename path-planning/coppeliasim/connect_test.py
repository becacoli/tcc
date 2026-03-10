from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import time

HOST = "localhost"
PORT = 23000
ROBOT_PATH = "/PioneerP3DX"
LEFT_MOTOR_PATH = "/PioneerP3DX/leftMotor"
RIGHT_MOTOR_PATH = "/PioneerP3DX/rightMotor"


def main():
    client = RemoteAPIClient(HOST, PORT)
    sim = client.getObject("sim")

    robot = sim.getObject(ROBOT_PATH)
    left_motor = sim.getObject(LEFT_MOTOR_PATH)
    right_motor = sim.getObject(RIGHT_MOTOR_PATH)

    pos = sim.getObjectPosition(robot, -1)
    print(f"Connected to CoppeliaSim on {HOST}:{PORT}")
    print(f"Robot handle: {robot}")
    print(f"Robot position: x={pos[0]:.3f}, y={pos[1]:.3f}, z={pos[2]:.3f}")

    sim.setJointTargetVelocity(left_motor, 2.0)
    sim.setJointTargetVelocity(right_motor, 2.0)
    time.sleep(2.0)
    sim.setJointTargetVelocity(left_motor, 0.0)
    sim.setJointTargetVelocity(right_motor, 0.0)

    print("Motor test done.")


if __name__ == "__main__":
    main()
