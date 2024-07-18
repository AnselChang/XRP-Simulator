from src.simulator import EnvironmentState, PhysicsEngine, RobotKinematicState, RobotSetpointState, Simulation

from dataclasses import dataclass
import time


@dataclass
class TestEnvironmentState(EnvironmentState):
    some_value: int

@dataclass
class TestRobotSetpointState(RobotSetpointState):
    delta: int

@dataclass
class TestRobotKinematicState(RobotKinematicState):
    value: int


class TestPhysicsEngine(PhysicsEngine):
    def simulate(self,
        delta_seconds: float,
        environment_state: TestEnvironmentState,
        robot_setpoint_state: TestRobotSetpointState,
        robot_kinematic_state: TestRobotKinematicState,
    ) -> TestRobotKinematicState:
        
        # Calculate the new kinematic state
        new_kinematic_state = TestRobotKinematicState(
            value=robot_kinematic_state.value + robot_setpoint_state.delta * delta_seconds
        )

        # Return the new kinematic state
        return new_kinematic_state


simulation = Simulation(
    initial_environment_state=TestEnvironmentState(0),
    initial_robot_setpoint_state=TestRobotSetpointState(0),
    initial_robot_kinematic_state=TestRobotKinematicState(0),
    physics_engine=TestPhysicsEngine(),
    delta_seconds=0.2
)

def set_delta(state: TestRobotSetpointState, delta: int):
    state.delta = delta
 
def get_value(state: TestRobotKinematicState):
    return state.value

simulation.start_simulation()
time.sleep(2)
simulation.simulation_input(lambda state: set_delta(state, 2))
simulation.simulation_input(lambda state: set_delta(state, 4))
simulation.simulation_input(lambda state: set_delta(state, 5))
simulation.simulation_input(lambda state: set_delta(state, 2))
time.sleep(1)
sensor_value = simulation.simulation_sensor(lambda state: get_value(state))
time.sleep(1)
simulation.simulation_input(lambda state: set_delta(state, -1))
time.sleep(1)
simulation.stop_simulation()

for step in simulation.get_full_simulation():
    print(step)

print(sensor_value)



