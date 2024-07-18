import dataclasses
from typing import Callable
import time, threading
from enum import Enum

"""
A constant state of the environment that the robot is in, which is fed into the physics engine.
Implement this class to define the environment state.
"""
@dataclasses.dataclass
class EnvironmentState:
    pass

"""
A collection of setpoints for the simulated robot's software, which will be fed into the robot's control loop.
RobotSetpointState are not affected by physics, but are set purely by simulation inputs.
Implement this class to define the setpoints for the robot.
"""
@dataclasses.dataclass
class RobotSetpointState:
    pass

"""
The physical state of the robot, which is controlled by the physics engine. It is not directly set by the
simulation inputs.
Implement this class to define the kinematic state of the robot.
"""
@dataclasses.dataclass
class RobotKinematicState:
    pass


"""
All the information stored in a single simulation step.
"""
@dataclasses.dataclass
class SimulationStep:
    timestep: float
    robot_setpoint_state: RobotSetpointState
    robot_kinematic_state: RobotKinematicState

    def __str__(self) -> str:
        return f"t={round(self.timestep, 3)}, Setpoint: {self.robot_setpoint_state}, Kinematic: {self.robot_kinematic_state}"


"""
The physics engine that will be used to simulate the robot and environment. Takes in the current
state of the environment, the current kinematic state of the robot, and the current setpoint state
of the robot. Apply physics and return the kinematic state of the robot at the next timestep.
Implement this class to create a custom physics engine.
"""
class PhysicsEngine:

    def simulate(self,
        delta_seconds: float,
        environment_state: EnvironmentState,
        robot_setpoint_state: RobotSetpointState,
        robot_kinematic_state: RobotKinematicState,
    ) -> RobotKinematicState:
        raise NotImplementedError("PhysicsEngine must implement simulate method")


class SimulationMode(Enum):
    REALTIME = "REALTIME" # The simulation runs in real-time in a separate thread
    ASYNC = "ASYNC" # The simulation runs in the same thread as the main program

"""
Runs a simulation of the robot in the environment using the physics engine.

In REALTIME mode, the simulation runs in a separate thread and simulates the robot in real-time.
In ASYNC mode, the simulation runs in the same thread as the main program and simulates the robot
as fast as possible whenever Simulation has control of the only thread.
"""
class Simulation:

    def __init__(self,
        mode: SimulationMode,
        initial_environment_state: EnvironmentState,
        initial_robot_setpoint_state: RobotSetpointState,
        initial_robot_kinematic_state: RobotKinematicState,
        physics_engine: PhysicsEngine,
        delta_seconds: float, # The number of seconds between each simulation step
        debug: bool = False
    ):
        self.mode = mode
        self.environment_state = initial_environment_state
        self.physics_engine = physics_engine
        self.debug = debug

        # After self.start_simulation() has been called and before self.stop_simulation()
        self.running = False

        # Whether self.start_simulation() has been called before
        self.has_started_running = False

        # The thread object for real-time simulation only
        self.realtime_simulation_thread = None

        # Initialize first timestep of the simulation
        self.simulation_steps = [SimulationStep(0, initial_robot_setpoint_state, initial_robot_kinematic_state)]

        # The number of milliseconds between each simulation step
        self.delta_seconds = delta_seconds

    def _execute_simulation_step(self):
        """
        Run the physics engine to simulate the next step of the robot in the environment.
        """

        # Copy the current setpoint state object for next step
        next_robot_setpoint_state = dataclasses.replace(self.current_simulation_step.robot_setpoint_state)

        # Simulate the next step
        next_robot_kinematic_state = self.physics_engine.simulate(
            self.delta_seconds,
            self.environment_state,
            self.current_simulation_step.robot_setpoint_state,
            self.current_simulation_step.robot_kinematic_state
        )

        # Add the next step to the simulation
        self.simulation_steps.append(SimulationStep(
            self.current_simulation_step.timestep + self.delta_seconds,
            next_robot_setpoint_state,
            next_robot_kinematic_state
        ))

        if self.debug:
            print(self.current_simulation_step)

    def _run_simulation_in_thread(self):
        """
        Run the simulation in a separate thread in real-time.
        Only used in REALTIME mode.
        """

        # Run the simulation in real-time
        self.start_time = time.time()
        timestep = 0
        while self.running:
            self._execute_simulation_step()
            
            # Wait until the next simulation step
            timestep += 1
            time_to_wait = self.start_time + timestep * self.delta_seconds - time.time()
            if time_to_wait > 0:
                time.sleep(time_to_wait)
            
            

    def _catch_up_simulation(self):
        """
        In the elapsed time since the last call to catch_up_simulation, execute as many simulation steps as necessary
        to catch up to the current time.
        Only used in ASYNC mode.
        """

        # Calculate the elapsed time since the last call to catch_up_simulation in seconds
        current_clock_time = time.time()
        elapsed_time = current_clock_time - self.last_clock_time

        # The number of simulation steps that need to be executed
        simulation_steps_to_execute = int(elapsed_time / self.delta_seconds)

        # The time between the last simulation step and now, which will be carried over to the next call
        time_carryover = elapsed_time % self.delta_seconds

        # Execute the simulation steps. Note that elapsed time does not advance while this loop is running.
        for _ in range(simulation_steps_to_execute):
            self._execute_simulation_step()

        # Update the last clock time. We call time.time() again to skip the time spent in the loop above.
        self.last_clock_time = time.time() - time_carryover


    def start_simulation(self):
        """
        If ASYNC, At the first catch_up_simulation call, all the steps from this point to the call will be simulated.
        If REALTIME, the simulation will start in a separate thread.
        """
        if self.has_started_running:
            raise Exception("Simulation cannot be started again after it has already started.")

        self.has_started_running = True
        self.running = True

        if self.debug:
            print(f"Starting simulation in {self.mode} mode.")

        if self.mode == SimulationMode.REALTIME:
            # Start the simulation in a separate thread
            self.realtime_simulation_thread = threading.Thread(target=self._run_simulation_in_thread)
            self.realtime_simulation_thread.start()
        else:
            self.last_clock_time = time.time()
    
    def simulation_input(self, input_function: Callable[[RobotSetpointState], None]):
        """
        Takes in a function that modifies the robot's setpoint state in-place.
        If ASYNC, executes all the previous simulation steps that would have happened before this input.
        """

        if self.mode == SimulationMode.ASYNC:
            # Catch up the simulation to the current time
            self._catch_up_simulation()

        # Modify the setpoint state
        input_function(self.current_simulation_step.robot_setpoint_state)

    def simulation_sensor(self, sensor_function: Callable[[RobotKinematicState], any]):
        """
        Takes in a function that reads the robot's kinematic state and returns a value.
        If ASYNC, executes all the previous simulation steps that would have happened before this sensor read.
        """

        if self.mode == SimulationMode.ASYNC:
            # Catch up the simulation to the current time
            self._catch_up_simulation()

        # Read the kinematic state
        return sensor_function(self.current_simulation_step.robot_kinematic_state)

    def stop_simulation(self):
        """
        Stop the simulation and return the full simulation.
        """

        self.running = False

        if self.mode == SimulationMode.ASYNC:
            # Simulate the remaining steps until the current time
            self._catch_up_simulation()
        else:
            # Wait for the simulation thread to finish
            self.realtime_simulation_thread.join()

        if self.debug:
            print("Simulation stopped.")

        return self.get_full_simulation()

    def get_full_simulation(self):
        return self.simulation_steps

    @property
    def current_simulation_step(self) -> SimulationStep:
        return self.simulation_steps[-1]
