"""
Drive the followed person model through a fixed demo scenario.

Publishes
---------
/person/cmd_vel : geometry_msgs/Twist
    Consumed by the VelocityControl plugin attached to the person model.

Scenario: walk a longer three-sided ground route -> climb the ramp -> sudden
sidestep (to test recovery when the target leaves the camera frame) -> stop.

The scenario is identical on every run so that the live demo, the backup video
and the plots in the report all come from the same motion.

Owner: Ahmet Melih ÜSTÜNER
"""

from dataclasses import dataclass
import math
import signal
from typing import Mapping, Sequence

from geometry_msgs.msg import Twist
from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions


@dataclass(frozen=True)
class MotionPhase:
    """One timed body-frame command in the demo."""

    name: str
    duration: float
    linear_x: float = 0.0
    linear_y: float = 0.0
    angular_z: float = 0.0


DEFAULTS = {
    'publish_rate': 20.0,
    'start_delay': 2.0,
    'linear_speed': 0.3,
    'turn_speed': math.pi / 6.0,
    'sidestep_speed': 0.3,
    'straight_duration': 12.0,
    'turn_duration': 3.0,
    'approach_duration': 10.0,
    'cross_duration': 22.0,
    'ramp_duration': 14.0,
    'sidestep_duration': 3.0,
}
STOPPED = MotionPhase('stopped', 0.0)


def build_phases(parameters: Mapping[str, float]) -> tuple[MotionPhase, ...]:
    """Validate startup settings and build the non-repeating sequence."""
    for name in DEFAULTS:
        value = parameters[name]
        if not math.isfinite(value) or value < 0.0:
            raise ValueError(f'{name} must be finite and non-negative')
    if parameters['publish_rate'] <= 0.0:
        raise ValueError('publish_rate must be positive')
    return (
        MotionPhase('start_delay', parameters['start_delay']),
        MotionPhase('straight', parameters['straight_duration'],
                    linear_x=parameters['linear_speed']),
        MotionPhase('first_turn', parameters['turn_duration'],
                    angular_z=parameters['turn_speed']),
        MotionPhase('approach', parameters['approach_duration'],
                    linear_x=parameters['linear_speed']),
        MotionPhase('second_turn', parameters['turn_duration'],
                    angular_z=parameters['turn_speed']),
        MotionPhase('cross', parameters['cross_duration'],
                    linear_x=parameters['linear_speed']),
        MotionPhase('third_turn', parameters['turn_duration'],
                    angular_z=parameters['turn_speed']),
        MotionPhase('ramp_leg', parameters['ramp_duration'],
                    linear_x=parameters['linear_speed']),
        MotionPhase('sidestep', parameters['sidestep_duration'],
                    linear_y=parameters['sidestep_speed']),
    )


def phase_at(elapsed_ns: int, phases: Sequence[MotionPhase]) -> MotionPhase:
    """Select a phase at nanosecond precision, then hold stop indefinitely."""
    end_ns = 0
    for phase in phases:
        end_ns += round(phase.duration * 1_000_000_000)
        if elapsed_ns < end_ns:
            return phase
    return STOPPED


class PersonWalker(Node):
    """Publish the fixed demo using ROS time and a steady command heartbeat."""

    def __init__(self):
        """Read immutable motion settings and wait for the clock and receiver."""
        super().__init__('person_walker')
        settings = {
            name: float(self.declare_parameter(
                name, default, ParameterDescriptor(read_only=True)).value)
            for name, default in DEFAULTS.items()
        }
        self._phases = build_phases(settings)
        self._publisher = self.create_publisher(Twist, '/person/cmd_vel', 10)
        self._epoch_ns: int | None = None
        self._last_time_ns: int | None = None
        self._phase_name: str | None = None
        # Keep sending commands even while Gazebo is paused. Only ROS time
        # advances the sequence, so pausing does not consume a motion phase.
        self._heartbeat_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self._timer = self.create_timer(
            1.0 / settings['publish_rate'], self._tick,
            clock=self._heartbeat_clock)
        self.get_logger().info('Waiting for a clock and /person/cmd_vel receiver')

    def _tick(self) -> None:
        """Publish the current phase, restarting after simulation time rewinds."""
        now_ns = self.get_clock().now().nanoseconds
        if self._last_time_ns is not None and now_ns < self._last_time_ns:
            self._epoch_ns = None
            self._phase_name = None
            self.get_logger().info('Clock moved backwards; restarting the demo')
        self._last_time_ns = now_ns
        if self.get_parameter('use_sim_time').value and now_ns == 0:
            self._publisher.publish(Twist())
            return
        if self._epoch_ns is None:
            if self._publisher.get_subscription_count() == 0:
                self._publisher.publish(Twist())
                return
            self._epoch_ns = now_ns
        phase = phase_at(now_ns - self._epoch_ns, self._phases)
        if phase.name != self._phase_name:
            self._phase_name = phase.name
            self.get_logger().info(f'Demo phase: {phase.name}')
        command = Twist()
        command.linear.x = phase.linear_x
        command.linear.y = phase.linear_y
        command.angular.z = phase.angular_z
        self._publisher.publish(command)

    def stop(self) -> None:
        """Cancel the timer and send zero velocity before context shutdown."""
        self._timer.cancel()
        self._publisher.publish(Twist())
        self._publisher.wait_for_all_acked(Duration(seconds=0.5))


def _interrupt(signum, frame) -> None:
    """Leave spin before shutting down ROS so the final stop can be sent."""
    raise KeyboardInterrupt


def main(args=None):
    """Run the scenario and send a stop on Ctrl+C or SIGTERM."""
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    previous_handlers = {
        sig: signal.signal(sig, _interrupt)
        for sig in (signal.SIGINT, signal.SIGTERM)
    }
    node = None
    try:
        node = PersonWalker()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if node is not None:
                if rclpy.ok():
                    node.stop()
                node.destroy_node()
        finally:
            rclpy.try_shutdown()
            for sig, handler in previous_handlers.items():
                signal.signal(sig, handler)


if __name__ == '__main__':
    main()
