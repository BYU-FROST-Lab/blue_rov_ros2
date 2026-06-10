#!/usr/bin/env python3
"""SBG status -> /diagnostics bridge.

Subscribes to the flag/status messages published by the sbg_driver and turns
selected fields into a diagnostic_msgs/DiagnosticArray on /diagnostics. What is
monitored is entirely data-driven: a standalone YAML file (path given by the
``config_file`` parameter) lists, per source topic, a set of "checks". Each
check names a field inside the message and the rule used to grade it
(boolean flag, enum value, or a numeric min/max threshold) plus the
warn/error levels.

This mirrors the existing time-sync ``topic_monitor`` node: it publishes on the
conventional absolute ``/diagnostics`` topic with names prefixed so the
diagnostic_aggregator can group them (default prefix ``sbg``) and shows up in
rqt_robot_monitor under /diagnostics_agg.

:author: Braden Meyers
"""

import importlib

import rclpy
import yaml
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.node import Node

# diagnostic_msgs level byte values keyed by name (for the config file).
LEVELS = {
    'OK': DiagnosticStatus.OK,
    'WARN': DiagnosticStatus.WARN,
    'ERROR': DiagnosticStatus.ERROR,
    'STALE': DiagnosticStatus.STALE,
}
LEVEL_NAMES = {v: k for k, v in LEVELS.items()}


def import_message_type(type_str):
    """Resolve a 'pkg/msg/Type' string to the message class."""
    pkg, _, type_name = type_str.split('/')
    module = importlib.import_module(f'{pkg}.msg')
    return getattr(module, type_name)


def get_field(msg, path):
    """Read a dotted field path (e.g. 'status.position_accuracy.x') from a msg."""
    obj = msg
    for part in path.split('.'):
        obj = getattr(obj, part)
    return obj


class SbgDiagnostics(Node):
    """Evaluates configured checks against the latest SBG messages."""

    def __init__(self):
        super().__init__('sbg_diagnostics')

        self.declare_parameter('config_file', '/home/frostlab/config/sbg_diagnostics.yaml')
        config_file = self.get_parameter('config_file').value

        with open(config_file, 'r') as f:
            config = yaml.safe_load(f)

        settings = config.get('settings', {}) or {}
        self.prefix = settings.get('diagnostic_name_prefix', 'sbg')
        self.stale_timeout = float(settings.get('stale_timeout_seconds', 2.0))
        publish_period = float(settings.get('publish_period_seconds', 1.0))

        # One entry per configured group. Each holds its definition, the latest
        # message received, and when it arrived (wall clock).
        self.groups = []
        for group in config.get('groups', []) or []:
            msg_type = import_message_type(group['type'])
            state = {
                'name': group['name'],
                'type': group['type'],
                'checks': group.get('checks', []) or [],
                'last_msg': None,
                'last_recv': None,
            }
            # Default to RELIABLE/depth 10; allow best_effort for completeness.
            depth = 10
            sub = self.create_subscription(
                msg_type,
                group['topic'],
                self._make_callback(state),
                depth,
            )
            state['sub'] = sub
            self.groups.append(state)
            self.get_logger().info(
                f"Monitoring '{group['name']}' on '{group['topic']}' "
                f"({group['type']}) with {len(state['checks'])} check(s)")

        self.pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.timer = self.create_timer(publish_period, self.publish_diagnostics)

        self.get_logger().info(
            f"sbg_diagnostics running: publishing /diagnostics every {publish_period:.2f}s "
            f"for {len(self.groups)} group(s).")

    def _make_callback(self, state):
        def callback(msg):
            state['last_msg'] = msg
            state['last_recv'] = self.get_clock().now()
        return callback

    # --- check evaluation ---------------------------------------------------

    def evaluate_check(self, msg, check):
        """Return (level, KeyValue) for a single check against a message."""
        field = check['field']
        name = check.get('name', field)
        ctype = check['type']
        try:
            raw = get_field(msg, field)
        except AttributeError as exc:
            kv = KeyValue(key=name, value=f'field error: {exc}')
            return DiagnosticStatus.ERROR, kv

        if ctype == 'bool':
            expect = bool(check.get('expect', True))
            actual = bool(raw)
            level = DiagnosticStatus.OK if actual == expect \
                else LEVELS.get(check.get('level_if_false', 'WARN'), DiagnosticStatus.WARN)
            value = f'{actual} (want {expect})'
            return level, KeyValue(key=name, value=value)

        if ctype == 'enum':
            actual = int(raw)
            label = (check.get('labels', {}) or {}).get(actual, str(actual))
            if actual in (check.get('ok_values', []) or []):
                level = DiagnosticStatus.OK
            elif actual in (check.get('warn_values', []) or []):
                level = DiagnosticStatus.WARN
            elif actual in (check.get('error_values', []) or []):
                level = DiagnosticStatus.ERROR
            else:
                level = LEVELS.get(check.get('else_level', 'ERROR'), DiagnosticStatus.ERROR)
            return level, KeyValue(key=name, value=f'{actual} ({label})')

        if ctype in ('min', 'max'):
            actual = float(raw)
            scale = float(check.get('scale', 1.0))
            scaled = actual * scale
            units = check.get('units', '')
            # uint8/uint16 'not available' sentinels (e.g. num_sv = 0xFF).
            invalid = check.get('invalid_values', []) or []
            if int(actual) in invalid:
                return DiagnosticStatus.ERROR, KeyValue(key=name, value=f'{actual} (N/A)')
            warn = check.get('warn')
            error = check.get('error')
            level = DiagnosticStatus.OK
            if ctype == 'max':
                if error is not None and scaled > error:
                    level = DiagnosticStatus.ERROR
                elif warn is not None and scaled > warn:
                    level = DiagnosticStatus.WARN
            else:  # min
                if error is not None and scaled < error:
                    level = DiagnosticStatus.ERROR
                elif warn is not None and scaled < warn:
                    level = DiagnosticStatus.WARN
            value = f'{scaled:g}{units}' if scale != 1.0 else f'{actual:g}{units}'
            return level, KeyValue(key=name, value=value)

        return DiagnosticStatus.ERROR, KeyValue(key=name, value=f'unknown check type: {ctype}')

    def build_status(self, state, now):
        """Build the DiagnosticStatus for one group from its latest message."""
        status = DiagnosticStatus()
        status.name = f'{self.prefix}: {state["name"]}'
        status.hardware_id = state['type']

        if state['last_msg'] is None:
            status.level = DiagnosticStatus.STALE
            status.message = 'No message received since startup'
            return status

        age = (now - state['last_recv']).nanoseconds * 1e-9
        if age > self.stale_timeout:
            status.level = DiagnosticStatus.STALE
            status.message = f'No message for {age:.1f}s (stale threshold {self.stale_timeout:.1f}s)'
            status.values.append(KeyValue(key='age (s)', value=f'{age:.2f}'))
            return status

        worst = DiagnosticStatus.OK
        num_warn = 0
        num_error = 0
        for check in state['checks']:
            level, kv = self.evaluate_check(state['last_msg'], check)
            kv.value = f'{kv.value} [{LEVEL_NAMES.get(level, "?")}]'
            status.values.append(kv)
            # STALE (3) sorts above ERROR numerically; clamp to ERROR for checks.
            effective = min(level, DiagnosticStatus.ERROR)
            worst = max(worst, effective)
            if level == DiagnosticStatus.WARN:
                num_warn += 1
            elif level == DiagnosticStatus.ERROR:
                num_error += 1

        status.level = worst
        if worst == DiagnosticStatus.OK:
            status.message = f'OK ({len(state["checks"])} checks)'
        else:
            status.message = f'{num_warn} warning(s), {num_error} error(s)'
        return status

    def publish_diagnostics(self):
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        now = self.get_clock().now()
        for state in self.groups:
            array.status.append(self.build_status(state, now))
        self.pub.publish(array)


def main(args=None):
    rclpy.init(args=args)
    node = SbgDiagnostics()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
