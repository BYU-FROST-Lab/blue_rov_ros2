import os
from datetime import datetime

from ament_index_python.packages import get_package_share_directory
from python_qt_binding import loadUi
from python_qt_binding.QtCore import QTimer, Signal
from python_qt_binding.QtWidgets import QWidget
from rclpy.qos import qos_profile_sensor_data
from rqt_gui_py.plugin import Plugin

from dvl_msgs.msg import CommandResponse, ConfigCommand, ConfigStatus, DVL, DVLDR


class DVLA50Plugin(Plugin):
    # Signals bridge ROS subscriber threads to the Qt UI thread
    _config_signal = Signal(object)
    _cmd_signal = Signal(object)
    _dvl_data_signal = Signal(object)
    _dvl_pos_signal = Signal(object)

    def __init__(self, context):
        super().__init__(context)
        self.setObjectName('DVLA50Plugin')

        self._node = context.node

        # Apply-config state machine: None | 'pre_check' | 'post_check'
        self._apply_state = None
        self._pending_config = {}

        self._widget = QWidget()
        loadUi(
            os.path.join(
                get_package_share_directory('dvl_a50_rqt'), 'ui', 'dvl_a50_panel.ui'
            ),
            self._widget,
        )
        self._widget.setObjectName('DVLA50PanelUi')
        if context.serial_number() > 1:
            self._widget.setWindowTitle(
                self._widget.windowTitle() + ' (%d)' % context.serial_number()
            )
        context.add_widget(self._widget)

        # Connect ROS → Qt signals before creating subscriptions
        self._config_signal.connect(self._update_config_ui)
        self._cmd_signal.connect(self._update_cmd_response_ui)
        self._dvl_data_signal.connect(self._update_dvl_data_ui)
        self._dvl_pos_signal.connect(self._update_dvl_pos_ui)

        qos = qos_profile_sensor_data

        self._pub = self._node.create_publisher(ConfigCommand, 'dvl/config/command', qos)

        self._node.create_subscription(
            ConfigStatus, 'dvl/config/status',
            lambda m: self._config_signal.emit(m), qos)
        self._node.create_subscription(
            CommandResponse, 'dvl/command/response',
            lambda m: self._cmd_signal.emit(m), qos)
        self._node.create_subscription(
            DVL, 'dvl/data',
            lambda m: self._dvl_data_signal.emit(m), qos)
        self._node.create_subscription(
            DVLDR, 'dvl/position',
            lambda m: self._dvl_pos_signal.emit(m), qos)

        # Connect button clicks
        self._widget.get_config_btn.clicked.connect(self._get_config)
        self._widget.apply_config_btn.clicked.connect(self._apply_config)
        self._widget.calibrate_gyro_btn.clicked.connect(self._calibrate_gyro)
        self._widget.reset_dr_btn.clicked.connect(self._reset_dead_reckoning)
        self._widget.set_ntp_btn.clicked.connect(self._set_ntp)
        self._widget.get_ntp_btn.clicked.connect(self._get_ntp_config)
        self._widget.get_time_status_btn.clicked.connect(self._get_time_status)
        self._widget.force_sync_btn.clicked.connect(self._force_sync_ntp)

        self._get_config()

    # ------------------------------------------------------------------
    # Command publishers
    # ------------------------------------------------------------------

    def _get_config(self):
        msg = ConfigCommand()
        msg.command = 'get_config'
        self._pub.publish(msg)
        self._log('→ get_config')

    def _apply_config(self):
        # Snapshot desired values from UI
        self._pending_config = {
            'speed_of_sound':         self._widget.sos_spinbox.value(),
            'acoustic_enabled':       self._widget.acoustic_cb.isChecked(),
            'dark_mode_enabled':      self._widget.dark_mode_cb.isChecked(),
            'periodic_cycling_enabled': self._widget.periodic_cycling_cb.isChecked(),
            'mounting_rotation_offset': self._widget.rotation_spinbox.value(),
            'range_mode':             self._widget.range_combo.currentText(),
        }
        self._apply_state = 'pre_check'
        self._widget.apply_config_btn.setEnabled(False)
        self._log('→ get_config (pre-apply check)')
        self._get_config()

    def _calibrate_gyro(self):
        msg = ConfigCommand()
        msg.command = 'calibrate_gyro'
        self._pub.publish(msg)
        self._log('→ calibrate_gyro')

    def _reset_dead_reckoning(self):
        msg = ConfigCommand()
        msg.command = 'reset_dead_reckoning'
        self._pub.publish(msg)
        self._log('→ reset_dead_reckoning')

    def _set_ntp(self):
        address = self._widget.ntp_address_input.text().strip() or 'auto'
        msg = ConfigCommand()
        msg.command = 'set_time_ntp'
        msg.parameter_value = address
        self._pub.publish(msg)
        self._log(f'→ set_time_ntp address={address}')

    def _get_ntp_config(self):
        msg = ConfigCommand()
        msg.command = 'get_time_ntp'
        self._pub.publish(msg)
        self._log('→ get_time_ntp')

    def _get_time_status(self):
        msg = ConfigCommand()
        msg.command = 'get_time_status'
        self._pub.publish(msg)
        self._log('→ get_time_status')

    def _force_sync_ntp(self):
        timeout = self._widget.force_sync_timeout.value()
        msg = ConfigCommand()
        msg.command = 'force_sync_ntp'
        msg.parameter_value = str(timeout)
        self._pub.publish(msg)
        self._log(f'→ force_sync_ntp timeout={timeout}s')

    # ------------------------------------------------------------------
    # Apply-config state machine helpers
    # ------------------------------------------------------------------

    def _sensor_config_from_msg(self, msg):
        return {
            'speed_of_sound':           msg.speed_of_sound,
            'acoustic_enabled':         msg.acoustic_enabled,
            'dark_mode_enabled':        msg.dark_mode_enabled,
            'periodic_cycling_enabled': msg.periodic_cycling_enabled,
            # msg field is int32; compare rounded
            'mounting_rotation_offset': msg.mounting_rotation_offset,
            'range_mode':               msg.range_mode,
        }

    def _diff_config(self, current):
        """Return {param_name: desired_value} for fields that differ."""
        p = self._pending_config
        diffs = {}
        if p['speed_of_sound'] != current['speed_of_sound']:
            diffs['speed_of_sound'] = str(p['speed_of_sound'])
        if p['acoustic_enabled'] != current['acoustic_enabled']:
            diffs['acoustic_enabled'] = str(p['acoustic_enabled']).lower()
        if p['dark_mode_enabled'] != current['dark_mode_enabled']:
            diffs['dark_mode_enabled'] = str(p['dark_mode_enabled']).lower()
        if p['periodic_cycling_enabled'] != current['periodic_cycling_enabled']:
            diffs['periodic_cycling_enabled'] = str(p['periodic_cycling_enabled']).lower()
        if int(round(p['mounting_rotation_offset'])) != current['mounting_rotation_offset']:
            diffs['mounting_rotation_offset'] = str(p['mounting_rotation_offset'])
        if p['range_mode'] != current['range_mode']:
            diffs['range_mode'] = p['range_mode']
        return diffs

    def _do_apply(self, current_msg):
        """Called after pre-check get_config response."""
        self._populate_config_fields(current_msg)
        current = self._sensor_config_from_msg(current_msg)
        changes = self._diff_config(current)

        if not changes:
            self._log('← No changes needed, config already matches')
            self._apply_state = None
            self._widget.apply_config_btn.setEnabled(True)
            return

        self._log(f'→ set_config: {list(changes.keys())}')
        for name, value in changes.items():
            msg = ConfigCommand()
            msg.command = 'set_config'
            msg.parameter_name = name
            msg.parameter_value = value
            self._pub.publish(msg)

        # Wait for driver to forward all commands (100ms/cmd in driver + network margin)
        delay_ms = len(changes) * 150 + 300
        self._apply_state = 'post_check'
        QTimer.singleShot(delay_ms, self._post_check_get_config)

    def _post_check_get_config(self):
        self._log('→ get_config (post-apply verify)')
        self._get_config()

    def _verify_apply(self, result_msg):
        """Called after post-check get_config response."""
        self._populate_config_fields(result_msg)
        current = self._sensor_config_from_msg(result_msg)
        remaining = self._diff_config(current)

        if remaining:
            self._log(f'← Apply PARTIAL — still differs: {list(remaining.keys())}')
        else:
            self._log('← Apply verified OK — all values match')

        self._apply_state = None
        self._pending_config = {}
        self._widget.apply_config_btn.setEnabled(True)

    # ------------------------------------------------------------------
    # UI update slots (always called on Qt thread via signals)
    # ------------------------------------------------------------------

    def _populate_config_fields(self, msg):
        self._widget.sos_spinbox.setValue(msg.speed_of_sound)
        self._widget.acoustic_cb.setChecked(msg.acoustic_enabled)
        self._widget.dark_mode_cb.setChecked(msg.dark_mode_enabled)
        self._widget.periodic_cycling_cb.setChecked(msg.periodic_cycling_enabled)
        self._widget.rotation_spinbox.setValue(float(msg.mounting_rotation_offset))
        idx = self._widget.range_combo.findText(msg.range_mode)
        if idx >= 0:
            self._widget.range_combo.setCurrentIndex(idx)

    def _update_config_ui(self, msg):
        if not msg.success:
            self._log(f'← get_config ERROR: {msg.error_message}')
            if self._apply_state is not None:
                self._apply_state = None
                self._pending_config = {}
                self._widget.apply_config_btn.setEnabled(True)
            return

        if self._apply_state == 'pre_check':
            self._do_apply(msg)
        elif self._apply_state == 'post_check':
            self._verify_apply(msg)
        else:
            self._populate_config_fields(msg)
            self._log(
                f'← get_config OK: sos={msg.speed_of_sound}, '
                f'acoustic={msg.acoustic_enabled}, dark={msg.dark_mode_enabled}, '
                f'cycling={msg.periodic_cycling_enabled}, '
                f'offset={msg.mounting_rotation_offset}, range={msg.range_mode}'
            )

    def _update_cmd_response_ui(self, msg):
        ok = 'OK' if msg.success else f'ERROR: {msg.error_message}'
        ts = msg.time_status
        if msg.response_to == 'get_time_ntp' and msg.success:
            self._widget.ntp_config_label.setText(f'address={ts.ntp_address}')
            self._widget.ntp_address_input.setText(ts.ntp_address)
            self._log(f'← get_time_ntp {ok}: address={ts.ntp_address}')
        elif msg.response_to in ('get_time_status', 'force_sync_ntp') and msg.success:
            synced = 'YES' if ts.ntp_synced else 'NO'
            last = f'{ts.ntp_seconds_since_last_sync:.0f}' if ts.ntp_seconds_since_last_sync >= 0 else '--'
            synced_to = ts.ntp_synced_to if ts.ntp_synced_to else '--'
            color = '#00aa00' if ts.ntp_synced else '#888888'
            self._widget.time_status_label.setText(
                f'synced={synced}  to={synced_to}  last={last} s'
            )
            self._widget.time_status_label.setStyleSheet(f'color: {color};')
            self._widget.system_time_label.setText(ts.system_time)
            self._log(
                f'← {msg.response_to} {ok}: synced={synced}, '
                f'to={synced_to}, time={ts.system_time}'
            )
        else:
            self._log(f'← {msg.response_to} {ok}')

    def _update_dvl_data_ui(self, msg):
        v = msg.velocity
        self._widget.vel_label.setText(
            f'vx={v.x:+.3f}  vy={v.y:+.3f}  vz={v.z:+.3f} m/s'
        )
        self._widget.alt_label.setText(f'{msg.altitude:.3f} m')
        valid = 'YES' if msg.velocity_valid else 'NO'
        color = '#00aa00' if msg.velocity_valid else '#cc0000'
        self._widget.valid_label.setText(valid)
        self._widget.valid_label.setStyleSheet(f'color: {color};')

    def _update_dvl_pos_ui(self, msg):
        p = msg.position
        self._widget.pos_label.setText(
            f'x={p.x:+.3f}  y={p.y:+.3f}  z={p.z:+.3f} m'
        )
        self._widget.att_label.setText(
            f'r={msg.roll:+.1f}  p={msg.pitch:+.1f}  y={msg.yaw:+.1f} deg'
        )

    def _log(self, text):
        ts = datetime.now().strftime('%H:%M:%S')
        self._widget.log_text.append(f'[{ts}] {text}')

    # ------------------------------------------------------------------
    # RQT lifecycle
    # ------------------------------------------------------------------

    def shutdown_plugin(self):
        pass

    def save_settings(self, _plugin_settings, _instance_settings):
        pass

    def restore_settings(self, _plugin_settings, _instance_settings):
        pass
