:github_url: https://github.com/ros-controls/ros2_controllers/blob/{REPOS_FILE_BRANCH}/battery_state_broadcaster/doc/userdoc.rst

.. _battery_state_broadcaster_userdoc:

Battery State Broadcaster
--------------------------------
The *Battery State Broadcaster* is a ros2 controller that publishes battery status information as
``sensor_msgs/msg/BatteryState`` messages.

It is designed to read battery-related state interfaces from one or more joints and convert them into
a standard ROS 2 battery message for monitoring, logging, and higher-level decision-making.

Interfaces
====================
The broadcaster can read the following state interfaces from each configured joint:

- ``battery_voltage`` *(mandatory)*
  Battery voltage [V]. Always required.

- ``battery_temperature`` *(optional)*
  Battery temperature [°C].

- ``battery_current`` *(optional)*
  Battery current [A].

- ``battery_charge`` *(optional)*
  Remaining battery charge [Ah].

- ``battery_percentage`` *(optional)*
  Charge level [%] (0.0–100.0).
  If not measured, estimated using ``minimum_voltage`` and ``maximum_voltage`` if provided.

- ``battery_power_supply_status`` *(optional)*
  Power supply status (e.g., Charging, Full, Not Charging).
  Defaults to *Unknown* if not provided.

- ``battery_power_supply_health`` *(optional)*
  Health indicator (e.g., Good, Overheat).
  Defaults to *Unknown* if not provided.

- ``battery_present`` *(optional)*
  Presence flag.
  Defaults to *true* if voltage value from this joint is available.

Metadata interfaces (read-only, optional):
- ``minimum_voltage`` / ``maximum_voltage`` – used to estimate percentage.
- ``capacity`` / ``design_capacity`` – reported as is.
- ``power_supply_technology`` – chemistry type enum.
- ``location`` – free-form string describing location.
- ``serial_number`` – serial number string.

Published Topics
================
The broadcaster publishes two topics:

- ``~/raw_battery_states`` (``control_msgs/msg/BatteryStates``)
  Publishes **one message per configured joint**, containing the raw values read directly from each joint’s interfaces.
  This is useful when you need detailed, per-battery diagnostics.

- ``~/battery_state`` (``sensor_msgs/msg/BatteryState``)
  Publishes a **single aggregated message** representing the combined battery state across all joints.
  This is useful for higher-level decision making (e.g., robot as a whole has 65% battery).

Aggregation rules:

+-----------------------------+----------------------------------------------+
| Field                       | Aggregation rule                             |
+=============================+==============================================+
| ``voltage``                 | Arithmetic mean across all joints            |
+-----------------------------+----------------------------------------------+
| ``temperature``             | Mean across joints reporting temperature     |
+-----------------------------+----------------------------------------------+
| ``current``                 | Mean across joints reporting current         |
+-----------------------------+----------------------------------------------+
| ``charge``                  | Sum across all joints                        |
+-----------------------------+----------------------------------------------+
| ``percentage``              | Mean across joints reporting/calculating     |
|                             | percentage                                   |
+-----------------------------+----------------------------------------------+
| ``power_supply_status``     | Combined into a representative status: the   |
|                             | (higher enum value is taken).                |
+-----------------------------+----------------------------------------------+
| ``power_supply_health``     | Combined into a representative health        |
|                             | (higher enum value is taken).                |
+-----------------------------+----------------------------------------------+

Parameters
^^^^^^^^^^^
This controller uses the
`generate_parameter_library <https://github.com/PickNikRobotics/generate_parameter_library>`_
to handle its parameters.
The parameter `definition file <https://github.com/ros-controls/ros2_controllers/blob/{REPOS_FILE_BRANCH}/battery_state_broadcaster/src/battery_state_broadcaster_parameters.yaml>`_
contains descriptions for all the parameters used by the controller.

List of parameters
=========================
.. generate_parameter_library_details:: ../src/battery_state_broadcaster_parameters.yaml

An example parameter file
=========================

An example parameter file for this controller can be found in
`the test directory <https://github.com/ros-controls/ros2_controllers/blob/{REPOS_FILE_BRANCH}/battery_state_broadcaster/test/battery_state_broadcaster_params.yaml>`_:

.. literalinclude:: ../test/battery_state_broadcaster_params.yaml
   :language: yaml
