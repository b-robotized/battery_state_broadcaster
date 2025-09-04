:github_url: https://github.com/ros-controls/ros2_controllers/blob/{REPOS_FILE_BRANCH}/battery_state_broadcaster/doc/userdoc.rst

.. _battery_state_broadcaster_userdoc:

Battery State Broadcaster
--------------------------------
The *Battery State Broadcaster* is a ROS 2 controller that publishes battery status information as
``sensor_msgs/msg/BatteryState`` messages.

It reads battery-related state interfaces from one or more joints and exposes them in a standard ROS 2 message format.
This allows easy integration with monitoring tools, logging systems, and higher-level decision-making nodes.

Interfaces
====================
The broadcaster can read the following state interfaces from each configured joint:

- ``battery_voltage`` *(mandatory)*
  Battery voltage [V].
  This interface is **always required** for the controller to function.

- ``battery_temperature`` *(optional)*
  Battery temperature [°C].

- ``battery_current`` *(optional)*
  Battery current [A].

- ``battery_charge`` *(optional)*
  Remaining battery charge [Ah].

- ``battery_percentage`` *(optional)*
  Charge level [%] (0.0–100.0).
  If not provided, it is estimated using ``minimum_voltage`` and ``maximum_voltage`` if available.

- ``battery_power_supply_status`` *(optional)*
  Power supply status (e.g., Charging, Full, Not Charging).
  Defaults to *Unknown* if not provided.

- ``battery_power_supply_health`` *(optional)*
  Health indicator (e.g., Good, Overheat).
  Defaults to *Unknown* if not provided.

- ``battery_present`` *(optional)*
  Indicates whether the battery is present.
  Defaults to *true* if a valid voltage is reported from this joint.

Metadata interfaces (read-only, optional):

- ``minimum_voltage`` / ``maximum_voltage`` – used for percentage estimation.
- ``capacity`` / ``design_capacity`` – reported as provided.
- ``power_supply_technology`` – chemistry type enum (e.g., Li-ion).
- ``location`` – free-form string describing physical location.
- ``serial_number`` – battery serial number.

Published Topics
================
The broadcaster publishes two topics:

- ``~/raw_battery_states`` (``control_msgs/msg/BatteryStates``)
  Publishes **per-joint battery state messages**, containing the raw values for each configured joint.

- ``~/battery_state`` (``sensor_msgs/msg/BatteryState``)
  Publishes a **single aggregated battery message** representing the combined status across all joints.

Aggregation Rules
=================

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
| ``percentage``              | Mean across joints with reported/calculated  |
|                             | percentage                                   |
+-----------------------------+----------------------------------------------+
| ``power_supply_status``     | Combined using highest reported enum value   |
+-----------------------------+----------------------------------------------+
| ``power_supply_health``     | Combined using highest reported enum value   |
+-----------------------------+----------------------------------------------+
| ``present``                 | True if any joint reports presence           |
+-----------------------------+----------------------------------------------+

Parameters
^^^^^^^^^^^
This controller uses the
`generate_parameter_library <https://github.com/PickNikRobotics/generate_parameter_library>`_
to manage parameters.
The parameter `definition file <https://github.com/ros-controls/ros2_controllers/blob/{REPOS_FILE_BRANCH}/battery_state_broadcaster/src/battery_state_broadcaster_parameters.yaml>`_
contains the full list and descriptions.

List of parameters
=========================
.. generate_parameter_library_details:: ../src/battery_state_broadcaster_parameters.yaml

Example Parameter File
=========================

An example parameter file for this controller is available in the
`test directory <https://github.com/ros-controls/ros2_controllers/blob/{REPOS_FILE_BRANCH}/battery_state_broadcaster/test/battery_state_broadcaster_params.yaml>`_:

.. literalinclude:: ../test/battery_state_broadcaster_params.yaml
   :language: yaml
