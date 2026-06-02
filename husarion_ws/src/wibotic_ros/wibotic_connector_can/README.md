# wibotic_ros

It reads a CAN Bus thanks to the uavcan library and sends the measurements to ROS 2.

## ROS Nodes

### wibotic_connector_can

It reads a CAN Bus thanks to the uavcan library and sends the measurements to ROS 2.

#### Publishes

- `wibotic_info` [*wibotic_msgs/WiboticInfo*]: Wibotic charger measurements.

#### Services

- `wibotic_charger_enable` [*std_srvs/SetBool*]: Set Wibotic Charger state.

#### Parameters

- `can_iface_name` [*string*, default: **can0**]: CAN BUS interface used for Wibotic receiver.
- `uavcan_node_id` [*int*, default: **20**]: Uavcan node ID.
- `uavcan_node_name` [*string*, default: **can0**]: Uavcan node name.
- `update_time` [*float*, default: **1.0**]: The period of reading WiboticInfo on a CAN BUS.
- `max_service_call_retries` [*int*, default:**10**]: Maximum retries to call `GetSet` uavcan service.
- `spin_duration` [*float*, default: **0.1**]: Spin duration of an uavcan node for every `update_time`.
