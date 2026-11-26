def validate_state_constraints(state, verbose=True):

    validation_result = {
        'is_valid': True,
        'errors': [],
        'warnings': [],
        'summary': {}
    }

    try:

        if not hasattr(state, 'customer_plan') or not hasattr(state, 'vehicle_routes'):
            validation_result['is_valid'] = False
            validation_result['errors'].append("State缺少必要的customer_plan或vehicle_routes属性")
            return validation_result

        for vehicle_id in range(len(state.V)):
            vehicle_errors = _validate_vehicle_drone_sequences(state, vehicle_id, verbose)
            validation_result['errors'].extend(vehicle_errors)

        validation_result['is_valid'] = len(validation_result['errors']) == 0

        if validation_result['errors']:
            validation_result['is_valid'] = False

    except Exception as e:
        validation_result['is_valid'] = False
        validation_result['errors'].append(f"验证过程中发生错误: {str(e)}")
        if verbose:
            print(f"❌ 验证过程异常: {str(e)}")

    return validation_result

def _validate_vehicle_drone_sequences(state, vehicle_id, verbose=False):

    errors = []

    try:

        vehicle_route = state.vehicle_routes[vehicle_id]
        if not vehicle_route:
            return errors

        vehicle_drones = _get_vehicle_drones(state, vehicle_id)
        if not vehicle_drones:
            return errors

        if verbose:
            print(f"\n🔍 检查车辆{vehicle_id}的无人机序列...")
            print(f"   路线: {vehicle_route}")
            print(f"   无人机: {vehicle_drones}")

        for drone_id in vehicle_drones:
            drone_errors = _validate_drone_sequence(state, vehicle_id, drone_id, vehicle_route, verbose)
            errors.extend(drone_errors)

    except Exception as e:
        errors.append(f"车辆{vehicle_id}验证过程中发生错误: {str(e)}")
        if verbose:
            print(f"❌ 车辆{vehicle_id}验证异常: {str(e)}")

    return errors

def _validate_drone_sequence(state, vehicle_id, drone_id, vehicle_route, verbose=False):

    errors = []

    try:

        drone_operations = _get_drone_operations_in_route(state, vehicle_id, drone_id, vehicle_route)

        if verbose:
            print(f"\n   🚁 检查无人机{drone_id}的操作序列:")
            for i, op in enumerate(drone_operations):
                print(f"     {i+1}. 节点{op['node']}: {op['operation']} (客户{op['customer_id']})")

        sequence_errors = _check_drone_operation_sequence(drone_id, drone_operations, vehicle_id, verbose)
        errors.extend(sequence_errors)

    except Exception as e:
        errors.append(f"无人机{drone_id}在车辆{vehicle_id}中验证失败: {str(e)}")
        if verbose:
            print(f"❌ 无人机{drone_id}验证异常: {str(e)}")

    return errors

def _get_vehicle_drones(state, vehicle_id):

    try:
        if hasattr(state, 'base_drone_assignment') and state.base_drone_assignment:
            return state.base_drone_assignment.get(vehicle_id, [])
        else:

            vehicle_drones = set()
            for customer_id, plan in state.customer_plan.items():
                if plan.get('vehicle_id') == vehicle_id:
                    drone_id = plan.get('drone_id')
                    if drone_id is not None:
                        vehicle_drones.add(drone_id)
            return list(vehicle_drones)
    except:
        return []

def _get_drone_operations_in_route(state, vehicle_id, drone_id, vehicle_route):

    operations = []

    try:

        for node_idx, node_id in enumerate(vehicle_route):

            node_operations = _get_drone_operations_at_node(state, vehicle_id, drone_id, node_id)
            operations.extend(node_operations)

    except Exception as e:
        pass

    return operations

def _get_drone_operations_at_node(state, vehicle_id, drone_id, node_id):

    operations = []

    try:

        for customer_id, plan in state.customer_plan.items():
            if (plan.get('vehicle_id') == vehicle_id and
                plan.get('drone_id') == drone_id and
                plan.get('node_id') == node_id):

                operation_type = None
                if plan.get('is_launch', False):
                    operation_type = 'launch'
                elif plan.get('is_recovery', False):
                    operation_type = 'recovery'

                if operation_type:
                    operations.append({
                        'node': node_id,
                        'operation': operation_type,
                        'customer_id': customer_id
                    })

    except Exception as e:
        pass

    return operations

def _check_drone_operation_sequence(drone_id, operations, vehicle_id, verbose=False):

    errors = []

    try:

        drone_launched = False

        for i, op in enumerate(operations):
            operation = op['operation']
            node_id = op['node']
            customer_id = op['customer_id']

            if operation == 'launch':
                if drone_launched:

                    error_msg = f"❌ 车辆{vehicle_id}在节点{node_id}连续发射无人机{drone_id}未回收 (客户{customer_id})"
                    errors.append(error_msg)
                    if verbose:
                        print(f"     {error_msg}")
                else:
                    drone_launched = True
                    if verbose:
                        print(f"     ✅ 节点{node_id}: 发射无人机{drone_id} (客户{customer_id})")

            elif operation == 'recovery':
                if not drone_launched:

                    error_msg = f"❌ 车辆{vehicle_id}在节点{node_id}未发射就回收无人机{drone_id} (客户{customer_id})"
                    errors.append(error_msg)
                    if verbose:
                        print(f"     {error_msg}")
                else:
                    drone_launched = False
                    if verbose:
                        print(f"     ✅ 节点{node_id}: 回收无人机{drone_id} (客户{customer_id})")

        if drone_launched:
            error_msg = f"❌ 车辆{vehicle_id}路线结束时，无人机{drone_id}处于发射状态但未被回收"
            errors.append(error_msg)
            if verbose:
                print(f"     {error_msg}")

    except Exception as e:
        error_msg = f"❌ 检查无人机{drone_id}操作序列时出错: {str(e)}"
        errors.append(error_msg)
        if verbose:
            print(f"     {error_msg}")

    return errors

def quick_validate(state):

    try:
        result = validate_state_constraints(state, verbose=False)
        return result['is_valid']
    except:
        return False
