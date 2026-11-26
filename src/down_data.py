import pickle
import json
from pathlib import Path
from copy import deepcopy
from datetime import datetime
def convert_for_saving(data):

    if isinstance(data, dict):
        new_dict = {}
        for key, value in data.items():
            try:

                pickle.dumps(value)
                new_dict[key] = value
            except (pickle.PicklingError, AttributeError):

                if callable(value):
                    new_dict[key] = f"<function {value.__name__}>"
                else:
                    new_dict[key] = str(value)
        return new_dict
    return data

def save_solution_data(time_uav_task_dict, time_customer_plan, time_uav_plan,
                      vehicle_plan_time, vehicle_task_data, save_dir='saved_solutions'):

    Path(save_dir).mkdir(parents=True, exist_ok=True)

    save_data = {
        'time_uav_task_dict': deepcopy(time_uav_task_dict),
        'time_customer_plan': deepcopy(time_customer_plan),
        'time_uav_plan': deepcopy(time_uav_plan),
        'vehicle_plan_time': deepcopy(vehicle_plan_time),
        'vehicle_task_data': deepcopy(vehicle_task_data)
    }

    for key in save_data:
        save_data[key] = convert_for_saving(save_data[key])

    from datetime import datetime
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    filename = f'solution_{timestamp}.pkl'

    try:

        with open(f'{save_dir}/{filename}', 'wb') as f:
            pickle.dump(save_data, f, protocol=pickle.HIGHEST_PROTOCOL)

        json_data = {
            'timestamp': timestamp,
            'summary': {
                'time_uav_task_count': len(time_uav_task_dict) if isinstance(time_uav_task_dict, dict) else 'N/A',
                'time_customer_plan_count': len(time_customer_plan) if isinstance(time_customer_plan, dict) else 'N/A',
                'vehicle_plan_time_count': len(vehicle_plan_time) if isinstance(vehicle_plan_time, dict) else 'N/A'
            }
        }

        with open(f'{save_dir}/{filename}.json', 'w') as f:
            json.dump(json_data, f, indent=4)

        print(f"解决方案已保存到: {save_dir}/{filename}")
        return filename

    except Exception as e:
        print(f"保存数据时发生错误: {e}")

        error_log = {
            'timestamp': timestamp,
            'error': str(e),
            'data_types': {
                'time_uav_task_dict': str(type(time_uav_task_dict)),
                'time_customer_plan': str(type(time_customer_plan)),
                'time_uav_plan': str(type(time_uav_plan)),
                'vehicle_plan_time': str(type(vehicle_plan_time)),
                'vehicle_task_data': str(type(vehicle_task_data))
            }
        }
        error_filename = f'error_log_{timestamp}.json'
        with open(f'{save_dir}/{error_filename}', 'w') as f:
            json.dump(error_log, f, indent=4)
        raise

def load_solution_data(filename, load_dir='saved_solutions'):

    try:
        filepath = f'{load_dir}/{filename}'
        with open(filepath, 'rb') as f:
            data = pickle.load(f)

        return (
            data['time_uav_task_dict'],
            data['time_customer_plan'],
            data['time_uav_plan'],
            data['vehicle_plan_time'],
            data['vehicle_task_data']
        )
    except Exception as e:
        print(f"加载数据时发生错误: {e}")
        print(f"尝试加载的文件: {filepath}")
        raise

def save_partial_solution(data, name, save_dir='saved_solutions'):

    Path(save_dir).mkdir(parents=True, exist_ok=True)
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    filename = f'{name}_{timestamp}.pkl'

    try:
        converted_data = convert_for_saving(deepcopy(data))
        with open(f'{save_dir}/{filename}', 'wb') as f:
            pickle.dump(converted_data, f, protocol=pickle.HIGHEST_PROTOCOL)
        print(f"部分数据已保存到: {save_dir}/{filename}")
        return filename
    except Exception as e:
        print(f"保存部分数据时发生错误: {e}")
        raise

import os
import pickle
import json
import datetime
import networkx as nx
from collections import defaultdict
import shutil
import numpy as np

def list_saved_files():

    save_dir = 'saved_solutions'
    if not os.path.exists(save_dir):
        print("没有找到saved_solutions目录")
        return []

    files = [f for f in os.listdir(save_dir) if f.startswith('input_data_') and f.endswith('.pkl')]
    if not files:
        print("没有找到保存的数据文件")
        return []

    print("\n当前保存的文件列表:")
    for i, f in enumerate(files, 1):
        file_path = os.path.join(save_dir, f)
        file_size = os.path.getsize(file_path) / (1024 * 1024)
        file_time = datetime.datetime.fromtimestamp(os.path.getctime(file_path))
        print(f"{i}. {f}")
        print(f"   创建时间: {file_time}")
        print(f"   文件大小: {file_size:.2f}MB")
    return files

def delete_saved_file(filename):

    save_dir = 'saved_solutions'
    if not filename.endswith('.pkl'):
        filename += '.pkl'
    if not filename.startswith('input_data_'):
        filename = 'input_data_' + filename

    file_path = os.path.join(save_dir, filename)
    json_file = file_path.replace('.pkl', '.json')

    try:
        if os.path.exists(file_path):
            os.remove(file_path)
            print(f"已删除文件: {filename}")
        if os.path.exists(json_file):
            os.remove(json_file)
            print(f"已删除对应的JSON文件")
        return True
    except Exception as e:
        print(f"删除文件时出错: {e}")
        return False

def backup_saved_file(filename, backup_name=None):

    save_dir = 'saved_solutions'
    backup_dir = os.path.join(save_dir, 'backups')

    if not filename.endswith('.pkl'):
        filename += '.pkl'
    if not filename.startswith('input_data_'):
        filename = 'input_data_' + filename

    if not os.path.exists(backup_dir):
        os.makedirs(backup_dir)

    source_path = os.path.join(save_dir, filename)
    source_json = source_path.replace('.pkl', '.json')

    if not os.path.exists(source_path):
        print(f"未找到源文件: {filename}")
        return False

    try:

        if backup_name is None:
            timestamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
            backup_name = f"backup_{timestamp}_{filename}"
        elif not backup_name.startswith('backup_'):
            backup_name = f"backup_{backup_name}"
        if not backup_name.endswith('.pkl'):
            backup_name += '.pkl'

        backup_path = os.path.join(backup_dir, backup_name)
        backup_json = backup_path.replace('.pkl', '.json')

        shutil.copy2(source_path, backup_path)
        if os.path.exists(source_json):
            shutil.copy2(source_json, backup_json)

        print(f"已创建备份: {backup_name}")
        return True
    except Exception as e:
        print(f"创建备份时出错: {e}")
        return False

def get_latest_saved_file():

    save_dir = 'saved_solutions'
    if not os.path.exists(save_dir):
        print("没有找到saved_solutions目录")
        return None

    files = [f for f in os.listdir(save_dir) if f.startswith('input_data_') and f.endswith('.pkl')]
    if not files:
        print("没有找到保存的数据文件")
        return None

    latest_file = sorted(files)[-1]
    return latest_file.replace('.pkl', '')

def convert_defaultdict_to_dict(d):

    if isinstance(d, defaultdict):
        d = {k: convert_defaultdict_to_dict(v) for k, v in d.items()}
    elif isinstance(d, dict):
        d = {k: convert_defaultdict_to_dict(v) for k, v in d.items()}
    elif isinstance(d, list):
        d = [convert_defaultdict_to_dict(v) for v in d]
    return d

def convert_str_to_tuple_key(s):

    if s.startswith('(') and s.endswith(')'):
        try:

            items = s[1:-1].split(',')

            items = [int(item.strip()) if item.strip().isdigit() else item.strip() for item in items]
            return tuple(items) if len(items) > 1 else items[0]
        except:
            return s
    return s

def convert_keys_back(obj):

    if isinstance(obj, dict):
        return {convert_str_to_tuple_key(k): convert_keys_back(v) for k, v in obj.items()}
    elif isinstance(obj, list):
        return [convert_keys_back(v) for v in obj]
    return obj

def create_object_from_dict(data_dict):

    if isinstance(data_dict, dict):

        if '_class_name' in data_dict:
            try:
                if data_dict['_class_name'] == 'Node':
                    from initialize import Node
                    node = Node()
                    for attr_name, attr_value in data_dict['attributes'].items():
                        setattr(node, attr_name, create_object_from_dict(attr_value))
                    return node
                elif data_dict['_class_name'] == 'Vehicle':
                    from initialize import Vehicle
                    vehicle = Vehicle()
                    for attr_name, attr_value in data_dict['attributes'].items():
                        setattr(vehicle, attr_name, create_object_from_dict(attr_value))
                    return vehicle
                elif data_dict['_class_name'] == 'vehicle_task':
                    from task_data import vehicle_task
                    attrs = data_dict['attributes']
                    task = vehicle_task(
                        attrs.get('id'),
                        attrs.get('vehicleType'),
                        attrs.get('node_id'),
                        attrs.get('node')
                    )
                    for attr_name, attr_value in attrs.items():
                        if attr_name not in ['id', 'vehicleType', 'node_id', 'node']:
                            setattr(task, attr_name, create_object_from_dict(attr_value))
                    return task
            except ImportError as e:
                print(f"警告: 无法导入类 {data_dict['_class_name']}, 使用字典替代: {str(e)}")
                return data_dict['attributes']

        return {convert_key_to_original(k): create_object_from_dict(v)
                for k, v in data_dict.items()}
    elif isinstance(data_dict, list):
        return [create_object_from_dict(item) for item in data_dict]
    elif isinstance(data_dict, tuple):
        return tuple(create_object_from_dict(item) for item in data_dict)
    return data_dict

def convert_dict_to_defaultdict(data):

    if isinstance(data, dict):

        dd = defaultdict(lambda: None)
        for k, v in data.items():
            if isinstance(v, dict):
                dd[k] = convert_dict_to_defaultdict(v)
            else:
                dd[k] = v
        return dd
    return data

def save_input_data(input_data):

    save_dir = 'saved_solutions'
    if not os.path.exists(save_dir):
        os.makedirs(save_dir)

    timestamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    base_filename = f'input_data_{timestamp}'

    serializable_data = {}
    for key, value in input_data.items():
        if isinstance(value, defaultdict):

            serializable_data[key] = {
                'type': 'defaultdict',
                'data': convert_defaultdict_to_dict(value)
            }
        elif isinstance(value, nx.Graph):
            serializable_data[key] = {
                'type': 'networkx_graph',
                'data': nx.node_link_data(value)
            }
        elif key in ['node', 'vehicle']:
            serializable_data[key] = {
                'type': 'object_dict',
                'data': {k: obj.__dict__ for k, obj in value.items()}
            }
        else:
            serializable_data[key] = {
                'type': 'regular',
                'data': value
            }

    pickle_filename = os.path.join(save_dir, f'{base_filename}.pkl')
    with open(pickle_filename, 'wb') as f:
        pickle.dump(serializable_data, f)

    return base_filename

def _convert_numpy_types(obj):

    if isinstance(obj, (np.integer,)):
        return int(obj)
    if isinstance(obj, (np.floating,)):
        return float(obj)
    if isinstance(obj, (np.bool_,)):
        return bool(obj)
    if isinstance(obj, np.ndarray):
        return obj.tolist()
    if isinstance(obj, dict):
        return { _convert_numpy_types(k): _convert_numpy_types(v) for k, v in obj.items() }
    if isinstance(obj, (list, tuple)):
        t = [_convert_numpy_types(x) for x in obj]
        return type(obj)(t) if isinstance(obj, tuple) else t
    return obj

def _convert_defaultdict_to_dict(dd):

    if isinstance(dd, defaultdict):
        dd = dict(dd)
    if isinstance(dd, dict):
        return {k: _convert_defaultdict_to_dict(v) for k, v in dd.items()}
    if isinstance(dd, list):
        return [_convert_defaultdict_to_dict(x) for x in dd]
    if isinstance(dd, tuple):
        return tuple(_convert_defaultdict_to_dict(x) for x in dd)
    return dd

def _convert_key_to_str(obj):

    if isinstance(obj, dict):
        return {str(k): _convert_key_to_str(v) for k, v in obj.items()}
    if isinstance(obj, list):
        return [_convert_key_to_str(x) for x in obj]
    if isinstance(obj, tuple):
        return tuple(_convert_key_to_str(x) for x in obj)
    return obj

def _is_vehicle_task(obj):
    return hasattr(obj, "__class__") and obj.__class__.__name__ == "vehicle_task"

def _is_make_node(obj):
    return hasattr(obj, "__class__") and obj.__class__.__name__ in ("make_node", "Node", "node")

def _safe_for_regular(obj):

    if _is_vehicle_task(obj) or _is_make_node(obj):
        return obj
    if isinstance(obj, nx.Graph):

        return obj
    obj = _convert_defaultdict_to_dict(obj)
    obj = _convert_key_to_str(obj)
    obj = _convert_numpy_types(obj)
    return obj

SAVE_DIR_ABS = os.path.join(os.path.dirname(os.path.abspath(__file__)), "saved_solutions")

def save_input_data_with_name(input_data, custom_name):

    def _sanitize_data(obj):

        if isinstance(obj, (np.integer, int)) and not isinstance(obj, bool):
             return int(obj)
        elif isinstance(obj, (np.floating, float)):
             return float(obj)
        elif isinstance(obj, np.ndarray):
             return [_sanitize_data(x) for x in obj.tolist()]

        elif isinstance(obj, defaultdict):
            return {k: _sanitize_data(v) for k, v in obj.items()}

        elif isinstance(obj, dict):
            return {k: _sanitize_data(v) for k, v in obj.items()}

        elif isinstance(obj, list):
            return [_sanitize_data(v) for v in obj]
        elif isinstance(obj, tuple):
            return tuple(_sanitize_data(v) for v in obj)

        else:
            return obj

    if not custom_name.endswith(".pkl"):
        custom_name += ".pkl"
    if not custom_name.startswith("input_data_"):
        custom_name = "input_data_" + custom_name

    save_dir = SAVE_DIR_ABS
    os.makedirs(save_dir, exist_ok=True)
    file_path = os.path.join(save_dir, custom_name)

    try:
        print(f"[SAVE] 正在预处理数据 (清洗 lambda 和 numpy)...")
        serializable_data = {}

        for key, value in input_data.items():

            if isinstance(value, nx.Graph):

                graph_data = nx.node_link_data(value)
                serializable_data[key] = {
                    'type': 'networkx_graph',
                    'data': _sanitize_data(graph_data)
                }

            elif isinstance(value, defaultdict):
                serializable_data[key] = {
                    'type': 'defaultdict',
                    'data': _sanitize_data(value)
                }

            elif key in ['node', 'vehicle']:
                serializable_data[key] = {
                    'type': 'object_dict',
                    'data': _sanitize_data(value)
                }

            elif key in ['vehicle_task_data', 'uav_task_dict']:
                serializable_data[key] = {
                    'type': 'task_data',
                    'data': _sanitize_data(value)
                }

            else:
                serializable_data[key] = {
                    'type': 'regular',
                    'data': _sanitize_data(value)
                }

        print(f"[SAVE] 正在写入文件: {file_path}")
        with open(file_path, "wb") as f:
            pickle.dump(serializable_data, f, protocol=pickle.HIGHEST_PROTOCOL)

        print(f"✅ 数据保存成功: {custom_name}")
        return custom_name.replace(".pkl", "")

    except Exception as e:
        print(f"❌ 保存失败: {str(e)}")
        import traceback
        print(traceback.format_exc())
        return None

def reconstruct_vehicle_task(data):

    if isinstance(data, dict):
        if data.get('_class_name') == 'vehicle_task':

            from task_data import vehicle_task

            task = vehicle_task(
                data['attributes'].get('id'),
                data['attributes'].get('vehicleType'),
                data['attributes'].get('node_id'),
                data['attributes'].get('node')
            )

            for attr_name, attr_value in data['attributes'].items():
                if attr_name not in ['id', 'vehicleType', 'node_id', 'node']:
                    if isinstance(attr_value, dict):
                        setattr(task, attr_name, reconstruct_vehicle_task(attr_value))
                    elif isinstance(attr_value, list):
                        setattr(task, attr_name, [reconstruct_vehicle_task(item) for item in attr_value])
                    else:
                        setattr(task, attr_name, attr_value)
            return task
        else:
            return {k: reconstruct_vehicle_task(v) for k, v in data.items()}
    elif isinstance(data, list):
        return [reconstruct_vehicle_task(item) for item in data]
    elif isinstance(data, tuple):
        return tuple(reconstruct_vehicle_task(item) for item in data)
    return data

def convert_vehicle_task(obj):

    if isinstance(obj, dict):
        return {k: convert_vehicle_task(v) for k, v in obj.items()}
    elif isinstance(obj, list):
        return [convert_vehicle_task(item) for item in obj]
    elif isinstance(obj, tuple):
        return tuple(convert_vehicle_task(item) for item in obj)
    elif hasattr(obj, '__class__') and obj.__class__.__name__ == 'vehicle_task':

        task_dict = {
            '_class_name': 'vehicle_task',
            'attributes': {}
        }
        for attr_name, attr_value in obj.__dict__.items():

            if isinstance(attr_value, (list, tuple)):
                task_dict['attributes'][attr_name] = [convert_vehicle_task(item) for item in attr_value]
            elif isinstance(attr_value, dict):
                task_dict['attributes'][attr_name] = {k: convert_vehicle_task(v) for k, v in attr_value.items()}
            elif hasattr(attr_value, '__dict__'):
                task_dict['attributes'][attr_name] = convert_vehicle_task(attr_value)
            else:
                task_dict['attributes'][attr_name] = attr_value
        return task_dict
    return obj

def convert_node_object(obj):

    if hasattr(obj, '__class__') and obj.__class__.__name__ == 'make_node':

        node_dict = {
            '_class_name': 'make_node',
            'attributes': {}
        }
        for attr_name, attr_value in obj.__dict__.items():

            if isinstance(attr_value, (list, tuple)):
                node_dict['attributes'][attr_name] = [convert_node_object(item) for item in attr_value]
            elif isinstance(attr_value, dict):
                node_dict['attributes'][attr_name] = {k: convert_node_object(v) for k, v in attr_value.items()}
            elif hasattr(attr_value, '__dict__'):
                node_dict['attributes'][attr_name] = convert_node_object(attr_value)
            else:
                node_dict['attributes'][attr_name] = attr_value
        return node_dict
    return obj

def convert_numpy_types(obj):

    if isinstance(obj, np.integer):
        return int(obj)
    elif isinstance(obj, np.floating):
        return float(obj)
    elif isinstance(obj, np.ndarray):
        return obj.tolist()
    elif isinstance(obj, dict):
        return {k: convert_numpy_types(v) for k, v in obj.items()}
    elif isinstance(obj, list):
        return [convert_numpy_types(v) for v in obj]
    elif isinstance(obj, tuple):
        return tuple(convert_numpy_types(v) for v in obj)
    return obj

def convert_key_to_original(key):

    try:

        return int(key)
    except (ValueError, TypeError):
        return key

def manage_saved_data():

    while True:
        print("\n=== 数据文件管理 ===")
        print("1. 列出所有保存的文件")
        print("2. 删除指定文件")
        print("3. 备份指定文件")
        print("4. 保存新数据")
        print("5. 退出")

        choice = input("请选择操作 (1-5): ")

        if choice == '1':
            list_saved_files()

        elif choice == '2':
            files = list_saved_files()
            if files:
                filename = input("请输入要删除的文件名: ")
                delete_saved_file(filename)

        elif choice == '3':
            files = list_saved_files()
            if files:
                filename = input("请输入要备份的文件名: ")
                backup_name = input("请输入备份名称（直接回车使用时间戳）: ")
                if not backup_name:
                    backup_name = None
                backup_saved_file(filename, backup_name)

        elif choice == '4':
            custom_name = input("请输入新数据文件名: ")

            print("请在代码中提供input_data后使用此功能")

        elif choice == '5':
            break

        else:
            print("无效的选择，请重试")

def load_input_data(filename):

    save_dir = SAVE_DIR_ABS

    if os.path.isabs(filename) and os.path.exists(filename):
        pickle_path = filename
    else:

        name = filename
        if name.endswith(".pkl"):
            name = name[:-4]
        if not name.startswith("input_data_"):
            name = "input_data_" + name

        pickle_path = os.path.join(save_dir, name + ".pkl")

    print(f"[LOAD] 正在读取文件: {pickle_path}")

    if not os.path.exists(pickle_path):
        print(f"[LOAD][ERROR] 文件不存在: {pickle_path}")
        return None

    try:
        with open(pickle_path, "rb") as f:
            loaded_wrapper = pickle.load(f)

        restored_data = {}

        for key, wrapper in loaded_wrapper.items():

            if isinstance(wrapper, dict) and 'type' in wrapper and 'data' in wrapper:
                data_type = wrapper['type']
                raw_data = wrapper['data']

                if data_type == 'defaultdict':

                    dd = defaultdict(lambda: None)
                    dd.update(raw_data)
                    restored_data[key] = dd

                elif data_type == 'networkx_graph':

                    restored_data[key] = nx.node_link_graph(raw_data)

                elif data_type == 'object_dict':

                    restored_data[key] = raw_data

                else:

                    restored_data[key] = raw_data
            else:

                restored_data[key] = wrapper

        print(f"[LOAD] ✅ 成功加载数据，对象方法已保留。")

        try:
            import gc as _gc
            _V = restored_data.get('V') or []
            _veh = restored_data.get('vehicle') or {}
            _ut = restored_data.get('uav_travel')
            _xe = restored_data.get('xeee')
            _canon_uav = {}
            for _u in _V:
                _vo = _veh.get(_u)
                if _vo is None:
                    continue
                _ukey = (
                    getattr(_vo, 'takeoffSpeed', None),
                    getattr(_vo, 'cruiseSpeed', None),
                    getattr(_vo, 'landingSpeed', None),
                    getattr(_vo, 'yawRateDeg', None),
                    getattr(_vo, 'cruiseAlt', None),
                    getattr(_vo, 'batteryPower', None),
                    getattr(_vo, 'capacityLbs', None),
                )
                if _ukey not in _canon_uav:
                    _canon_uav[_ukey] = _u
                else:
                    _src_u = _canon_uav[_ukey]
                    if _ut is not None and _u in _ut and _src_u in _ut:
                        _ut[_u] = _ut[_src_u]
                    if _xe is not None and _u in _xe and _src_u in _xe:
                        _xe[_u] = _xe[_src_u]
            _override_uav_cost = os.environ.get('VDAC_PER_UAV_COST')
            if _override_uav_cost is not None and _override_uav_cost != '':
                _target_c = float(_override_uav_cost)
                _bup = restored_data.get('best_uav_plan') or {}
                for _u in _V:
                    _vo = _veh.get(_u)
                    if _vo is not None:
                        _old_c = float(getattr(_vo, 'per_cost', 1.0) or 1.0)
                        _vo.per_cost = _target_c
                        if _old_c > 0 and abs(_target_c - _old_c) > 1e-12:
                            _ratio = _target_c / _old_c
                            for _mk, _md in _bup.items():
                                if isinstance(_md, dict) and _md.get('drone_id') == _u:
                                    for _ck in ('cost', 'uav_cost', 'uav_route_cost'):
                                        if _ck in _md and _md[_ck] is not None:
                                            _md[_ck] = float(_md[_ck]) * _ratio
            _gc.collect()
        except Exception as _dedup_err:
            print(f"[LOAD][WARN] post-load dedup/cost hook: {_dedup_err}")
        return restored_data

    except Exception as e:
        print(f"[LOAD][ERROR] 加载失败: {str(e)}")
        import traceback
        print(traceback.format_exc())
        return None

def convert_object_to_dict(obj):

    if hasattr(obj, '__class__'):
        class_name = obj.__class__.__name__
        if class_name in ['Node', 'Vehicle', 'vehicle_task']:
            return {
                '_class_name': class_name,
                'attributes': {
                    attr_name: convert_object_to_dict(attr_value)
                    for attr_name, attr_value in obj.__dict__.items()
                }
            }
    if isinstance(obj, dict):

        return {convert_key_to_original(k): convert_object_to_dict(v) for k, v in obj.items()}
    elif isinstance(obj, (list, tuple)):
        return [convert_object_to_dict(item) for item in obj]
    elif isinstance(obj, (int, float, str, bool, type(None))):
        return obj
    elif isinstance(obj, np.integer):
        return int(obj)
    elif isinstance(obj, np.floating):
        return float(obj)
    elif isinstance(obj, np.ndarray):
        return obj.tolist()
    return obj

def run_low_update_time_from_saved(input_filename):

    input_data = load_input_data(input_filename)
    if input_data is None:
        print("错误: 无法加载输入数据")
        return None

    try:
        return (input_data['uav_task_dict'],
                input_data['best_customer_plan'],
                input_data['best_uav_plan'],
                input_data['best_vehicle_route'],
                input_data['vehicle_task_data'],
                input_data['vehicle_arrival_time'],
                input_data['node'],
                input_data['DEPOT_nodeID'],
                input_data['V'],
                input_data['T'],
                input_data['vehicle'],
                input_data['uav_travel'],
                input_data['veh_distance'],
                input_data['veh_travel'],
                input_data['N'],
                input_data['N_zero'],
                input_data['N_plus'],
                input_data['A_total'],
                input_data['A_cvtp'],
                input_data['A_vtp'],
                input_data['A_aerial_relay_node'],
                input_data['G_air'],
                input_data['G_ground'],
                input_data['air_matrix'],
                input_data['ground_matrix'],
                input_data['air_node_types'],
                input_data['ground_node_types'],
                input_data['A_c'],
                input_data['xeee'])
    except KeyError as e:
        print(f"错误: 缺少必要的数据键 {str(e)}")
        return None
    except Exception as e:
        print(f"错误: 处理数据时出错: {str(e)}")
        return None
