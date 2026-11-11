# Copyright 2026 Maik Knof
# SPDX-License-Identifier: Apache-2.0
"""Echo-compatible dotted field selection and bounded YAML output."""


def select_field(message, path):
    if not path:
        return message
    for name in path.split('.'):
        if not name or name.startswith('_') or not hasattr(message, name):
            raise ValueError(f'Invalid field: {path}')
        message = getattr(message, name)
    return message


def format_message(value, *, truncate_length=128, no_arr=False, no_str=False, flow_style=False):
    from rosidl_runtime_py import message_to_yaml
    import yaml

    if hasattr(value, 'get_fields_and_field_types'):
        return message_to_yaml(
            value, truncate_length=truncate_length, no_arr=no_arr, no_str=no_str,
            flow_style=flow_style)

    def plain(item):
        from rosidl_runtime_py import message_to_ordereddict

        if hasattr(item, 'get_fields_and_field_types'):
            return plain(message_to_ordereddict(
                item, truncate_length=truncate_length, no_arr=no_arr, no_str=no_str))
        if isinstance(item, dict):
            return {key: plain(entry) for key, entry in item.items()}
        if isinstance(item, str):
            return item
        if hasattr(item, '__iter__'):
            if truncate_length and len(item) > truncate_length:
                return [plain(entry) for entry in item[:truncate_length]] + ['...']
            return [plain(entry) for entry in item]
        if hasattr(item, 'item'):
            return item.item()
        return item

    return yaml.safe_dump(plain(value), sort_keys=False, default_flow_style=flow_style)
