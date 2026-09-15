"""Export a ROS 2 bag as a stable, long-form CSV file."""

from __future__ import annotations

import argparse
import csv
from collections.abc import Sequence
from pathlib import Path
import sys
from typing import Any, Iterator

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


BASE_COLUMNS = ('time_ns', 'time_s', 'topic', 'message_type')


def flatten_message(
    value: Any,
    prefix: str = '',
    output=None,
) -> dict[str, Any]:
    """Flatten nested ROS messages and arrays into stable CSV columns."""
    output = {} if output is None else output
    fields = getattr(value, 'get_fields_and_field_types', None)
    if callable(fields):
        for field_name in fields():
            child_prefix = f'{prefix}.{field_name}' if prefix else field_name
            flatten_message(getattr(value, field_name), child_prefix, output)
    elif (
        isinstance(value, Sequence)
        and not isinstance(value, (str, bytes, bytearray))
    ):
        for index, item in enumerate(value):
            flatten_message(item, f'{prefix}[{index}]', output)
    elif hasattr(value, 'tolist'):
        # Humble represents fixed-size numeric arrays as NumPy arrays.
        flatten_message(value.tolist(), prefix, output)
    else:
        output[prefix] = value
    return output


def _open_reader(bag_path: Path, storage_id: str):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_path), storage_id=storage_id),
        rosbag2_py.ConverterOptions('', ''),
    )
    return reader


def _rows(bag_path: Path, storage_id: str) -> Iterator[dict[str, Any]]:
    reader = _open_reader(bag_path, storage_id)
    topic_types = {}
    for topic in reader.get_all_topics_and_types():
        try:
            message_class = get_message(topic.type)
        except (AttributeError, ImportError, ModuleNotFoundError) as error:
            package = topic.type.split('/', maxsplit=1)[0]
            raise RuntimeError(
                f'cannot load {topic.type}; build and source the workspace '
                f'that provides {package} before converting the bag'
            ) from error
        topic_types[topic.name] = (topic.type, message_class)
    first_timestamp = None
    while reader.has_next():
        topic, serialized, timestamp = reader.read_next()
        first_timestamp = (
            timestamp if first_timestamp is None else first_timestamp)
        type_name, message_class = topic_types[topic]
        row = {
            'time_ns': timestamp,
            'time_s': (timestamp - first_timestamp) * 1e-9,
            'topic': topic,
            'message_type': type_name,
        }
        message = deserialize_message(serialized, message_class)
        row.update(flatten_message(message))
        yield row


def export_bag(
    bag_path: Path,
    output_path: Path,
    storage_id: str = 'sqlite3',
) -> int:
    """Export *bag_path* to *output_path* and return the message count."""
    if not bag_path.is_dir():
        raise FileNotFoundError(f'bag directory does not exist: {bag_path}')

    # Discover all columns first, then write in a second pass. This bounds
    # memory use even when an experiment contains millions of messages.
    dynamic_columns: set[str] = set()
    message_count = 0
    for row in _rows(bag_path, storage_id):
        dynamic_columns.update(row.keys())
        message_count += 1
    columns = [
        *BASE_COLUMNS,
        *sorted(dynamic_columns.difference(BASE_COLUMNS)),
    ]

    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=columns)
        writer.writeheader()
        writer.writerows(_rows(bag_path, storage_id))
    return message_count


def _arguments():
    parser = argparse.ArgumentParser(
        description=(
            'Convert every message in a ROS 2 bag to one long-form CSV row.'))
    parser.add_argument(
        'bag', help='rosbag directory containing metadata.yaml')
    parser.add_argument(
        '-o', '--output',
        help='output CSV (default: BAG/BAG_NAME.csv)',
    )
    parser.add_argument(
        '--storage-id', default='sqlite3',
        help='rosbag storage plugin (default: sqlite3)',
    )
    return parser.parse_args()


def main() -> int:
    """Parse command-line arguments and export the requested bag."""
    args = _arguments()
    bag_path = Path(args.bag).expanduser().resolve()
    output_path = (
        Path(args.output).expanduser().resolve()
        if args.output else bag_path / f'{bag_path.name}.csv'
    )
    try:
        count = export_bag(bag_path, output_path, args.storage_id)
    except (FileNotFoundError, RuntimeError) as error:
        print(f'bag_to_csv: error: {error}', file=sys.stderr)
        return 1
    print(f'Exported {count} messages to {output_path}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
