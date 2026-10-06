"""Replay recorded sensors only until the first changed route decision.

The old trajectory ceases to be valid after that tick. This tool deliberately
stops there and makes no solved or travel claim.
"""
import csv
import sys
import types

try:
    import paho.mqtt.client
except ImportError:
    paho = types.ModuleType('paho')
    mqtt = types.ModuleType('paho.mqtt')
    client = types.ModuleType('paho.mqtt.client')
    paho.mqtt = mqtt
    mqtt.client = client
    sys.modules.update({'paho': paho, 'paho.mqtt': mqtt,
                        'paho.mqtt.client': client})

from task_1b_boilerplate import CenteringController


def first_divergence(path):
    ctl = CenteringController()
    with open(path, newline='') as f:
        for row in csv.DictReader(f):
            vals = [float(row[k]) if row[k] else None
                    for k in ('fl', 'fr', 'sl', 'sr')]
            ctl.update(*vals, float(row.get('gyro_z') or 0),
                       float(row.get('dt_rep') or 0))
            old = row.get('state')
            if ctl.state != old:
                return {'t_wall': row.get('t_wall'), 'old': old,
                        'new': ctl.state, 'reason': ctl.state_reason,
                        'observation': ctl.observation}
    return None


if __name__ == '__main__':
    for path in sys.argv[1:]:
        print(path, first_divergence(path))
