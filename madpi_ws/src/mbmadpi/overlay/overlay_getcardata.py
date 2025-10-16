"""
Car state aggregator for MAD76 camera overlay.
Collects live telemetry from ROS topics and management server.
"""

import time
import threading

class CarState:
    def __init__(self):
        self._cars = {}
        self._lock = threading.Lock()

    def update_from_outputs_ext_list(self, msg_list):
        """
        Update car state from CarOutputsExtList message.
        
        Args:
            msg_list: mbmadmsgs/CarOutputsExtList message with .list attribute
        """
        with self._lock:
            now = time.time()
            for entry in getattr(msg_list, 'list', []):
                cid = int(entry.carid)
                rec = self._cars.setdefault(cid, {})
                rec['car'] = cid
                rec['speed'] = float(getattr(entry, 'v', 0.0))
                s = getattr(entry, 's', None)
                if s and len(s) >= 2:
                    rec['pos_xy'] = (float(s[0]), float(s[1]))
                rec['prob'] = float(getattr(entry, 'prob', 0.0))
                rec['last_seen'] = now

    def update_from_ranking(self, ranking_json):
        """
        Update car state from management server ranking JSON.
        
        Args:
            ranking_json: list of {driver, laptime, avgspeed, active}
        """
        with self._lock:
            for idx, item in enumerate(ranking_json, start=1):
                rec = self._cars.setdefault(idx, {})
                rec['driver'] = item.get('driver')
                rec['pos'] = idx
                rec['laptime'] = item.get('laptime') or item.get('time')
                rec['avgspeed'] = item.get('avgspeed') or item.get('speed')

    def snapshot_list(self):
        """
        Get a snapshot of current car states as a list.
        
        Returns:
            List of dicts with keys: car, pos, driver, lap, time, speed, mode
        """
        with self._lock:
            out = []
            # sort by pos when available, else by carid
            items = sorted(self._cars.items(), key=lambda kv: kv[1].get('pos', kv[0]))
            for cid, rec in items:
                item = {
                    'car': rec.get('car', cid),
                    'pos': rec.get('pos', None) or cid,
                    'driver': rec.get('driver', '') or f'car_{cid}',
                    'lap': rec.get('lap', 0),
                    'time': rec.get('laptime', 0.0),
                    'speed': rec.get('speed', 0.0),
                    'mode': 'On Track' if rec.get('prob', 0.0) > 0.1 else 'Not On Track'
                }
                out.append(item)
            return out

CAR_STATE = CarState()
