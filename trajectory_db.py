"""
trajectory_db.py - SQLite-based trajectory storage for real-time query
轨迹数据库 - 使用SQLite存储以支持实时查询

Stores all radar target positions across frames for trajectory visualization.
Also stores camera detections for matched visualization.
"""

import sqlite3
import os
import json
from typing import List, Tuple, Dict, Optional


class TrajectoryDB:
    """
    SQLite database for radar/camera trajectories and calibration state.
    Can be in-memory or file-based.
    """
    
    def __init__(self, db_path: Optional[str] = None):
        self.db_path = db_path
        print(f"[TrajectoryDB] Init with path: {db_path}")
        if db_path:
            self.conn = sqlite3.connect(db_path)
        else:
            self.conn = sqlite3.connect(':memory:')
            
        self.cursor = self.conn.cursor()
        self._create_schema()
        
        # If file-based and has data, mark as loaded
        if db_path and os.path.exists(db_path):
            self.cursor.execute("SELECT count(*) FROM radar_trajectories")
            count = self.cursor.fetchone()[0]
            self.loaded = count > 0
            print(f"[TrajectoryDB] DB loaded status: {self.loaded} (rows: {count})")
        else:
            self.loaded = False
    
    def add_matched_pair(self, radar_id: int, camera_id: int):
        """Add a matched pair to the database."""
        try:
            print(f"[TrajectoryDB] Adding pair R{radar_id}-C{camera_id}...")
            self.cursor.execute('''
                INSERT OR IGNORE INTO matched_pairs (radar_id, camera_id)
                VALUES (?, ?)
            ''', (radar_id, camera_id))
            self.conn.commit()
            print(f"[TrajectoryDB] Pair R{radar_id}-C{camera_id} saved.")
        except Exception as e:
            print(f"[TrajectoryDB] Error adding pair: {e}")
            
    def get_matched_pairs(self) -> List[Tuple[int, int]]:
        """Get all matched pairs."""
        try:
            self.cursor.execute('SELECT radar_id, camera_id FROM matched_pairs')
            rows = self.cursor.fetchall()
            print(f"[TrajectoryDB] Retrieved {len(rows)} matched pairs")
            return rows
        except Exception as e:
            print(f"[TrajectoryDB] Error getting pairs: {e}")
            return []
        
    def _create_schema(self):
        """Create database schema."""
        # Radar trajectories
        self.cursor.execute('''
            CREATE TABLE IF NOT EXISTS radar_trajectories (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                target_id INTEGER,
                frame_id INTEGER,
                x REAL,
                y REAL,
                range_val REAL,
                velocity REAL,
                rcs REAL
            )
        ''')
        self.cursor.execute('CREATE INDEX IF NOT EXISTS idx_radar_target ON radar_trajectories(target_id)')
        self.cursor.execute('CREATE INDEX IF NOT EXISTS idx_radar_frame ON radar_trajectories(frame_id)')
        
        # Camera trajectories
        self.cursor.execute('''
            CREATE TABLE IF NOT EXISTS camera_trajectories (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                target_id INTEGER,
                frame_id INTEGER,
                u REAL,
                v REAL,
                x_bev REAL,
                y_bev REAL,
                confidence REAL
            )
        ''')
        self.cursor.execute('CREATE INDEX IF NOT EXISTS idx_camera_target ON camera_trajectories(target_id)')
        self.cursor.execute('CREATE INDEX IF NOT EXISTS idx_camera_frame ON camera_trajectories(frame_id)')
        
        # Matched pairs (Persistent)
        self.cursor.execute('''
            CREATE TABLE IF NOT EXISTS matched_pairs (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                radar_id INTEGER,
                camera_id INTEGER,
                UNIQUE(radar_id, camera_id)
            )
        ''')
        
        # Calibration Parameters (Persistent)
        self.cursor.execute('''
            CREATE TABLE IF NOT EXISTS calibration_params (
                key TEXT PRIMARY KEY,
                value TEXT
            )
        ''')
        
        # Calibration Points (Persistent)
        self.cursor.execute('''
            CREATE TABLE IF NOT EXISTS calibration_points (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                type TEXT, -- 'pair' or 'lane'
                data TEXT  -- JSON string
            )
        ''')
        
        self.conn.commit()
        
    def close(self):
        """Close database connection."""
        if self.conn:
            self.conn.close()
            
    def clear(self):
        """Clear trajectory data (but keep persistent data if file-based)."""
        self.cursor.execute('DELETE FROM radar_trajectories')
        self.cursor.execute('DELETE FROM camera_trajectories')
        # If in-memory, we also clear persistent tables to be safe/clean
        # If file-based, we might want to keep matched_pairs/calibration?
        # For now, let's keep matched_pairs and calibration even for in-memory 
        # (assuming it's a session DB).
        self.conn.commit()
        self.loaded = False
        
    def save_calibration_state(self, camera_params: dict, radar_params: dict):
        """Save calibration parameters to DB."""
        try:
            self.cursor.execute('INSERT OR REPLACE INTO calibration_params (key, value) VALUES (?, ?)', 
                              ('camera', json.dumps(camera_params)))
            self.cursor.execute('INSERT OR REPLACE INTO calibration_params (key, value) VALUES (?, ?)', 
                              ('radar', json.dumps(radar_params)))
            self.conn.commit()
        except Exception as e:
            print(f"[TrajectoryDB] Error saving calibration: {e}")
            
    def load_calibration_state(self) -> Tuple[dict, dict]:
        """Load calibration parameters from DB. Returns (camera, radar) dicts."""
        self.cursor.execute('SELECT key, value FROM calibration_params')
        rows = self.cursor.fetchall()
        camera = {}
        radar = {}
        for key, val in rows:
            if key == 'camera':
                camera = json.loads(val)
            elif key == 'radar':
                radar = json.loads(val)
        return camera, radar
        
    def save_calibration_points(self, point_pairs: list, lanes: list = None):
        """Save selected point pairs and lanes (expects list of dicts)."""
        try:
            self.cursor.execute('DELETE FROM calibration_points')
            
            # Save pairs
            for p_dict in point_pairs:
                self.cursor.execute('INSERT INTO calibration_points (type, data) VALUES (?, ?)', 
                                  ('pair', json.dumps(p_dict)))
            
            # Save lanes
            if lanes:
                for l_dict in lanes:
                    self.cursor.execute('INSERT INTO calibration_points (type, data) VALUES (?, ?)', 
                                      ('lane', json.dumps(l_dict)))
            
            self.conn.commit()
        except Exception as e:
            print(f"[TrajectoryDB] Error saving points: {e}")
            
    def load_calibration_points(self) -> List[dict]:
        """Load calibration points. Returns list of dicts."""
        self.cursor.execute('SELECT data FROM calibration_points WHERE type="pair"')
        return [json.loads(row[0]) for row in self.cursor.fetchall()]
        """Add a matched pair to the database."""
        try:
            self.cursor.execute('''
                INSERT OR IGNORE INTO matched_pairs (radar_id, camera_id)
                VALUES (?, ?)
            ''', (radar_id, camera_id))
            self.conn.commit()
        except Exception as e:
            print(f"[TrajectoryDB] Error adding pair: {e}")
            
    def get_matched_pairs(self) -> List[Tuple[int, int]]:
        """Get all matched pairs."""
        self.cursor.execute('SELECT radar_id, camera_id FROM matched_pairs')
        return self.cursor.fetchall()
        
    def remove_matched_pair(self, radar_id: int, camera_id: int):
        """Remove a matched pair."""
        self.cursor.execute('''
            DELETE FROM matched_pairs 
            WHERE radar_id = ? AND camera_id = ?
        ''', (radar_id, camera_id))
        self.conn.commit()
    
    def save_pairs_to_disk(self, filepath: str = 'matched_pairs.json'):
        """Save matched pairs to a JSON file."""
        pairs = self.get_matched_pairs()
        data = [{'radar_id': r, 'camera_id': c} for r, c in pairs]
        try:
            with open(filepath, 'w') as f:
                json.dump(data, f, indent=2)
            print(f"[TrajectoryDB] Saved {len(pairs)} pairs to {filepath}")
        except Exception as e:
            print(f"[TrajectoryDB] Error saving pairs: {e}")
            
    def load_pairs_from_disk(self, filepath: str = 'matched_pairs.json'):
        """Load matched pairs from a JSON file."""
        if not os.path.exists(filepath):
            return
        try:
            with open(filepath, 'r') as f:
                data = json.load(f)
            count = 0
            for item in data:
                self.add_matched_pair(item.get('radar_id'), item.get('camera_id'))
                count += 1
            print(f"[TrajectoryDB] Loaded {count} pairs from {filepath}")
        except Exception as e:
            print(f"[TrajectoryDB] Error loading pairs: {e}")

    def load_all_radar_files(self, data_root: str) -> int:
        """
        Load all radar JSON files from data_root/radar/ directory.
        Returns number of trajectory points loaded.
        """
        self.clear()
        
        radar_dir = os.path.join(data_root, 'radar')
        camera_dir = os.path.join(data_root, 'camera')
        
        count = 0
        
        # Load radar data
        if os.path.exists(radar_dir):
            for fname in sorted(os.listdir(radar_dir)):
                if not fname.endswith('.json'):
                    continue
                    
                try:
                    frame_id = int(os.path.splitext(fname)[0])
                except ValueError:
                    continue
                    
                fpath = os.path.join(radar_dir, fname)
                try:
                    with open(fpath, 'r', encoding='utf-8') as f:
                        data = json.load(f)
                        
                    targets = data.get('targets', [])
                    for t in targets:
                        self.cursor.execute('''
                            INSERT INTO radar_trajectories 
                            (target_id, frame_id, x, y, range_val, velocity, rcs)
                            VALUES (?, ?, ?, ?, ?, ?, ?)
                        ''', (
                            t.get('id', 0),
                            frame_id,
                            t.get('x', 0.0),
                            t.get('y', 0.0),
                            t.get('range', 0.0),
                            t.get('velocity', 0.0),
                            t.get('rcs', 0.0)
                        ))
                        count += 1
                except Exception as e:
                    print(f"[TrajectoryDB] Error loading radar {fname}: {e}")
        
        # Load camera data
        if os.path.exists(camera_dir):
            for fname in sorted(os.listdir(camera_dir)):
                if not fname.endswith('.json'):
                    continue
                    
                try:
                    frame_id = int(os.path.splitext(fname)[0])
                except ValueError:
                    continue
                    
                fpath = os.path.join(camera_dir, fname)
                try:
                    with open(fpath, 'r', encoding='utf-8') as f:
                        data = json.load(f)
                        
                    detections = data.get('detections', [])
                    for d in detections:
                        self.cursor.execute('''
                            INSERT INTO camera_trajectories 
                            (target_id, frame_id, u, v, x_bev, y_bev, confidence)
                            VALUES (?, ?, ?, ?, ?, ?, ?)
                        ''', (
                            d.get('id', 0),
                            frame_id,
                            d.get('u', 0.0),
                            d.get('v', 0.0),
                            d.get('x_bev', 0.0),
                            d.get('y_bev', 0.0),
                            d.get('confidence', 0.0)
                        ))
                except Exception as e:
                    print(f"[TrajectoryDB] Error loading camera {fname}: {e}")
                        
        self.conn.commit()
        self.loaded = True
        print(f"[TrajectoryDB] Loaded {count} radar points")
        return count
        
    def get_all_target_ids(self) -> List[int]:
        """Get list of unique target IDs (from radar)."""
        self.cursor.execute('SELECT DISTINCT target_id FROM radar_trajectories ORDER BY target_id')
        return [row[0] for row in self.cursor.fetchall()]
        
    def get_trajectory(self, target_id: int) -> List[Tuple[int, float, float]]:
        """
        Get all radar points for a target ID.
        Returns list of (frame_id, x, y) sorted by frame.
        """
        self.cursor.execute('''
            SELECT frame_id, x, y FROM radar_trajectories 
            WHERE target_id = ? ORDER BY frame_id
        ''', (target_id,))
        return self.cursor.fetchall()
    
    def get_camera_trajectory(self, target_id: int) -> List[Tuple[int, float, float, float, float]]:
        """
        Get all camera points for a target ID.
        Returns list of (frame_id, u, v, x_bev, y_bev) sorted by frame.
        """
        self.cursor.execute('''
            SELECT frame_id, u, v, x_bev, y_bev FROM camera_trajectories 
            WHERE target_id = ? ORDER BY frame_id
        ''', (target_id,))
        return self.cursor.fetchall()
        
    def get_all_trajectories(self) -> Dict[int, List[Tuple[int, float, float]]]:
        """Get all radar trajectories grouped by target_id."""
        target_ids = self.get_all_target_ids()
        result = {}
        for tid in target_ids:
            result[tid] = self.get_trajectory(tid)
        return result
    
    def get_all_camera_trajectories(self) -> Dict[int, List[Tuple[int, float, float, float, float]]]:
        """Get all camera trajectories grouped by target_id."""
        target_ids = self.get_all_target_ids()
        result = {}
        for tid in target_ids:
            traj = self.get_camera_trajectory(tid)
            if traj:
                result[tid] = traj
        return result
        
    def get_point_at_frame(self, target_id: int, frame_id: int) -> Optional[Tuple[float, float, float, float, float]]:
        """
        Get radar target position at specific frame.
        Returns (x, y, range, velocity, rcs) or None if not found.
        """
        self.cursor.execute('''
            SELECT x, y, range_val, velocity, rcs FROM radar_trajectories 
            WHERE target_id = ? AND frame_id = ?
        ''', (target_id, frame_id))
        row = self.cursor.fetchone()
        return row if row else None
    
    def get_camera_point_at_frame(self, target_id: int, frame_id: int) -> Optional[Tuple[float, float]]:
        """
        Get camera detection pixel position at specific frame.
        Returns (u, v) or None if not found.
        """
        self.cursor.execute('''
            SELECT u, v FROM camera_trajectories 
            WHERE target_id = ? AND frame_id = ?
        ''', (target_id, frame_id))
        row = self.cursor.fetchone()
        return row if row else None
        
    def get_frame_count(self) -> int:
        """Get number of unique frames."""
        self.cursor.execute('SELECT COUNT(DISTINCT frame_id) FROM radar_trajectories')
        return self.cursor.fetchone()[0]
        
    def get_target_count(self) -> int:
        """Get number of unique targets."""
        self.cursor.execute('SELECT COUNT(DISTINCT target_id) FROM radar_trajectories')
        return self.cursor.fetchone()[0]
        
    def find_nearest_point(self, x_bev: float, y_bev: float, max_dist: float = 5.0) -> Optional[Tuple[int, int, float, float]]:
        """
        Find nearest trajectory point to given BEV coordinates.
        Returns (target_id, frame_id, x, y) or None if no point within max_dist.
        """
        self.cursor.execute('''
            SELECT target_id, frame_id, x, y,
                   ((x - ?) * (x - ?) + (y - ?) * (y - ?)) as dist_sq
            FROM radar_trajectories
            ORDER BY dist_sq
            LIMIT 1
        ''', (x_bev, x_bev, y_bev, y_bev))
        
        row = self.cursor.fetchone()
        if row and row[4] <= max_dist * max_dist:
            return (row[0], row[1], row[2], row[3])
        return None
