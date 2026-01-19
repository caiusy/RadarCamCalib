import sqlite3
import os
import json
import sys

def inspect_db(db_path):
    if not os.path.exists(db_path):
        print(f"File not found: {db_path}")
        return

    print(f"Inspecting: {db_path}")
    conn = sqlite3.connect(db_path)
    cursor = conn.cursor()

    # Check tables
    cursor.execute("SELECT name FROM sqlite_master WHERE type='table';")
    tables = cursor.fetchall()
    print(f"Tables: {[t[0] for t in tables]}")

    # Check params
    try:
        cursor.execute("SELECT * FROM calibration_params")
        rows = cursor.fetchall()
        print(f"\n--- Calibration Params ({len(rows)}) ---")
        for r in rows:
            print(r)
    except Exception as e:
        print(f"Error reading params: {e}")

    # Check pairs
    try:
        cursor.execute("SELECT * FROM matched_pairs")
        rows = cursor.fetchall()
        print(f"\n--- Matched Pairs ({len(rows)}) ---")
        for r in rows:
            print(r)
    except Exception as e:
        print(f"Error reading pairs: {e}")

    # Check points
    try:
        cursor.execute("SELECT id, type, data FROM calibration_points")
        rows = cursor.fetchall()
        print(f"\n--- Calibration Points ({len(rows)}) ---")
        for r in rows:
            print(f"ID: {r[0]}, Type: {r[1]}")
            print(f"Data: {r[2]}")
    except Exception as e:
        print(f"Error reading calibration points: {e}")
        
    conn.close()

if __name__ == "__main__":
    db_path = "/Users/caius/Downloads/radardemo/dataset/radar_data.db"
    inspect_db(db_path)
