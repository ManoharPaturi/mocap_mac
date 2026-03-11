import sqlite3

try:
    conn = sqlite3.connect("mocap_data.db")
    cursor = conn.cursor()

    # STRICTLY grab only the dynamically generated data tables (e.g. session_2026...)
    cursor.execute("SELECT name FROM sqlite_master WHERE type='table' AND name LIKE 'session_202%' ORDER BY name DESC LIMIT 1")
    session_table = cursor.fetchone()

    if session_table:
        table_name = session_table[0]
        print(f"🔥 Checking Table: {table_name}")
        
        # Count rows where PC2 data actually exists and isn't empty
        cursor.execute(f"SELECT COUNT(*) FROM {table_name} WHERE pc2_pose_data != '[]' AND pc2_pose_data IS NOT NULL")
        pc2_count = cursor.fetchone()[0]
        
        # Count rows where 3D data exists
        cursor.execute(f"SELECT COUNT(*) FROM {table_name} WHERE pose_3d_data != '[]' AND pose_3d_data IS NOT NULL")
        pose3d_count = cursor.fetchone()[0]
        
        print(f"✅ 3D Frames Saved: {pose3d_count}")
        print(f"👀 PC2 (Windows) Frames Saved: {pc2_count}")
        
    else:
        print("No session tables found!")

except Exception as e:
    print(f"Error: {e}")