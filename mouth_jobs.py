# mouth_jobs.py
def process_mouth_position(x, y, z, face_points, mouth_open):
    """This function will be executed by RQ workers"""
    mouth_state = "OPEN" if mouth_open else "CLOSED"
    print(f"Processing mouth position: X={x}, Y={y}, Z={z}, Points={face_points}, Mouth={mouth_state}")
    
    
    
    return {"status": "processed", "x": x, "y": y, "z": z, "mouth_open": mouth_open}
