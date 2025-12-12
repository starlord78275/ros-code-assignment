from flask import Flask, request, jsonify, send_from_directory
from flask_cors import CORS
from werkzeug.utils import secure_filename
import os
from pathlib import Path
import sys

# Import our modules
sys.path.append('./backend')
from code_checker import ROSCodeChecker
from simulation_runner import SimulationRunner

app = Flask(__name__)
CORS(app)  # Enable CORS for React frontend

# Configuration
UPLOAD_FOLDER = './uploads'
ALLOWED_EXTENSIONS = {'zip'}
app.config['UPLOAD_FOLDER'] = UPLOAD_FOLDER
app.config['MAX_CONTENT_LENGTH'] = 50 * 1024 * 1024  # 50MB max

# Ensure upload folder exists
Path(UPLOAD_FOLDER).mkdir(exist_ok=True)

def allowed_file(filename):
    return '.' in filename and filename.rsplit('.', 1)[1].lower() in ALLOWED_EXTENSIONS

@app.route('/')
def home():
    return jsonify({
        "message": "ROS Code Checker API",
        "status": "running",
        "version": "1.0.0"
    })

@app.route('/api/health', methods=['GET'])
def health():
    return jsonify({"status": "healthy"})

@app.route('/api/upload', methods=['POST'])
def upload_code():
    """Upload and validate ROS package"""
    if 'file' not in request.files:
        return jsonify({"error": "No file provided"}), 400
    
    file = request.files['file']
    
    if file.filename == '':
        return jsonify({"error": "No file selected"}), 400
    
    if file and allowed_file(file.filename):
        filename = secure_filename(file.filename)
        filepath = os.path.join(app.config['UPLOAD_FOLDER'], filename)
        file.save(filepath)
        
        # Run code checker
        checker = ROSCodeChecker(upload_dir=UPLOAD_FOLDER)
        report = checker.validate_package(filepath)
        
        return jsonify({
            "message": "Package uploaded and validated",
            "report": report,
            "filename": filename
        })
    
    return jsonify({"error": "Invalid file type. Only .zip allowed"}), 400

@app.route('/api/validate/<filename>', methods=['GET'])
def validate_code(filename):
    """Validate already uploaded package"""
    filepath = os.path.join(app.config['UPLOAD_FOLDER'], secure_filename(filename))
    
    if not os.path.exists(filepath):
        return jsonify({"error": "File not found"}), 404
    
    checker = ROSCodeChecker(upload_dir=UPLOAD_FOLDER)
    report = checker.validate_package(filepath)
    
    return jsonify(report)

@app.route('/api/simulate', methods=['POST'])
def run_simulation():
    """Run simulation for validated package"""
    data = request.get_json()
    filename = data.get('filename')
    
    if not filename:
        return jsonify({"error": "No filename provided"}), 400
    
    filepath = os.path.join(app.config['UPLOAD_FOLDER'], secure_filename(filename))
    
    if not os.path.exists(filepath):
        return jsonify({"error": "File not found"}), 404
    
    # Run simulation
    runner = SimulationRunner()
    results = runner.run_node(filepath, data.get('node_name', 'main'))
    
    return jsonify(results)

@app.route('/api/report/<filename>', methods=['GET'])
def get_report(filename):
    """Get full report for a package"""
    filepath = os.path.join(app.config['UPLOAD_FOLDER'], secure_filename(filename))
    
    if not os.path.exists(filepath):
        return jsonify({"error": "File not found"}), 404
    
    checker = ROSCodeChecker(upload_dir=UPLOAD_FOLDER)
    report = checker.validate_package(filepath)
    text_report = checker.generate_report(format="text")
    
    return jsonify({
        "json_report": report,
        "text_report": text_report
    })

if __name__ == '__main__':
    print("\n" + "="*60)
    print("🚀 ROS Code Checker API Starting...")
    print("="*60)
    print("📡 Server: http://localhost:5000")
    print("📋 Health Check: http://localhost:5000/api/health")
    print("="*60 + "\n")
    
    app.run(debug=True, host='0.0.0.0', port=5000)
