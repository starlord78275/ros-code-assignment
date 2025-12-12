import os
import json
import zipfile
import tempfile
import subprocess
from pathlib import Path
from datetime import datetime
from typing import Dict, List, Tuple

class ROSCodeChecker:
    """Main class for checking ROS/ROS2 code packages"""
    
    def __init__(self, upload_dir: str = "./uploads"):
        self.upload_dir = Path(upload_dir)
        self.upload_dir.mkdir(exist_ok=True)
        self.report = {
            "timestamp": "",
            "package_name": "",
            "status": "pending",
            "errors": [],
            "warnings": [],
            "info": [],
            "checks": {
                "syntax": {"passed": False, "details": []},
                "structure": {"passed": False, "details": []},
                "ros_components": {"passed": False, "details": []},
                "safety": {"passed": False, "details": []}
            }
        }
    
    def extract_package(self, zip_path: str) -> Tuple[bool, str]:
        """Extract uploaded ZIP file"""
        try:
            extract_path = tempfile.mkdtemp(dir=self.upload_dir)
            with zipfile.ZipFile(zip_path, 'r') as zip_ref:
                zip_ref.extractall(extract_path)
            return True, extract_path
        except Exception as e:
            self.report["errors"].append(f"Extraction failed: {str(e)}")
            return False, ""
    
    def check_syntax(self, package_path: str) -> bool:
        """Check Python syntax with flake8 and C++ with g++"""
        passed = True
        details = []
        
        # Check Python files
        python_files = list(Path(package_path).rglob("*.py"))
        for py_file in python_files:
            try:
                result = subprocess.run(
                    ['flake8', '--max-line-length=120', str(py_file)],
                    capture_output=True, text=True, timeout=10
                )
                if result.returncode != 0:
                    passed = False
                    details.append(f"Flake8 errors in {py_file.name}: {result.stdout}")
                else:
                    details.append(f"✓ {py_file.name} passed syntax check")
            except subprocess.TimeoutExpired:
                details.append(f"⚠ Timeout checking {py_file.name}")
            except FileNotFoundError:
                details.append("⚠ flake8 not installed, skipping Python syntax check")
                break
        
        # Check C++ files
        cpp_files = list(Path(package_path).rglob("*.cpp"))
        for cpp_file in cpp_files:
            try:
                result = subprocess.run(
                    ['g++', '-fsyntax-only', '-std=c++14', str(cpp_file)],
                    capture_output=True, text=True, timeout=10
                )
                if result.returncode != 0:
                    passed = False
                    details.append(f"C++ errors in {cpp_file.name}: {result.stderr}")
                else:
                    details.append(f"✓ {cpp_file.name} passed syntax check")
            except subprocess.TimeoutExpired:
                details.append(f"⚠ Timeout checking {cpp_file.name}")
            except FileNotFoundError:
                details.append("⚠ g++ not installed, skipping C++ syntax check")
                break
        
        self.report["checks"]["syntax"]["passed"] = passed
        self.report["checks"]["syntax"]["details"] = details
        return passed
    
    def check_ros_structure(self, package_path: str) -> bool:
        """Validate ROS package structure"""
        passed = True
        details = []
        package_path = Path(package_path)
        
        # Check for package.xml
        package_xml = package_path / "package.xml"
        if not package_xml.exists():
            package_xmls = list(package_path.rglob("package.xml"))
            if package_xmls:
                package_xml = package_xmls[0]
                details.append(f"✓ Found package.xml at {package_xml.relative_to(package_path)}")
            else:
                passed = False
                details.append("✗ Missing package.xml")
        else:
            details.append("✓ Found package.xml")
        
        # Check for build files
        cmake = list(package_path.rglob("CMakeLists.txt"))
        setup_py = list(package_path.rglob("setup.py"))
        
        if cmake or setup_py:
            details.append(f"✓ Found build configuration")
        else:
            passed = False
            details.append("✗ Missing CMakeLists.txt or setup.py")
        
        self.report["checks"]["structure"]["passed"] = passed
        self.report["checks"]["structure"]["details"] = details
        return passed
    
    def check_ros_components(self, package_path: str) -> bool:
        """Detect ROS publishers, subscribers, services"""
        details = []
        components_found = {
            "init_node": False,
            "publishers": [],
            "subscribers": [],
            "services": []
        }
        
        python_files = list(Path(package_path).rglob("*.py"))
        for py_file in python_files:
            try:
                with open(py_file, 'r') as f:
                    content = f.read()
                    
                    # Check for ROS1/ROS2 initialization
                    if 'rospy.init_node' in content or 'rclpy.init' in content:
                        components_found["init_node"] = True
                    
                    # Check for publishers
                    if 'Publisher(' in content or '.Publisher(' in content:
                        components_found["publishers"].append(py_file.name)
                    
                    # Check for subscribers
                    if 'Subscriber(' in content or '.Subscriber(' in content:
                        components_found["subscribers"].append(py_file.name)
                    
                    # Check for services
                    if 'Service(' in content or '.Service(' in content:
                        components_found["services"].append(py_file.name)
            except Exception as e:
                details.append(f"⚠ Could not read {py_file.name}: {str(e)}")
        
        if components_found["init_node"]:
            details.append("✓ Found ROS node initialization")
        else:
            details.append("⚠ No ROS node initialization found")
        
        if components_found["publishers"]:
            details.append(f"✓ Found publishers in: {', '.join(components_found['publishers'])}")
        
        if components_found["subscribers"]:
            details.append(f"✓ Found subscribers in: {', '.join(components_found['subscribers'])}")
        
        if components_found["services"]:
            details.append(f"✓ Found services in: {', '.join(components_found['services'])}")
        
        passed = components_found["init_node"]
        self.report["checks"]["ros_components"]["passed"] = passed
        self.report["checks"]["ros_components"]["details"] = details
        return passed
    
    def check_safety(self, package_path: str) -> bool:
        """Check for basic safety issues in robot control code"""
        passed = True
        details = []
        
        python_files = list(Path(package_path).rglob("*.py"))
        for py_file in python_files:
            try:
                with open(py_file, 'r') as f:
                    content = f.read()
                    lines = content.split('\n')
                    
                    in_loop = False
                    loop_has_sleep = False
                    
                    for i, line in enumerate(lines):
                        if 'while' in line or 'for' in line:
                            in_loop = True
                            loop_has_sleep = False
                        
                        if in_loop and ('rate.sleep()' in line or 'rospy.sleep' in line or 'time.sleep' in line):
                            loop_has_sleep = True
                        
                        # Check for large joint values
                        if 'joint' in line.lower() and any(op in line for op in ['=', 'append', 'publish']):
                            import re
                            numbers = re.findall(r'[-+]?\d*\.\d+|[-+]?\d+', line)
                            for num in numbers:
                                try:
                                    val = float(num)
                                    if abs(val) > 6.28:
                                        details.append(f"⚠ Line {i+1} in {py_file.name}: Large joint value {val} (check if in radians)")
                                except:
                                    pass
                    
                    if in_loop and not loop_has_sleep:
                        details.append(f"⚠ {py_file.name}: Loop without sleep/rate - may cause high CPU usage")
                
            except Exception as e:
                details.append(f"⚠ Could not analyze {py_file.name}: {str(e)}")
        
        if not details:
            details.append("✓ No obvious safety issues detected")
        
        self.report["checks"]["safety"]["passed"] = passed
        self.report["checks"]["safety"]["details"] = details
        return passed
    
    def validate_package(self, zip_path: str) -> Dict:
        """Main validation method"""
        self.report["timestamp"] = datetime.now().isoformat()
        
        success, package_path = self.extract_package(zip_path)
        if not success:
            self.report["status"] = "failed"
            return self.report
        
        package_name = Path(zip_path).stem
        self.report["package_name"] = package_name
        
        # Run all checks
        syntax_ok = self.check_syntax(package_path)
        structure_ok = self.check_ros_structure(package_path)
        components_ok = self.check_ros_components(package_path)
        safety_ok = self.check_safety(package_path)
        
        # Determine overall status
        if syntax_ok and structure_ok and components_ok:
            self.report["status"] = "passed"
        elif structure_ok and components_ok:
            self.report["status"] = "warning"
        else:
            self.report["status"] = "failed"
        
        return self.report
    
    def generate_report(self, format: str = "json") -> str:
        """Generate validation report"""
        if format == "json":
            return json.dumps(self.report, indent=2)
        else:
            lines = []
            lines.append("="*60)
            lines.append("ROS CODE VALIDATION REPORT")
            lines.append("="*60)
            lines.append(f"Package: {self.report['package_name']}")
            lines.append(f"Timestamp: {self.report['timestamp']}")
            lines.append(f"Status: {self.report['status'].upper()}")
            lines.append("="*60)
            
            for check_name, check_data in self.report["checks"].items():
                lines.append(f"\n{check_name.upper()} CHECK: {'PASSED' if check_data['passed'] else 'FAILED'}")
                for detail in check_data['details']:
                    lines.append(f"  {detail}")
            
            return "\n".join(lines)

if __name__ == "__main__":
    checker = ROSCodeChecker()
    print("ROS Code Checker initialized")
