
# ROS Code Checker & Simulator

A web-based tool for validating ROS/ROS2 packages and running robotic arm simulations. Built for MavenAI Technologies Robotics Internship.

## 🚀 Features

- ✅ **Code Syntax Validation** - Python (flake8) and C++ (g++) checking
- 📦 **ROS Package Structure Validation** - Verifies package.xml, CMakeLists.txt, setup.py
- 🔍 **ROS Component Detection** - Finds publishers, subscribers, services, node initialization
- 🛡️ **Safety Checks** - Detects unsafe joint values, missing rate limiting, infinite loops
- 🤖 **Gazebo Simulation Integration** - Run validated code in UR5 simulation
- 📊 **Detailed Reports** - JSON and text format validation reports
- 🌐 **Modern Web Interface** - Clean, responsive UI with real-time feedback

## 🛠️ Tech Stack

**Backend:**
- Python 3.8+
- Flask (REST API)
- ROS2 Jazzy/Humble (Ubuntu 24.04/22.04)

**Frontend:**
- React.js
- Modern CSS (Tech/Robotics theme)

**Tools:**
- flake8 (Python linting)
- g++ (C++ syntax checking)
- Gazebo (Robot simulation)




```
## 📁 Project Structure
ros-code-checker/
├── backend/
│   ├── code_checker.py       # Main validation logic
│   ├── simulation_runner.py  # Gazebo simulation controller
│   └── utils/                # Helper modules
├── frontend/
│   ├── src/
│   │   ├── App.js           # Main React component
│   │   └── App.css          # Styling
│   └── package.json
├── test_packages/
│   ├── correct_package/     # Valid ROS package (pick-and-place)
│   ├── faulty_package/      # Invalid package with errors
│   ├── correct_package.zip
│   └── faulty_package.zip
├── uploads/                 # Uploaded packages directory
├── app.py                   # Flask API server
├── requirements.txt
└── README.md
```
```

## ⚙️ Installation

### Prerequisites

- Ubuntu 24.04 (Noble) or Ubuntu 22.04 (Jammy)
- Python 3.8+
- Node.js 18+

### Step 1: Install ROS2 Jazzy (Ubuntu 24.04)

```
```
# Set up locale
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Enable Ubuntu Universe repository
sudo apt install software-properties-common
sudo add-apt-repository universe

# Add ROS2 repository
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2 Jazzy Desktop
sudo apt update
sudo apt upgrade
sudo apt install ros-jazzy-desktop

# Source ROS2
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Install development tools
sudo apt install python3-rosdep python3-colcon-common-extensions
sudo rosdep init
rosdep update
```
```

**For Ubuntu 22.04, use ROS2 Humble instead:**

sudo apt install ros-humble-desktop
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Step 2: Install Gazebo & ROS Packages

```
# Update package list
sudo apt-get update

# Install Gazebo-ROS bridge and packages
sudo apt install ros-humble-gazebo-ros-pkgs
sudo apt install ros-humble-ros-gz

# Install robotic arm packages (optional)
sudo apt-get install ros-humble-bcr-arm

# Verify Gazebo installation
gazebo --version
```

### Step 3: Backend Setup

```
# Clone repository
git clone https://github.com/YOUR_USERNAME/ros-code-checker.git
cd ros-code-checker

# Install Python dependencies
pip install -r requirements.txt

# Install code checking tools
sudo apt install flake8 g++ build-essential

# Create uploads directory
mkdir -p uploads

# Run Flask server
python3 app.py
```

Server runs on `http://localhost:5000`

### Step 4: Frontend Setup

```
# Navigate to frontend directory
cd frontend

# Install Node.js dependencies
npm install

# Start React development server
npm start
```

Frontend runs on `http://localhost:3000`

### Step 5: Verify Installation

**Test ROS2:**
```
ros2 --help
```

**Test Backend API:**
```
curl http://localhost:5000/api/health
```

**Test Frontend:**
Open browser to `http://localhost:3000`

## 🎮 Usage

### 1. Upload Package
- Click "Choose ZIP file"
- Select your ROS package (must be .zip)
- Click "Upload & Validate"

### 2. View Report
- Check validation status (PASSED/FAILED/WARNING)
- Review detailed check results:
  - ✅ **Syntax** - Python flake8 and C++ compilation checks
  - ✅ **Structure** - package.xml, CMakeLists.txt/setup.py verification
  - ✅ **ROS Components** - Node initialization, publishers, subscribers detection
  - ✅ **Safety** - Joint value ranges, rate limiting, infinite loop detection

### 3. Run Simulation
- If validation passes, click "Run Simulation"
- View real-time simulation logs
- Analyze joint motion data in tabular format

## 📦 Test Packages

Two test packages are included for demonstration:

### Correct Package (`correct_package.zip`)
- ✅ Proper ROS2 node initialization with rclpy
- ✅ Safe joint values (< 2π radians)
- ✅ Rate limiting in control loops
- ✅ Complete package structure (package.xml, setup.py)
- ✅ Clean code style (PEP 8 compliant)
- ✅ Proper publishers and subscribers
- **Use Case:** Pick-and-place manipulation task

### Faulty Package (`faulty_package.zip`)
- ❌ Missing ROS node initialization
- ❌ Unsafe joint values (>10 radians)
- ❌ No rate limiting (infinite loop without sleep)
- ❌ Missing setup.py build file
- ❌ Style violations (spacing, naming)
- ❌ Missing sensor_msgs imports
- **Purpose:** Demonstrates error detection capabilities

## 🔌 API Endpoints

| Method | Endpoint | Description |
|--------|----------|-------------|
| GET | `/` | API status and version |
| GET | `/api/health` | Health check endpoint |
| POST | `/api/upload` | Upload and validate ROS package |
| GET | `/api/validate/<filename>` | Re-validate existing package |
| POST | `/api/simulate` | Run Gazebo simulation |
| GET | `/api/report/<filename>` | Get detailed validation report |

**Example Request:**
```
# Upload package
curl -X POST -F "file=@correct_package.zip" http://localhost:5000/api/upload

# Check health
curl http://localhost:5000/api/health
```

## 🎯 Assignment Requirements

This project fulfills all requirements from MavenAI Technologies Robotics Internship Task 1:

- [x] **Code Checker Script**
  - Syntax validation (flake8, g++)
  - ROS structure check (package.xml, CMakeLists.txt)
  - Component detection (publishers, subscribers, services)
  - Safety heuristics (joint values, rate limiting)
  - JSON and text report generation

- [x] **Simulation Runner**
  - Gazebo integration with UR5 robotic arm
  - Scene creation (cube, target position)
  - Joint motion recording
  - Success/failure detection

- [x] **Minimal Web Interface**
  - Package upload functionality
  - Validation report display
  - Simulation trigger and visualization
  - Modern, responsive design

- [x] **Deliverables**
  - GitHub repository with complete code
  - Two test packages (correct and faulty)
  - Comprehensive documentation
  - Demo video (link below)

## 🔧 Troubleshooting

### ROS2 Not Found
```
source /opt/ros/jazzy/setup.bash
# or for Ubuntu 22.04
source /opt/ros/humble/setup.bash
```

### Gazebo Not Starting
```
sudo apt install gazebo
gazebo --verbose
```

### Port Already in Use
```
# Kill process on port 5000
sudo lsof -t -i:5000 | xargs kill -9

# Or change port in app.py
app.run(debug=True, host='0.0.0.0', port=5001)
```

### flake8 Not Found
```
pip install flake8
# or
sudo apt install flake8
```

### CORS Errors in Browser
Make sure both Flask (port 5000) and React (port 3000) are running:
```
# Terminal 1
python3 app.py

# Terminal 2
cd frontend && npm start
```

### Upload Fails
Check that uploads directory exists:
```
mkdir -p uploads
chmod 755 uploads
```

## 🏗️ Development

### Running Tests
```
# Test code checker
cd backend
python3 code_checker.py

# Test simulation runner
python3 simulation_runner.py
```

### Adding New Checks
Edit `backend/code_checker.py` and add methods to `ROSCodeChecker` class:
```
def check_custom_rule(self, package_path: str) -> bool:
    # Your validation logic
    pass
```

### Modifying UI
Edit `frontend/src/App.css` for styling or `frontend/src/App.js` for functionality.

## 📝 Code Validation Rules

### Syntax Checks
- Python: flake8 with max line length 120
- C++: g++ syntax-only compilation with C++14 standard

### Structure Checks
- Required: package.xml (ROS package manifest)
- Required: CMakeLists.txt OR setup.py (build configuration)

### ROS Component Checks
- Node initialization: `rospy.init_node()` or `rclpy.init()`
- Publishers: `.Publisher()` declarations
- Subscribers: `.Subscriber()` declarations
- Services: `.Service()` declarations

### Safety Checks
- Joint values should be < 6.28 radians (2π)
- Loops must contain rate limiting (`rate.sleep()`, `time.sleep()`)
- No infinite loops without sleep

## 👨‍💻 Author

Built by Nitin Gavande for MavenAI Technologies Robotics Internship

**Contact:**
- Email: bhimgavande.777@.com
- GitHub: [starlord78275](https://github.com/starlord78275/ros-code-assignment.git)
- LinkedIn: https://www.linkedin.com/in/nitin-gavande-891a7b31a/

## 📄 License

MIT License

Copyright (c) 2025 [Nitin Gavande]

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.


---

**⭐ If you found this project helpful, please give it a star on GitHub!**

