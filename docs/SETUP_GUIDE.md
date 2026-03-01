# 🚀 Setup Guide - Platform-Specific Instructions

This guide helps **all team members** set up the RoboCup UR5e system on different platforms.

---

## 📋 Table of Contents

1. [Prerequisites](#prerequisites)
2. [Platform-Specific Setup](#platform-specific-setup)
   - [Ubuntu 22.04 (Recommended)](#ubuntu-2204-recommended)
   - [Windows WSL2](#windows-wsl2)
   - [macOS (M-chip or Intel)](#macos-m-chip-or-intel)
3. [Understanding the System Architecture](#understanding-the-system-architecture)
4. [First-Time Setup](#first-time-setup)
5. [Verifying Installation](#verifying-installation)
6. [Troubleshooting](#troubleshooting)

---

## ✅ Prerequisites

### Required for All Platforms:
- **Git** (for cloning repository)
- **Docker** (for running containers)
- **Docker Compose** (for orchestrating services)

### Optional (for GPU users):
- **NVIDIA GPU** + **NVIDIA Container Toolkit** (for GPU acceleration)

---

## 🖥️ Platform-Specific Setup

### Ubuntu 22.04 (Recommended)

**This is the primary development platform used by the team leader (Suhang).**

#### 1. Install Docker

```bash
# Update package index
sudo apt-get update

# Install Docker
sudo apt-get install -y docker.io docker-compose

# Add your user to docker group (avoid sudo)
sudo usermod -aG docker $USER

# Log out and log back in, then verify
docker --version
docker-compose --version
```

#### 2. Install NVIDIA Container Toolkit (If you have NVIDIA GPU)

```bash
# Setup repository
distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -
curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | sudo tee /etc/apt/sources.list.d/nvidia-docker.list

# Install nvidia-docker2
sudo apt-get update
sudo apt-get install -y nvidia-docker2

# Restart Docker
sudo systemctl restart docker

# Verify GPU access
docker run --rm --gpus all nvidia/cuda:12.0.1-base-ubuntu20.04 nvidia-smi
```

#### 3. Clone Repository

```bash
cd ~
git clone https://github.com/your-username/robocup_ur5e.git
cd robocup_ur5e
```

#### 4. Configure Environment

The system is pre-configured for Ubuntu 22.04. Check `.env` file:

```bash
cat .env
```

Should contain:
```
ROS_MASTER_URI=http://192.168.56.101:11311
ROS_IP=192.168.56.1
NVIDIA_VISIBLE_DEVICES=all
NVIDIA_DRIVER_CAPABILITIES=compute,utility
```

**✅ Ready to go! Skip to [First-Time Setup](#first-time-setup)**

---

### Windows WSL2

**For Windows users, we recommend WSL2 with Ubuntu 22.04.**

#### 1. Install WSL2 with Ubuntu 22.04

```powershell
# In PowerShell (Administrator)
wsl --install -d Ubuntu-22.04

# After installation, launch Ubuntu
wsl
```

#### 2. Install Docker Desktop for Windows

- Download Docker Desktop: https://www.docker.com/products/docker-desktop/
- Enable **WSL2 backend** in Docker Desktop settings
- Enable **WSL Integration** for your Ubuntu-22.04 distro

#### 3. Verify Docker in WSL2

```bash
# In WSL2 Ubuntu terminal
docker --version
docker-compose --version

# Test Docker
docker run hello-world
```

#### 4. Install NVIDIA Container Toolkit (If you have NVIDIA GPU)

**IMPORTANT**: NVIDIA GPU support in WSL2 requires:
- Windows 11 or Windows 10 (21H2+)
- NVIDIA Driver 470.76+ (Windows host)
- WSL2 kernel 5.10.43.3+

```bash
# In WSL2 Ubuntu terminal
distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -
curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | sudo tee /etc/apt/sources.list.d/nvidia-docker.list

sudo apt-get update
sudo apt-get install -y nvidia-docker2

# Restart Docker (via Docker Desktop)

# Verify GPU access
docker run --rm --gpus all nvidia/cuda:12.0.1-base-ubuntu20.04 nvidia-smi
```

#### 5. Clone Repository in WSL2

```bash
cd ~
git clone https://github.com/your-username/robocup_ur5e.git
cd robocup_ur5e
```

#### 6. Configure Network for WSL2

**IMPORTANT**: WSL2 uses a virtual network. Update `.env` file:

```bash
# Find your WSL2 IP
ip addr show eth0 | grep inet

# Edit .env
vim .env

# Update ROS_IP to your WSL2 IP (e.g., 172.x.x.x)
ROS_IP=<your_wsl2_ip>
```

**✅ Ready! Continue to [First-Time Setup](#first-time-setup)**

---

### macOS (M-chip or Intel)

**macOS users will run on CPU only (no GPU acceleration).**

#### 1. Install Docker Desktop for Mac

- Download Docker Desktop: https://www.docker.com/products/docker-desktop/
- Install and start Docker Desktop
- Allocate resources in settings:
  - CPUs: At least 4 cores
  - Memory: At least 8 GB
  - Disk: At least 60 GB

#### 2. Verify Docker

```bash
docker --version
docker-compose --version
docker run hello-world
```

#### 3. Clone Repository

```bash
cd ~
git clone https://github.com/your-username/robocup_ur5e.git
cd robocup_ur5e
```

#### 4. Configure for CPU-Only Execution

The perception nodes automatically detect the device and fall back to CPU if CUDA is unavailable.

**Edit `.env` file** to disable GPU settings:

```bash
vim .env

# Comment out or remove NVIDIA settings:
# NVIDIA_VISIBLE_DEVICES=all
# NVIDIA_DRIVER_CAPABILITIES=compute,utility
```

**Edit `docker-compose.yml`** to remove GPU reservations:

```bash
vim docker-compose.yml

# Comment out the 'deploy' sections for perception_yolo and perception_grasp:
# deploy:
#   resources:
#     reservations:
#       devices:
#         - driver: nvidia
#           count: 1
#           capabilities: [gpu]
```

**✅ Ready! Continue to [First-Time Setup](#first-time-setup)**

---

## 🏗️ Understanding the System Architecture

### The UR5e VirtualBox VM

The team uses a **VirtualBox VM** that contains:
- **Ubuntu 20.04** guest OS
- **ROS Noetic**
- **Gazebo simulation** with UR5e robot model (`arm_gazebo`)
- **ROS Master** (runs `roscore`)

**Relationship:**
```
┌───────────────────────────────────────────────────────────┐
│  Your Machine (Ubuntu/WSL2/Mac)                           │
│                                                            │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐   │
│  │  robocup_    │  │ perception_  │  │ perception_  │   │
│  │    brain     │  │    yolo      │  │    grasp     │   │
│  │  (Docker)    │  │  (Docker)    │  │  (Docker)    │   │
│  └──────┬───────┘  └──────┬───────┘  └──────┬───────┘   │
│         │                  │                  │            │
│         └──────────────────┼──────────────────┘            │
│                            │                               │
│                   Network (Host Mode)                      │
│                            │                               │
└────────────────────────────┼───────────────────────────────┘
                             │
                   ┌─────────▼─────────┐
                   │  VirtualBox VM    │
                   │ (192.168.56.101)  │
                   │                   │
                   │  ┌─────────────┐  │
                   │  │  roscore    │  │
                   │  │  (ROS Master)│  │
                   │  └─────────────┘  │
                   │                   │
                   │  ┌─────────────┐  │
                   │  │  Gazebo     │  │
                   │  │  (UR5e Sim) │  │
                   │  └─────────────┘  │
                   └───────────────────┘
```

**Why this architecture?**
- **Leader (Suhang)** has the VirtualBox VM with Gazebo
- **Team members** connect to the VM's ROS Master via network
- **Docker containers** on your machine communicate with VM over network
- This allows **distributed development** - everyone works on their own machine

### 使用本地 arm_gazebo Docker（同机 Gazebo + RoboCup）

若你在本机用 **arm_gazebo** 的 Docker 跑 Gazebo 和 roscore，而不是连 VirtualBox VM，请按以下方式配置：

1. **启动顺序**（必须先生效环境 Docker，再启动 RoboCup）：
   ```bash
   # 终端 1：先启动 arm_gazebo 环境（含 roscore / Gazebo）
   cd arm_gazebo/docker
   sudo ./run.bash
   # 等待 Gazebo 和 roscore 完全起来后再进行下一步

   # 终端 2：再启动 RoboCup 总 Docker 及子容器
   cd robocup_ur5e
   ./scripts/start.sh
   # 选择 1 启动所有服务
   ```

2. **`.env` 配置**：  
   确保 RoboCup 指向本机 ROS Master（与 arm_gazebo 同机）：
   ```bash
   ROS_MASTER_URI=http://127.0.0.1:11311
   ROS_IP=127.0.0.1
   ```
   项目根目录下的 `.env` 已默认包含上述“本地 arm_gazebo”配置；若曾改为 VM 的 IP，请改回上述两行。

3. **为何能连上**：  
   arm_gazebo 与 robocup_ur5e 的 compose 均使用 `network_mode: "host"`，roscore 在宿主机 `localhost:11311`，故 RoboCup 容器通过 `127.0.0.1:11311` 即可连接。

---

## 🎯 First-Time Setup

### Step 1: Clone Repository (if not done)

```bash
git clone https://github.com/your-username/robocup_ur5e.git
cd robocup_ur5e
```

### Step 2: Review Environment Configuration

```bash
cat .env
```

**Key variables:**
- `ROS_MASTER_URI`: Points to the VirtualBox VM (192.168.56.101:11311)
- `ROS_IP`: Your machine's IP (auto-detected or set manually)

**For most team members**, you'll connect to Suhang's VM. Make sure:
1. VirtualBox VM is running (Suhang will provide access)
2. You can ping the VM: `ping 192.168.56.101`
3. ROS Master is running in VM: `nc -zv 192.168.56.101 11311`

### Step 3: Build Docker Images

**This will take 30-60 minutes on first build (downloads models and dependencies).**

```bash
# Option 1: Build all images
./rebuild_all.sh

# Option 2: Build manually
docker-compose build --no-cache

# Option 3: Pull pre-built images (if leader pushed to Docker Hub)
docker-compose pull
```

### Step 4: Start System

```bash
./start.sh
```

This interactive script will:
- ✓ Verify all images are built
- ✓ Check network configuration
- ✓ Start all containers
- ✓ Display system status

### Step 5: Verify System is Running

```bash
# Check container status
docker-compose ps

# Should show:
# robocup_brain      Up
# perception_yolo    Up
# perception_grasp   Up

# View logs
docker-compose logs -f
```

---

## 🔍 Verifying Installation

### Test 1: Check Docker Containers

```bash
./status.sh
```

Expected output:
```
✅ robocup_ur5e/brain              4.7GB
✅ robocup_ur5e/perception_yolo   18.2GB
✅ robocup_ur5e/perception_grasp   17GB

✅ robocup_brain      Up
✅ perception_yolo    Up
✅ perception_grasp   Up
```

### Test 2: Check ROS Connectivity

```bash
# Enter brain container
docker-compose exec brain bash

# Inside container
source /workspace/devel/setup.bash
rostopic list

# Should show topics like:
# /perception/detected_objects
# /perception/grasp_candidates
# /move_group/...
```

### Test 3: Verify GPU Access (NVIDIA users only)

```bash
# Check YOLO container GPU
docker-compose exec perception_yolo nvidia-smi

# Check Grasp container GPU
docker-compose exec perception_grasp nvidia-smi
```

### Test 4: Check CPU Fallback (Mac users)

```bash
# View YOLO logs - should show "Using device: cpu"
docker-compose logs perception_yolo | grep -i device

# View Grasp logs - should show "Using device: cpu"
docker-compose logs perception_grasp | grep -i device
```

---

## 🐛 Troubleshooting

### Problem: Cannot connect to Docker daemon

**Solution (Ubuntu/WSL2):**
```bash
# Start Docker service
sudo systemctl start docker

# OR restart Docker Desktop (WSL2)
```

**Solution (Mac):**
- Start Docker Desktop application

---

### Problem: "Cannot connect to ROS Master"

**Causes:**
1. VirtualBox VM is not running
2. VM IP address is incorrect
3. `roscore` is not running in VM
4. Firewall blocking port 11311

**Solution:**
```bash
# Test VM connectivity
ping 192.168.56.101

# Test ROS Master port
nc -zv 192.168.56.101 11311

# If fails, check .env configuration
cat .env | grep ROS_MASTER_URI

# Update if needed
echo "ROS_MASTER_URI=http://192.168.56.101:11311" > .env
```

---

### Problem: "NVIDIA GPU not detected" (NVIDIA users)

**Solution:**
```bash
# Verify NVIDIA Docker runtime
docker run --rm --gpus all nvidia/cuda:12.0.1-base-ubuntu20.04 nvidia-smi

# If fails, reinstall nvidia-docker2
sudo apt-get install -y nvidia-docker2
sudo systemctl restart docker
```

---

### Problem: "Out of disk space"

Docker images are large (~40GB total). **Solution:**

```bash
# Check disk usage
df -h

# Clean unused Docker images
docker system prune -a

# Remove old images
docker images | grep "<none>" | awk '{print $3}' | xargs docker rmi
```

---

### Problem: Container fails to build (dependency errors)

**Solution:**
```bash
# Clean Docker cache and rebuild
docker-compose down
docker system prune -a
docker-compose build --no-cache
```

See `BUILD_FIX_FINAL.md` for specific dependency fixes.

---

### Problem: 机械臂乱动导致 Gazebo 中物品飞出

**现象**：运行 arm_gazebo 时机械臂乱甩，环境中物品被撞飞。

**已做优化**（arm_gazebo 配置）：
- 物理：`real_time_update_rate` 从 1000 降至 500，增加 `max_contacts`
- 控制器：`goal_time` 从 0.6s 增至 1.0s，关节约束放宽

**进一步排查**：
```bash
# 1. 确认是否有节点在向机械臂发命令
rostopic echo /pos_joint_traj_controller/command

# 2. 单独测试：只启动 arm_gazebo，不启动 RoboCup
# 若单独 arm_gazebo 时稳定，问题可能在 RoboCup 侧

# 3. 若仍不稳定，可进一步降低 physics 的 real_time_update_rate（如 250）
# 编辑 arm_gazebo/worlds/arm_empty.world
```

---

### Problem: Permission denied (Ubuntu/WSL2)

**Solution:**
```bash
# Add user to docker group
sudo usermod -aG docker $USER

# Log out and log back in
exit
# (Re-login)

# Verify
docker ps
```

---

## 🔧 Advanced Configuration

### For Team Members WITHOUT Access to VirtualBox VM

If you want to run the system **without** the VirtualBox VM (local testing only):

1. **Install ROS Noetic** locally or in a container
2. **Start local ROS Master**:
   ```bash
   roscore
   ```
3. **Update `.env`** to point to localhost:
   ```bash
   ROS_MASTER_URI=http://localhost:11311
   ROS_IP=localhost
   ```
4. **Start Docker containers**:
   ```bash
   docker-compose up -d
   ```

**Note**: You won't have the UR5e Gazebo simulation, but you can still test perception and FSM logic.

---

### For Mac Users (M-chip)

**PyTorch will automatically use MPS (Metal Performance Shaders) if available:**

The perception nodes check device availability in this order:
1. CUDA (NVIDIA GPU)
2. MPS (Apple Silicon)
3. CPU (fallback)

Logs will show:
```
[YOLO] Using device: mps
```

**Performance**: MPS is slower than CUDA but faster than CPU.

---

## 📊 Expected Build Times

| Platform | GPU | Brain | YOLO | Grasp | Total |
|----------|-----|-------|------|-------|-------|
| Ubuntu 22.04 + NVIDIA RTX 5070 Ti | ✅ | 5 min | 15 min | 20 min | ~40 min |
| Ubuntu 22.04 (CPU only) | ❌ | 5 min | 10 min | 15 min | ~30 min |
| WSL2 + NVIDIA GPU | ✅ | 5 min | 15 min | 20 min | ~40 min |
| WSL2 (CPU only) | ❌ | 5 min | 10 min | 15 min | ~30 min |
| Mac M-chip (MPS) | 🟡 | 5 min | 12 min | 18 min | ~35 min |
| Mac Intel (CPU) | ❌ | 5 min | 10 min | 15 min | ~30 min |

---

## 🎯 Next Steps

After successful setup:

1. **Read Team Development Guide**: [`TEAM_README.md`](TEAM_README.md)
   - Find your responsibilities
   - Locate your file to edit
   - Review your TODO list

2. **Start Developing**:
   ```bash
   # Edit code on your machine
   vim src/your_package/nodes/your_node.py
   
   # Restart container to apply changes
   docker-compose restart your_service
   
   # View logs
   docker-compose logs -f your_service
   ```

3. **Test Your Code**:
   ```bash
   # Enter container
   docker-compose exec your_service bash
   
   # Run your node
   source /workspace/devel/setup.bash
   rosrun your_package your_node.py
   ```

4. **Commit and Push**:
   ```bash
   git add src/your_package/
   git commit -m "feat(package): implement feature"
   git push origin main
   ```

---

## 🆘 Getting Help

### Documentation
- **Setup Issues**: This file (SETUP_GUIDE.md)
- **Development Tasks**: TEAM_README.md
- **Architecture Questions**: ARCHITECTURE_SUMMARY.md
- **Command Reference**: QUICKSTART.md

### Contact
- **Technical Lead**: Suhang Xia - suhang@robocup.org
- **GitHub Issues**: Open an issue for bugs or questions

---

## ✅ Setup Checklist

Before starting development, ensure:

- [ ] Docker and Docker Compose installed
- [ ] Repository cloned
- [ ] All 3 Docker images built successfully
- [ ] Containers start without errors (`docker-compose ps` shows "Up")
- [ ] Can view container logs (`docker-compose logs -f`)
- [ ] Read TEAM_README.md and know your tasks
- [ ] Found your file and located TODO markers

**Once all checked, you're ready to develop!** 🚀

---

**Need help? Ask in the team chat or open a GitHub issue!**

*Last Updated: January 26, 2026*
