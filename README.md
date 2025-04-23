# dia_ros
## ROS1 Wrapper for [nari-labs/Dia](https://github.com/nari-labs/dia)
`dia_ros` is a ROS1 interface for [Dia](https://github.com/nari-labs/dia), a large-scale open-weights text-to-dialogue model. This wrapper allows you to receive text via a ROS topic and generate corresponding speech audio using Dia, which is then played back through the `sound_play` package.
---

## ✅ Prerequisites
### 1. Python Environment (Python ≥ 3.10) We recommend using **Conda** or **Docker** to create an isolated environment compatible with Dia. Example using Conda:
```conda create -n dia_env python=3.10 conda activate dia_env ``` Follow setup instructions from the original Dia repository: 👉 https://github.com/nari-labs/dia

### 2. ROS1 Dependencies in Conda Environment To allow the Conda Python interpreter to communicate with ROS1:
```pip install rospkg ```
---



## 🎧 Install sound_play Install `sound_play`, which is used to play generated `.wav` files.
- For **Melodic**:
```sudo apt install ros-melodic-sound-play ```
- For **Noetic**:
```sudo apt install ros-noetic-sound-play ```
---


## 🚀 Installation
### 1. Clone this repository
```cd ~/catkin_ws/src git clone https://github.com/MinSungjae/dia_ros ```

### 2. Make the main script executable
```cd dia_ros/scripts chmod +x dia_ros.py ``` 

### 3. ⚠️ Edit the Shebang Open `dia_ros.py` and modify the first line to match your Conda Python interpreter:
```python #!/home/your_username/miniconda3/envs/dia_env/bin/python ``` 

### 4. Build your workspace
```cd ~/catkin_ws catkin_make # or catkin build source devel/setup.bash ```
---



## 🧪 How to Run 

### 1. Start Dia ROS node
```rosrun dia_ros dia_ros.py ``` 

### 2. Start the sound_play node (must run in parallel)
```rosrun sound_play soundplay_node.py ```
---



## 📡 Publishing Input Text To send text to be synthesized and played:
```rostopic pub /input_text std_msgs/String "data: '[S1] Hello, this is a ROS-Dia integration demo.'" ```
--- 



## 📁 Output
- Generated audio is saved as:
``` /tmp/dia_output.wav ```
- Audio is automatically played via `sound_play`. ---



## 💬 Notes
- Make sure your Conda environment has all required dependencies for Dia, including:
    - `torch`, `soundfile`, and other Dia dependencies
- You can extend `dia_ros.py` to:
    - Handle audio queueing
    - Monitor playback status
    - Provide feedback via topics or services --- 