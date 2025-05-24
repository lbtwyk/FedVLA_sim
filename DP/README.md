# Diffusion Policy Training System

A PyTorch-based training system for visuomotor diffusion policies. This system trains neural networks to predict robot actions from camera images using diffusion models for cube stacking tasks.

## Overview

The diffusion policy training system implements:

- **ResNet34 Vision Backbone**: Pre-trained feature extraction from 424x240 camera images
- **Diffusion Model Architecture**: Generative model for robot control
- **Multi-Layer Perceptron**: Action prediction with 6 joint angles + 1 gripper value
- **Training Pipeline**: Automated training with checkpointing

## System Architecture

```
DP/
├── model.py                         # Neural network architecture
├── train.py                         # Training script and diffusion sampling
├── dataset.py                       # Data loading and preprocessing
├── inference.py                     # Standalone inference script
├── inference_gazebo.py              # Gazebo integration inference
├── inference_modified.py            # Modified inference variants
├── inference_realtime.py            # Real-time inference
├── unet_model.py                    # Alternative UNet architecture
├── requirements.txt                 # Python dependencies
├── run.sh                          # Training execution script
├── resume_training.sh              # Resume training from checkpoint
└── checkpoints/                    # Trained model checkpoints
    ├── model_best-2.pth            # Best performing model
    ├── model_best-3.pth            # Alternative checkpoint
    └── ...                         # Additional checkpoints
```

## Model Architecture

### DiffusionPolicyModel (model.py)

**Vision Backbone**: ResNet34
- Input: 424x240 RGB images
- Output: 512-dimensional feature vectors
- Pre-trained on ImageNet
- Frozen weights for stable training

**State Prediction Network**:
- Input: Image features + timestep embedding
- Hidden layers: 4-layer MLP with 256 hidden units
- Output: 7-dimensional action (6 joints + 1 gripper)
- Activation: ReLU with dropout

**Diffusion Process**:
- Timesteps: 1000 (configurable)
- Beta schedule: Linear from 1e-4 to 0.02
- Sampling: DDPM with noise prediction

### Key Features

- **State Dimension**: 7 (6 joint angles + 1 gripper value)
- **Image Resolution**: 424x240 pixels (optimized for inference speed)
- **Training Data**: Degrees format for improved model sensitivity
- **Inference Rate**: 10Hz for real-time robot control

## Training

### Data Requirements

**Input Data Structure**:
```
~/mycobot_episodes_degrees/
├── episode_YYYYMMDD_HHMMSS_mmm/
│   ├── states.json              # Joint states in degrees
│   └── frame_dir/               # Camera images
│       ├── image_00000.png
│       ├── image_00001.png
│       └── ...
```

**states.json Format**:
```json
[
  {
    "angles": [10.5, -15.2, 30.8, -45.1, 60.3, -75.6],
    "gripper_value": [50],
    "image": "frame_dir/image_00000.png"
  }
]
```

### Training Configuration

**Default Parameters**:
```python
# Model Architecture
state_dim = 7                    # 6 joints + 1 gripper
image_feature_dim = 512          # ResNet34 output
hidden_dim = 256                 # MLP hidden size
num_mlp_layers = 4               # MLP depth

# Training
batch_size = 32                  # Batch size
learning_rate = 1e-4             # Adam learning rate
num_epochs = 1000                # Training epochs
weight_decay = 1e-5              # L2 regularization

# Diffusion
diffusion_timesteps = 1000       # Total timesteps
beta_start = 1e-4                # Beta schedule start
beta_end = 0.02                  # Beta schedule end
```

### Training Execution

**Basic Training**:
```bash
cd ~/ros2_ws/DP

# Activate virtual environment
source ~/.venvs/diffusion_policy/bin/activate

# Run training with default parameters
./run.sh

# Or run manually
python train.py \
  --data_dir ~/mycobot_episodes_degrees \
  --output_dir ./checkpoints
```

**Resume Training**:
```bash
# Resume from checkpoint
./resume_training.sh
```

### Training Parameters

**Data Configuration**:
- `--data_dir`: Path to episode data directory
- `--image_size`: Input image size (default: 224)
- `--frame_skip`: Skip frames for temporal diversity (default: 2)

**Model Configuration**:
- `--state_dim`: Robot state dimension (default: 7)
- `--hidden_dim`: MLP hidden dimension (default: 256)
- `--num_mlp_layers`: Number of MLP layers (default: 4)

**Training Configuration**:
- `--batch_size`: Training batch size (default: 32)
- `--learning_rate`: Adam learning rate (default: 1e-4)
- `--num_epochs`: Total training epochs (default: 1000)
- `--save_interval`: Checkpoint save interval (default: 100)

**Diffusion Configuration**:
- `--diffusion_timesteps`: Total diffusion timesteps (default: 1000)
- `--beta_start`: Beta schedule start value (default: 1e-4)
- `--beta_end`: Beta schedule end value (default: 0.02)

## Inference

### Standalone Inference

```python
import torch
from model import DiffusionPolicyModel
from train import p_sample_loop, linear_beta_schedule

# Load model
checkpoint = torch.load('checkpoints/model_best-2.pth')
model = DiffusionPolicyModel(
    state_dim=7,
    hidden_dim=256,
    num_layers=4,
    image_feature_dim=512
)
model.load_state_dict(checkpoint['model_state_dict'])
model.eval()

# Run inference
with torch.no_grad():
    predicted_action = p_sample_loop(
        model,
        shape=(1, 7),
        timesteps=1000,
        image_input=image_tensor
    )
```

### ROS 2 Integration

The trained models integrate with the ROS 2 inference system:

```bash
# Run inference in simulation
source ~/.venvs/diffusion_policy/bin/activate
ros2 launch diffusion_policy_inference simulation_inference.launch.py \
  checkpoint_path:=~/ros2_ws/DP/checkpoints/model_best-2.pth \
  model_dir:=~/ros2_ws/DP
```

## Model Checkpoints

### Available Models

**model_best-2.pth**:
- Training episodes: ~200 episodes
- Data format: Degrees
- Performance: Stable cube stacking
- Recommended for inference

**model_best-3.pth**:
- Training episodes: ~300 episodes
- Data format: Degrees
- Performance: Enhanced precision
- Alternative checkpoint

### Checkpoint Contents

```python
checkpoint = {
    'epoch': 1000,
    'model_state_dict': model.state_dict(),
    'optimizer_state_dict': optimizer.state_dict(),
    'train_loss': 0.0123,
    'args': {
        'state_dim': 7,
        'hidden_dim': 256,
        'num_mlp_layers': 4,
        'image_feature_dim': 512,
        # ... other training arguments
    }
}
```

## Data Preprocessing

### Image Processing

```python
transform = transforms.Compose([
    transforms.Resize((224, 224)),           # Resize to model input
    transforms.ToTensor(),                   # Convert to tensor
    transforms.Normalize(                    # ImageNet normalization
        mean=[0.485, 0.456, 0.406],
        std=[0.229, 0.224, 0.225]
    )
])
```

### State Processing

```python
# Convert radians to degrees (training data)
angles_degrees = [math.degrees(angle) for angle in angles_radians]

# Normalize gripper values
gripper_normalized = (gripper_raw - gripper_min) / (gripper_max - gripper_min) * 100
```

## Performance Optimization

### Training Optimization

- **Mixed Precision**: Use automatic mixed precision for faster training
- **Data Loading**: Multi-worker data loading for I/O efficiency
- **Batch Size**: Adjust based on GPU memory (32 recommended for RTX 3070)
- **Learning Rate**: Use learning rate scheduling for better convergence

### Inference Optimization

- **Model Quantization**: Reduce model size for deployment
- **Batch Inference**: Process multiple timesteps together
- **GPU Memory**: Optimize memory usage for real-time inference

## Troubleshooting

### Training Issues

**CUDA Out of Memory**:
```bash
# Reduce batch size
python train.py --batch_size 16

# Use gradient accumulation
python train.py --gradient_accumulation_steps 2
```

**Slow Training**:
```bash
# Increase number of workers
python train.py --num_workers 8

# Use mixed precision
python train.py --use_amp
```

### Model Loading Issues

**Checkpoint Compatibility**:
```python
# Check checkpoint contents
checkpoint = torch.load('model.pth', map_location='cpu')
print(checkpoint.keys())
print(checkpoint['args'])
```

**Version Compatibility**:
```bash
# Check PyTorch version
python -c "import torch; print(torch.__version__)"

# Ensure CUDA compatibility
python -c "import torch; print(torch.cuda.is_available())"
```

## Dependencies

Install required packages:

```bash
pip install -r requirements.txt
```

**Core Dependencies**:
- PyTorch >= 1.12.0
- torchvision >= 0.13.0
- Pillow >= 8.0.0
- OpenCV >= 4.5.0
- NumPy >= 1.21.0
- tqdm >= 4.62.0

## Related Documentation

- **[Diffusion Policy Inference](../src/diffusion_policy_inference/README.md)** - ROS 2 inference integration
- **[Data Collection Guide](../DATA_COLLECTION.md)** - Training data collection
- **[Main Project README](../README.md)** - Overall project documentation

## Back to Main Documentation

← [Main Project README](../README.md)
