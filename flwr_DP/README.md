# Federated Learning for Diffusion Policy

Distributed training system using Flower framework. Enables collaborative training across multiple clients while preserving data privacy.

## Overview

- **Flower Integration**: Industry-standard federated learning
- **Distributed Training**: Multiple clients train on local data partitions
- **FedAvg Aggregation**: Combines client model updates
- **Privacy Preservation**: Data stays local to each client
- **Scalable**: Support for multiple clients and rounds
- **Monitoring**: Comprehensive logging and metrics

## System Architecture

```
flwr_DP/
├── model.py                         # Federated-compatible neural network architecture
├── train.py                         # Federated training script with diffusion sampling
├── dataset.py                       # Federated data loading and preprocessing
├── flwr_client.py                   # Flower federated learning client implementation
├── flwr_server.py                   # Flower federated learning server with custom strategy
├── run_simulation.py                # Federated learning simulation orchestrator
├── run_clients.py                   # Multi-client execution script
├── inference.py                     # Federated model inference
└── checkpoints/                     # Federated trained model files
    ├── client_0_round_X.pth         # Client-specific checkpoints
    ├── client_1_round_X.pth         # Client-specific checkpoints
    └── ...                          # Additional federated checkpoints
```

## Features

### Client-Server Architecture

**Client (flwr_client.py)**: Local training on data partitions, parameter sharing, MPS/CUDA support
**Server (flwr_server.py)**: FedAvg aggregation, metrics collection, round coordination

### Data Partitioning

Episodes automatically split among clients with non-overlapping distribution:
```python
# Example: 3 clients, 300 episodes
# Client 0: Episodes 0-99
# Client 1: Episodes 100-199  
# Client 2: Episodes 200-299
```

## Usage

### Federated Learning Simulation

**Quick Start**:
```bash
cd ~/ros2_ws/flwr_DP

# Activate virtual environment
source ~/.venvs/diffusion_policy/bin/activate

# Run federated learning simulation
python run_simulation.py
```

**Custom Configuration**:
```bash
# Edit run_simulation.py to configure:
NUM_CLIENTS_TO_SIMULATE = 5      # Number of federated clients
NUM_SIMULATION_ROUNDS = 100      # Number of federated rounds
```

### Manual Client-Server Setup

**Start Server**:
```bash
cd ~/ros2_ws/flwr_DP
source ~/.venvs/diffusion_policy/bin/activate

# Start Flower server
python -c "
import flwr as fl
from flwr_server import app as server_app
fl.server.run_server(server_app=server_app)
"
```

**Start Clients** (in separate terminals):
```bash
cd ~/ros2_ws/flwr_DP
source ~/.venvs/diffusion_policy/bin/activate

# Start client 0
python -c "
import flwr as fl
from flwr_client import app as client_app
fl.client.run_client(server_address='localhost:8080', client_app=client_app)
"
```

### Multi-Client Script

```bash
# Run multiple clients automatically
python run_clients.py --num_clients 3 --server_address localhost:8080
```

## Configuration

### Federated Learning Parameters

**Client Configuration**:
```python
# In flwr_client.py
NUM_TOTAL_CLIENTS = 3           # Total number of clients
LOCAL_EPOCHS = 1                # Local training epochs per round
BATCH_SIZE = 16                 # Client batch size
LEARNING_RATE = 1e-4            # Client learning rate
```

**Server Configuration**:
```python
# In flwr_server.py
NUM_ROUNDS = 500                # Total federated rounds
MIN_FIT_CLIENTS = 3             # Minimum clients for training
MIN_EVALUATE_CLIENTS = 3        # Minimum clients for evaluation
FRACTION_FIT = 1.0              # Fraction of clients to use for training
```

**Simulation Configuration**:
```python
# In run_simulation.py
NUM_CLIENTS_TO_SIMULATE = 3     # Simulated clients
NUM_SIMULATION_ROUNDS = 500     # Simulation rounds
```

### Hardware Support

**Apple Silicon (MPS)**:
- Automatic MPS detection and usage
- Enhanced memory management for Apple Silicon
- Periodic cache cleaning to prevent memory issues

**NVIDIA GPU (CUDA)**:
- Automatic CUDA detection and usage
- GPU memory optimization
- Multi-GPU support (if available)

**CPU Fallback**:
- Automatic fallback to CPU if no GPU available
- Optimized CPU training parameters

## Training Process

1. **Server Init**: Start with initial model parameters
2. **Client Selection**: Select clients for current round
3. **Model Distribution**: Send global model to clients
4. **Local Training**: Clients train on local data
5. **Model Upload**: Clients send updates to server
6. **Aggregation**: Server aggregates using FedAvg
7. **Repeat**: Continue for specified rounds

**Privacy**: Raw data never leaves clients, only model parameters shared
**Efficiency**: Compressed parameter sharing, minimal network overhead

## Monitoring and Evaluation

### Metrics Collection

**Client Metrics**:
- Local training loss per round
- Local evaluation accuracy
- Number of training samples
- Training time per round

**Server Metrics**:
- Aggregated training loss across clients
- Global model evaluation metrics
- Round completion times
- Client participation rates

### Logging

**Comprehensive Logging**:
```bash
# Example log output
2024-01-15 10:30:15 - INFO - Round 1 - Aggregating fit results from 3 clients
2024-01-15 10:30:15 - INFO - Client 0 - Training loss: 0.0234, Num Examples: 2400
2024-01-15 10:30:15 - INFO - Client 1 - Training loss: 0.0198, Num Examples: 2400
2024-01-15 10:30:15 - INFO - Client 2 - Training loss: 0.0256, Num Examples: 2400
2024-01-15 10:30:15 - INFO - Round 1 - Successfully aggregated model weights
```

## Troubleshooting

### Common Issues

**Memory Issues on Apple Silicon**:
```bash
# Reduce batch size in flwr_client.py
BATCH_SIZE = 8  # Instead of 16

# Enable more frequent cache cleaning
clean_mps_cache(force_gc=True)
```

**Client Connection Issues**:
```bash
# Check server address and port
python -c "import socket; print(socket.gethostbyname('localhost'))"

# Verify server is running
netstat -an | grep 8080
```

**Data Loading Issues**:
```bash
# Verify data directory exists
ls -la ../mycobot_episodes_degrees/

# Check episode count
python -c "
from dataset import RobotEpisodeDataset
dataset = RobotEpisodeDataset('../mycobot_episodes_degrees/')
print(f'Total episodes: {len(dataset)}')
"
```

### Performance Optimization

**Training Speed**:
- Adjust `LOCAL_EPOCHS` based on data size
- Optimize `BATCH_SIZE` for available memory
- Use appropriate `NUM_WORKERS` for data loading

**Memory Usage**:
- Enable periodic cache cleaning
- Reduce model size if necessary
- Monitor memory usage during training

## Comparison with Centralized Training

| Aspect | Centralized (DP/) | Federated (flwr_DP/) |
|--------|------------------|---------------------|
| **Data Location** | Single machine | Distributed across clients |
| **Privacy** | All data visible | Data remains local |
| **Scalability** | Limited by single machine | Scales with number of clients |
| **Communication** | None required | Model parameters only |
| **Training Speed** | Faster per epoch | Slower due to communication |
| **Robustness** | Single point of failure | Distributed resilience |

## Related Documentation

- **[Centralized Training](../DP/README.md)** - Traditional centralized training approach
- **[Main Project README](../README.md)** - Overall project documentation
- **[Data Collection Guide](../DATA_COLLECTION.md)** - Training data collection

## Dependencies

**Additional Federated Learning Dependencies**:
```bash
pip install flwr[simulation]>=1.0.0
pip install ray>=2.0.0  # For simulation backend
```

**Core Dependencies** (same as centralized training):
- PyTorch >= 1.12.0
- torchvision >= 0.13.0
- Pillow >= 8.0.0
- OpenCV >= 4.5.0
- NumPy >= 1.21.0
- tqdm >= 4.62.0

## Back to Main Documentation

← [Main Project README](../README.md) 