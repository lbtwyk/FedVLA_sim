# FedVLA/DP/model.py

import torch
import torch.nn as nn
import torch.nn.functional as F
import torchvision.models as models
import math
import logging
from typing import Optional, Tuple

# Configure logging if run as main, otherwise assume it's configured elsewhere
if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

class SinusoidalPosEmb(nn.Module):
    """
    Generates sinusoidal positional embeddings for the diffusion timestep.
    Taken from: https://github.com/lucidrains/denoising-diffusion-pytorch/
    """
    def __init__(self, dim: int):
        """
        Initializes the sinusoidal positional embedding module.

        Args:
            dim (int): The dimension of the embeddings to generate.
        """
        super().__init__()
        self.dim = dim

    def forward(self, time: torch.Tensor) -> torch.Tensor:
        """
        Generates the embeddings.

        Args:
            time (torch.Tensor): A tensor of timesteps, shape (batch_size,).

        Returns:
            torch.Tensor: The generated embeddings, shape (batch_size, dim).
        """
        device = time.device
        half_dim = self.dim // 2
        # Handle potential division by zero if dim=0 or 1, although unlikely for embeddings
        if half_dim <= 1:
             denominator = 1.0 # Avoid log(10000) / 0
        else:
             denominator = half_dim - 1
        # Prevent potential overflow with large denominators
        if denominator == 0:
            embeddings = 0.0 # Or handle appropriately
        else:
            embeddings = math.log(10000) / denominator
        embeddings = torch.exp(torch.arange(half_dim, device=device) * -embeddings)
        # Ensure time is broadcastable: (batch_size,) -> (batch_size, 1)
        embeddings = time[:, None] * embeddings[None, :]
        embeddings = torch.cat((embeddings.sin(), embeddings.cos()), dim=-1)
        # Handle odd dimensions
        if self.dim % 2 == 1:
            embeddings = F.pad(embeddings, (0, 1)) # Pad the last dimension
        return embeddings

class MLPBlock(nn.Module):
    """A simple MLP block with LayerNorm and GELU activation."""
    def __init__(self, input_dim: int, output_dim: int):
        super().__init__()
        self.layers = nn.Sequential(
            nn.Linear(input_dim, output_dim),
            nn.LayerNorm(output_dim), # LayerNorm before activation
            nn.GELU()
        )

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        return self.layers(x)

class DiffusionPolicyModel(nn.Module):
    """
    A diffusion policy model that predicts noise based on state, timestep,
    and image features extracted via a ResNet34 backbone.
    """
    def __init__(self,
                 state_dim: int,
                 time_emb_dim: int = 64,
                 time_emb_mult: int = 2,  # Multiplier for time embedding MLP hidden dimension
                 hidden_dim: int = 128,   # Changed default to 128 to match your trained model
                 num_layers: int = 4,
                 image_feature_dim: Optional[int] = None, # Made optional, will be derived
                 use_pretrained_resnet: bool = True,
                 freeze_resnet: bool = True
                ):
        """
        Initializes the Diffusion Policy Model with a ResNet34 image backbone.

        Args:
            state_dim (int): The dimensionality of the input state vector.
            time_emb_dim (int, optional): Dimensionality of timestep embedding. Defaults to 64.
            hidden_dim (int, optional): Dimensionality of hidden layers. Defaults to 128.
            num_layers (int, optional): Number of MLP blocks. Defaults to 4.
            image_feature_dim (Optional[int], optional): Expected dimensionality of ResNet features.
                                               If None, it's set to 512 (default for ResNet34).
                                               If provided, it's validated against the ResNet34 output.
            use_pretrained_resnet (bool, optional): Load pretrained weights. Defaults to True.
            freeze_resnet (bool, optional): Freeze ResNet weights. Defaults to True.
        """
        super().__init__()
        self.state_dim = state_dim
        self.time_emb_dim = time_emb_dim
        self.hidden_dim = hidden_dim
        self.backbone_name = "resnet34"  # Fixed to ResNet34

        # --- Image Backbone ---
        # Use ResNet34 as the backbone
        weights_arg = models.ResNet34_Weights.DEFAULT if use_pretrained_resnet else None
        resnet = models.resnet34(weights=weights_arg)
        actual_resnet_output_channels = 512

        self.image_backbone = nn.Sequential(*list(resnet.children())[:-2]) # Remove fc and avgpool
        self.adaptive_pool = nn.AdaptiveAvgPool2d((1, 1))

        # Validate or set image_feature_dim
        if image_feature_dim is None:
            self.image_feature_dim = actual_resnet_output_channels
            logging.info(f"image_feature_dim not provided, using default {self.image_feature_dim} for ResNet34.")
        elif image_feature_dim != actual_resnet_output_channels:
            # If a user explicitly provides a value that doesn't match ResNet34's output,
            # we prioritize the backbone's true output for compatibility.
            logging.warning(f"Provided image_feature_dim ({image_feature_dim}) doesn't match "
                            f"ResNet34 output channels ({actual_resnet_output_channels}). "
                            f"Using {actual_resnet_output_channels} for compatibility.")
            self.image_feature_dim = actual_resnet_output_channels
        else:
            self.image_feature_dim = image_feature_dim # Provided and matches ResNet34
            logging.info(f"Using provided image_feature_dim {self.image_feature_dim} for ResNet34.")

        if freeze_resnet:
            for param in self.image_backbone.parameters():
                param.requires_grad = False
            logging.info(f"ResNet backbone ({self.backbone_name}) weights frozen.")
        else:
             logging.info(f"ResNet backbone ({self.backbone_name}) weights will be fine-tuned.")
        # --- End Image Backbone ---


        # --- Timestep Embedding ---
        self.time_emb_dim = time_emb_dim
        self.time_emb_mult = time_emb_mult
        self.time_mlp = nn.Sequential(
            SinusoidalPosEmb(time_emb_dim),
            nn.Linear(time_emb_dim, time_emb_dim * time_emb_mult),
            nn.GELU(),
            nn.Linear(time_emb_dim * time_emb_mult, time_emb_dim),
        )
        # --- End Timestep Embedding ---


        # --- Main Policy Network ---
        # Input projection layer now takes state + image features
        input_proj_dim = state_dim + self.image_feature_dim
        self.input_projection = MLPBlock(input_proj_dim, hidden_dim)

        # MLP layers conditioned on time
        self.layers = nn.ModuleList()
        for _ in range(num_layers):
            # Each layer takes the hidden state + time embedding
            self.layers.append(MLPBlock(hidden_dim + time_emb_dim, hidden_dim))

        # Final output layer to predict noise
        self.output_projection = nn.Linear(hidden_dim, state_dim)
        # --- End Main Policy Network ---

        logging.info(f"Initialized DiffusionPolicyModel:")
        logging.info(f"  State Dim: {state_dim}")
        logging.info(f"  Image Feature Dim: {self.image_feature_dim}")
        logging.info(f"  Time Emb Dim: {time_emb_dim}")
        logging.info(f"  Time Emb Mult: {time_emb_mult}")
        logging.info(f"  Hidden Dim: {hidden_dim}")
        logging.info(f"  Num Layers: {num_layers}")
        logging.info(f"  Backbone: ResNet34")
        logging.info(f"  Using Pretrained ResNet: {use_pretrained_resnet}")
        logging.info(f"  Freezing ResNet: {freeze_resnet}")

    def load_state_dict_with_conversion(self, state_dict: dict, strict: bool = False):
        """
        Load state dict with automatic conversion from legacy checkpoint format.

        This method handles mismatches in time embedding dimensions by creating
        new time_mlp layers with the correct dimensions and copying weights where possible.
        It also handles different model architectures by mapping keys from the old format
        to the new format.

        Args:
            state_dict (dict): The state dictionary to load
            strict (bool): Whether to strictly enforce that the keys in state_dict match

        Returns:
            dict: The return value from the superclass load_state_dict call
        """
        logging.info(f"Starting state dict conversion with {len(state_dict)} keys")

        # Check if this is a legacy checkpoint with image_encoder and mlp keys
        has_image_encoder = any(k.startswith('image_encoder.') for k in state_dict.keys())
        has_mlp = any(k.startswith('mlp.') for k in state_dict.keys())

        if has_image_encoder or has_mlp:
            logging.info("Detected legacy checkpoint format with image_encoder/mlp keys")
            converted_dict = {}

            # Map image_encoder keys to image_backbone keys
            for k, v in state_dict.items():
                if k.startswith('image_encoder.'):
                    # For ResNet, we need to map the layers correctly
                    new_key = k.replace('image_encoder.', 'image_backbone.')
                    converted_dict[new_key] = v
                    logging.debug(f"Mapped {k} -> {new_key}")
                elif k.startswith('mlp.'):
                    # For MLP layers, we need to map to the new structure
                    # This is a simplistic mapping and may need to be adjusted
                    if k.startswith('mlp.0.'):
                        new_key = k.replace('mlp.0.', 'input_projection.layers.0.')
                        converted_dict[new_key] = v
                    elif k.startswith('mlp.3.') or k.startswith('mlp.6.') or k.startswith('mlp.9.'):
                        layer_idx = (int(k.split('.')[1]) // 3) - 1
                        rest = k.split('.', 2)[2]
                        new_key = f'layers.{layer_idx}.layers.0.{rest}'
                        converted_dict[new_key] = v
                    elif k.startswith('mlp.12.'):
                        new_key = k.replace('mlp.12.', 'output_projection.')
                        converted_dict[new_key] = v
                else:
                    # Keep other keys as is
                    converted_dict[k] = v

            # Use the converted dictionary
            state_dict = converted_dict
            logging.info(f"Converted legacy format to {len(state_dict)} keys")

        # Check if there's a mismatch in time_mlp dimensions
        if 'time_mlp.1.weight' in state_dict:
            checkpoint_time_emb_mult = None

            # Determine the time embedding multiplier from the checkpoint
            if state_dict['time_mlp.1.weight'].shape[0] == self.time_emb_dim * 4:
                checkpoint_time_emb_mult = 4
            elif state_dict['time_mlp.1.weight'].shape[0] == self.time_emb_dim * 2:
                checkpoint_time_emb_mult = 2
            else:
                # Try to infer from dimensions
                checkpoint_time_emb_mult = state_dict['time_mlp.1.weight'].shape[0] // self.time_emb_dim
                if checkpoint_time_emb_mult <= 0:
                    checkpoint_time_emb_mult = 2  # Default to 2 if we can't determine

            logging.info(f"Detected checkpoint time_emb_mult={checkpoint_time_emb_mult}")

            # If we have a mismatch, log it and create new layers
            if not hasattr(self, 'time_emb_mult') or checkpoint_time_emb_mult != getattr(self, 'time_emb_mult', 4):
                logging.warning(f"Time embedding multiplier mismatch: checkpoint uses {checkpoint_time_emb_mult}, "
                               f"model initialized with {getattr(self, 'time_emb_mult', 4)}. Creating compatible layers.")

                # Create new time_mlp with the checkpoint's dimensions
                self.time_mlp = nn.Sequential(
                    SinusoidalPosEmb(self.time_emb_dim),
                    nn.Linear(self.time_emb_dim, self.time_emb_dim * checkpoint_time_emb_mult),
                    nn.GELU(),
                    nn.Linear(self.time_emb_dim * checkpoint_time_emb_mult, self.time_emb_dim),
                )

                # Update the stored multiplier if it exists
                if hasattr(self, 'time_emb_mult'):
                    self.time_emb_mult = checkpoint_time_emb_mult
                logging.info(f"Adjusted model to use time_emb_mult={checkpoint_time_emb_mult}")

        # Handle size mismatches in MLP layers
        # This is needed when the model architecture has changed between training and inference
        if 'input_projection.layers.0.weight' in state_dict:
            # Get the checkpoint dimensions
            checkpoint_input_dim = state_dict['input_projection.layers.0.weight'].shape[1]
            model_input_dim = None

            # Get the model's input dimension
            if hasattr(self, 'input_projection') and hasattr(self.input_projection, 'layers'):
                if len(self.input_projection.layers) > 0 and hasattr(self.input_projection.layers[0], 'weight'):
                    model_input_dim = self.input_projection.layers[0].weight.shape[1]

            if model_input_dim is not None and checkpoint_input_dim != model_input_dim:
                logging.warning(f"Input dimension mismatch: checkpoint uses {checkpoint_input_dim}, "
                               f"model initialized with {model_input_dim}. Recreating input projection.")

                # Create a new input projection with the checkpoint's dimensions
                # This assumes the output dimension is the same (typically hidden_dim)
                output_dim = state_dict['input_projection.layers.0.weight'].shape[0]
                self.input_projection = MLPBlock(checkpoint_input_dim, output_dim)

                # Also need to adjust the first layer of each MLP block
                for i in range(len(self.layers)):
                    if f'layers.{i}.layers.0.weight' in state_dict:
                        layer_input_dim = state_dict[f'layers.{i}.layers.0.weight'].shape[1]
                        layer_output_dim = state_dict[f'layers.{i}.layers.0.weight'].shape[0]
                        self.layers[i] = MLPBlock(layer_input_dim, layer_output_dim)

                logging.info(f"Adjusted model layers to match checkpoint dimensions")

        # Try to load the state dict with strict=False first to see what's missing
        missing_keys, unexpected_keys = [], []
        try:
            result = super().load_state_dict(state_dict, strict=False)
            missing_keys = result.missing_keys
            unexpected_keys = result.unexpected_keys

            if missing_keys:
                logging.warning(f"Missing keys: {missing_keys}")
            if unexpected_keys:
                logging.warning(f"Unexpected keys: {unexpected_keys}")

            if not missing_keys and not unexpected_keys:
                logging.info("All keys matched successfully!")

            return result
        except Exception as e:
            logging.error(f"Error during state dict loading: {e}")
            raise


    def forward(self,
                state: torch.Tensor,
                timestep: torch.Tensor,
                image_input: torch.Tensor
               ) -> torch.Tensor:
        """
        Forward pass of the diffusion model.

        Args:
            state (torch.Tensor): The current state tensor, shape (batch_size, state_dim).
            timestep (torch.Tensor): The current diffusion timestep, shape (batch_size,).
            image_input (torch.Tensor): Batch of input images, expected shape
                                        (batch_size, 3, H, W). Should be normalized
                                        as expected by ResNet.

        Returns:
            torch.Tensor: The predicted noise, shape (batch_size, state_dim).
        """
        # Ensure all model components are on the same device
        device = next(self.parameters()).device

        # Make sure the model components are on the correct device
        self.to(device)

        # Ensure all inputs are on the same device
        state = state.to(device)
        timestep = timestep.to(device)
        image_input = image_input.to(device)

        # Log device information (only once)
        if not hasattr(self, '_device_logged'):
            logging.info(f"Model is on device: {device}")
            self._device_logged = True

        # 1. Generate timestep embeddings
        time_emb = self.time_mlp(timestep.float())  # Shape: (batch_size, time_emb_dim)
        time_emb = time_emb.to(device)  # Ensure on correct device

        # 2. Extract image features using ResNet34
        image_features = self.image_backbone(image_input) # (batch_size, C, H', W')
        image_features = self.adaptive_pool(image_features) # (batch_size, C, 1, 1)
        image_features = torch.flatten(image_features, 1) # (batch_size, C)
        image_features = image_features.to(device)  # Ensure on correct device

        # Handle dimension mismatch between ResNet output and expected feature dimension
        actual_feature_dim = image_features.shape[-1]
        if actual_feature_dim != self.image_feature_dim:
            # Use a class variable to track if we've already logged this message
            if not hasattr(self, '_dimension_adaptation_logged'):
                logging.info(f"Adapting ResNet output dimension {actual_feature_dim} to expected {self.image_feature_dim}")
                self._dimension_adaptation_logged = True

            if actual_feature_dim < self.image_feature_dim:
                # If ResNet output is smaller than expected, pad with zeros
                padding_size = self.image_feature_dim - actual_feature_dim
                zero_padding = torch.zeros(image_features.shape[0], padding_size, device=device)
                image_features = torch.cat([image_features, zero_padding], dim=1)

                # Log this only once
                if not hasattr(self, '_padding_logged'):
                    logging.info(f"Padded ResNet output with {padding_size} zeros to match expected dimension")
                    self._padding_logged = True
            else:
                # If ResNet output is larger than expected, truncate
                image_features = image_features[:, :self.image_feature_dim]

                # Log this only once
                if not hasattr(self, '_truncation_logged'):
                    logging.info(f"Truncated ResNet output to match expected dimension {self.image_feature_dim}")
                    self._truncation_logged = True

        # 3. Concatenate state and image features
        combined_input = torch.cat([state, image_features], dim=1)
        combined_input = combined_input.to(device)  # Ensure on correct device

        # Ensure input_projection is on the correct device
        self.input_projection = self.input_projection.to(device)

        # 4. Process through the MLP layers with time conditioning
        x = self.input_projection(combined_input)
        x = x.to(device)  # Ensure on correct device

        for i, layer in enumerate(self.layers):
            # Ensure layer is on the correct device
            self.layers[i] = layer.to(device)

            # Condition on time embedding for each layer
            time_emb_conditioned = torch.cat([x, time_emb], dim=1)
            time_emb_conditioned = time_emb_conditioned.to(device)  # Ensure on correct device

            x = self.layers[i](time_emb_conditioned)
            x = x.to(device)  # Ensure on correct device

        # Ensure output_projection is on the correct device
        self.output_projection = self.output_projection.to(device)

        # 5. Final output projection
        predicted_noise = self.output_projection(x)
        predicted_noise = predicted_noise.to(device)  # Ensure on correct device

        return predicted_noise

# --- Example Usage ---
if __name__ == "__main__":
    # Configuration
    STATE_DIM = 7  # 6 joint angles + 1 gripper value
    IMAGE_H, IMAGE_W = 224, 224 # Example ResNet standard input size
    BATCH_SIZE = 4
    # IMAGE_FEATURE_DIM = 512 # No longer needed here, will be derived or passed

    # Instantiate the model with ResNet34
    model = DiffusionPolicyModel(
        state_dim=STATE_DIM,
        use_pretrained_resnet=True,
        freeze_resnet=True
    )
    print("-" * 30)
    print(f"ResNet34 model initialized. Image feature dim: {model.image_feature_dim}")
    num_params = sum(p.numel() for p in model.parameters() if p.requires_grad)
    print(f"Trainable Parameters: {num_params:,}")
    print("-" * 30)

    # Example with explicit image_feature_dim that matches ResNet34
    model_explicit_dim = DiffusionPolicyModel(
        state_dim=STATE_DIM,
        image_feature_dim=512, # Matches ResNet34
        use_pretrained_resnet=True,
        freeze_resnet=True
    )
    print("-" * 30)
    print(f"ResNet34 (explicit dim) model initialized. Image feature dim: {model_explicit_dim.image_feature_dim}")
    print("-" * 30)

    # Example that should trigger a warning (mismatched explicit dim)
    try:
        model_mismatch = DiffusionPolicyModel(
            state_dim=STATE_DIM,
            image_feature_dim=2048, # Mismatch for ResNet34
            use_pretrained_resnet=True,
            freeze_resnet=True
        )
        print(f"ResNet34 (mismatched dim) model initialized. Image feature dim: {model_mismatch.image_feature_dim}")
    except ValueError as ve:
        print(f"Caught expected ValueError for mismatch: {ve}") # Should not happen if logic is correct, warning instead
    print("-" * 30)


    # Create dummy input data (ensure image tensor has correct shape and type)
    dummy_state = torch.randn(BATCH_SIZE, STATE_DIM)
    dummy_timestep = torch.randint(0, 1000, (BATCH_SIZE,)) # Example diffusion timesteps
    # Image input needs shape (B, C, H, W) and be float
    dummy_image_input = torch.randn(BATCH_SIZE, 3, IMAGE_H, IMAGE_W)

    # Perform a forward pass with the model
    try:
        predicted_noise = model(dummy_state, dummy_timestep, dummy_image_input)
        print("Forward pass successful with ResNet34 model!")
        print(f"Output predicted noise shape: {predicted_noise.shape}")
        assert predicted_noise.shape == (BATCH_SIZE, STATE_DIM)
        print("Output shape matches state dimension. Basic check passed.")

    except Exception as e:
        print(f"Error during forward pass: {e}")
        logging.exception("Forward pass failed")

    print("-" * 30)
