import torch
import time

# Check if CUDA is available
if not torch.cuda.is_available():
    print("CUDA is not available on your system.")
    exit()

# Function to perform operation on GPU
def gpu_operation(size):
    # Create tensors on the GPU
    tensor1 = torch.randn(size, device='cuda')
    tensor2 = torch.randn(size, device='cuda')

    start_time = time.time()
    # Perform addition
    for i in range(2):
        tensor1 += tensor2
    torch.cuda.synchronize()  # Wait for the operations to finish
    return time.time() - start_time



# Function to perform operation on CPU
def cpu_operation(size):
    # Create tensors on the CPU
    tensor1 = torch.randn(size)
    tensor2 = torch.randn(size)

    start_time = time.time()
    # Perform addition
    for i in range(2):
        tensor1 += tensor2
    return time.time() - start_time

# Test with a large array
array_size = (1000000, 100)
cpu_time = cpu_operation(array_size)
gpu_time = gpu_operation(array_size)

print(f"CPU time: {cpu_time:.4f} seconds")
print(f"GPU time: {gpu_time:.4f} seconds")


