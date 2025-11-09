import numpy as np
from std_msgs.msg import Float32MultiArray, MultiArrayDimension


def numpy_to_multiarray(arr):
    """Convert a numpy array to a Float32MultiArray message."""
    msg = Float32MultiArray()
    shape = arr.shape

    # Build the layout
    stride = 1
    for size in reversed(shape):
        dim = MultiArrayDimension()
        dim.label = ""
        dim.size = size
        dim.stride = stride
        msg.layout.dim.insert(0, dim)
        stride *= size

    # Set the numpy arr data to Float32MultiArray msg
    msg.layout.data_offset = 0
    msg.data = arr.flatten().tolist()
    return msg


def multiarray_to_numpy(msg):
    """Convert a Float32MultiArray message back to a numpy array."""
    shape = [dim.size for dim in msg.layout.dim]
    arr = np.array(msg.data, dtype=np.float32)
    if shape:
        arr = arr.reshape(shape)
    return arr
