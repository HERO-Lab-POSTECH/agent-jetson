"""Frozen sim-to-real policy contract shared by the board runtime modules."""
import numpy as np


POLICY_OBS_DIM = 72
PROPRIO_DIM = 20
LATENT_DIM = 9
ACTION_DIM = 8
CONTROL_DT = 0.02

HIST_LEN = 3
HIST_ACTION_LEN = 2
HIST_STRIDE = 3
HIST_FEAT_DIM = 18
HIST_JB_DIM = 10
INTEGRAL_DIM = 3
INTEGRAL_LEAK = 0.99
INTEGRAL_CLAMP = 2.0
INTEGRAL_GATED = True
INTEGRAL_SIGMA = np.array([0.10, 0.10, 0.10], dtype=np.float32)
BIAS_EMA_DIM = 3
BIAS_EMA_ALPHA = 0.99
NOMINAL_JOINT_POS = np.array([0.0, np.pi / 2.0], dtype=np.float32)
DELTA_SCALE = 0.10
TCN_HISTORY = 9
JOINT1_TRAIN_LIMIT = 4.0 * np.pi

L_LINK = 0.233
LPF_ALPHA = 0.2
THR_TAU_UP = 0.1
THR_TAU_DOWN = 0.05
THR_FILTER_DT = 0.02

TOPICS = {}


def wrap_angle(a):
    """Wrap angle(s) to [-pi, pi] via atan2(sin, cos)."""
    return np.arctan2(np.sin(a), np.cos(a))
