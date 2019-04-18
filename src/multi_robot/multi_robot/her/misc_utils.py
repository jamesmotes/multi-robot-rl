import numpy as np
import random
import zipfile

def set_global_seeds(i):
    try:
        import MPI
        rank = MPI.COMM_WORLD.Get_rank()
    except ImportError:
        rank = 0

    myseed = i  + 1000 * rank if i is not None else None
    try:
        import tensorflow as tf
        tf.set_random_seed(myseed)
    except ImportError:
        pass
    np.random.seed(myseed)
    random.seed(myseed)



def zipsame(*seqs):
    L = len(seqs[0])
    assert all(len(seq) == L for seq in seqs[1:])
    return zip(*seqs)