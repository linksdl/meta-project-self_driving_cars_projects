import argparse
import glob
import os
import random

import numpy as np

from utils import get_module_logger


def split(data_dir):
    """
    Create three splits from the processed records. The files should be moved to new folders in the 
    same directory. This folder should be named train, val and test.

    args:
        - data_dir [str]: data directory, /home/workspace/data/
    """
    # Use os.symlink() instead of os.rename() to symbolically link the data to newly created train, val and test directories
    # - https://www.howtogeek.com/287014/how-to-create-and-use-symbolic-links-aka-symlinks-on-linux/
    
    records = glob.glob(data_dir + 'waymo/segment-*.tfrecord')
    # shuffle records
    random.shuffle(records)
    cut1 = int(0.7*len(records))
    cut2 = int(0.9*len(records))
    
    train_records = records[:cut1]
    val_records = records[cut1:cut2]
    test_records = records[cut2:]
    logger.info(f'Number of training files: {len(train_records)}')
    logger.info(f'Number of validation files: {len(val_records)}')
    logger.info(f'Number of testing files: {len(test_records)}')
    
    train_dir = os.path.join(data_dir, "train")
    val_dir = os.path.join(data_dir, "val")
    test_dir = os.path.join(data_dir, "test")

    os.makedirs(train_dir, exist_ok=True)
    os.makedirs(val_dir, exist_ok=True)
    os.makedirs(test_dir, exist_ok=True)
    
    [os.symlink(p, train_dir+ '/'+ p.split('/')[-1]) for p in train_records]
    [os.symlink(p, val_dir+'/'+p.split('/')[-1]) for p in val_records]
    [os.symlink(p, test_dir+'/'+p.split('/')[-1]) for p in test_records]
    

if __name__ == "__main__": 
    parser = argparse.ArgumentParser(description='Split data into training / validation / testing')
    parser.add_argument('--data_dir', required=True,
                        help='data directory')
    args = parser.parse_args()

    logger = get_module_logger(__name__)
    logger.info('Creating splits...')
    split(args.data_dir)